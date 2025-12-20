#!/usr/bin/env python3
"""
FastAPI Application for AGV Transport Web Dashboard

This module provides the main FastAPI application with CORS middleware,
template configuration, static file serving, and application lifecycle events.
"""

import asyncio
import logging
import os
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Dict, Any

from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from fastapi.responses import HTMLResponse

from .ros2_bridge import AsyncROS2Bridge
from .station_manager import StationManager
from .api_routes import setup_api_routes
from .error_handlers import setup_error_handlers, add_request_id_middleware
from .config import get_config, validate_config, reload_config


# Configure logging
def setup_logging():
    """Configure application logging."""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.StreamHandler(),
            logging.FileHandler('agv_transport_web.log', mode='a')
        ]
    )
    
    # Set specific log levels for different components
    logging.getLogger('uvicorn').setLevel(logging.INFO)
    logging.getLogger('fastapi').setLevel(logging.INFO)
    logging.getLogger('agv_transport_web').setLevel(logging.DEBUG)

setup_logging()
logger = logging.getLogger(__name__)

# Global instances
ros2_bridge: AsyncROS2Bridge = None
station_manager: StationManager = None


@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Application lifespan context manager for startup and shutdown events.
    
    Handles initialization and cleanup of ROS2 bridge and other resources.
    """
    global ros2_bridge, station_manager
    
    # Startup
    logger.info("Starting AGV Transport Web Dashboard...")
    
    try:
        # Load and validate configuration
        config = get_config()
        if not validate_config():
            logger.error("Configuration validation failed")
            raise RuntimeError("Invalid configuration")
        
        logger.info(f"Configuration loaded - Host: {config.host}:{config.port}")
        
        # Initialize station manager
        logger.info("Initializing station manager...")
        station_manager = StationManager(config_path=config.station_config_path)
        if not station_manager.is_loaded:
            logger.warning("No stations loaded - some functionality may be limited")
        else:
            logger.info(f"Loaded {station_manager.station_count} stations")
        
        # Initialize ROS2 bridge
        logger.info("Initializing ROS2 bridge...")
        ros2_bridge = AsyncROS2Bridge(
            node_name=config.ros2_node_name,
            timeout=config.ros2_timeout,
            retry_attempts=config.ros2_retry_attempts
        )
        await ros2_bridge.initialize()
        logger.info("ROS2 bridge initialized successfully")
        
        # Verify connection health
        connection_info = await ros2_bridge.get_connection_info()
        if connection_info.get('healthy', False):
            logger.info("ROS2 connection is healthy")
        else:
            logger.warning("ROS2 connection may not be healthy - check transport system")
        
        logger.info("AGV Transport Web Dashboard startup complete")
        
        # Setup API routes now that dependencies are initialized
        from .api_routes import setup_api_routes
        setup_api_routes(app, ros2_bridge, station_manager)
        logger.info("API routes configured")
        
    except Exception as e:
        logger.error(f"Failed to initialize application: {e}")
        # Continue startup even if some components fail
        # This allows the web interface to show error states
    
    yield
    
    # Shutdown
    logger.info("Shutting down AGV Transport Web Dashboard...")
    
    try:
        if ros2_bridge:
            await ros2_bridge.shutdown()
            logger.info("ROS2 bridge shutdown complete")
    except Exception as e:
        logger.error(f"Error during shutdown: {e}")
    
    logger.info("AGV Transport Web Dashboard shutdown complete")


def create_app() -> FastAPI:
    """
    Create and configure the FastAPI application.
    
    Returns:
        Configured FastAPI application instance.
    """
    # Get configuration
    config = get_config()
    
    # Create FastAPI app with lifespan management
    app = FastAPI(
        title="AGV Transport Web Dashboard",
        description="Web interface for monitoring and controlling AGV transport operations",
        version="1.0.0",
        lifespan=lifespan,
        docs_url="/docs" if config.enable_docs else None,
        redoc_url="/redoc" if config.enable_docs else None
    )
    
    # Configure CORS middleware
    app.add_middleware(
        CORSMiddleware,
        allow_origins=config.cors_origins,
        allow_credentials=config.cors_allow_credentials,
        allow_methods=["GET", "POST", "PUT", "DELETE"],
        allow_headers=["*"],
    )
    
    # Add request ID middleware for error tracking
    app.middleware("http")(add_request_id_middleware)
    
    # Setup error handlers
    setup_error_handlers(app)
    
    # Setup static files and templates
    setup_static_files(app)
    
    # Add health check endpoints
    setup_health_endpoints(app)
    
    # Add main dashboard route
    setup_dashboard_routes(app)
    
    # Add configuration management endpoints
    setup_config_endpoints(app)
    
    # Setup API routes (will be configured during startup)
    # This is done in the lifespan function after dependencies are initialized
    
    return app


def setup_static_files(app: FastAPI):
    """
    Configure static file serving and Jinja2 templates.
    
    Args:
        app: FastAPI application instance.
    """
    # Get the package directory (agv_transport_web/agv_transport_web/)
    package_dir = Path(__file__).parent
    
    # Go up one level to get the project root (agv_transport_web/)
    project_dir = package_dir.parent
    
    # Helper function to check if directory has actual files (not just empty or .gitkeep)
    def has_real_files(directory: Path) -> bool:
        if not directory.exists():
            return False
        files = list(directory.glob("*"))
        # Filter out .gitkeep and check if there are real files
        real_files = [f for f in files if f.name != ".gitkeep" and not f.name.startswith(".")]
        return len(real_files) > 0
    
    # Setup static files directory - prefer project root if it has files
    static_dir = project_dir / "static"
    if not has_real_files(static_dir):
        static_dir = package_dir / "static"
        if not static_dir.exists():
            static_dir.mkdir(parents=True, exist_ok=True)
            logger.info(f"Created static directory: {static_dir}")
    
    logger.info(f"Using static directory: {static_dir}")
    
    # Mount static files
    app.mount("/static", StaticFiles(directory=str(static_dir)), name="static")
    
    # Setup templates directory - prefer project root if it has template files
    templates_dir = project_dir / "templates"
    if not has_real_files(templates_dir):
        templates_dir = package_dir / "templates"
        if not templates_dir.exists():
            templates_dir.mkdir(parents=True, exist_ok=True)
            logger.info(f"Created templates directory: {templates_dir}")
    
    logger.info(f"Using templates directory: {templates_dir}")
    
    # List template files for debugging
    if templates_dir.exists():
        template_files = list(templates_dir.glob("*.html"))
        logger.info(f"Found {len(template_files)} template files: {[f.name for f in template_files]}")
    
    # Configure Jinja2 templates
    app.state.templates = Jinja2Templates(directory=str(templates_dir))
    
    logger.info(f"Static files configured: {static_dir}")
    logger.info(f"Templates configured: {templates_dir}")


def setup_health_endpoints(app: FastAPI):
    """
    Setup health check endpoints for monitoring.
    
    Args:
        app: FastAPI application instance.
    """
    
    @app.get("/health", tags=["Health"])
    async def health_check() -> Dict[str, Any]:
        """
        Basic health check endpoint.
        
        Returns:
            Health status information.
        """
        return {
            "status": "healthy",
            "service": "AGV Transport Web Dashboard",
            "version": "1.0.0"
        }
    
    @app.get("/health/detailed", tags=["Health"])
    async def detailed_health_check() -> Dict[str, Any]:
        """
        Detailed health check including ROS2 and station manager status.
        
        Returns:
            Comprehensive health status information.
        """
        global ros2_bridge, station_manager
        
        health_info = {
            "status": "healthy",
            "service": "AGV Transport Web Dashboard",
            "version": "1.0.0",
            "timestamp": asyncio.get_event_loop().time(),
            "components": {}
        }
        
        # Check ROS2 bridge health
        if ros2_bridge:
            try:
                connection_info = await ros2_bridge.get_connection_info()
                health_info["components"]["ros2_bridge"] = {
                    "status": "healthy" if connection_info.get("healthy", False) else "unhealthy",
                    "initialized": ros2_bridge.is_initialized,
                    "connection_info": connection_info
                }
            except Exception as e:
                health_info["components"]["ros2_bridge"] = {
                    "status": "error",
                    "error": str(e)
                }
        else:
            health_info["components"]["ros2_bridge"] = {
                "status": "not_initialized"
            }
        
        # Check station manager health
        if station_manager:
            health_info["components"]["station_manager"] = {
                "status": "healthy" if station_manager.is_loaded else "warning",
                "stations_loaded": station_manager.station_count,
                "config_path": station_manager.config_path
            }
        else:
            health_info["components"]["station_manager"] = {
                "status": "not_initialized"
            }
        
        # Determine overall status
        component_statuses = [
            comp.get("status", "unknown") 
            for comp in health_info["components"].values()
        ]
        
        if "error" in component_statuses:
            health_info["status"] = "error"
        elif "unhealthy" in component_statuses:
            health_info["status"] = "unhealthy"
        elif "warning" in component_statuses:
            health_info["status"] = "warning"
        
        return health_info


def setup_config_endpoints(app: FastAPI):
    """
    Setup configuration management endpoints.
    
    Args:
        app: FastAPI application instance.
    """
    
    @app.get("/config", tags=["Configuration"])
    async def get_configuration() -> Dict[str, Any]:
        """
        Get current application configuration.
        
        Returns:
            Current configuration dictionary.
        """
        config = get_config()
        return {
            "status": "success",
            "configuration": config.to_dict(),
            "validation_status": "valid" if validate_config() else "invalid"
        }
    
    @app.post("/config/reload", tags=["Configuration"])
    async def reload_configuration() -> Dict[str, Any]:
        """
        Reload application configuration from environment variables.
        
        Returns:
            Reload status and updated configuration.
        """
        try:
            reloaded = reload_config()
            config = get_config()
            
            return {
                "status": "success",
                "reloaded": reloaded,
                "message": "Configuration reloaded successfully" if reloaded else "Configuration unchanged",
                "configuration": config.to_dict(),
                "validation_status": "valid" if validate_config() else "invalid"
            }
        except Exception as e:
            logger.error(f"Failed to reload configuration: {e}")
            return {
                "status": "error",
                "message": f"Failed to reload configuration: {str(e)}",
                "reloaded": False
            }
    
    @app.get("/config/validate", tags=["Configuration"])
    async def validate_configuration() -> Dict[str, Any]:
        """
        Validate current application configuration.
        
        Returns:
            Configuration validation status.
        """
        try:
            is_valid = validate_config()
            return {
                "status": "success",
                "valid": is_valid,
                "message": "Configuration is valid" if is_valid else "Configuration validation failed"
            }
        except Exception as e:
            logger.error(f"Configuration validation error: {e}")
            return {
                "status": "error",
                "valid": False,
                "message": f"Configuration validation error: {str(e)}"
            }


def setup_dashboard_routes(app: FastAPI):
    """
    Setup main dashboard routes.
    
    Args:
        app: FastAPI application instance.
    """
    
    @app.get("/", response_class=HTMLResponse, tags=["Dashboard"])
    async def dashboard_root(request: Request):
        """
        Redirect root to dashboard.
        
        Args:
            request: FastAPI request object.
            
        Returns:
            Redirect response to dashboard.
        """
        return await dashboard(request)
    
    @app.get("/dashboard", response_class=HTMLResponse, tags=["Dashboard"])
    async def dashboard(request: Request):
        """
        Main dashboard page.
        
        Args:
            request: FastAPI request object.
            
        Returns:
            HTML response with dashboard template.
        """
        global ros2_bridge, station_manager
        
        # Prepare template context
        context = {
            "request": request,
            "title": "AGV Transport Dashboard",
            "ros2_connected": ros2_bridge and ros2_bridge.is_healthy if ros2_bridge else False,
            "stations_loaded": station_manager and station_manager.is_loaded if station_manager else False,
            "station_count": station_manager.station_count if station_manager else 0
        }
        
        # Render dashboard template (will be created in later tasks)
        try:
            return app.state.templates.TemplateResponse("dashboard.html", context)
        except Exception as e:
            logger.error(f"Error rendering dashboard template: {e}")
            # Return basic HTML if template is not available
            return HTMLResponse(
                content=f"""
                <!DOCTYPE html>
                <html>
                <head>
                    <title>AGV Transport Dashboard</title>
                </head>
                <body>
                    <h1>AGV Transport Dashboard</h1>
                    <p>Dashboard template not yet available.</p>
                    <p>ROS2 Connected: {context['ros2_connected']}</p>
                    <p>Stations Loaded: {context['stations_loaded']} ({context['station_count']} stations)</p>
                    <p>Health Check: <a href="/health/detailed">/health/detailed</a></p>
                </body>
                </html>
                """,
                status_code=200
            )


# Create the FastAPI app instance
app = create_app()


if __name__ == "__main__":
    import uvicorn
    from .config import get_config, config_manager
    
    # Load configuration
    config = get_config()
    
    # Get uvicorn configuration
    uvicorn_config = config_manager.get_uvicorn_config()
    
    # Run the application
    uvicorn.run(
        "agv_transport_web.app:app",
        **uvicorn_config
    )