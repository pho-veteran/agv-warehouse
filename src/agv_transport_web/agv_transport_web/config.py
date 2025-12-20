#!/usr/bin/env python3
"""
Configuration Management for AGV Transport Web Dashboard

This module provides environment variable configuration for the web server,
default configuration files, configuration validation on startup, and
configuration reload capability.
"""

import os
import logging
from pathlib import Path
from typing import Dict, Any, Optional
from dataclasses import dataclass, field
from pydantic import BaseModel, Field, validator


logger = logging.getLogger(__name__)


@dataclass
class WebServerConfig:
    """Web server configuration with environment variable support."""
    
    # Server settings
    host: str = field(default_factory=lambda: os.getenv('AGV_WEB_HOST', '0.0.0.0'))
    port: int = field(default_factory=lambda: int(os.getenv('AGV_WEB_PORT', '8000')))
    reload: bool = field(default_factory=lambda: os.getenv('AGV_WEB_RELOAD', 'false').lower() == 'true')
    log_level: str = field(default_factory=lambda: os.getenv('AGV_WEB_LOG_LEVEL', 'info'))
    
    # ROS2 settings
    ros2_node_name: str = field(default_factory=lambda: os.getenv('AGV_ROS2_NODE_NAME', 'agv_transport_web_dashboard'))
    ros2_timeout: float = field(default_factory=lambda: float(os.getenv('AGV_ROS2_TIMEOUT', '10.0')))
    ros2_retry_attempts: int = field(default_factory=lambda: int(os.getenv('AGV_ROS2_RETRY_ATTEMPTS', '3')))
    
    # Station configuration
    station_config_path: str = field(default_factory=lambda: os.getenv(
        'AGV_STATION_CONFIG_PATH', 
        '/ros2_ws/src/agv_transport/config/warehouse_stations.yaml'
    ))
    station_config_reload_interval: float = field(default_factory=lambda: float(os.getenv(
        'AGV_STATION_CONFIG_RELOAD_INTERVAL', '60.0'
    )))
    
    # Update intervals (seconds)
    status_update_interval: float = field(default_factory=lambda: float(os.getenv('AGV_STATUS_UPDATE_INTERVAL', '3.0')))
    queue_update_interval: float = field(default_factory=lambda: float(os.getenv('AGV_QUEUE_UPDATE_INTERVAL', '5.0')))
    health_check_interval: float = field(default_factory=lambda: float(os.getenv('AGV_HEALTH_CHECK_INTERVAL', '30.0')))
    
    # CORS settings
    cors_origins: list = field(default_factory=lambda: os.getenv('AGV_CORS_ORIGINS', '*').split(','))
    cors_allow_credentials: bool = field(default_factory=lambda: os.getenv('AGV_CORS_ALLOW_CREDENTIALS', 'true').lower() == 'true')
    
    # Security settings
    max_request_size: int = field(default_factory=lambda: int(os.getenv('AGV_MAX_REQUEST_SIZE', '1048576')))  # 1MB
    rate_limit_requests: int = field(default_factory=lambda: int(os.getenv('AGV_RATE_LIMIT_REQUESTS', '100')))
    rate_limit_window: int = field(default_factory=lambda: int(os.getenv('AGV_RATE_LIMIT_WINDOW', '60')))
    
    # Development settings
    debug_mode: bool = field(default_factory=lambda: os.getenv('AGV_DEBUG_MODE', 'false').lower() == 'true')
    enable_docs: bool = field(default_factory=lambda: os.getenv('AGV_ENABLE_DOCS', 'true').lower() == 'true')
    
    def __post_init__(self):
        """Validate configuration after initialization."""
        self.validate()
    
    def validate(self) -> None:
        """Validate configuration values."""
        # Validate port range
        if not (1 <= self.port <= 65535):
            raise ValueError(f"Invalid port number: {self.port}. Must be between 1 and 65535.")
        
        # Validate log level
        valid_log_levels = ['debug', 'info', 'warning', 'error', 'critical']
        if self.log_level.lower() not in valid_log_levels:
            raise ValueError(f"Invalid log level: {self.log_level}. Must be one of {valid_log_levels}")
        
        # Validate timeout values
        if self.ros2_timeout <= 0:
            raise ValueError(f"ROS2 timeout must be positive: {self.ros2_timeout}")
        
        if self.station_config_reload_interval <= 0:
            raise ValueError(f"Station config reload interval must be positive: {self.station_config_reload_interval}")
        
        # Validate update intervals
        if self.status_update_interval <= 0:
            raise ValueError(f"Status update interval must be positive: {self.status_update_interval}")
        
        if self.queue_update_interval <= 0:
            raise ValueError(f"Queue update interval must be positive: {self.queue_update_interval}")
        
        # Validate retry attempts
        if self.ros2_retry_attempts < 0:
            raise ValueError(f"ROS2 retry attempts must be non-negative: {self.ros2_retry_attempts}")
        
        # Validate rate limiting
        if self.rate_limit_requests <= 0:
            raise ValueError(f"Rate limit requests must be positive: {self.rate_limit_requests}")
        
        if self.rate_limit_window <= 0:
            raise ValueError(f"Rate limit window must be positive: {self.rate_limit_window}")
        
        logger.info("Configuration validation passed")
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert configuration to dictionary."""
        return {
            'host': self.host,
            'port': self.port,
            'reload': self.reload,
            'log_level': self.log_level,
            'ros2_node_name': self.ros2_node_name,
            'ros2_timeout': self.ros2_timeout,
            'ros2_retry_attempts': self.ros2_retry_attempts,
            'station_config_path': self.station_config_path,
            'station_config_reload_interval': self.station_config_reload_interval,
            'status_update_interval': self.status_update_interval,
            'queue_update_interval': self.queue_update_interval,
            'health_check_interval': self.health_check_interval,
            'cors_origins': self.cors_origins,
            'cors_allow_credentials': self.cors_allow_credentials,
            'max_request_size': self.max_request_size,
            'rate_limit_requests': self.rate_limit_requests,
            'rate_limit_window': self.rate_limit_window,
            'debug_mode': self.debug_mode,
            'enable_docs': self.enable_docs
        }
    
    @classmethod
    def from_env(cls) -> 'WebServerConfig':
        """Create configuration from environment variables."""
        return cls()
    
    def reload_from_env(self) -> None:
        """Reload configuration from environment variables."""
        logger.info("Reloading configuration from environment variables")
        new_config = self.from_env()
        
        # Update all fields
        for field_name in self.__dataclass_fields__:
            setattr(self, field_name, getattr(new_config, field_name))
        
        # Re-validate
        self.validate()
        logger.info("Configuration reloaded successfully")


class ConfigurationManager:
    """Manages application configuration with reload capability."""
    
    def __init__(self):
        """Initialize configuration manager."""
        self._config: Optional[WebServerConfig] = None
        self._config_file_path: Optional[Path] = None
        self._last_reload_time: float = 0.0
        
    @property
    def config(self) -> WebServerConfig:
        """Get current configuration."""
        if self._config is None:
            self.load_config()
        return self._config
    
    def load_config(self, config_file_path: Optional[str] = None) -> None:
        """
        Load configuration from environment variables and optional config file.
        
        Args:
            config_file_path: Optional path to configuration file.
        """
        logger.info("Loading configuration...")
        
        # Load from environment variables
        self._config = WebServerConfig.from_env()
        
        # Load from config file if provided
        if config_file_path:
            self._config_file_path = Path(config_file_path)
            if self._config_file_path.exists():
                logger.info(f"Loading configuration from file: {config_file_path}")
                # TODO: Implement config file loading (JSON/YAML)
                # For now, we only support environment variables
            else:
                logger.warning(f"Configuration file not found: {config_file_path}")
        
        self._last_reload_time = os.path.getmtime(__file__)
        logger.info("Configuration loaded successfully")
    
    def reload_config(self) -> bool:
        """
        Reload configuration if it has changed.
        
        Returns:
            True if configuration was reloaded, False otherwise.
        """
        try:
            # Check if we should reload based on file modification time
            current_time = os.path.getmtime(__file__)
            if current_time <= self._last_reload_time:
                return False
            
            logger.info("Configuration file changed, reloading...")
            old_config = self._config.to_dict() if self._config else {}
            
            # Reload configuration
            self.load_config(str(self._config_file_path) if self._config_file_path else None)
            
            # Check if configuration actually changed
            new_config = self._config.to_dict()
            if old_config != new_config:
                logger.info("Configuration updated with new values")
                return True
            else:
                logger.info("Configuration file changed but values are the same")
                return False
                
        except Exception as e:
            logger.error(f"Failed to reload configuration: {e}")
            return False
    
    def validate_config(self) -> bool:
        """
        Validate current configuration.
        
        Returns:
            True if configuration is valid, False otherwise.
        """
        try:
            if self._config is None:
                logger.error("No configuration loaded")
                return False
            
            self._config.validate()
            logger.info("Configuration validation passed")
            return True
            
        except Exception as e:
            logger.error(f"Configuration validation failed: {e}")
            return False
    
    def get_uvicorn_config(self) -> Dict[str, Any]:
        """
        Get configuration dictionary for uvicorn server.
        
        Returns:
            Dictionary with uvicorn configuration parameters.
        """
        config = self.config
        return {
            'host': config.host,
            'port': config.port,
            'reload': config.reload,
            'log_level': config.log_level,
            'access_log': config.debug_mode,
            'use_colors': True,
            'server_header': False,
            'date_header': False
        }


# Global configuration manager instance
config_manager = ConfigurationManager()


def get_config() -> WebServerConfig:
    """
    Get the current application configuration.
    
    Returns:
        Current WebServerConfig instance.
    """
    return config_manager.config


def reload_config() -> bool:
    """
    Reload application configuration.
    
    Returns:
        True if configuration was reloaded, False otherwise.
    """
    return config_manager.reload_config()


def validate_config() -> bool:
    """
    Validate current application configuration.
    
    Returns:
        True if configuration is valid, False otherwise.
    """
    return config_manager.validate_config()


def create_default_env_file(file_path: str = '.env') -> None:
    """
    Create a default .env file with all available configuration options.
    
    Args:
        file_path: Path where to create the .env file.
    """
    env_content = """# AGV Transport Web Dashboard Configuration
# Copy this file to .env and modify values as needed

# Web Server Settings
AGV_WEB_HOST=0.0.0.0
AGV_WEB_PORT=8000
AGV_WEB_RELOAD=false
AGV_WEB_LOG_LEVEL=info

# ROS2 Settings
AGV_ROS2_NODE_NAME=agv_transport_web_dashboard
AGV_ROS2_TIMEOUT=10.0
AGV_ROS2_RETRY_ATTEMPTS=3

# Station Configuration
AGV_STATION_CONFIG_PATH=/ros2_ws/src/agv_transport/config/warehouse_stations.yaml
AGV_STATION_CONFIG_RELOAD_INTERVAL=60.0

# Update Intervals (seconds)
AGV_STATUS_UPDATE_INTERVAL=3.0
AGV_QUEUE_UPDATE_INTERVAL=5.0
AGV_HEALTH_CHECK_INTERVAL=30.0

# CORS Settings
AGV_CORS_ORIGINS=*
AGV_CORS_ALLOW_CREDENTIALS=true

# Security Settings
AGV_MAX_REQUEST_SIZE=1048576
AGV_RATE_LIMIT_REQUESTS=100
AGV_RATE_LIMIT_WINDOW=60

# Development Settings
AGV_DEBUG_MODE=false
AGV_ENABLE_DOCS=true
"""
    
    with open(file_path, 'w') as f:
        f.write(env_content)
    
    logger.info(f"Default .env file created at: {file_path}")


if __name__ == "__main__":
    # Test configuration loading
    config = get_config()
    print("Configuration loaded successfully:")
    print(f"  Host: {config.host}")
    print(f"  Port: {config.port}")
    print(f"  ROS2 Node: {config.ros2_node_name}")
    print(f"  Station Config: {config.station_config_path}")
    
    # Create default .env file
    create_default_env_file('example.env')
    print("Example .env file created")