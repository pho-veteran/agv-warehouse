#!/usr/bin/env python3
"""
API Routes for AGV Transport Web Dashboard

This module provides REST API endpoints for the web dashboard including:
- AGV status information
- Station management
- Transport order operations
- Queue status
"""

import asyncio
import logging
import time
import uuid
from datetime import datetime
from typing import Dict, List, Any, Optional

from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel, Field, validator

# Import transport models
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../../agv_transport'))
from agv_transport.models import TransportOrder, TransportStatus

from .ros2_bridge import AsyncROS2Bridge
from .station_manager import StationManager
from .error_handlers import (
    AGVTransportError, ROS2ConnectionError, StationConfigurationError,
    TransportOrderError, ValidationError, validate_station_name,
    validate_pickup_station, validate_different_stations, validate_priority
)


logger = logging.getLogger(__name__)

# Global references (will be set by main app)
ros2_bridge: Optional[AsyncROS2Bridge] = None
station_manager: Optional[StationManager] = None


def set_global_dependencies(bridge: AsyncROS2Bridge, manager: StationManager):
    """
    Set global dependencies for API routes.
    
    Args:
        bridge: AsyncROS2Bridge instance
        manager: StationManager instance
    """
    global ros2_bridge, station_manager
    ros2_bridge = bridge
    station_manager = manager


# Pydantic models for API requests and responses
class TransportOrderRequest(BaseModel):
    """Request model for creating transport orders."""
    pickup_station: str = Field(..., description="Name of the pickup station")
    dropoff_station: str = Field(..., description="Name of the dropoff station")
    priority: int = Field(default=0, ge=0, le=10, description="Order priority (0-10)")
    
    @validator('pickup_station', 'dropoff_station')
    def validate_station_names(cls, v):
        """Validate station names are not empty."""
        if not v or not v.strip():
            raise ValueError('Station name cannot be empty')
        return v.strip()
    
    @validator('dropoff_station')
    def validate_different_stations(cls, v, values):
        """Validate pickup and dropoff stations are different."""
        if 'pickup_station' in values and v == values['pickup_station']:
            raise ValueError('Pickup and dropoff stations must be different')
        return v


class TransportOrderResponse(BaseModel):
    """Response model for transport order operations."""
    order_id: str
    pickup_station: str
    dropoff_station: str
    priority: int
    status: str
    created_at: str
    message: str


class StationInfo(BaseModel):
    """Response model for station information."""
    station_name: str
    station_type: str
    x: float
    y: float
    yaw: float
    description: str


class StatusResponse(BaseModel):
    """Response model for AGV status."""
    current_state: str
    battery_level: float
    current_station: Optional[str]
    target_station: Optional[str]
    active_order_id: Optional[str]
    queue_length: int
    last_update: str
    navigation_progress: float
    connection_healthy: bool
    age_seconds: float
    is_stale: bool


class OrderQueueResponse(BaseModel):
    """Response model for order queue status."""
    orders: List[Dict[str, Any]]
    total_count: int
    active_orders: int
    pending_orders: int
    completed_orders: int
    last_updated: str


# Dependency functions
async def get_ros2_bridge() -> AsyncROS2Bridge:
    """
    Dependency to get ROS2 bridge instance.
    
    Returns:
        AsyncROS2Bridge instance
        
    Raises:
        ROS2ConnectionError: If ROS2 bridge is not available
    """
    if not ros2_bridge or not ros2_bridge.is_initialized:
        raise ROS2ConnectionError("ROS2 bridge not available - transport system may be offline")
    return ros2_bridge


async def get_station_manager() -> StationManager:
    """
    Dependency to get station manager instance.
    
    Returns:
        StationManager instance
        
    Raises:
        StationConfigurationError: If station manager is not available
    """
    if not station_manager:
        raise StationConfigurationError("Station manager not available")
    return station_manager


# Create API router
router = APIRouter(prefix="/api", tags=["API"])


@router.get("/status", response_model=StatusResponse)
async def get_agv_status(bridge: AsyncROS2Bridge = Depends(get_ros2_bridge)):
    """
    Get current AGV status from ROS2 transport system.
    
    Args:
        bridge: ROS2 bridge dependency
        
    Returns:
        Current AGV status information
        
    Raises:
        HTTPException: If status data is not available
    """
    try:
        status_data = await bridge.get_status()
        
        if not status_data:
            raise ROS2ConnectionError("AGV status data not available - transport system may be offline")
        
        status_info = status_data.get('status', {})
        
        # Convert to response model
        response = StatusResponse(
            current_state=status_info.get('current_state', 'UNKNOWN'),
            battery_level=status_info.get('battery_level', 0.0),
            current_station=status_info.get('current_station'),
            target_station=status_info.get('target_station'),
            active_order_id=status_info.get('active_order_id'),
            queue_length=status_info.get('queue_length', 0),
            last_update=datetime.fromtimestamp(
                status_data.get('timestamp', time.time())
            ).isoformat(),
            navigation_progress=status_info.get('navigation_progress', 0.0),
            connection_healthy=status_data.get('connection_healthy', False),
            age_seconds=status_data.get('age_seconds', 0.0),
            is_stale=status_data.get('is_stale', True)
        )
        
        return response
        
    except (ROS2ConnectionError, AGVTransportError):
        raise
    except Exception as e:
        logger.error(f"Error getting AGV status: {e}")
        raise AGVTransportError(f"Internal error getting AGV status: {str(e)}")


@router.get("/stations", response_model=List[StationInfo])
async def get_stations(manager: StationManager = Depends(get_station_manager)):
    """
    Get list of all available stations.
    
    Args:
        manager: Station manager dependency
        
    Returns:
        List of all stations with their information
        
    Raises:
        HTTPException: If station data cannot be loaded
    """
    try:
        # Reload configuration to get latest data
        manager.reload_configuration()
        
        stations_data = manager.get_all_stations()
        
        # Convert to response model
        stations = [
            StationInfo(
                station_name=station['station_name'],
                station_type=station['station_type'],
                x=station['x'],
                y=station['y'],
                yaw=station['yaw'],
                description=station['description']
            )
            for station in stations_data
        ]
        
        return stations
        
    except Exception as e:
        logger.error(f"Error getting stations: {e}")
        raise StationConfigurationError(f"Error loading station configuration: {str(e)}")


@router.get("/stations/pickup", response_model=List[StationInfo])
async def get_pickup_stations(manager: StationManager = Depends(get_station_manager)):
    """
    Get list of stations suitable for pickup (excludes charging stations).
    
    Args:
        manager: Station manager dependency
        
    Returns:
        List of pickup-suitable stations
    """
    try:
        stations_data = manager.get_pickup_stations()
        
        stations = [
            StationInfo(
                station_name=station['station_name'],
                station_type=station['station_type'],
                x=station['x'],
                y=station['y'],
                yaw=station['yaw'],
                description=station['description']
            )
            for station in stations_data
        ]
        
        return stations
        
    except Exception as e:
        logger.error(f"Error getting pickup stations: {e}")
        raise StationConfigurationError(f"Error loading pickup stations: {str(e)}")


@router.get("/stations/dropoff", response_model=List[StationInfo])
async def get_dropoff_stations(manager: StationManager = Depends(get_station_manager)):
    """
    Get list of stations suitable for dropoff (includes all stations).
    
    Args:
        manager: Station manager dependency
        
    Returns:
        List of dropoff-suitable stations
    """
    try:
        stations_data = manager.get_dropoff_stations()
        
        stations = [
            StationInfo(
                station_name=station['station_name'],
                station_type=station['station_type'],
                x=station['x'],
                y=station['y'],
                yaw=station['yaw'],
                description=station['description']
            )
            for station in stations_data
        ]
        
        return stations
        
    except Exception as e:
        logger.error(f"Error getting dropoff stations: {e}")
        raise StationConfigurationError(f"Error loading dropoff stations: {str(e)}")


@router.get("/orders", response_model=OrderQueueResponse)
async def get_order_queue(bridge: AsyncROS2Bridge = Depends(get_ros2_bridge)):
    """
    Get current transport order queue status.
    
    Args:
        bridge: ROS2 bridge dependency
        
    Returns:
        Current order queue information
    """
    try:
        # Get tracked orders from bridge
        orders = await bridge.get_orders()
        
        # Count orders by status
        active_orders = sum(1 for o in orders if o.get('status') == 'ACTIVE')
        pending_orders = sum(1 for o in orders if o.get('status') == 'QUEUED')
        completed_orders = sum(1 for o in orders if o.get('status') == 'COMPLETED')
        
        # Format orders for response
        formatted_orders = []
        for order in orders:
            created_at = order.get('created_at') or order.get('timestamp', time.time())
            formatted_orders.append({
                'order_id': order.get('order_id', 'Unknown'),
                'pickup_station': order.get('pickup_station', 'Unknown'),
                'dropoff_station': order.get('dropoff_station', 'Unknown'),
                'status': order.get('status', 'UNKNOWN'),
                'priority': order.get('priority', 0),
                'created_at': datetime.fromtimestamp(created_at).isoformat()
            })
        
        response = OrderQueueResponse(
            orders=formatted_orders,
            total_count=len(formatted_orders),
            active_orders=active_orders,
            pending_orders=pending_orders,
            completed_orders=completed_orders,
            last_updated=datetime.now().isoformat()
        )
        
        return response
        
    except Exception as e:
        logger.error(f"Error getting order queue: {e}")
        raise AGVTransportError(f"Error getting order queue: {str(e)}")


@router.post("/orders", response_model=TransportOrderResponse, status_code=201)
async def create_transport_order(
    order_request: TransportOrderRequest,
    bridge: AsyncROS2Bridge = Depends(get_ros2_bridge),
    manager: StationManager = Depends(get_station_manager)
):
    """
    Create a new transport order.
    
    Args:
        order_request: Transport order request data
        bridge: ROS2 bridge dependency
        manager: Station manager dependency
        
    Returns:
        Created transport order information
        
    Raises:
        HTTPException: If order creation fails or validation errors occur
    """
    try:
        # Validate station names using custom validators
        pickup_station = validate_pickup_station(order_request.pickup_station, manager)
        dropoff_station = validate_station_name(order_request.dropoff_station, manager)
        
        # Validate stations are different
        pickup_station, dropoff_station = validate_different_stations(pickup_station, dropoff_station)
        
        # Validate priority
        priority = validate_priority(order_request.priority)
        
        # Create transport order
        order_id = f"order_{int(time.time())}_{uuid.uuid4().hex[:8]}"
        
        transport_order = TransportOrder(
            order_id=order_id,
            pickup_station=pickup_station,
            dropoff_station=dropoff_station,
            priority=priority,
            timestamp=time.time()
        )
        
        # Publish order to ROS2
        success = await bridge.publish_order(transport_order)
        
        if not success:
            raise TransportOrderError("Failed to publish transport order to ROS2 system")
        
        # Create response
        response = TransportOrderResponse(
            order_id=order_id,
            pickup_station=pickup_station,
            dropoff_station=dropoff_station,
            priority=priority,
            status="QUEUED",
            created_at=datetime.fromtimestamp(transport_order.timestamp).isoformat(),
            message="Transport order created successfully"
        )
        
        logger.info(f"Created transport order: {order_id}")
        return response
        
    except (ValidationError, TransportOrderError, ROS2ConnectionError, StationConfigurationError):
        raise
    except Exception as e:
        logger.error(f"Error creating transport order: {e}")
        raise TransportOrderError(f"Internal error creating transport order: {str(e)}")


@router.delete("/orders/{order_id}")
async def cancel_transport_order(
    order_id: str,
    bridge: AsyncROS2Bridge = Depends(get_ros2_bridge)
):
    """
    Cancel a transport order (placeholder implementation).
    
    Args:
        order_id: ID of the order to cancel
        bridge: ROS2 bridge dependency
        
    Returns:
        Cancellation confirmation
        
    Note:
        This is a placeholder implementation. Full cancellation would require
        integration with the transport task manager's cancellation system.
    """
    # This is a placeholder - full implementation would require
    # integration with transport task manager cancellation
    return {
        "message": f"Order cancellation requested for {order_id}",
        "note": "Cancellation feature not yet fully implemented"
    }


# Function to register routes with the main app
def setup_api_routes(app, bridge: AsyncROS2Bridge, manager: StationManager):
    """
    Setup API routes with the FastAPI app.
    
    Args:
        app: FastAPI application instance
        bridge: AsyncROS2Bridge instance
        manager: StationManager instance
    """
    # Set global dependencies
    set_global_dependencies(bridge, manager)
    
    # Include the router
    app.include_router(router)
    
    logger.info("API routes configured successfully")