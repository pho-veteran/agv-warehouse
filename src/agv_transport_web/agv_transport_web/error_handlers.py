#!/usr/bin/env python3
"""
Error Handling and Validation for AGV Transport Web Dashboard

This module provides comprehensive error handling middleware, custom exception
classes, and validation utilities for the FastAPI application.
"""

import logging
import traceback
from typing import Dict, Any, Optional, Union

from fastapi import FastAPI, Request, HTTPException
from fastapi.responses import JSONResponse
from fastapi.exceptions import RequestValidationError
from pydantic import ValidationError
from starlette.exceptions import HTTPException as StarletteHTTPException


logger = logging.getLogger(__name__)


class AGVTransportError(Exception):
    """Base exception class for AGV transport system errors."""
    
    def __init__(self, message: str, error_code: Optional[str] = None, details: Optional[Dict[str, Any]] = None):
        self.message = message
        self.error_code = error_code or "AGV_TRANSPORT_ERROR"
        self.details = details or {}
        super().__init__(self.message)


class ROS2ConnectionError(AGVTransportError):
    """Exception for ROS2 connection related errors."""
    
    def __init__(self, message: str, details: Optional[Dict[str, Any]] = None):
        super().__init__(message, "ROS2_CONNECTION_ERROR", details)


class StationConfigurationError(AGVTransportError):
    """Exception for station configuration related errors."""
    
    def __init__(self, message: str, details: Optional[Dict[str, Any]] = None):
        super().__init__(message, "STATION_CONFIG_ERROR", details)


class TransportOrderError(AGVTransportError):
    """Exception for transport order related errors."""
    
    def __init__(self, message: str, details: Optional[Dict[str, Any]] = None):
        super().__init__(message, "TRANSPORT_ORDER_ERROR", details)


class ValidationError(AGVTransportError):
    """Exception for validation related errors."""
    
    def __init__(self, message: str, field: Optional[str] = None, value: Optional[Any] = None):
        details = {}
        if field:
            details["field"] = field
        if value is not None:
            details["value"] = value
        super().__init__(message, "VALIDATION_ERROR", details)


def create_error_response(
    status_code: int,
    message: str,
    error_code: Optional[str] = None,
    details: Optional[Dict[str, Any]] = None,
    request_id: Optional[str] = None
) -> Dict[str, Any]:
    """
    Create a standardized error response.
    
    Args:
        status_code: HTTP status code
        message: Error message
        error_code: Optional error code for categorization
        details: Optional additional error details
        request_id: Optional request ID for tracking
        
    Returns:
        Standardized error response dictionary
    """
    error_response = {
        "error": {
            "status_code": status_code,
            "message": message,
            "error_code": error_code or "UNKNOWN_ERROR",
            "timestamp": logger.handlers[0].formatter.formatTime(logging.LogRecord(
                name="", level=0, pathname="", lineno=0, msg="", args=(), exc_info=None
            )) if logger.handlers else None
        }
    }
    
    if details:
        error_response["error"]["details"] = details
    
    if request_id:
        error_response["error"]["request_id"] = request_id
    
    return error_response


async def agv_transport_exception_handler(request: Request, exc: AGVTransportError) -> JSONResponse:
    """
    Handle AGV transport system specific exceptions.
    
    Args:
        request: FastAPI request object
        exc: AGVTransportError exception
        
    Returns:
        JSON error response
    """
    # Determine appropriate HTTP status code based on error type
    status_code = 500
    if isinstance(exc, ROS2ConnectionError):
        status_code = 503  # Service Unavailable
    elif isinstance(exc, StationConfigurationError):
        status_code = 500  # Internal Server Error
    elif isinstance(exc, TransportOrderError):
        status_code = 400  # Bad Request
    elif isinstance(exc, ValidationError):
        status_code = 422  # Unprocessable Entity
    
    # Log the error
    logger.error(f"AGV Transport Error: {exc.message}", extra={
        "error_code": exc.error_code,
        "details": exc.details,
        "request_url": str(request.url),
        "request_method": request.method
    })
    
    # Create error response
    error_response = create_error_response(
        status_code=status_code,
        message=exc.message,
        error_code=exc.error_code,
        details=exc.details,
        request_id=getattr(request.state, 'request_id', None)
    )
    
    return JSONResponse(
        status_code=status_code,
        content=error_response
    )


async def http_exception_handler(request: Request, exc: HTTPException) -> JSONResponse:
    """
    Handle FastAPI HTTP exceptions.
    
    Args:
        request: FastAPI request object
        exc: HTTPException
        
    Returns:
        JSON error response
    """
    # Log the error
    logger.warning(f"HTTP Exception: {exc.detail}", extra={
        "status_code": exc.status_code,
        "request_url": str(request.url),
        "request_method": request.method
    })
    
    # Create error response
    error_response = create_error_response(
        status_code=exc.status_code,
        message=exc.detail,
        error_code="HTTP_ERROR",
        request_id=getattr(request.state, 'request_id', None)
    )
    
    return JSONResponse(
        status_code=exc.status_code,
        content=error_response
    )


async def validation_exception_handler(request: Request, exc: RequestValidationError) -> JSONResponse:
    """
    Handle Pydantic validation errors.
    
    Args:
        request: FastAPI request object
        exc: RequestValidationError
        
    Returns:
        JSON error response with validation details
    """
    # Extract validation error details
    validation_errors = []
    for error in exc.errors():
        validation_errors.append({
            "field": ".".join(str(loc) for loc in error["loc"]),
            "message": error["msg"],
            "type": error["type"],
            "input": error.get("input")
        })
    
    # Log the validation error
    logger.warning(f"Validation Error: {len(validation_errors)} field(s) failed validation", extra={
        "validation_errors": validation_errors,
        "request_url": str(request.url),
        "request_method": request.method
    })
    
    # Create error response
    error_response = create_error_response(
        status_code=422,
        message="Request validation failed",
        error_code="VALIDATION_ERROR",
        details={"validation_errors": validation_errors},
        request_id=getattr(request.state, 'request_id', None)
    )
    
    return JSONResponse(
        status_code=422,
        content=error_response
    )


async def general_exception_handler(request: Request, exc: Exception) -> JSONResponse:
    """
    Handle unexpected exceptions.
    
    Args:
        request: FastAPI request object
        exc: Exception
        
    Returns:
        JSON error response
    """
    # Log the unexpected error with full traceback
    logger.error(f"Unexpected error: {str(exc)}", extra={
        "exception_type": type(exc).__name__,
        "traceback": traceback.format_exc(),
        "request_url": str(request.url),
        "request_method": request.method
    })
    
    # Create error response (don't expose internal details in production)
    error_response = create_error_response(
        status_code=500,
        message="An unexpected error occurred",
        error_code="INTERNAL_ERROR",
        details={"exception_type": type(exc).__name__} if logger.level <= logging.DEBUG else None,
        request_id=getattr(request.state, 'request_id', None)
    )
    
    return JSONResponse(
        status_code=500,
        content=error_response
    )


def setup_error_handlers(app: FastAPI):
    """
    Setup error handlers for the FastAPI application.
    
    Args:
        app: FastAPI application instance
    """
    # Add custom exception handlers
    app.add_exception_handler(AGVTransportError, agv_transport_exception_handler)
    app.add_exception_handler(HTTPException, http_exception_handler)
    app.add_exception_handler(StarletteHTTPException, http_exception_handler)
    app.add_exception_handler(RequestValidationError, validation_exception_handler)
    app.add_exception_handler(Exception, general_exception_handler)
    
    logger.info("Error handlers configured successfully")


# Validation utilities
def validate_station_name(station_name: str, station_manager) -> str:
    """
    Validate a station name exists in the configuration.
    
    Args:
        station_name: Station name to validate
        station_manager: StationManager instance
        
    Returns:
        Validated station name
        
    Raises:
        ValidationError: If station name is invalid
    """
    if not station_name or not station_name.strip():
        raise ValidationError("Station name cannot be empty", field="station_name", value=station_name)
    
    station_name = station_name.strip()
    
    if not station_manager.validate_station_name(station_name):
        available_stations = station_manager.get_station_names()
        raise ValidationError(
            f"Station '{station_name}' not found",
            field="station_name",
            value=station_name
        )
    
    return station_name


def validate_pickup_station(station_name: str, station_manager) -> str:
    """
    Validate a pickup station (cannot be a charging station).
    
    Args:
        station_name: Station name to validate
        station_manager: StationManager instance
        
    Returns:
        Validated station name
        
    Raises:
        ValidationError: If station is invalid for pickup
    """
    station_name = validate_station_name(station_name, station_manager)
    
    station = station_manager.get_station(station_name)
    if station and station.type == 'charger':
        raise ValidationError(
            "Cannot pickup from charging station",
            field="pickup_station",
            value=station_name
        )
    
    return station_name


def validate_different_stations(pickup_station: str, dropoff_station: str) -> tuple[str, str]:
    """
    Validate that pickup and dropoff stations are different.
    
    Args:
        pickup_station: Pickup station name
        dropoff_station: Dropoff station name
        
    Returns:
        Tuple of validated station names
        
    Raises:
        ValidationError: If stations are the same
    """
    if pickup_station == dropoff_station:
        raise ValidationError(
            "Pickup and dropoff stations must be different",
            field="dropoff_station",
            value=dropoff_station
        )
    
    return pickup_station, dropoff_station


def validate_priority(priority: int) -> int:
    """
    Validate transport order priority.
    
    Args:
        priority: Priority value to validate
        
    Returns:
        Validated priority
        
    Raises:
        ValidationError: If priority is out of range
    """
    if not isinstance(priority, int):
        raise ValidationError(
            "Priority must be an integer",
            field="priority",
            value=priority
        )
    
    if priority < 0 or priority > 10:
        raise ValidationError(
            "Priority must be between 0 and 10",
            field="priority",
            value=priority
        )
    
    return priority


# Request ID middleware for error tracking
async def add_request_id_middleware(request: Request, call_next):
    """
    Middleware to add request ID for error tracking.
    
    Args:
        request: FastAPI request object
        call_next: Next middleware/handler
        
    Returns:
        Response with request ID header
    """
    import uuid
    
    # Generate request ID
    request_id = str(uuid.uuid4())
    request.state.request_id = request_id
    
    # Process request
    response = await call_next(request)
    
    # Add request ID to response headers
    response.headers["X-Request-ID"] = request_id
    
    return response