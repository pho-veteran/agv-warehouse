"""Data models for AGV Transport Task Manager.

This module contains dataclasses for TransportOrder, TransportStatus, and StationConfig
used throughout the transport task management system.
"""

from dataclasses import dataclass, field
from typing import Optional, Dict, Any
import time
import uuid


@dataclass
class TransportOrder:
    """Represents a transport order for moving cargo between stations.
    
    Attributes:
        order_id: Unique identifier (UUID)
        pickup_station: Station name for pickup
        dropoff_station: Station name for dropoff
        priority: Priority level (higher = more urgent)
        timestamp: Creation timestamp
    """
    order_id: str = field(default_factory=lambda: str(uuid.uuid4()))
    pickup_station: str = ""
    dropoff_station: str = ""
    priority: int = 0
    timestamp: float = field(default_factory=time.time)
    
    def validate(self) -> tuple[bool, str]:
        """Validate the transport order.
        
        Returns:
            Tuple of (is_valid, error_message)
        """
        if not self.pickup_station:
            return False, "pickup_station is required"
        if not self.dropoff_station:
            return False, "dropoff_station is required"
        if self.pickup_station == self.dropoff_station:
            return False, "pickup_station and dropoff_station must be different"
        return True, ""
    
    def to_dict(self) -> Dict[str, Any]:
        """Serialize to dictionary."""
        return {
            'order_id': self.order_id,
            'pickup_station': self.pickup_station,
            'dropoff_station': self.dropoff_station,
            'priority': self.priority,
            'timestamp': self.timestamp,
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'TransportOrder':
        """Deserialize from dictionary."""
        return cls(
            order_id=data.get('order_id', str(uuid.uuid4())),
            pickup_station=data.get('pickup_station', ''),
            dropoff_station=data.get('dropoff_station', ''),
            priority=data.get('priority', 0),
            timestamp=data.get('timestamp', time.time()),
        )


@dataclass
class TransportStatus:
    """Represents the current status of the transport system.
    
    Attributes:
        current_state: FSM state name
        current_order_id: Current order being processed
        queue_length: Number of orders in queue
        current_station: Current target station
        navigation_progress: Progress 0.0 - 1.0
        error_message: Error description if any
        timestamp: Status timestamp
    """
    current_state: str = "IDLE"
    current_order_id: str = ""
    queue_length: int = 0
    current_station: str = ""
    navigation_progress: float = 0.0
    error_message: str = ""
    timestamp: float = field(default_factory=time.time)
    
    def is_complete(self) -> bool:
        """Check if all required fields are present and non-null.
        
        Returns:
            True if all required fields are present and valid
        """
        # current_state must be non-empty
        if not self.current_state:
            return False
        # current_order_id can be empty when IDLE, but must be a string
        if self.current_order_id is None:
            return False
        # queue_length must be non-negative
        if self.queue_length is None or self.queue_length < 0:
            return False
        # current_station can be empty, but must be a string
        if self.current_station is None:
            return False
        # timestamp must be positive
        if self.timestamp is None or self.timestamp <= 0:
            return False
        # navigation_progress must be in range [0, 1]
        if self.navigation_progress is None:
            return False
        if not (0.0 <= self.navigation_progress <= 1.0):
            return False
        # error_message can be empty, but must be a string
        if self.error_message is None:
            return False
        return True
    
    def to_dict(self) -> Dict[str, Any]:
        """Serialize to dictionary."""
        return {
            'current_state': self.current_state,
            'current_order_id': self.current_order_id,
            'queue_length': self.queue_length,
            'current_station': self.current_station,
            'navigation_progress': self.navigation_progress,
            'error_message': self.error_message,
            'timestamp': self.timestamp,
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'TransportStatus':
        """Deserialize from dictionary."""
        return cls(
            current_state=data.get('current_state', 'IDLE'),
            current_order_id=data.get('current_order_id', ''),
            queue_length=data.get('queue_length', 0),
            current_station=data.get('current_station', ''),
            navigation_progress=data.get('navigation_progress', 0.0),
            error_message=data.get('error_message', ''),
            timestamp=data.get('timestamp', time.time()),
        )


@dataclass
class StationConfig:
    """Configuration for a station in the warehouse.
    
    Attributes:
        name: Station identifier
        x: X coordinate in map frame
        y: Y coordinate in map frame
        yaw: Orientation in radians
        station_type: Type of station (dock, charger)
        description: Human-readable description
    """
    name: str = ""
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    station_type: str = ""
    description: str = ""
    
    def validate(self) -> tuple[bool, str]:
        """Validate the station configuration.
        
        Returns:
            Tuple of (is_valid, error_message)
        """
        if not self.name:
            return False, "name is required"
        if not self.station_type:
            return False, "station_type is required"
        return True, ""
    
    def to_dict(self) -> Dict[str, Any]:
        """Serialize to dictionary."""
        return {
            'name': self.name,
            'x': self.x,
            'y': self.y,
            'yaw': self.yaw,
            'station_type': self.station_type,
            'description': self.description,
        }
    
    @classmethod
    def from_dict(cls, name: str, data: Dict[str, Any]) -> 'StationConfig':
        """Deserialize from dictionary (YAML format).
        
        Args:
            name: Station name (key from YAML)
            data: Station data dictionary
        """
        return cls(
            name=name,
            x=float(data.get('x', 0.0)),
            y=float(data.get('y', 0.0)),
            yaw=float(data.get('yaw', 0.0)),
            station_type=data.get('type', ''),
            description=data.get('description', ''),
        )
