"""Station Manager for AGV Transport Task Manager.

This module manages station configurations loaded from YAML files,
providing station lookup and pose conversion functionality.
"""

import math
import os
from typing import Dict, List, Optional
import yaml

from geometry_msgs.msg import PoseStamped, Quaternion
from agv_transport.models import StationConfig


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert yaw angle (radians) to quaternion.
    
    Args:
        yaw: Yaw angle in radians
        
    Returns:
        Quaternion representing the yaw rotation
    """
    # For rotation around Z-axis only (2D navigation)
    # q = [0, 0, sin(yaw/2), cos(yaw/2)]
    half_yaw = yaw / 2.0
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half_yaw)
    q.w = math.cos(half_yaw)
    return q


def quaternion_to_yaw(q: Quaternion) -> float:
    """Convert quaternion to yaw angle (radians).
    
    Args:
        q: Quaternion (assumes rotation around Z-axis only)
        
    Returns:
        Yaw angle in radians
    """
    # For rotation around Z-axis only
    # yaw = 2 * atan2(z, w)
    return 2.0 * math.atan2(q.z, q.w)


class StationManager:
    """Manages station configurations from YAML.
    
    Loads station configurations from a YAML file and provides
    methods for station lookup and pose retrieval.
    
    Attributes:
        stations: Dictionary mapping station names to StationConfig
        config_path: Path to the YAML configuration file
        frame_id: Frame ID for poses (default: 'map')
    """
    
    def __init__(self, config_path: str, frame_id: str = 'map'):
        """Initialize StationManager.
        
        Args:
            config_path: Path to warehouse_stations.yaml
            frame_id: Frame ID for poses (default: 'map')
        """
        self.config_path = config_path
        self.frame_id = frame_id
        self.stations: Dict[str, StationConfig] = {}
        self._load_stations()
    
    def _load_stations(self) -> bool:
        """Load stations from YAML configuration file.
        
        Returns:
            True if loading succeeded, False otherwise
        """
        if not os.path.exists(self.config_path):
            return False
        
        try:
            with open(self.config_path, 'r') as f:
                data = yaml.safe_load(f)
            
            if not data or 'stations' not in data:
                return False
            
            stations_data = data['stations']
            self.stations = {}
            
            for name, station_data in stations_data.items():
                config = StationConfig.from_dict(name, station_data)
                self.stations[name] = config
            
            return True
            
        except (yaml.YAMLError, IOError):
            return False
    
    def load_stations(self) -> bool:
        """Reload stations from YAML configuration file.
        
        Public method to reload stations (e.g., via service call).
        
        Returns:
            True if loading succeeded, False otherwise
        """
        return self._load_stations()
    
    def get_station_pose(self, station_name: str) -> Optional[PoseStamped]:
        """Get PoseStamped for a station.
        
        Args:
            station_name: Name of the station
            
        Returns:
            PoseStamped with x, y, and quaternion orientation,
            or None if station doesn't exist
        """
        if station_name not in self.stations:
            return None
        
        station = self.stations[station_name]
        
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = station.x
        pose.pose.position.y = station.y
        pose.pose.position.z = 0.0
        pose.pose.orientation = yaw_to_quaternion(station.yaw)
        
        return pose
    
    def station_exists(self, station_name: str) -> bool:
        """Check if a station exists.
        
        Args:
            station_name: Name of the station
            
        Returns:
            True if station exists, False otherwise
        """
        return station_name in self.stations
    
    def get_stations_by_type(self, station_type: str) -> List[str]:
        """Get list of station names by type.
        
        Args:
            station_type: Type of station (e.g., 'dock', 'charger')
            
        Returns:
            List of station names matching the type
        """
        return [
            name for name, config in self.stations.items()
            if config.station_type == station_type
        ]
    
    def get_all_station_names(self) -> List[str]:
        """Get list of all station names.
        
        Returns:
            List of all station names
        """
        return list(self.stations.keys())
    
    def get_station_config(self, station_name: str) -> Optional[StationConfig]:
        """Get StationConfig for a station.
        
        Args:
            station_name: Name of the station
            
        Returns:
            StationConfig or None if station doesn't exist
        """
        return self.stations.get(station_name)
