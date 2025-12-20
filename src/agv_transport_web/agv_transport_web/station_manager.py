"""
Station Manager Module for AGV Transport Web Dashboard

This module provides functionality to load, validate, and manage station configurations
from YAML files for the AGV transport system.
"""

import os
import yaml
import logging
from typing import Dict, List, Optional, Any
from dataclasses import dataclass
from pathlib import Path


@dataclass
class Station:
    """Data class representing a station in the AGV system."""
    name: str
    type: str
    x: float
    y: float
    yaw: float
    description: str


class StationManagerError(Exception):
    """Custom exception for station manager errors."""
    pass


class StationManager:
    """
    Manages station configuration loading, validation, and lookup operations.
    
    This class handles loading station data from warehouse_stations.yaml,
    validates the configuration, and provides lookup functionality for the
    web dashboard API.
    """
    
    def __init__(self, config_path: Optional[str] = None):
        """
        Initialize the StationManager.
        
        Args:
            config_path: Optional path to the warehouse_stations.yaml file.
                        If None, will search for it in common locations.
        """
        self.logger = logging.getLogger(__name__)
        self.config_path = config_path
        self.stations: Dict[str, Station] = {}
        self._last_modified = None
        
        # Try to find and load the configuration file
        if not self.config_path:
            self.config_path = self._find_config_file()
        
        if self.config_path:
            self.load_stations()
    
    def _find_config_file(self) -> Optional[str]:
        """
        Find the warehouse_stations.yaml configuration file in common locations.
        
        Returns:
            Path to the configuration file if found, None otherwise.
        """
        # Common locations to search for the config file
        search_paths = [
            "config/warehouse_stations.yaml",
            "../agv_transport/config/warehouse_stations.yaml",
            "../../agv_transport/config/warehouse_stations.yaml",
            "/opt/ros/humble/share/agv_transport/config/warehouse_stations.yaml",
        ]
        
        for path in search_paths:
            full_path = Path(path).resolve()
            if full_path.exists():
                self.logger.info(f"Found station configuration at: {full_path}")
                return str(full_path)
        
        self.logger.warning("Could not find warehouse_stations.yaml in common locations")
        return None
    
    def load_stations(self) -> None:
        """
        Load station configuration from the YAML file.
        
        Raises:
            StationManagerError: If the configuration file cannot be loaded or is invalid.
        """
        if not self.config_path:
            raise StationManagerError("No configuration file path specified")
        
        config_file = Path(self.config_path)
        
        if not config_file.exists():
            raise StationManagerError(f"Configuration file not found: {self.config_path}")
        
        try:
            # Check if file has been modified since last load
            current_modified = config_file.stat().st_mtime
            if self._last_modified and current_modified == self._last_modified:
                self.logger.debug("Configuration file unchanged, skipping reload")
                return
            
            with open(config_file, 'r', encoding='utf-8') as file:
                config_data = yaml.safe_load(file)
            
            self._validate_config(config_data)
            self._parse_stations(config_data)
            
            self._last_modified = current_modified
            self.logger.info(f"Successfully loaded {len(self.stations)} stations from {self.config_path}")
            
        except yaml.YAMLError as e:
            raise StationManagerError(f"Invalid YAML format in {self.config_path}: {e}")
        except Exception as e:
            raise StationManagerError(f"Error loading station configuration: {e}")
    
    def _validate_config(self, config_data: Dict[str, Any]) -> None:
        """
        Validate the structure of the configuration data.
        
        Args:
            config_data: The loaded YAML configuration data.
            
        Raises:
            StationManagerError: If the configuration structure is invalid.
        """
        if not isinstance(config_data, dict):
            raise StationManagerError("Configuration must be a dictionary")
        
        if 'stations' not in config_data:
            raise StationManagerError("Configuration must contain 'stations' section")
        
        stations_data = config_data['stations']
        if not isinstance(stations_data, dict):
            raise StationManagerError("'stations' section must be a dictionary")
        
        if not stations_data:
            raise StationManagerError("No stations defined in configuration")
        
        # Validate each station
        for station_name, station_data in stations_data.items():
            self._validate_station(station_name, station_data)
    
    def _validate_station(self, station_name: str, station_data: Dict[str, Any]) -> None:
        """
        Validate a single station configuration.
        
        Args:
            station_name: The name of the station.
            station_data: The station configuration data.
            
        Raises:
            StationManagerError: If the station configuration is invalid.
        """
        if not isinstance(station_data, dict):
            raise StationManagerError(f"Station '{station_name}' must be a dictionary")
        
        required_fields = ['x', 'y', 'yaw', 'type', 'description']
        for field in required_fields:
            if field not in station_data:
                raise StationManagerError(f"Station '{station_name}' missing required field: {field}")
        
        # Validate coordinate types
        for coord in ['x', 'y', 'yaw']:
            try:
                float(station_data[coord])
            except (ValueError, TypeError):
                raise StationManagerError(f"Station '{station_name}' field '{coord}' must be a number")
        
        # Validate type
        valid_types = ['dock', 'shelf', 'charger']
        if station_data['type'] not in valid_types:
            raise StationManagerError(
                f"Station '{station_name}' has invalid type '{station_data['type']}'. "
                f"Valid types: {valid_types}"
            )
        
        # Validate description
        if not isinstance(station_data['description'], str):
            raise StationManagerError(f"Station '{station_name}' description must be a string")
    
    def _parse_stations(self, config_data: Dict[str, Any]) -> None:
        """
        Parse station data from the configuration and create Station objects.
        
        Args:
            config_data: The loaded YAML configuration data.
        """
        self.stations.clear()
        
        for station_name, station_data in config_data['stations'].items():
            station = Station(
                name=station_name,
                type=station_data['type'],
                x=float(station_data['x']),
                y=float(station_data['y']),
                yaw=float(station_data['yaw']),
                description=station_data['description']
            )
            self.stations[station_name] = station
    
    def get_all_stations(self) -> List[Dict[str, Any]]:
        """
        Get all stations as a list of dictionaries suitable for JSON serialization.
        
        Returns:
            List of station dictionaries with all station information.
        """
        return [
            {
                'station_name': station.name,
                'station_type': station.type,
                'x': station.x,
                'y': station.y,
                'yaw': station.yaw,
                'description': station.description
            }
            for station in self.stations.values()
        ]
    
    def get_station(self, station_name: str) -> Optional[Station]:
        """
        Get a specific station by name.
        
        Args:
            station_name: The name of the station to retrieve.
            
        Returns:
            Station object if found, None otherwise.
        """
        return self.stations.get(station_name)
    
    def get_stations_by_type(self, station_type: str) -> List[Station]:
        """
        Get all stations of a specific type.
        
        Args:
            station_type: The type of stations to retrieve (dock, shelf, charger).
            
        Returns:
            List of Station objects matching the specified type.
        """
        return [
            station for station in self.stations.values()
            if station.type == station_type
        ]
    
    def validate_station_name(self, station_name: str) -> bool:
        """
        Validate that a station name exists in the configuration.
        
        Args:
            station_name: The station name to validate.
            
        Returns:
            True if the station exists, False otherwise.
        """
        return station_name in self.stations
    
    def reload_configuration(self) -> None:
        """
        Reload the station configuration from the file.
        
        This method can be called to refresh the station data if the
        configuration file has been updated.
        
        Raises:
            StationManagerError: If the configuration cannot be reloaded.
        """
        self.logger.info("Reloading station configuration")
        self.load_stations()
    
    def get_station_names(self) -> List[str]:
        """
        Get a list of all station names.
        
        Returns:
            List of station names.
        """
        return list(self.stations.keys())
    
    def get_pickup_stations(self) -> List[Dict[str, Any]]:
        """
        Get stations suitable for pickup operations (excludes charging stations).
        
        Returns:
            List of station dictionaries excluding charging stations.
        """
        return [
            {
                'station_name': station.name,
                'station_type': station.type,
                'x': station.x,
                'y': station.y,
                'yaw': station.yaw,
                'description': station.description
            }
            for station in self.stations.values()
            if station.type != 'charger'
        ]
    
    def get_dropoff_stations(self) -> List[Dict[str, Any]]:
        """
        Get stations suitable for dropoff operations (includes all stations).
        
        Returns:
            List of all station dictionaries.
        """
        return self.get_all_stations()
    
    @property
    def station_count(self) -> int:
        """Get the total number of loaded stations."""
        return len(self.stations)
    
    @property
    def is_loaded(self) -> bool:
        """Check if stations have been successfully loaded."""
        return len(self.stations) > 0