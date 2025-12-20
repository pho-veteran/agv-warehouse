#!/usr/bin/env python3
"""
Tests for Station Manager functionality.

This module contains unit tests for the StationManager class,
verifying configuration loading, validation, and lookup operations.
"""

import os
import tempfile
import pytest
import yaml
from pathlib import Path
from unittest.mock import patch, Mock

# Import the module under test
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), '../agv_transport_web'))

from agv_transport_web.station_manager import StationManager, StationManagerError, Station


class TestStationManager:
    """Unit tests for StationManager class."""
    
    def create_test_config(self, stations_data):
        """Helper method to create a temporary config file."""
        config = {
            'stations': stations_data,
            'metadata': {
                'created': '2025-12-20T10:00:00',
                'purpose': 'Test configuration',
                'total_stations': len(stations_data)
            }
        }
        
        temp_file = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
        yaml.dump(config, temp_file, default_flow_style=False)
        temp_file.close()
        return temp_file.name
    
    def test_station_manager_initialization_with_valid_config(self):
        """Test StationManager initialization with valid configuration."""
        stations_data = {
            'dock_in_1': {
                'x': -0.143,
                'y': -20.679,
                'yaw': 0.0,
                'type': 'dock',
                'description': 'Incoming goods receiving dock 1'
            },
            'charging_station': {
                'x': 13.157,
                'y': -30.282,
                'yaw': 0.0,
                'type': 'charger',
                'description': 'AGV charging station'
            }
        }
        
        config_file = self.create_test_config(stations_data)
        
        try:
            manager = StationManager(config_file)
            
            # Verify initialization
            assert manager.is_loaded is True
            assert manager.station_count == 2
            assert 'dock_in_1' in manager.get_station_names()
            assert 'charging_station' in manager.get_station_names()
            
        finally:
            os.unlink(config_file)
    
    def test_station_manager_initialization_without_config(self):
        """Test StationManager initialization without config file."""
        with patch.object(StationManager, '_find_config_file', return_value=None):
            manager = StationManager()
            
            # Should initialize but not load any stations
            assert manager.is_loaded is False
            assert manager.station_count == 0
    
    def test_station_validation_missing_required_fields(self):
        """Test station validation with missing required fields."""
        stations_data = {
            'invalid_station': {
                'x': 1.0,
                'y': 2.0,
                # Missing 'yaw', 'type', 'description'
            }
        }
        
        config_file = self.create_test_config(stations_data)
        
        try:
            with pytest.raises(StationManagerError) as exc_info:
                StationManager(config_file)
            
            assert "missing required field" in str(exc_info.value)
            
        finally:
            os.unlink(config_file)
    
    def test_station_validation_invalid_coordinates(self):
        """Test station validation with invalid coordinate types."""
        stations_data = {
            'invalid_station': {
                'x': 'not_a_number',
                'y': -20.679,
                'yaw': 0.0,
                'type': 'dock',
                'description': 'Test station'
            }
        }
        
        config_file = self.create_test_config(stations_data)
        
        try:
            with pytest.raises(StationManagerError) as exc_info:
                StationManager(config_file)
            
            assert "must be a number" in str(exc_info.value)
            
        finally:
            os.unlink(config_file)
    
    def test_station_validation_invalid_type(self):
        """Test station validation with invalid station type."""
        stations_data = {
            'invalid_station': {
                'x': 1.0,
                'y': 2.0,
                'yaw': 0.0,
                'type': 'invalid_type',
                'description': 'Test station'
            }
        }
        
        config_file = self.create_test_config(stations_data)
        
        try:
            with pytest.raises(StationManagerError) as exc_info:
                StationManager(config_file)
            
            assert "invalid type" in str(exc_info.value)
            assert "Valid types:" in str(exc_info.value)
            
        finally:
            os.unlink(config_file)
    
    def test_get_all_stations(self):
        """Test getting all stations as JSON-serializable format."""
        stations_data = {
            'dock_in_1': {
                'x': -0.143,
                'y': -20.679,
                'yaw': 0.0,
                'type': 'dock',
                'description': 'Incoming goods receiving dock 1'
            },
            'charging_station': {
                'x': 13.157,
                'y': -30.282,
                'yaw': 0.0,
                'type': 'charger',
                'description': 'AGV charging station'
            }
        }
        
        config_file = self.create_test_config(stations_data)
        
        try:
            manager = StationManager(config_file)
            all_stations = manager.get_all_stations()
            
            # Verify structure and content
            assert len(all_stations) == 2
            
            # Check first station
            dock_station = next(s for s in all_stations if s['station_name'] == 'dock_in_1')
            assert dock_station['station_type'] == 'dock'
            assert dock_station['x'] == -0.143
            assert dock_station['y'] == -20.679
            assert dock_station['yaw'] == 0.0
            assert 'description' in dock_station
            
        finally:
            os.unlink(config_file)
    
    def test_get_station_by_name(self):
        """Test getting a specific station by name."""
        stations_data = {
            'dock_in_1': {
                'x': -0.143,
                'y': -20.679,
                'yaw': 0.0,
                'type': 'dock',
                'description': 'Incoming goods receiving dock 1'
            }
        }
        
        config_file = self.create_test_config(stations_data)
        
        try:
            manager = StationManager(config_file)
            
            # Test existing station
            station = manager.get_station('dock_in_1')
            assert station is not None
            assert station.name == 'dock_in_1'
            assert station.type == 'dock'
            assert station.x == -0.143
            
            # Test non-existing station
            non_existing = manager.get_station('non_existing')
            assert non_existing is None
            
        finally:
            os.unlink(config_file)
    
    def test_get_stations_by_type(self):
        """Test getting stations filtered by type."""
        stations_data = {
            'dock_in_1': {
                'x': -0.143,
                'y': -20.679,
                'yaw': 0.0,
                'type': 'dock',
                'description': 'Incoming goods receiving dock 1'
            },
            'dock_in_2': {
                'x': -3.756,
                'y': -6.553,
                'yaw': 0.0,
                'type': 'dock',
                'description': 'Incoming goods receiving dock 2'
            },
            'charging_station': {
                'x': 13.157,
                'y': -30.282,
                'yaw': 0.0,
                'type': 'charger',
                'description': 'AGV charging station'
            }
        }
        
        config_file = self.create_test_config(stations_data)
        
        try:
            manager = StationManager(config_file)
            
            # Test dock stations
            dock_stations = manager.get_stations_by_type('dock')
            assert len(dock_stations) == 2
            assert all(station.type == 'dock' for station in dock_stations)
            
            # Test charger stations
            charger_stations = manager.get_stations_by_type('charger')
            assert len(charger_stations) == 1
            assert charger_stations[0].type == 'charger'
            
            # Test non-existing type
            shelf_stations = manager.get_stations_by_type('shelf')
            assert len(shelf_stations) == 0
            
        finally:
            os.unlink(config_file)
    
    def test_validate_station_name(self):
        """Test station name validation."""
        stations_data = {
            'dock_in_1': {
                'x': -0.143,
                'y': -20.679,
                'yaw': 0.0,
                'type': 'dock',
                'description': 'Incoming goods receiving dock 1'
            }
        }
        
        config_file = self.create_test_config(stations_data)
        
        try:
            manager = StationManager(config_file)
            
            # Test valid station name
            assert manager.validate_station_name('dock_in_1') is True
            
            # Test invalid station name
            assert manager.validate_station_name('non_existing') is False
            
        finally:
            os.unlink(config_file)
    
    def test_get_pickup_stations_excludes_chargers(self):
        """Test that pickup stations exclude charging stations."""
        stations_data = {
            'dock_in_1': {
                'x': -0.143,
                'y': -20.679,
                'yaw': 0.0,
                'type': 'dock',
                'description': 'Incoming goods receiving dock 1'
            },
            'charging_station': {
                'x': 13.157,
                'y': -30.282,
                'yaw': 0.0,
                'type': 'charger',
                'description': 'AGV charging station'
            }
        }
        
        config_file = self.create_test_config(stations_data)
        
        try:
            manager = StationManager(config_file)
            
            pickup_stations = manager.get_pickup_stations()
            
            # Should only include dock, not charger
            assert len(pickup_stations) == 1
            assert pickup_stations[0]['station_name'] == 'dock_in_1'
            assert pickup_stations[0]['station_type'] == 'dock'
            
            # Verify charger is excluded
            station_names = [s['station_name'] for s in pickup_stations]
            assert 'charging_station' not in station_names
            
        finally:
            os.unlink(config_file)
    
    def test_get_dropoff_stations_includes_all(self):
        """Test that dropoff stations include all station types."""
        stations_data = {
            'dock_in_1': {
                'x': -0.143,
                'y': -20.679,
                'yaw': 0.0,
                'type': 'dock',
                'description': 'Incoming goods receiving dock 1'
            },
            'charging_station': {
                'x': 13.157,
                'y': -30.282,
                'yaw': 0.0,
                'type': 'charger',
                'description': 'AGV charging station'
            }
        }
        
        config_file = self.create_test_config(stations_data)
        
        try:
            manager = StationManager(config_file)
            
            dropoff_stations = manager.get_dropoff_stations()
            
            # Should include all stations
            assert len(dropoff_stations) == 2
            
            station_names = [s['station_name'] for s in dropoff_stations]
            assert 'dock_in_1' in station_names
            assert 'charging_station' in station_names
            
        finally:
            os.unlink(config_file)
    
    def test_configuration_reload(self):
        """Test configuration reload functionality."""
        initial_stations = {
            'dock_in_1': {
                'x': -0.143,
                'y': -20.679,
                'yaw': 0.0,
                'type': 'dock',
                'description': 'Incoming goods receiving dock 1'
            }
        }
        
        config_file = self.create_test_config(initial_stations)
        
        try:
            manager = StationManager(config_file)
            assert manager.station_count == 1
            
            # Update the config file
            updated_stations = {
                'dock_in_1': {
                    'x': -0.143,
                    'y': -20.679,
                    'yaw': 0.0,
                    'type': 'dock',
                    'description': 'Updated description'
                },
                'dock_in_2': {
                    'x': -3.756,
                    'y': -6.553,
                    'yaw': 0.0,
                    'type': 'dock',
                    'description': 'New dock station'
                }
            }
            
            # Write updated config
            config = {
                'stations': updated_stations,
                'metadata': {'total_stations': len(updated_stations)}
            }
            
            with open(config_file, 'w') as f:
                yaml.dump(config, f)
            
            # Reload configuration
            manager.reload_configuration()
            
            # Verify updated data
            assert manager.station_count == 2
            assert 'dock_in_2' in manager.get_station_names()
            
            # Verify updated description
            station = manager.get_station('dock_in_1')
            assert station.description == 'Updated description'
            
        finally:
            os.unlink(config_file)
    
    def test_malformed_yaml_handling(self):
        """Test handling of malformed YAML files."""
        # Create a malformed YAML file
        temp_file = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
        temp_file.write("invalid: yaml: content: [unclosed")
        temp_file.close()
        
        try:
            with pytest.raises(StationManagerError) as exc_info:
                StationManager(temp_file.name)
            
            assert "Invalid YAML format" in str(exc_info.value)
            
        finally:
            os.unlink(temp_file.name)
    
    def test_missing_config_file_handling(self):
        """Test handling of missing configuration file."""
        non_existing_file = "/path/that/does/not/exist/config.yaml"
        
        with pytest.raises(StationManagerError) as exc_info:
            StationManager(non_existing_file)
        
        assert "Configuration file not found" in str(exc_info.value)
    
    def test_empty_stations_configuration(self):
        """Test handling of empty stations configuration."""
        config = {
            'stations': {},
            'metadata': {'total_stations': 0}
        }
        
        temp_file = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
        yaml.dump(config, temp_file, default_flow_style=False)
        temp_file.close()
        
        try:
            with pytest.raises(StationManagerError) as exc_info:
                StationManager(temp_file.name)
            
            assert "No stations defined" in str(exc_info.value)
            
        finally:
            os.unlink(temp_file.name)


if __name__ == '__main__':
    pytest.main([__file__])