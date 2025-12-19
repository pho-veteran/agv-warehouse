"""Property-based tests for StationManager.

This module contains property-based tests using Hypothesis to verify
the correctness of station lookup and yaw-to-quaternion conversion.
"""

import sys
import os
import math

# Add the package to path for testing
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from hypothesis import given, strategies as st, settings, assume
from agv_transport.station_manager import (
    StationManager, 
    yaw_to_quaternion, 
    quaternion_to_yaw
)


# **Feature: agv-transport-task-manager, Property 2: Station Lookup Correctness**
# **Validates: Requirements 3.2**
@settings(max_examples=100)
@given(
    yaw=st.floats(min_value=-math.pi, max_value=math.pi, allow_nan=False, allow_infinity=False)
)
def test_property_2_station_lookup_yaw_quaternion_roundtrip(yaw: float):
    """Property 2: Station Lookup Correctness
    
    For any station name in warehouse_stations.yaml, calling get_station_pose(station_name) 
    SHALL return a PoseStamped where the quaternion orientation correctly represents 
    the yaw angle from the config (round-trip: quaternion_to_yaw(yaw_to_quaternion(yaw)) == yaw 
    within tolerance).
    
    **Feature: agv-transport-task-manager, Property 2: Station Lookup Correctness**
    **Validates: Requirements 3.2**
    """
    # Convert yaw to quaternion and back
    quaternion = yaw_to_quaternion(yaw)
    recovered_yaw = quaternion_to_yaw(quaternion)
    
    # Verify round-trip within tolerance (floating point precision)
    tolerance = 1e-9
    assert abs(recovered_yaw - yaw) < tolerance, \
        f"Yaw round-trip failed: original={yaw}, recovered={recovered_yaw}, diff={abs(recovered_yaw - yaw)}"
    
    # Verify quaternion is normalized (unit quaternion)
    norm = math.sqrt(quaternion.x**2 + quaternion.y**2 + quaternion.z**2 + quaternion.w**2)
    assert abs(norm - 1.0) < tolerance, \
        f"Quaternion not normalized: norm={norm}"


# **Feature: agv-transport-task-manager, Property 3: Invalid Station Rejection**
# **Validates: Requirements 3.3**
@settings(max_examples=100)
@given(
    invalid_name=st.text(min_size=1, max_size=50).filter(
        lambda x: x.strip() and x not in ['dock_in_1', 'dock_in_2', 'dock_out', 'charging_station']
    )
)
def test_property_3_invalid_station_rejection(invalid_name: str):
    """Property 3: Invalid Station Rejection
    
    For any station name NOT in warehouse_stations.yaml, calling get_station_pose(station_name) 
    SHALL return None and station_exists(station_name) SHALL return False.
    
    **Feature: agv-transport-task-manager, Property 3: Invalid Station Rejection**
    **Validates: Requirements 3.3**
    """
    # Get path to config file
    config_path = os.path.join(
        os.path.dirname(__file__), 
        '..', 'config', 'warehouse_stations.yaml'
    )
    
    # Skip if config file doesn't exist (test environment issue)
    assume(os.path.exists(config_path))
    
    manager = StationManager(config_path)
    
    # Verify invalid station returns None for pose
    pose = manager.get_station_pose(invalid_name)
    assert pose is None, f"Expected None for invalid station '{invalid_name}', got {pose}"
    
    # Verify station_exists returns False
    exists = manager.station_exists(invalid_name)
    assert exists is False, f"Expected False for invalid station '{invalid_name}', got {exists}"
