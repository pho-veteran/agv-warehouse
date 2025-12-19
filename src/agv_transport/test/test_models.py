"""Property-based tests for AGV Transport data models.

This module contains property-based tests using Hypothesis to verify
the correctness of TransportOrder, TransportStatus, and StationConfig models.
"""

import sys
import os

# Add the package to path for testing
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from hypothesis import given, strategies as st, settings
from agv_transport.models import TransportOrder, TransportStatus, StationConfig


# Strategies for generating test data
valid_states = st.sampled_from([
    "IDLE", "GO_PICKUP", "WAIT_LOADING", "GO_DROPOFF", 
    "WAIT_UNLOADING", "DONE", "ERROR"
])

valid_station_names = st.sampled_from([
    "dock_in_1", "dock_in_2", "dock_out", "charging_station"
])

non_empty_strings = st.text(min_size=1, max_size=50).filter(lambda x: x.strip())

# **Feature: agv-transport-task-manager, Property 6: Status Message Completeness**
# **Validates: Requirements 5.2, 5.3**
@settings(max_examples=100)
@given(
    current_state=valid_states,
    current_order_id=st.text(max_size=50),
    queue_length=st.integers(min_value=0, max_value=100),
    current_station=st.text(max_size=50),
    navigation_progress=st.floats(min_value=0.0, max_value=1.0, allow_nan=False),
    error_message=st.text(max_size=200),
    timestamp=st.floats(min_value=0.001, max_value=1e12, allow_nan=False)
)
def test_property_6_status_message_completeness(
    current_state: str,
    current_order_id: str,
    queue_length: int,
    current_station: str,
    navigation_progress: float,
    error_message: str,
    timestamp: float
):
    """Property 6: Status Message Completeness
    
    For any published TransportStatus message, all required fields 
    (current_state, current_order_id, queue_length, current_station, timestamp) 
    SHALL be present and non-null.
    
    **Feature: agv-transport-task-manager, Property 6: Status Message Completeness**
    **Validates: Requirements 5.2, 5.3**
    """
    status = TransportStatus(
        current_state=current_state,
        current_order_id=current_order_id,
        queue_length=queue_length,
        current_station=current_station,
        navigation_progress=navigation_progress,
        error_message=error_message,
        timestamp=timestamp
    )
    
    # Verify all fields are present and non-null
    assert status.is_complete(), f"Status should be complete: {status}"
    
    # Verify serialization round-trip preserves all fields
    status_dict = status.to_dict()
    assert 'current_state' in status_dict
    assert 'current_order_id' in status_dict
    assert 'queue_length' in status_dict
    assert 'current_station' in status_dict
    assert 'navigation_progress' in status_dict
    assert 'error_message' in status_dict
    assert 'timestamp' in status_dict
    
    # Verify deserialization produces equivalent object
    restored = TransportStatus.from_dict(status_dict)
    assert restored.current_state == status.current_state
    assert restored.current_order_id == status.current_order_id
    assert restored.queue_length == status.queue_length
    assert restored.current_station == status.current_station
    assert restored.navigation_progress == status.navigation_progress
    assert restored.error_message == status.error_message
    assert restored.timestamp == status.timestamp
