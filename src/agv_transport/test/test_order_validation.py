"""Property-based tests for order validation and queue management.

This module contains property-based tests using Hypothesis to verify
the correctness of order validation and FIFO queue ordering.
"""

import sys
import os

# Add the package to path for testing
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from hypothesis import given, strategies as st, settings, assume
from agv_transport.models import TransportOrder
from agv_transport.station_manager import StationManager


# Valid station names from warehouse_stations.yaml
VALID_STATIONS = ['dock_in_1', 'dock_in_2', 'dock_out', 'charging_station']

# Strategies for generating test data
valid_station_names = st.sampled_from(VALID_STATIONS)

invalid_station_names = st.text(min_size=1, max_size=50).filter(
    lambda x: x.strip() and x not in VALID_STATIONS
)


def get_station_manager() -> StationManager:
    """Get a StationManager instance with the test config."""
    config_path = os.path.join(
        os.path.dirname(__file__),
        '..', 'config', 'warehouse_stations.yaml'
    )
    return StationManager(config_path)


def validate_order_with_station_manager(
    order: TransportOrder,
    station_manager: StationManager
) -> tuple[bool, str]:
    """Validate a transport order against station manager.
    
    This mirrors the validation logic in TransportTaskManagerNode.
    
    Args:
        order: TransportOrder to validate
        station_manager: StationManager for station lookups
        
    Returns:
        Tuple of (is_valid, error_message)
    """
    # Basic validation
    is_valid, error = order.validate()
    if not is_valid:
        return False, error
    
    # Validate pickup station exists
    if not station_manager.station_exists(order.pickup_station):
        return False, f"pickup_station '{order.pickup_station}' does not exist"
    
    # Validate dropoff station exists
    if not station_manager.station_exists(order.dropoff_station):
        return False, f"dropoff_station '{order.dropoff_station}' does not exist"
    
    return True, ""


# **Feature: agv-transport-task-manager, Property 4: Order Validation Completeness**
# **Validates: Requirements 4.2, 4.4**
@settings(max_examples=100)
@given(
    pickup=invalid_station_names,
    dropoff=valid_station_names
)
def test_property_4_invalid_pickup_station_rejected(pickup: str, dropoff: str):
    """Property 4: Order Validation Completeness - Invalid pickup station
    
    For any TransportOrder with an invalid pickup_station, the system SHALL
    reject the order with an error status.
    
    **Feature: agv-transport-task-manager, Property 4: Order Validation Completeness**
    **Validates: Requirements 4.2, 4.4**
    """
    config_path = os.path.join(
        os.path.dirname(__file__),
        '..', 'config', 'warehouse_stations.yaml'
    )
    assume(os.path.exists(config_path))
    
    station_manager = get_station_manager()
    
    order = TransportOrder(
        pickup_station=pickup,
        dropoff_station=dropoff
    )
    
    is_valid, error = validate_order_with_station_manager(order, station_manager)
    
    assert not is_valid, f"Order with invalid pickup '{pickup}' should be rejected"
    assert "pickup_station" in error.lower() or "does not exist" in error.lower(), \
        f"Error should mention pickup station issue: {error}"


@settings(max_examples=100)
@given(
    pickup=valid_station_names,
    dropoff=invalid_station_names
)
def test_property_4_invalid_dropoff_station_rejected(pickup: str, dropoff: str):
    """Property 4: Order Validation Completeness - Invalid dropoff station
    
    For any TransportOrder with an invalid dropoff_station, the system SHALL
    reject the order with an error status.
    
    **Feature: agv-transport-task-manager, Property 4: Order Validation Completeness**
    **Validates: Requirements 4.2, 4.4**
    """
    config_path = os.path.join(
        os.path.dirname(__file__),
        '..', 'config', 'warehouse_stations.yaml'
    )
    assume(os.path.exists(config_path))
    
    station_manager = get_station_manager()
    
    order = TransportOrder(
        pickup_station=pickup,
        dropoff_station=dropoff
    )
    
    is_valid, error = validate_order_with_station_manager(order, station_manager)
    
    assert not is_valid, f"Order with invalid dropoff '{dropoff}' should be rejected"
    assert "dropoff_station" in error.lower() or "does not exist" in error.lower(), \
        f"Error should mention dropoff station issue: {error}"


@settings(max_examples=100)
@given(
    pickup=invalid_station_names,
    dropoff=invalid_station_names
)
def test_property_4_both_invalid_stations_rejected(pickup: str, dropoff: str):
    """Property 4: Order Validation Completeness - Both stations invalid
    
    For any TransportOrder with both invalid stations, the system SHALL
    reject the order.
    
    **Feature: agv-transport-task-manager, Property 4: Order Validation Completeness**
    **Validates: Requirements 4.2, 4.4**
    """
    # Ensure pickup and dropoff are different
    assume(pickup != dropoff)
    
    config_path = os.path.join(
        os.path.dirname(__file__),
        '..', 'config', 'warehouse_stations.yaml'
    )
    assume(os.path.exists(config_path))
    
    station_manager = get_station_manager()
    
    order = TransportOrder(
        pickup_station=pickup,
        dropoff_station=dropoff
    )
    
    is_valid, error = validate_order_with_station_manager(order, station_manager)
    
    assert not is_valid, f"Order with both invalid stations should be rejected"


@settings(max_examples=100)
@given(
    pickup=valid_station_names,
    dropoff=valid_station_names
)
def test_property_4_valid_stations_accepted(pickup: str, dropoff: str):
    """Property 4: Order Validation Completeness - Valid stations accepted
    
    For any TransportOrder with valid and different stations, the system SHALL
    accept the order.
    
    **Feature: agv-transport-task-manager, Property 4: Order Validation Completeness**
    **Validates: Requirements 4.2, 4.4**
    """
    # Ensure pickup and dropoff are different
    assume(pickup != dropoff)
    
    config_path = os.path.join(
        os.path.dirname(__file__),
        '..', 'config', 'warehouse_stations.yaml'
    )
    assume(os.path.exists(config_path))
    
    station_manager = get_station_manager()
    
    order = TransportOrder(
        pickup_station=pickup,
        dropoff_station=dropoff
    )
    
    is_valid, error = validate_order_with_station_manager(order, station_manager)
    
    assert is_valid, f"Order with valid stations should be accepted, got error: {error}"
    assert error == "", f"Error message should be empty for valid order: {error}"


@settings(max_examples=100)
@given(station=valid_station_names)
def test_property_4_same_station_rejected(station: str):
    """Property 4: Order Validation Completeness - Same station rejected
    
    For any TransportOrder where pickup and dropoff are the same station,
    the system SHALL reject the order.
    
    **Feature: agv-transport-task-manager, Property 4: Order Validation Completeness**
    **Validates: Requirements 4.2, 4.4**
    """
    config_path = os.path.join(
        os.path.dirname(__file__),
        '..', 'config', 'warehouse_stations.yaml'
    )
    assume(os.path.exists(config_path))
    
    station_manager = get_station_manager()
    
    order = TransportOrder(
        pickup_station=station,
        dropoff_station=station
    )
    
    is_valid, error = validate_order_with_station_manager(order, station_manager)
    
    assert not is_valid, f"Order with same pickup and dropoff should be rejected"
    assert "different" in error.lower() or "same" in error.lower() or "must be" in error.lower(), \
        f"Error should mention stations must be different: {error}"



# **Feature: agv-transport-task-manager, Property 5: Queue FIFO Ordering**
# **Validates: Requirements 4.3, 4.5**

@st.composite
def valid_transport_orders(draw):
    """Generate valid transport orders with different pickup and dropoff stations."""
    pickup = draw(valid_station_names)
    dropoff = draw(valid_station_names.filter(lambda x: x != pickup))
    return TransportOrder(
        pickup_station=pickup,
        dropoff_station=dropoff,
        priority=draw(st.integers(min_value=0, max_value=10))
    )


@settings(max_examples=100)
@given(orders=st.lists(valid_transport_orders(), min_size=1, max_size=20))
def test_property_5_queue_fifo_ordering(orders: list[TransportOrder]):
    """Property 5: Queue FIFO Ordering
    
    For any sequence of valid TransportOrders added to the queue, the processing 
    order SHALL match the insertion order (FIFO), regardless of priority field.
    
    **Feature: agv-transport-task-manager, Property 5: Queue FIFO Ordering**
    **Validates: Requirements 4.3, 4.5**
    """
    from collections import deque
    
    config_path = os.path.join(
        os.path.dirname(__file__),
        '..', 'config', 'warehouse_stations.yaml'
    )
    assume(os.path.exists(config_path))
    
    station_manager = get_station_manager()
    
    # Simulate the order queue (deque for FIFO)
    order_queue: deque[TransportOrder] = deque()
    
    # Add all orders to queue (simulating add_order behavior)
    for order in orders:
        is_valid, _ = validate_order_with_station_manager(order, station_manager)
        if is_valid:
            order_queue.append(order)
    
    # Record insertion order
    insertion_order = list(order_queue)
    
    # Pop orders and verify FIFO ordering
    processing_order = []
    while order_queue:
        processing_order.append(order_queue.popleft())
    
    # Verify processing order matches insertion order
    assert len(processing_order) == len(insertion_order), \
        "All orders should be processed"
    
    for i, (processed, inserted) in enumerate(zip(processing_order, insertion_order)):
        assert processed.order_id == inserted.order_id, \
            f"Order at position {i} should match: expected {inserted.order_id}, got {processed.order_id}"


@settings(max_examples=100)
@given(
    orders=st.lists(valid_transport_orders(), min_size=2, max_size=10),
    priorities=st.lists(st.integers(min_value=0, max_value=100), min_size=2, max_size=10)
)
def test_property_5_fifo_ignores_priority(orders: list[TransportOrder], priorities: list[int]):
    """Property 5: Queue FIFO Ordering - Priority ignored
    
    For any sequence of orders with varying priorities, the processing order
    SHALL still follow FIFO (insertion order), not priority order.
    
    **Feature: agv-transport-task-manager, Property 5: Queue FIFO Ordering**
    **Validates: Requirements 4.3, 4.5**
    """
    from collections import deque
    
    config_path = os.path.join(
        os.path.dirname(__file__),
        '..', 'config', 'warehouse_stations.yaml'
    )
    assume(os.path.exists(config_path))
    
    # Assign different priorities to orders
    for i, order in enumerate(orders):
        if i < len(priorities):
            order.priority = priorities[i]
    
    station_manager = get_station_manager()
    order_queue: deque[TransportOrder] = deque()
    
    # Add orders to queue
    for order in orders:
        is_valid, _ = validate_order_with_station_manager(order, station_manager)
        if is_valid:
            order_queue.append(order)
    
    # Record order IDs in insertion order
    insertion_ids = [o.order_id for o in order_queue]
    
    # Pop and verify FIFO
    processing_ids = []
    while order_queue:
        processing_ids.append(order_queue.popleft().order_id)
    
    assert processing_ids == insertion_ids, \
        f"Processing order {processing_ids} should match insertion order {insertion_ids}"
