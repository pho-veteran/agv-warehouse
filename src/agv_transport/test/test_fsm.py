"""Property-based tests for AGV Transport FSM.

This module contains property-based tests using Hypothesis to verify
the correctness of the TransportFSM state machine.
"""

import sys
import os

# Add the package to path for testing
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from hypothesis import given, strategies as st, settings, assume
from agv_transport.transport_fsm import TransportFSM, TransportState, VALID_TRANSITIONS
from agv_transport.models import TransportOrder


# Strategies for generating test data
valid_station_names = st.sampled_from([
    "dock_in_1", "dock_in_2", "dock_out", "charging_station"
])

# Strategy for generating valid transport orders
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


# **Feature: agv-transport-task-manager, Property 1: FSM State Transition Validity**
# **Validates: Requirements 1.2, 1.3, 1.4, 1.5, 1.6, 1.7**
@settings(max_examples=100)
@given(order=valid_transport_orders())
def test_property_1_fsm_state_transition_validity(order: TransportOrder):
    """Property 1: FSM State Transition Validity
    
    For any valid TransportOrder received in IDLE state, the FSM SHALL 
    transition through the sequence IDLE → GO_PICKUP → WAIT_LOADING → 
    GO_DROPOFF → WAIT_UNLOADING → DONE → IDLE, with each transition 
    occurring only when the appropriate trigger condition is met.
    
    **Feature: agv-transport-task-manager, Property 1: FSM State Transition Validity**
    **Validates: Requirements 1.2, 1.3, 1.4, 1.5, 1.6, 1.7**
    """
    fsm = TransportFSM()
    
    # Verify initial state is IDLE (Requirement 1.1)
    assert fsm.get_current_state() == TransportState.IDLE, \
        "FSM should start in IDLE state"
    
    # Track state sequence
    state_sequence = [fsm.get_current_state()]
    
    # Start order - should transition IDLE → GO_PICKUP (Requirement 1.2)
    assert fsm.start_order(order), "Should be able to start order from IDLE"
    state_sequence.append(fsm.get_current_state())
    assert fsm.get_current_state() == TransportState.GO_PICKUP, \
        "Should transition to GO_PICKUP after starting order"
    
    # Simulate reaching pickup station - GO_PICKUP → WAIT_LOADING (Requirement 1.3)
    assert fsm.transition_to(TransportState.WAIT_LOADING), \
        "Should be able to transition to WAIT_LOADING from GO_PICKUP"
    state_sequence.append(fsm.get_current_state())
    assert fsm.get_current_state() == TransportState.WAIT_LOADING
    
    # Simulate loading complete - WAIT_LOADING → GO_DROPOFF (Requirement 1.4)
    assert fsm.transition_to(TransportState.GO_DROPOFF), \
        "Should be able to transition to GO_DROPOFF from WAIT_LOADING"
    state_sequence.append(fsm.get_current_state())
    assert fsm.get_current_state() == TransportState.GO_DROPOFF
    
    # Simulate reaching dropoff station - GO_DROPOFF → WAIT_UNLOADING (Requirement 1.5)
    assert fsm.transition_to(TransportState.WAIT_UNLOADING), \
        "Should be able to transition to WAIT_UNLOADING from GO_DROPOFF"
    state_sequence.append(fsm.get_current_state())
    assert fsm.get_current_state() == TransportState.WAIT_UNLOADING
    
    # Simulate unloading complete - WAIT_UNLOADING → DONE (Requirement 1.6)
    assert fsm.transition_to(TransportState.DONE), \
        "Should be able to transition to DONE from WAIT_UNLOADING"
    state_sequence.append(fsm.get_current_state())
    assert fsm.get_current_state() == TransportState.DONE
    
    # Complete order - DONE → IDLE (Requirement 1.7)
    assert fsm.complete_order(), "Should be able to complete order from DONE"
    state_sequence.append(fsm.get_current_state())
    assert fsm.get_current_state() == TransportState.IDLE, \
        "Should return to IDLE after completing order"
    
    # Verify the complete sequence matches expected
    expected_sequence = fsm.get_expected_sequence()
    assert state_sequence == expected_sequence, \
        f"State sequence {state_sequence} should match expected {expected_sequence}"


@settings(max_examples=100)
@given(
    current_state=st.sampled_from(list(TransportState)),
    target_state=st.sampled_from(list(TransportState))
)
def test_fsm_only_allows_valid_transitions(
    current_state: TransportState,
    target_state: TransportState
):
    """Test that FSM only allows transitions defined in VALID_TRANSITIONS.
    
    For any current state and target state, the transition should only
    succeed if it's in the VALID_TRANSITIONS map.
    """
    fsm = TransportFSM()
    fsm.current_state = current_state  # Force state for testing
    
    valid_targets = VALID_TRANSITIONS.get(current_state, set())
    transition_result = fsm.transition_to(target_state)
    
    if target_state in valid_targets:
        assert transition_result, \
            f"Transition from {current_state} to {target_state} should succeed"
        assert fsm.get_current_state() == target_state
    else:
        assert not transition_result, \
            f"Transition from {current_state} to {target_state} should fail"
        assert fsm.get_current_state() == current_state, \
            "State should remain unchanged after invalid transition"


@settings(max_examples=100)
@given(order=valid_transport_orders())
def test_fsm_start_order_only_from_idle(order: TransportOrder):
    """Test that orders can only be started from IDLE state."""
    fsm = TransportFSM()
    
    # Should work from IDLE
    assert fsm.start_order(order)
    assert fsm.current_order == order
    
    # Should not work from non-IDLE states
    for state in TransportState:
        if state != TransportState.IDLE:
            fsm.current_state = state
            new_order = TransportOrder(
                pickup_station="dock_in_1",
                dropoff_station="dock_out"
            )
            assert not fsm.start_order(new_order), \
                f"Should not be able to start order from {state}"


@settings(max_examples=100)
@given(order=valid_transport_orders())
def test_fsm_reset_returns_to_idle(order: TransportOrder):
    """Test that reset() always returns FSM to IDLE state."""
    fsm = TransportFSM()
    
    # Start an order and progress through some states
    fsm.start_order(order)
    fsm.transition_to(TransportState.WAIT_LOADING)
    
    # Reset should return to IDLE
    fsm.reset()
    
    assert fsm.get_current_state() == TransportState.IDLE
    assert fsm.current_order is None
    assert fsm.retry_count == 0
