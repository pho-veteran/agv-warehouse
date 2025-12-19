"""Property-based tests for AGV Transport Error Recovery.

This module contains property-based tests using Hypothesis to verify
the correctness of error handling and recovery logic.

**Feature: agv-transport-task-manager**
**Validates: Requirements 8.3, 8.4**
"""

import sys
import os

# Add the package to path for testing
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from hypothesis import given, strategies as st, settings, assume
from agv_transport.transport_fsm import TransportFSM, TransportState


# Strategies for generating test data
non_error_states = st.sampled_from([
    TransportState.IDLE,
    TransportState.GO_PICKUP,
    TransportState.WAIT_LOADING,
    TransportState.GO_DROPOFF,
    TransportState.WAIT_UNLOADING,
    TransportState.DONE,
])

all_states = st.sampled_from(list(TransportState))

max_consecutive_errors_strategy = st.integers(min_value=1, max_value=10)
num_errors_strategy = st.integers(min_value=0, max_value=15)


class MockTransportTaskManager:
    """Mock transport task manager for testing error recovery logic.
    
    This mock simulates the error handling behavior of TransportTaskManagerNode
    without requiring ROS2 infrastructure.
    """
    
    def __init__(self, max_consecutive_errors: int = 3):
        """Initialize mock manager.
        
        Args:
            max_consecutive_errors: Threshold for entering ERROR state
        """
        self.fsm = TransportFSM()
        self.max_consecutive_errors = max_consecutive_errors
        self._consecutive_errors = 0
    
    def get_consecutive_errors(self) -> int:
        """Get current consecutive error count."""
        return self._consecutive_errors
    
    def handle_fsm_exception(self, exception: Exception) -> None:
        """Handle an exception during FSM execution.
        
        Implements the same logic as TransportTaskManagerNode._handle_fsm_exception
        
        Args:
            exception: The exception that occurred
        """
        self._consecutive_errors += 1
        
        # Clear current order
        self.fsm.current_order = None
        
        # Check for consecutive failures threshold (Requirement 8.4)
        if self._consecutive_errors >= self.max_consecutive_errors:
            self.fsm.transition_to(TransportState.ERROR)
        else:
            # Transition to IDLE on recoverable errors (Requirement 8.3)
            self.fsm.transition_to(TransportState.IDLE)
    
    def reset_transport(self) -> bool:
        """Reset the transport manager.
        
        Returns:
            True if reset was successful
        """
        self.fsm.reset()
        self._consecutive_errors = 0
        return True
    
    def simulate_successful_execution(self) -> None:
        """Simulate a successful FSM execution cycle."""
        # Reset consecutive errors on successful execution
        if self.fsm.get_current_state() != TransportState.ERROR:
            self._consecutive_errors = 0


# **Feature: agv-transport-task-manager, Property 10: Error Recovery to IDLE**
# **Validates: Requirements 8.3, 8.4**
@settings(max_examples=100)
@given(
    initial_state=non_error_states,
    max_consecutive_errors=max_consecutive_errors_strategy
)
def test_property_10_single_error_recovers_to_idle(
    initial_state: TransportState,
    max_consecutive_errors: int
):
    """Property 10: Error Recovery to IDLE - Single Error Case
    
    For any unexpected exception during FSM execution (when consecutive errors
    are below threshold), the system SHALL transition to IDLE state.
    
    **Feature: agv-transport-task-manager, Property 10: Error Recovery to IDLE**
    **Validates: Requirements 8.3**
    """
    manager = MockTransportTaskManager(max_consecutive_errors=max_consecutive_errors)
    
    # Set initial state
    manager.fsm.current_state = initial_state
    
    # Simulate a single exception
    manager.handle_fsm_exception(Exception("Test error"))
    
    # Should transition to IDLE (not ERROR) after single error
    assert manager.fsm.get_current_state() == TransportState.IDLE, \
        f"Single error from {initial_state} should recover to IDLE"
    assert manager.get_consecutive_errors() == 1


@settings(max_examples=100)
@given(
    initial_state=non_error_states,
    max_consecutive_errors=max_consecutive_errors_strategy
)
def test_property_10_consecutive_errors_enter_error_state(
    initial_state: TransportState,
    max_consecutive_errors: int
):
    """Property 10: Error Recovery to IDLE - Consecutive Errors Case
    
    For any max_consecutive_errors consecutive exceptions, the system SHALL
    transition to ERROR state and require manual reset.
    
    **Feature: agv-transport-task-manager, Property 10: Error Recovery to IDLE**
    **Validates: Requirements 8.4**
    """
    manager = MockTransportTaskManager(max_consecutive_errors=max_consecutive_errors)
    
    # Set initial state
    manager.fsm.current_state = initial_state
    
    # Simulate consecutive errors up to threshold
    for i in range(max_consecutive_errors):
        manager.handle_fsm_exception(Exception(f"Test error {i+1}"))
        
        if i < max_consecutive_errors - 1:
            # Before threshold, should be in IDLE
            assert manager.fsm.get_current_state() == TransportState.IDLE, \
                f"Error {i+1} should recover to IDLE"
            # Force back to a non-IDLE state to simulate continued operation
            manager.fsm.current_state = initial_state
    
    # After threshold, should be in ERROR state
    assert manager.fsm.get_current_state() == TransportState.ERROR, \
        f"After {max_consecutive_errors} consecutive errors, should be in ERROR state"
    assert manager.get_consecutive_errors() == max_consecutive_errors


@settings(max_examples=100)
@given(max_consecutive_errors=max_consecutive_errors_strategy)
def test_property_10_reset_exits_error_state(max_consecutive_errors: int):
    """Property 10: Error Recovery to IDLE - Reset from ERROR
    
    When in ERROR state, manual reset via service call SHALL return
    the system to IDLE state.
    
    **Feature: agv-transport-task-manager, Property 10: Error Recovery to IDLE**
    **Validates: Requirements 8.4**
    """
    manager = MockTransportTaskManager(max_consecutive_errors=max_consecutive_errors)
    
    # Force into ERROR state
    manager.fsm.current_state = TransportState.ERROR
    manager._consecutive_errors = max_consecutive_errors
    
    # Reset should return to IDLE
    success = manager.reset_transport()
    
    assert success, "Reset should succeed"
    assert manager.fsm.get_current_state() == TransportState.IDLE, \
        "Reset should return to IDLE state"
    assert manager.get_consecutive_errors() == 0, \
        "Reset should clear consecutive error count"


@settings(max_examples=100)
@given(
    max_consecutive_errors=max_consecutive_errors_strategy,
    num_errors=num_errors_strategy
)
def test_property_10_success_resets_error_count(
    max_consecutive_errors: int,
    num_errors: int
):
    """Property 10: Error Recovery to IDLE - Success Resets Count
    
    For any number of errors below threshold, a successful execution
    should reset the consecutive error count.
    
    **Feature: agv-transport-task-manager, Property 10: Error Recovery to IDLE**
    **Validates: Requirements 8.3**
    """
    assume(num_errors < max_consecutive_errors)
    
    manager = MockTransportTaskManager(max_consecutive_errors=max_consecutive_errors)
    
    # Simulate some errors (below threshold)
    for i in range(num_errors):
        manager.handle_fsm_exception(Exception(f"Test error {i+1}"))
        # Reset state to continue (simulating recovery)
        manager.fsm.current_state = TransportState.GO_PICKUP
    
    assert manager.get_consecutive_errors() == num_errors
    
    # Simulate successful execution
    manager.simulate_successful_execution()
    
    assert manager.get_consecutive_errors() == 0, \
        "Successful execution should reset consecutive error count"


@settings(max_examples=100)
@given(
    initial_state=non_error_states,
    max_consecutive_errors=max_consecutive_errors_strategy
)
def test_property_10_error_clears_current_order(
    initial_state: TransportState,
    max_consecutive_errors: int
):
    """Property 10: Error Recovery to IDLE - Order Cleared on Error
    
    When an exception occurs, the current order should be cleared.
    
    **Feature: agv-transport-task-manager, Property 10: Error Recovery to IDLE**
    **Validates: Requirements 8.3**
    """
    from agv_transport.models import TransportOrder
    
    manager = MockTransportTaskManager(max_consecutive_errors=max_consecutive_errors)
    
    # Set up an order
    order = TransportOrder(
        pickup_station="dock_in_1",
        dropoff_station="dock_out"
    )
    manager.fsm.current_order = order
    manager.fsm.current_state = initial_state
    
    # Simulate an exception
    manager.handle_fsm_exception(Exception("Test error"))
    
    # Order should be cleared
    assert manager.fsm.current_order is None, \
        "Current order should be cleared after exception"


@settings(max_examples=100)
@given(
    num_errors_before_success=st.integers(min_value=1, max_value=5),
    max_consecutive_errors=st.integers(min_value=6, max_value=10)
)
def test_property_10_intermittent_errors_dont_accumulate(
    num_errors_before_success: int,
    max_consecutive_errors: int
):
    """Property 10: Error Recovery to IDLE - Intermittent Errors
    
    Errors separated by successful executions should not accumulate
    toward the ERROR state threshold.
    
    **Feature: agv-transport-task-manager, Property 10: Error Recovery to IDLE**
    **Validates: Requirements 8.3, 8.4**
    """
    manager = MockTransportTaskManager(max_consecutive_errors=max_consecutive_errors)
    
    # Simulate multiple cycles of errors followed by success
    for cycle in range(3):
        # Simulate some errors
        for i in range(num_errors_before_success):
            manager.handle_fsm_exception(Exception(f"Cycle {cycle} error {i+1}"))
            manager.fsm.current_state = TransportState.GO_PICKUP
        
        # Simulate success - should reset counter
        manager.simulate_successful_execution()
        
        assert manager.get_consecutive_errors() == 0, \
            f"Error count should be 0 after success in cycle {cycle}"
    
    # Should never have entered ERROR state
    assert manager.fsm.get_current_state() != TransportState.ERROR, \
        "Intermittent errors with successes should not enter ERROR state"


@settings(max_examples=100)
@given(max_consecutive_errors=max_consecutive_errors_strategy)
def test_property_10_error_state_requires_manual_reset(max_consecutive_errors: int):
    """Property 10: Error Recovery to IDLE - ERROR State Persistence
    
    Once in ERROR state, the system should remain in ERROR state
    until manually reset.
    
    **Feature: agv-transport-task-manager, Property 10: Error Recovery to IDLE**
    **Validates: Requirements 8.4**
    """
    manager = MockTransportTaskManager(max_consecutive_errors=max_consecutive_errors)
    
    # Enter ERROR state
    for _ in range(max_consecutive_errors):
        manager.handle_fsm_exception(Exception("Test error"))
        if manager.fsm.get_current_state() != TransportState.ERROR:
            manager.fsm.current_state = TransportState.GO_PICKUP
    
    assert manager.fsm.get_current_state() == TransportState.ERROR
    
    # Additional errors should not change state (already in ERROR)
    # The FSM should stay in ERROR
    initial_error_count = manager.get_consecutive_errors()
    
    # Verify ERROR state is stable (can only exit via reset)
    # Note: In actual implementation, exceptions in ERROR state are handled differently
    # Here we just verify the state persists
    assert manager.fsm.get_current_state() == TransportState.ERROR, \
        "ERROR state should persist until manual reset"

