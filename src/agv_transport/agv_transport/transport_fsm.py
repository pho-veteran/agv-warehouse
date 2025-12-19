"""Finite State Machine for AGV Transport Task Manager.

This module implements the FSM that controls the transport workflow,
managing state transitions for pickup and dropoff operations.

**Feature: agv-transport-task-manager**
**Validates: Requirements 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7**
"""

from enum import Enum
from typing import Optional, Set, Dict
from dataclasses import dataclass, field

from agv_transport.models import TransportOrder


class TransportState(Enum):
    """Enumeration of transport FSM states.
    
    States:
        IDLE: Waiting for transport orders
        GO_PICKUP: Navigating to pickup station
        WAIT_LOADING: Waiting for cargo loading
        GO_DROPOFF: Navigating to dropoff station
        WAIT_UNLOADING: Waiting for cargo unloading
        DONE: Transport completed
        ERROR: Error state requiring manual reset
    """
    IDLE = "IDLE"
    GO_PICKUP = "GO_PICKUP"
    WAIT_LOADING = "WAIT_LOADING"
    GO_DROPOFF = "GO_DROPOFF"
    WAIT_UNLOADING = "WAIT_UNLOADING"
    DONE = "DONE"
    ERROR = "ERROR"


# Valid state transitions map
# Key: current state, Value: set of valid next states
VALID_TRANSITIONS: Dict[TransportState, Set[TransportState]] = {
    TransportState.IDLE: {TransportState.GO_PICKUP, TransportState.ERROR},
    TransportState.GO_PICKUP: {TransportState.WAIT_LOADING, TransportState.IDLE, TransportState.ERROR},
    TransportState.WAIT_LOADING: {TransportState.GO_DROPOFF, TransportState.ERROR},
    TransportState.GO_DROPOFF: {TransportState.WAIT_UNLOADING, TransportState.IDLE, TransportState.ERROR},
    TransportState.WAIT_UNLOADING: {TransportState.DONE, TransportState.ERROR},
    TransportState.DONE: {TransportState.IDLE, TransportState.ERROR},
    TransportState.ERROR: {TransportState.IDLE},  # Only manual reset can exit ERROR
}


@dataclass
class TransportFSM:
    """Finite State Machine for transport workflow.
    
    Manages state transitions for the transport task, ensuring only valid
    transitions occur and tracking the current order being processed.
    
    Attributes:
        current_state: Current FSM state
        current_order: The transport order being processed (None when IDLE)
        retry_count: Number of navigation retries for current order
        max_retries: Maximum allowed retries before failure
    """
    current_state: TransportState = field(default=TransportState.IDLE)
    current_order: Optional[TransportOrder] = field(default=None)
    retry_count: int = field(default=0)
    max_retries: int = field(default=3)
    
    def get_current_state(self) -> TransportState:
        """Get the current FSM state.
        
        Returns:
            Current TransportState
        """
        return self.current_state
    
    def transition_to(self, new_state: TransportState) -> bool:
        """Attempt to transition to a new state.
        
        Validates that the transition is allowed before changing state.
        
        Args:
            new_state: The target state to transition to
            
        Returns:
            True if transition was successful, False if invalid transition
        """
        if not self._is_valid_transition(new_state):
            return False
        
        self.current_state = new_state
        return True
    
    def _is_valid_transition(self, new_state: TransportState) -> bool:
        """Check if a transition to the new state is valid.
        
        Args:
            new_state: The target state
            
        Returns:
            True if transition is valid, False otherwise
        """
        valid_next_states = VALID_TRANSITIONS.get(self.current_state, set())
        return new_state in valid_next_states
    
    def get_valid_transitions(self) -> Set[TransportState]:
        """Get the set of valid next states from current state.
        
        Returns:
            Set of valid TransportState values that can be transitioned to
        """
        return VALID_TRANSITIONS.get(self.current_state, set()).copy()
    
    def start_order(self, order: TransportOrder) -> bool:
        """Start processing a new transport order.
        
        Can only be called when in IDLE state.
        
        Args:
            order: The transport order to process
            
        Returns:
            True if order was started, False if not in IDLE state
        """
        if self.current_state != TransportState.IDLE:
            return False
        
        self.current_order = order
        self.retry_count = 0
        return self.transition_to(TransportState.GO_PICKUP)
    
    def complete_order(self) -> bool:
        """Complete the current order and return to IDLE.
        
        Can only be called when in DONE state.
        
        Returns:
            True if order was completed, False if not in DONE state
        """
        if self.current_state != TransportState.DONE:
            return False
        
        self.current_order = None
        self.retry_count = 0
        return self.transition_to(TransportState.IDLE)
    
    def increment_retry(self) -> bool:
        """Increment retry count and check if max retries exceeded.
        
        Returns:
            True if still within retry limit, False if exceeded
        """
        self.retry_count += 1
        return self.retry_count < self.max_retries
    
    def reset(self) -> None:
        """Reset FSM to initial IDLE state.
        
        Clears current order and retry count.
        """
        self.current_state = TransportState.IDLE
        self.current_order = None
        self.retry_count = 0
    
    def enter_error_state(self) -> bool:
        """Transition to ERROR state.
        
        Can be called from any state except ERROR itself.
        
        Returns:
            True if transition was successful
        """
        if self.current_state == TransportState.ERROR:
            return False
        return self.transition_to(TransportState.ERROR)
    
    def is_busy(self) -> bool:
        """Check if FSM is currently processing an order.
        
        Returns:
            True if not in IDLE or ERROR state
        """
        return self.current_state not in {TransportState.IDLE, TransportState.ERROR}
    
    def get_expected_sequence(self) -> list[TransportState]:
        """Get the expected state sequence for a successful transport.
        
        Returns:
            List of states in order for successful transport
        """
        return [
            TransportState.IDLE,
            TransportState.GO_PICKUP,
            TransportState.WAIT_LOADING,
            TransportState.GO_DROPOFF,
            TransportState.WAIT_UNLOADING,
            TransportState.DONE,
            TransportState.IDLE,
        ]
