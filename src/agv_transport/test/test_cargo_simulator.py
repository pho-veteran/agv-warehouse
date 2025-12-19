"""Property-based tests for AGV Transport Cargo Simulator.

This module contains property-based tests using Hypothesis to verify
the correctness of the CargoSimulator cargo lifecycle management.
"""

import sys
import os

# Add the package to path for testing
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from hypothesis import given, strategies as st, settings, assume
from agv_transport.cargo_simulator import (
    CargoSimulator, CargoState, CargoInfo
)


# Strategies for generating test data
station_coordinates = st.floats(min_value=-100.0, max_value=100.0, allow_nan=False, allow_infinity=False)
station_names = st.sampled_from([
    "dock_in_1", "dock_in_2", "dock_out", "charging_station", "test_station"
])


# **Feature: agv-transport-task-manager, Property 8: Cargo Lifecycle Consistency**
# **Validates: Requirements 7.1, 7.2, 7.3, 7.4, 7.5**
@settings(max_examples=100)
@given(
    station_x=station_coordinates,
    station_y=station_coordinates,
    station_name=station_names
)
def test_property_8_cargo_lifecycle_consistency(
    station_x: float, station_y: float, station_name: str
):
    """Property 8: Cargo Lifecycle Consistency
    
    For any transport order, the cargo simulation SHALL follow the lifecycle:
    spawn (at WAIT_LOADING entry) → attach (after loading timeout) → 
    detach (at WAIT_UNLOADING entry) → delete (after unloading timeout).
    Gazebo failures SHALL NOT block FSM progression.
    
    **Feature: agv-transport-task-manager, Property 8: Cargo Lifecycle Consistency**
    **Validates: Requirements 7.1, 7.2, 7.3, 7.4, 7.5**
    """
    simulator = CargoSimulator()
    
    # Clear any previous state
    simulator.clear_lifecycle_history()
    
    # Step 1: Spawn cargo at station (Requirement 7.1)
    spawn_result = simulator.spawn_cargo_at_station(station_x, station_y, station_name)
    assert spawn_result, "Spawn should succeed"
    assert simulator.get_cargo_state() == CargoState.SPAWNED, \
        "Cargo should be in SPAWNED state after spawn"
    
    # Verify cargo info
    cargo_info = simulator.get_cargo_info()
    assert cargo_info is not None, "Cargo info should exist after spawn"
    assert cargo_info.spawn_x == station_x, "Spawn X should match"
    assert cargo_info.spawn_y == station_y, "Spawn Y should match"
    
    # Step 2: Attach cargo to robot (Requirement 7.2)
    attach_result = simulator.attach_cargo_to_robot()
    assert attach_result, "Attach should succeed"
    assert simulator.get_cargo_state() == CargoState.ATTACHED, \
        "Cargo should be in ATTACHED state after attach"
    assert simulator.is_attached, "is_attached flag should be True"
    
    # Step 3: Detach cargo from robot (Requirement 7.3)
    detach_result = simulator.detach_cargo()
    assert detach_result, "Detach should succeed"
    assert simulator.get_cargo_state() == CargoState.DETACHED, \
        "Cargo should be in DETACHED state after detach"
    assert not simulator.is_attached, "is_attached flag should be False"
    
    # Step 4: Delete cargo (Requirement 7.4)
    delete_result = simulator.delete_cargo()
    assert delete_result, "Delete should succeed"
    assert simulator.get_cargo_state() == CargoState.NONE, \
        "Cargo should be in NONE state after delete"
    assert simulator.get_cargo_info() is None, "Cargo info should be None after delete"
    
    # Verify complete lifecycle sequence
    is_valid, error_msg = simulator.verify_lifecycle_sequence()
    assert is_valid, f"Lifecycle sequence invalid: {error_msg}"
    
    # Verify lifecycle history matches expected sequence
    expected_history = [
        CargoState.SPAWNED,
        CargoState.ATTACHED,
        CargoState.DETACHED,
        CargoState.NONE
    ]
    actual_history = simulator.get_lifecycle_history()
    assert actual_history == expected_history, \
        f"Lifecycle history {actual_history} should match expected {expected_history}"


@settings(max_examples=100)
@given(
    station_x=station_coordinates,
    station_y=station_coordinates,
)
def test_cargo_spawn_creates_valid_cargo_info(station_x: float, station_y: float):
    """Test that spawning cargo creates valid CargoInfo with correct coordinates."""
    simulator = CargoSimulator()
    
    simulator.spawn_cargo_at_station(station_x, station_y, "test_station")
    
    cargo_info = simulator.get_cargo_info()
    assert cargo_info is not None
    assert cargo_info.state == CargoState.SPAWNED
    assert cargo_info.spawn_x == station_x
    assert cargo_info.spawn_y == station_y
    assert cargo_info.spawn_z == simulator.CARGO_SPAWN_HEIGHT
    assert cargo_info.name.startswith("cargo_box_")


@settings(max_examples=100)
@given(station_x=station_coordinates, station_y=station_coordinates)
def test_cargo_cannot_attach_without_spawn(station_x: float, station_y: float):
    """Test that attach fails when no cargo has been spawned."""
    simulator = CargoSimulator()
    
    # Try to attach without spawning first
    result = simulator.attach_cargo_to_robot()
    assert not result, "Attach should fail when no cargo exists"
    assert simulator.get_cargo_state() == CargoState.NONE


@settings(max_examples=100)
@given(station_x=station_coordinates, station_y=station_coordinates)
def test_cargo_cannot_detach_without_attach(station_x: float, station_y: float):
    """Test that detach fails when cargo is not attached."""
    simulator = CargoSimulator()
    
    # Spawn but don't attach
    simulator.spawn_cargo_at_station(station_x, station_y, "test_station")
    
    # Try to detach without attaching first
    result = simulator.detach_cargo()
    assert not result, "Detach should fail when cargo is not attached"
    assert simulator.get_cargo_state() == CargoState.SPAWNED


@settings(max_examples=100)
@given(station_x=station_coordinates, station_y=station_coordinates)
def test_cargo_delete_without_cargo_fails(station_x: float, station_y: float):
    """Test that delete fails when no cargo exists."""
    simulator = CargoSimulator()
    
    # Try to delete without any cargo
    result = simulator.delete_cargo()
    assert not result, "Delete should fail when no cargo exists"


@settings(max_examples=100)
@given(
    station_x1=station_coordinates,
    station_y1=station_coordinates,
    station_x2=station_coordinates,
    station_y2=station_coordinates,
)
def test_spawn_replaces_existing_cargo(
    station_x1: float, station_y1: float,
    station_x2: float, station_y2: float
):
    """Test that spawning new cargo replaces existing cargo."""
    simulator = CargoSimulator()
    
    # Spawn first cargo
    simulator.spawn_cargo_at_station(station_x1, station_y1, "station1")
    first_cargo_name = simulator.get_cargo_info().name
    
    # Spawn second cargo (should replace first)
    simulator.spawn_cargo_at_station(station_x2, station_y2, "station2")
    second_cargo_name = simulator.get_cargo_info().name
    
    # Verify new cargo replaced old
    assert first_cargo_name != second_cargo_name, "New cargo should have different name"
    assert simulator.get_cargo_info().spawn_x == station_x2
    assert simulator.get_cargo_info().spawn_y == station_y2


@settings(max_examples=100)
@given(station_x=station_coordinates, station_y=station_coordinates)
def test_reset_clears_all_state(station_x: float, station_y: float):
    """Test that reset() clears all cargo state."""
    simulator = CargoSimulator()
    
    # Create some state
    simulator.spawn_cargo_at_station(station_x, station_y, "test_station")
    simulator.attach_cargo_to_robot()
    
    # Reset
    simulator.reset()
    
    # Verify all state is cleared
    assert simulator.current_cargo is None
    assert not simulator.is_attached
    assert len(simulator.get_lifecycle_history()) == 0
    assert simulator.get_cargo_state() == CargoState.NONE


@settings(max_examples=100)
@given(station_x=station_coordinates, station_y=station_coordinates)
def test_simulate_full_lifecycle_helper(station_x: float, station_y: float):
    """Test the simulate_full_lifecycle helper method."""
    simulator = CargoSimulator()
    
    result = simulator.simulate_full_lifecycle(station_x, station_y, "test_station")
    
    assert result, "Full lifecycle simulation should succeed"
    
    # Verify lifecycle was completed correctly
    is_valid, error_msg = simulator.verify_lifecycle_sequence()
    assert is_valid, f"Lifecycle sequence invalid: {error_msg}"
    
    # Verify final state
    assert simulator.get_cargo_state() == CargoState.NONE
    assert simulator.current_cargo is None
