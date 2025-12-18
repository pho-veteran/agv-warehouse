#!/usr/bin/env python3
"""
Property-based tests for goal selection functions.

Tests Properties 5-7 from the design document:
- Property 5: Distance-Based Sorting
- Property 6: Minimum Distance Filtering
- Property 7: Goal Reached Threshold
"""

import pytest
import numpy as np
from hypothesis import given, strategies as st, settings, assume
from typing import List, Tuple, Optional


# =============================================================================
# Pure functions extracted for testing (no ROS dependencies)
# =============================================================================

def select_best_frontier(
    frontiers: List[Tuple[float, float]],
    robot_position: Tuple[float, float],
    min_goal_distance: float
) -> Optional[List[Tuple[float, float]]]:
    """
    Sort frontiers by distance from robot (nearest first).
    Filter out frontiers closer than min_goal_distance.
    
    Args:
        frontiers: List of frontier coordinates
        robot_position: Current robot (x, y) position
        min_goal_distance: Minimum acceptable distance
        
    Returns:
        Sorted list of valid frontiers (nearest first), or None if empty
    """
    if not frontiers:
        return None
    
    robot_pos = np.array(robot_position)
    
    # Calculate distances
    distances = [
        np.linalg.norm(robot_pos - np.array(frontier)) 
        for frontier in frontiers
    ]
    
    # Pair frontiers with distances
    frontier_distances = list(zip(frontiers, distances))
    
    # Filter by minimum distance
    valid_frontier_distances = [
        (f, d) for f, d in frontier_distances 
        if d >= min_goal_distance
    ]
    
    if not valid_frontier_distances:
        return None
    
    # Sort by distance (nearest first)
    sorted_frontiers = sorted(valid_frontier_distances, key=lambda x: x[1])
    
    return [f for f, _ in sorted_frontiers]


def is_goal_reached(
    robot_position: Tuple[float, float],
    goal_position: Tuple[float, float],
    threshold: float = 0.25
) -> bool:
    """
    Check if robot has reached the goal.
    
    Args:
        robot_position: Current robot (x, y) position
        goal_position: Goal (x, y) position
        threshold: Distance threshold for goal reached (default 0.25m)
        
    Returns:
        True if distance < threshold, False otherwise
    """
    robot_pos = np.array(robot_position)
    goal_pos = np.array(goal_position)
    distance = np.linalg.norm(robot_pos - goal_pos)
    return distance < threshold


def calculate_distance(
    point1: Tuple[float, float],
    point2: Tuple[float, float]
) -> float:
    """Calculate Euclidean distance between two points."""
    return np.linalg.norm(np.array(point1) - np.array(point2))


# =============================================================================
# Property-Based Tests
# =============================================================================

@given(
    st.lists(
        st.tuples(
            st.floats(min_value=-100.0, max_value=100.0, allow_nan=False, allow_infinity=False),
            st.floats(min_value=-100.0, max_value=100.0, allow_nan=False, allow_infinity=False)
        ),
        min_size=2,
        max_size=50
    ),
    st.tuples(
        st.floats(min_value=-100.0, max_value=100.0, allow_nan=False, allow_infinity=False),
        st.floats(min_value=-100.0, max_value=100.0, allow_nan=False, allow_infinity=False)
    ),
    st.floats(min_value=0.0, max_value=1.0, allow_nan=False, allow_infinity=False)
)
@settings(max_examples=100)
def test_distance_based_sorting(
    frontiers: List[Tuple[float, float]],
    robot_position: Tuple[float, float],
    min_goal_distance: float
):
    """
    **Feature: agv-auto-explore, Property 5: Distance-Based Sorting**
    **Validates: Requirements 3.1**
    
    For any list of sorted frontiers and robot position, for all consecutive 
    pairs (f_i, f_{i+1}) in the list: distance(robot, f_i) <= distance(robot, f_{i+1}).
    """
    result = select_best_frontier(frontiers, robot_position, min_goal_distance)
    
    if result is None or len(result) < 2:
        return  # Nothing to verify for empty or single-element results
    
    # Verify sorting: each frontier should be no farther than the next
    for i in range(len(result) - 1):
        dist_i = calculate_distance(robot_position, result[i])
        dist_i_plus_1 = calculate_distance(robot_position, result[i + 1])
        
        assert dist_i <= dist_i_plus_1 + 1e-9, \
            f"Sorting violated: distance to frontier {i} ({dist_i}) > distance to frontier {i+1} ({dist_i_plus_1})"


@given(
    st.lists(
        st.tuples(
            st.floats(min_value=-100.0, max_value=100.0, allow_nan=False, allow_infinity=False),
            st.floats(min_value=-100.0, max_value=100.0, allow_nan=False, allow_infinity=False)
        ),
        min_size=1,
        max_size=50
    ),
    st.tuples(
        st.floats(min_value=-100.0, max_value=100.0, allow_nan=False, allow_infinity=False),
        st.floats(min_value=-100.0, max_value=100.0, allow_nan=False, allow_infinity=False)
    ),
    st.floats(min_value=0.1, max_value=10.0, allow_nan=False, allow_infinity=False)
)
@settings(max_examples=100)
def test_minimum_distance_filtering(
    frontiers: List[Tuple[float, float]],
    robot_position: Tuple[float, float],
    min_goal_distance: float
):
    """
    **Feature: agv-auto-explore, Property 6: Minimum Distance Filtering**
    **Validates: Requirements 3.2**
    
    For any frontier in the selected frontiers list and robot position, 
    distance(robot, frontier) >= min_goal_distance.
    """
    result = select_best_frontier(frontiers, robot_position, min_goal_distance)
    
    if result is None:
        # Verify that all original frontiers were indeed too close
        for frontier in frontiers:
            dist = calculate_distance(robot_position, frontier)
            assert dist < min_goal_distance, \
                f"Frontier at {frontier} with distance {dist} should have been included (min_dist={min_goal_distance})"
        return
    
    # Verify all returned frontiers are at least min_goal_distance away
    for frontier in result:
        dist = calculate_distance(robot_position, frontier)
        assert dist >= min_goal_distance - 1e-9, \
            f"Frontier at {frontier} is too close: distance {dist} < min_goal_distance {min_goal_distance}"


@given(
    st.tuples(
        st.floats(min_value=-100.0, max_value=100.0, allow_nan=False, allow_infinity=False),
        st.floats(min_value=-100.0, max_value=100.0, allow_nan=False, allow_infinity=False)
    ),
    st.tuples(
        st.floats(min_value=-100.0, max_value=100.0, allow_nan=False, allow_infinity=False),
        st.floats(min_value=-100.0, max_value=100.0, allow_nan=False, allow_infinity=False)
    )
)
@settings(max_examples=100)
def test_goal_reached_threshold(
    robot_position: Tuple[float, float],
    goal_position: Tuple[float, float]
):
    """
    **Feature: agv-auto-explore, Property 7: Goal Reached Threshold**
    **Validates: Requirements 4.5**
    
    For any robot position and goal position, the goal is considered reached 
    if and only if distance(robot, goal) < 0.25 meters.
    """
    threshold = 0.25
    distance = calculate_distance(robot_position, goal_position)
    
    result = is_goal_reached(robot_position, goal_position, threshold)
    
    if distance < threshold:
        assert result is True, \
            f"Goal should be reached: distance {distance} < threshold {threshold}"
    else:
        assert result is False, \
            f"Goal should NOT be reached: distance {distance} >= threshold {threshold}"


# =============================================================================
# Unit Tests for Edge Cases
# =============================================================================

def test_select_best_frontier_empty_list():
    """Test with empty frontier list."""
    result = select_best_frontier([], (0.0, 0.0), 0.3)
    assert result is None, "Empty input should return None"


def test_select_best_frontier_all_too_close():
    """Test when all frontiers are within min_goal_distance."""
    frontiers = [(0.1, 0.1), (0.2, 0.0), (-0.1, 0.1)]
    result = select_best_frontier(frontiers, (0.0, 0.0), 0.5)
    assert result is None, "All frontiers too close should return None"


def test_select_best_frontier_single_valid():
    """Test with single valid frontier."""
    frontiers = [(1.0, 0.0)]
    result = select_best_frontier(frontiers, (0.0, 0.0), 0.3)
    assert result is not None
    assert len(result) == 1
    assert result[0] == (1.0, 0.0)


def test_goal_reached_exact_threshold():
    """Test goal reached at exact threshold boundary."""
    # At exactly 0.25m, should NOT be reached (< not <=)
    robot = (0.0, 0.0)
    goal = (0.25, 0.0)
    assert is_goal_reached(robot, goal, 0.25) is False


def test_goal_reached_just_under_threshold():
    """Test goal reached just under threshold."""
    robot = (0.0, 0.0)
    goal = (0.24, 0.0)
    assert is_goal_reached(robot, goal, 0.25) is True


def test_goal_reached_same_position():
    """Test goal reached when robot is at goal."""
    robot = (1.5, 2.5)
    goal = (1.5, 2.5)
    assert is_goal_reached(robot, goal, 0.25) is True
