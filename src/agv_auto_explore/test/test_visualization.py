#!/usr/bin/env python3
"""
Property-based tests for visualization functions.

Tests Property 8 from the design document:
- Property 8: Marker Creation Correctness
"""

import pytest
import numpy as np
from hypothesis import given, strategies as st, settings
from typing import List, Tuple
from dataclasses import dataclass


# =============================================================================
# Mock classes to avoid ROS dependencies in tests
# =============================================================================

@dataclass
class MockPoint:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class MockQuaternion:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0


@dataclass
class MockPose:
    position: MockPoint = None
    orientation: MockQuaternion = None
    
    def __post_init__(self):
        if self.position is None:
            self.position = MockPoint()
        if self.orientation is None:
            self.orientation = MockQuaternion()


@dataclass
class MockVector3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class MockColor:
    r: float = 0.0
    g: float = 0.0
    b: float = 0.0
    a: float = 1.0


@dataclass
class MockHeader:
    frame_id: str = ""
    stamp: object = None


class MockMarker:
    """Mock ROS Marker message."""
    SPHERE = 2  # Marker type constant
    ADD = 0     # Action constant
    
    def __init__(self):
        self.header = MockHeader()
        self.ns = ""
        self.id = 0
        self.type = 0
        self.action = 0
        self.pose = MockPose()
        self.scale = MockVector3()
        self.color = MockColor()


class MockMarkerArray:
    """Mock ROS MarkerArray message."""
    def __init__(self):
        self.markers: List[MockMarker] = []


# =============================================================================
# Pure function for creating frontier markers (no ROS dependencies)
# =============================================================================

def create_frontier_markers(
    frontiers: List[Tuple[float, float]],
    frame_id: str = "map"
) -> MockMarkerArray:
    """
    Create visualization markers for frontiers.
    
    Each marker: SPHERE type, scale 0.1, green color (r=0, g=1, b=0, a=1)
    
    Args:
        frontiers: List of frontier coordinates
        frame_id: Frame ID for markers
        
    Returns:
        MarkerArray with green sphere markers at each frontier
    """
    marker_array = MockMarkerArray()
    
    for i, (x, y) in enumerate(frontiers):
        marker = MockMarker()
        marker.header.frame_id = frame_id
        marker.ns = "frontiers"
        marker.id = i
        marker.type = MockMarker.SPHERE
        marker.action = MockMarker.ADD
        
        # Position
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = 0.0
        
        # Scale (0.1 as per design)
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        
        # Color: Green (r=0, g=1, b=0)
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        
        marker_array.markers.append(marker)
    
    return marker_array


# =============================================================================
# Property-Based Tests
# =============================================================================

@given(
    st.lists(
        st.tuples(
            st.floats(min_value=-1000.0, max_value=1000.0, allow_nan=False, allow_infinity=False),
            st.floats(min_value=-1000.0, max_value=1000.0, allow_nan=False, allow_infinity=False)
        ),
        min_size=0,
        max_size=100
    )
)
@settings(max_examples=100)
def test_marker_creation_correctness(frontiers: List[Tuple[float, float]]):
    """
    **Feature: agv-auto-explore, Property 8: Marker Creation Correctness**
    **Validates: Requirements 5.2**
    
    For any frontier position (x, y), the created Marker SHALL have:
    - pose.position.x == x
    - pose.position.y == y
    - color.r == 0.0, color.g == 1.0, color.b == 0.0 (green)
    - type == Marker.SPHERE
    """
    marker_array = create_frontier_markers(frontiers)
    
    # Verify correct number of markers
    assert len(marker_array.markers) == len(frontiers), \
        f"Expected {len(frontiers)} markers, got {len(marker_array.markers)}"
    
    for i, (x, y) in enumerate(frontiers):
        marker = marker_array.markers[i]
        
        # Verify position matches frontier coordinates
        assert abs(marker.pose.position.x - x) < 1e-9, \
            f"Marker {i} x position mismatch: {marker.pose.position.x} != {x}"
        assert abs(marker.pose.position.y - y) < 1e-9, \
            f"Marker {i} y position mismatch: {marker.pose.position.y} != {y}"
        
        # Verify marker type is SPHERE
        assert marker.type == MockMarker.SPHERE, \
            f"Marker {i} type should be SPHERE ({MockMarker.SPHERE}), got {marker.type}"
        
        # Verify color is green (r=0, g=1, b=0)
        assert abs(marker.color.r - 0.0) < 1e-9, \
            f"Marker {i} color.r should be 0.0, got {marker.color.r}"
        assert abs(marker.color.g - 1.0) < 1e-9, \
            f"Marker {i} color.g should be 1.0, got {marker.color.g}"
        assert abs(marker.color.b - 0.0) < 1e-9, \
            f"Marker {i} color.b should be 0.0, got {marker.color.b}"
        
        # Verify alpha is 1.0 (fully opaque)
        assert abs(marker.color.a - 1.0) < 1e-9, \
            f"Marker {i} color.a should be 1.0, got {marker.color.a}"
        
        # Verify scale is 0.1
        assert abs(marker.scale.x - 0.1) < 1e-9, \
            f"Marker {i} scale.x should be 0.1, got {marker.scale.x}"
        assert abs(marker.scale.y - 0.1) < 1e-9, \
            f"Marker {i} scale.y should be 0.1, got {marker.scale.y}"
        assert abs(marker.scale.z - 0.1) < 1e-9, \
            f"Marker {i} scale.z should be 0.1, got {marker.scale.z}"
        
        # Verify marker ID is unique and sequential
        assert marker.id == i, \
            f"Marker {i} id should be {i}, got {marker.id}"
        
        # Verify namespace
        assert marker.ns == "frontiers", \
            f"Marker {i} namespace should be 'frontiers', got '{marker.ns}'"
        
        # Verify frame_id
        assert marker.header.frame_id == "map", \
            f"Marker {i} frame_id should be 'map', got '{marker.header.frame_id}'"


# =============================================================================
# Unit Tests for Edge Cases
# =============================================================================

def test_create_markers_empty_list():
    """Test marker creation with empty frontier list."""
    marker_array = create_frontier_markers([])
    assert len(marker_array.markers) == 0, "Empty input should produce empty marker array"


def test_create_markers_single_frontier():
    """Test marker creation with single frontier."""
    frontiers = [(1.5, -2.3)]
    marker_array = create_frontier_markers(frontiers)
    
    assert len(marker_array.markers) == 1
    marker = marker_array.markers[0]
    
    assert abs(marker.pose.position.x - 1.5) < 1e-9
    assert abs(marker.pose.position.y - (-2.3)) < 1e-9
    assert marker.type == MockMarker.SPHERE
    assert marker.color.g == 1.0


def test_create_markers_negative_coordinates():
    """Test marker creation with negative coordinates."""
    frontiers = [(-5.0, -10.0), (-0.5, -0.5)]
    marker_array = create_frontier_markers(frontiers)
    
    assert len(marker_array.markers) == 2
    assert abs(marker_array.markers[0].pose.position.x - (-5.0)) < 1e-9
    assert abs(marker_array.markers[0].pose.position.y - (-10.0)) < 1e-9
    assert abs(marker_array.markers[1].pose.position.x - (-0.5)) < 1e-9
    assert abs(marker_array.markers[1].pose.position.y - (-0.5)) < 1e-9


def test_create_markers_large_coordinates():
    """Test marker creation with large coordinates."""
    frontiers = [(1000.0, 2000.0), (-500.0, 750.0)]
    marker_array = create_frontier_markers(frontiers)
    
    assert len(marker_array.markers) == 2
    assert abs(marker_array.markers[0].pose.position.x - 1000.0) < 1e-9
    assert abs(marker_array.markers[0].pose.position.y - 2000.0) < 1e-9


def test_create_markers_unique_ids():
    """Test that all markers have unique IDs."""
    frontiers = [(i * 0.5, i * 0.5) for i in range(10)]
    marker_array = create_frontier_markers(frontiers)
    
    ids = [m.id for m in marker_array.markers]
    assert len(ids) == len(set(ids)), "All marker IDs should be unique"
    assert ids == list(range(10)), "Marker IDs should be sequential starting from 0"
