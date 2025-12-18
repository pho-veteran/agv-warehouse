#!/usr/bin/env python3
"""
Property-based tests for frontier detection functions.

Tests Properties 1-4 from the design document:
- Property 1: Frontier Detection Correctness
- Property 2: Grid to World Coordinate Conversion
- Property 3: DBSCAN Clustering Invariant
- Property 4: Obstacle Clearance Filtering
"""

import pytest
import numpy as np
from hypothesis import given, strategies as st, settings, assume
from sklearn.cluster import DBSCAN
from typing import List, Tuple


# =============================================================================
# Pure functions extracted for testing (no ROS dependencies)
# =============================================================================

def detect_frontiers_from_grid(
    map_data: np.ndarray,
    resolution: float = 0.05,
    origin_x: float = 0.0,
    origin_y: float = 0.0
) -> List[Tuple[int, int]]:
    """
    Detect frontier cells from a 2D occupancy grid.
    
    A frontier cell is a free cell (value != -1 AND < 90) that is adjacent to 
    at least one unknown cell (value -1) in 8-connected neighborhood.
    
    Args:
        map_data: 2D numpy array of occupancy values (-1=unknown, 0-100=free to occupied)
        resolution: Map resolution in meters/cell
        origin_x: Map origin x coordinate
        origin_y: Map origin y coordinate
        
    Returns:
        List of (grid_x, grid_y) coordinates for frontier cells
    """
    unknown_space = -1
    occupied_threshold = 90
    
    # 8-connected neighborhood kernel
    kernel = np.array([
        [1, 1, 1],
        [1, 0, 1],
        [1, 1, 1]
    ])
    
    # Pad map with unknown values
    padded_map = np.pad(
        map_data, 
        pad_width=1, 
        mode='constant', 
        constant_values=unknown_space
    )
    
    frontiers = []
    
    # Find frontier cells
    for y in range(1, padded_map.shape[0] - 1):
        for x in range(1, padded_map.shape[1] - 1):
            # Check if cell is free (not unknown and not occupied)
            if padded_map[y, x] != unknown_space and padded_map[y, x] < occupied_threshold:
                # Check 8-connected neighborhood for unknown cells
                neighborhood = padded_map[y-1:y+2, x-1:x+2]
                if np.any(neighborhood[kernel == 1] == unknown_space):
                    frontiers.append((x-1, y-1))  # Adjust for padding
    
    return frontiers


def grid_to_world(
    grid_x: int, 
    grid_y: int, 
    resolution: float, 
    origin_x: float, 
    origin_y: float
) -> Tuple[float, float]:
    """
    Convert grid coordinates to world coordinates.
    
    Args:
        grid_x, grid_y: Grid cell coordinates
        resolution: Map resolution in meters/cell
        origin_x, origin_y: Map origin in world coordinates
        
    Returns:
        (world_x, world_y) tuple
    """
    world_x = grid_x * resolution + origin_x
    world_y = grid_y * resolution + origin_y
    return (world_x, world_y)


def world_to_grid(
    world_x: float, 
    world_y: float, 
    resolution: float, 
    origin_x: float, 
    origin_y: float
) -> Tuple[int, int]:
    """
    Convert world coordinates to grid coordinates.
    
    Args:
        world_x, world_y: World coordinates in meters
        resolution: Map resolution in meters/cell
        origin_x, origin_y: Map origin in world coordinates
        
    Returns:
        (grid_x, grid_y) tuple
    """
    grid_x = int((world_x - origin_x) / resolution)
    grid_y = int((world_y - origin_y) / resolution)
    return (grid_x, grid_y)


def cluster_frontiers(
    frontiers: List[Tuple[float, float]],
    cluster_tolerance: float,
    min_frontier_size: int
) -> List[np.ndarray]:
    """
    Cluster frontier points using DBSCAN algorithm.
    
    Args:
        frontiers: List of frontier world coordinates
        cluster_tolerance: DBSCAN eps parameter
        min_frontier_size: DBSCAN min_samples parameter
        
    Returns:
        List of cluster centroids as numpy arrays
    """
    if len(frontiers) < 2:
        return [np.array(f) for f in frontiers]
    
    frontier_array = np.array(frontiers)
    db = DBSCAN(
        eps=cluster_tolerance, 
        min_samples=min_frontier_size
    ).fit(frontier_array)
    
    labels = db.labels_
    unique_labels = set(labels)
    
    clustered_frontiers = []
    for label in unique_labels:
        if label == -1:  # Noise points - skip
            continue
        class_member_mask = (labels == label)
        cluster_points = frontier_array[class_member_mask]
        centroid = np.mean(cluster_points, axis=0)
        clustered_frontiers.append(centroid)
    
    return clustered_frontiers


def filter_obstacles(
    frontiers: List[np.ndarray],
    map_data: np.ndarray,
    resolution: float,
    origin_x: float,
    origin_y: float,
    obstacle_clearance: float
) -> List[Tuple[float, float]]:
    """
    Filter frontiers that are too close to obstacles.
    
    Args:
        frontiers: Clustered frontier centroids
        map_data: 2D numpy array of map values
        resolution: Map resolution
        origin_x, origin_y: Map origin
        obstacle_clearance: Minimum distance from obstacles
        
    Returns:
        Filtered list of frontier coordinates
    """
    filtered_frontiers = []
    height, width = map_data.shape
    
    for frontier in frontiers:
        x, y = frontier
        # Convert world to grid coordinates
        grid_x = int((x - origin_x) / resolution)
        grid_y = int((y - origin_y) / resolution)
        
        clearance_cells = int(obstacle_clearance / resolution)
        is_clear = True
        
        # Check circular area around frontier
        for dx in range(-clearance_cells, clearance_cells + 1):
            for dy in range(-clearance_cells, clearance_cells + 1):
                # Check if within circular clearance
                if dx*dx + dy*dy <= clearance_cells*clearance_cells:
                    check_x = grid_x + dx
                    check_y = grid_y + dy
                    # Check bounds
                    if 0 <= check_x < width and 0 <= check_y < height:
                        # Check if obstacle (value > 50)
                        if map_data[check_y, check_x] > 50:
                            is_clear = False
                            break
            if not is_clear:
                break
        
        if is_clear:
            filtered_frontiers.append((float(x), float(y)))
    
    return filtered_frontiers


# =============================================================================
# Property-Based Tests
# =============================================================================

# Strategy for generating valid occupancy grid values
occupancy_value = st.integers(min_value=-1, max_value=100)


@given(st.integers(min_value=5, max_value=30), st.integers(min_value=5, max_value=30))
@settings(max_examples=100)
def test_frontier_detection_correctness(width: int, height: int):
    """
    **Feature: agv-auto-explore, Property 1: Frontier Detection Correctness**
    **Validates: Requirements 2.1**
    
    For any OccupancyGrid map, all detected frontier cells SHALL be cells where:
    - Cell value != -1 (not unknown) AND cell value < 90 (occupied_threshold)
    - At least one of the 8-connected neighbors has value -1 (unknown)
    """
    # Generate random map with mix of free, occupied, and unknown cells
    np.random.seed()
    map_data = np.random.choice([-1, 0, 50, 100], size=(height, width), p=[0.3, 0.4, 0.2, 0.1])
    
    frontiers = detect_frontiers_from_grid(map_data)
    
    unknown_space = -1
    occupied_threshold = 90
    
    for gx, gy in frontiers:
        # Verify cell is within bounds
        assert 0 <= gx < width, f"Frontier x={gx} out of bounds (width={width})"
        assert 0 <= gy < height, f"Frontier y={gy} out of bounds (height={height})"
        
        # Verify cell is free (not unknown and not occupied)
        cell_value = map_data[gy, gx]
        assert cell_value != unknown_space, f"Frontier at ({gx}, {gy}) is unknown"
        assert cell_value < occupied_threshold, f"Frontier at ({gx}, {gy}) is occupied (value={cell_value})"
        
        # Verify at least one adjacent unknown (-1) in 8-connected neighborhood
        has_unknown_neighbor = False
        for dx, dy in [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]:
            nx, ny = gx + dx, gy + dy
            if 0 <= nx < width and 0 <= ny < height:
                if map_data[ny, nx] == unknown_space:
                    has_unknown_neighbor = True
                    break
            else:
                # Out of bounds treated as unknown (due to padding)
                has_unknown_neighbor = True
                break
        
        assert has_unknown_neighbor, f"Frontier at ({gx}, {gy}) has no unknown neighbors"


@given(
    st.integers(min_value=0, max_value=100),
    st.integers(min_value=0, max_value=100),
    st.floats(min_value=0.01, max_value=1.0, allow_nan=False, allow_infinity=False),
    st.floats(min_value=-10.0, max_value=10.0, allow_nan=False, allow_infinity=False),
    st.floats(min_value=-10.0, max_value=10.0, allow_nan=False, allow_infinity=False)
)
@settings(max_examples=100)
def test_grid_to_world_coordinate_conversion(
    grid_x: int, 
    grid_y: int, 
    resolution: float, 
    origin_x: float, 
    origin_y: float
):
    """
    **Feature: agv-auto-explore, Property 2: Grid to World Coordinate Conversion**
    **Validates: Requirements 2.2**
    
    For any grid coordinate (gx, gy) and map metadata (resolution, origin_x, origin_y):
    - world_x = gx * resolution + origin_x
    - world_y = gy * resolution + origin_y
    - Round-trip: grid_to_world(world_to_grid(point)) ≈ point (within resolution tolerance)
    """
    # Test forward conversion
    world_x, world_y = grid_to_world(grid_x, grid_y, resolution, origin_x, origin_y)
    
    expected_world_x = grid_x * resolution + origin_x
    expected_world_y = grid_y * resolution + origin_y
    
    assert abs(world_x - expected_world_x) < 1e-9, f"world_x mismatch: {world_x} != {expected_world_x}"
    assert abs(world_y - expected_world_y) < 1e-9, f"world_y mismatch: {world_y} != {expected_world_y}"
    
    # Test round-trip conversion (grid -> world -> grid)
    recovered_grid_x, recovered_grid_y = world_to_grid(world_x, world_y, resolution, origin_x, origin_y)
    
    assert recovered_grid_x == grid_x, f"Round-trip grid_x mismatch: {recovered_grid_x} != {grid_x}"
    assert recovered_grid_y == grid_y, f"Round-trip grid_y mismatch: {recovered_grid_y} != {grid_y}"


@given(
    st.lists(
        st.tuples(
            st.floats(min_value=-10.0, max_value=10.0, allow_nan=False, allow_infinity=False),
            st.floats(min_value=-10.0, max_value=10.0, allow_nan=False, allow_infinity=False)
        ),
        min_size=10,
        max_size=100
    ),
    st.floats(min_value=0.05, max_value=0.5, allow_nan=False, allow_infinity=False)
)
@settings(max_examples=100)
def test_dbscan_clustering_invariant(
    frontiers: List[Tuple[float, float]], 
    cluster_tolerance: float
):
    """
    **Feature: agv-auto-explore, Property 3: DBSCAN Clustering Invariant**
    **Validates: Requirements 2.3**
    
    For any set of frontier points and cluster_tolerance parameter, all points 
    within the same cluster SHALL have at least one other point in the cluster 
    within cluster_tolerance distance.
    """
    min_frontier_size = 3  # Use small value to ensure clusters form
    
    if len(frontiers) < min_frontier_size:
        return  # Skip if not enough points
    
    frontier_array = np.array(frontiers)
    db = DBSCAN(eps=cluster_tolerance, min_samples=min_frontier_size).fit(frontier_array)
    labels = db.labels_
    
    unique_labels = set(labels)
    
    for label in unique_labels:
        if label == -1:  # Skip noise points
            continue
        
        cluster_mask = (labels == label)
        cluster_points = frontier_array[cluster_mask]
        
        # For each point in cluster, verify at least one other point is within tolerance
        for i, point in enumerate(cluster_points):
            distances = np.linalg.norm(cluster_points - point, axis=1)
            # Exclude self (distance 0)
            other_distances = distances[distances > 0]
            
            if len(other_distances) > 0:
                min_distance = np.min(other_distances)
                assert min_distance <= cluster_tolerance, \
                    f"Point {point} has no neighbor within {cluster_tolerance}, min_dist={min_distance}"


@given(
    st.integers(min_value=10, max_value=30),
    st.integers(min_value=10, max_value=30),
    st.floats(min_value=0.05, max_value=0.2, allow_nan=False, allow_infinity=False),
    st.floats(min_value=0.1, max_value=0.5, allow_nan=False, allow_infinity=False)
)
@settings(max_examples=100)
def test_obstacle_clearance_filtering(
    width: int, 
    height: int, 
    resolution: float, 
    obstacle_clearance: float
):
    """
    **Feature: agv-auto-explore, Property 4: Obstacle Clearance Filtering**
    **Validates: Requirements 2.4**
    
    For any filtered frontier and map data, the frontier position SHALL be at 
    least obstacle_clearance distance away from all obstacle cells (value > 50).
    """
    # Generate map with some obstacles
    np.random.seed()
    map_data = np.random.choice([0, 100], size=(height, width), p=[0.8, 0.2])
    
    origin_x, origin_y = 0.0, 0.0
    
    # Generate some frontier candidates in free space
    frontiers = []
    for _ in range(20):
        gx = np.random.randint(0, width)
        gy = np.random.randint(0, height)
        if map_data[gy, gx] <= 50:  # Only add if in free space
            world_x = gx * resolution + origin_x
            world_y = gy * resolution + origin_y
            frontiers.append(np.array([world_x, world_y]))
    
    if not frontiers:
        return  # Skip if no valid frontiers generated
    
    filtered = filter_obstacles(
        frontiers, map_data, resolution, origin_x, origin_y, obstacle_clearance
    )
    
    clearance_cells = int(obstacle_clearance / resolution)
    
    # Verify each filtered frontier is clear of obstacles
    for fx, fy in filtered:
        grid_x = int((fx - origin_x) / resolution)
        grid_y = int((fy - origin_y) / resolution)
        
        # Check all cells within clearance radius
        for dx in range(-clearance_cells, clearance_cells + 1):
            for dy in range(-clearance_cells, clearance_cells + 1):
                if dx*dx + dy*dy <= clearance_cells*clearance_cells:
                    check_x = grid_x + dx
                    check_y = grid_y + dy
                    if 0 <= check_x < width and 0 <= check_y < height:
                        assert map_data[check_y, check_x] <= 50, \
                            f"Filtered frontier ({fx}, {fy}) is within clearance of obstacle at ({check_x}, {check_y})"


# =============================================================================
# Unit Tests for Edge Cases
# =============================================================================

def test_detect_frontiers_empty_map():
    """Test frontier detection on empty (all unknown) map."""
    map_data = np.full((10, 10), -1)
    frontiers = detect_frontiers_from_grid(map_data)
    assert len(frontiers) == 0, "Should find no frontiers in all-unknown map"


def test_detect_frontiers_fully_explored():
    """Test frontier detection on fully explored (no unknown) map."""
    map_data = np.zeros((10, 10))
    frontiers = detect_frontiers_from_grid(map_data)
    assert len(frontiers) == 0, "Should find no frontiers in fully explored map"


def test_cluster_frontiers_single_point():
    """Test clustering with single point."""
    frontiers = [(1.0, 1.0)]
    result = cluster_frontiers(frontiers, 0.1, 5)
    assert len(result) == 1, "Single point should return as-is"


def test_filter_obstacles_empty_list():
    """Test filtering with empty frontier list."""
    map_data = np.zeros((10, 10))
    result = filter_obstacles([], map_data, 0.05, 0.0, 0.0, 0.2)
    assert len(result) == 0, "Empty input should return empty output"
