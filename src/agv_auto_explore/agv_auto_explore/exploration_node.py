#!/usr/bin/env python3
"""
AGV Auto-Explore - Exploration Node

Autonomous exploration for AGV warehouse system using frontier-based exploration.
Ported from Autonomous-Turtlebot project for ROS2 Jazzy.

This node implements:
- Frontier detection from OccupancyGrid maps (vectorized for performance)
- DBSCAN clustering for frontier grouping
- Nav2 integration for navigation
- Visualization of frontiers in RViz
- Utility-based goal selection for better coverage
- Stuck detection and adaptive parameters
"""

import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import IsPathValid
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from visualization_msgs.msg import Marker, MarkerArray
from action_msgs.msg import GoalStatus
import numpy as np
import numpy.typing as npt
from scipy.ndimage import binary_dilation
from sklearn.cluster import DBSCAN
import asyncio
import threading
import time
from typing import List, Tuple, Optional


class ExplorationNode(Node):
    """
    ROS2 node for autonomous frontier-based exploration.
    
    Implements Requirements: 9.1, 9.2, 10.1, 10.3
    """

    def __init__(self):
        super().__init__('exploration_node')
        
        # =====================================================================
        # Task 2.1: ExplorationNode class initialization
        # Requirements: 9.1, 9.2, 10.1, 10.3
        # =====================================================================
        
        # Create separate callback groups for multi-threaded execution
        # Requirement 10.1: Use separate callback groups
        self.service_group = MutuallyExclusiveCallbackGroup()
        self.timer_group = ReentrantCallbackGroup()
        self.map_callback_group = MutuallyExclusiveCallbackGroup()
        
        # Create NavigateToPose action client
        self.navigate_to_pose_client = ActionClient(
            self, 
            NavigateToPose, 
            'navigate_to_pose'
        )
        
        # Create subscriptions
        # Requirement 9.1: Subscribe to /odom topic
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            20
        )
        
        # Requirement 9.2: Subscribe to amcl_pose topic
        self.amcl_pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.amcl_pose_callback,
            10
        )
        
        # Map subscription with dedicated callback group
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            5,
            callback_group=self.map_callback_group
        )
        
        # Create publishers
        self.frontier_publisher = self.create_publisher(
            MarkerArray, 
            'frontiers', 
            30
        )
        
        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            'initialpose',
            10
        )
        
        # Create IsPathValid service client
        self.is_path_valid_client = self.create_client(
            IsPathValid,
            'is_path_valid',
            callback_group=self.service_group
        )
        
        # Wait for IsPathValid service
        while not self.is_path_valid_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('IsPathValid service not available, waiting...')
        
        # Initialize state variables (matching Autonomous-Turtlebot)
        self.map: Optional[OccupancyGrid] = None
        self.latest_odom: Optional[Odometry] = None
        self.latest_amcl_pose: Optional[PoseWithCovarianceStamped] = None
        self.current_goal: Optional[Tuple[float, float]] = None
        self.start_position: Optional[Tuple[float, float]] = None
        self.goal_handle = None
        
        # Flags
        self.initial_pose_sent: bool = False
        self.exploration_initialized: bool = False
        self.navigation_in_progress: bool = False
        
        # Goal blacklisting for warehouse exploration
        self.blacklisted_goals: List[Tuple[float, float]] = []
        
        # Stuck detection
        self.last_position: Optional[np.ndarray] = None
        self.last_position_time: float = time.time()
        self.stuck_threshold_time: float = 30.0  # seconds
        self.stuck_threshold_distance: float = 0.5  # meters
        
        # Exploration progress tracking
        self.total_cells: int = 0
        self.explored_cells: int = 0
        self.exploration_complete: bool = False
        
        # Parameters - tuned for warehouse (35x50m)
        # Requirement 8.1, 8.2
        self.declare_parameter('cluster_tolerance', 0.3)
        self.declare_parameter('min_frontier_size', 10)
        self.declare_parameter('obstacle_clearance', 0.35)
        self.declare_parameter('min_goal_distance', 1.2)
        self.declare_parameter('blacklist_radius', 1.2)
        self.declare_parameter('max_blacklist_size', 100)
        self.declare_parameter('min_frontier_size_final', 3)
        self.declare_parameter('adaptive_threshold', 0.8)
        
        # Information gain parameters
        self.declare_parameter('information_gain_radius', 2.0)
        self.declare_parameter('scale_radius_with_map', True)
        
        self.cluster_tolerance = self.get_parameter('cluster_tolerance').value
        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.min_frontier_size_original = self.min_frontier_size
        self.obstacle_clearance = self.get_parameter('obstacle_clearance').value
        self.min_goal_distance = self.get_parameter('min_goal_distance').value
        self.blacklist_radius = self.get_parameter('blacklist_radius').value
        self.max_blacklist_size = self.get_parameter('max_blacklist_size').value
        self.min_frontier_size_final = self.get_parameter('min_frontier_size_final').value
        self.adaptive_threshold = self.get_parameter('adaptive_threshold').value
        
        # Information gain parameters
        self.information_gain_radius = self.get_parameter('information_gain_radius').value
        self.scale_radius_with_map = self.get_parameter('scale_radius_with_map').value
        
        self.get_logger().info(f'Parameters: cluster_tolerance={self.cluster_tolerance}, '
                               f'min_frontier_size={self.min_frontier_size}, '
                               f'obstacle_clearance={self.obstacle_clearance}, '
                               f'min_goal_distance={self.min_goal_distance}, '
                               f'blacklist_radius={self.blacklist_radius}, '
                               f'info_gain_radius={self.information_gain_radius}')
        
        # Create timers
        # Status check timer - every 5 seconds
        self.create_timer(5.0, self.check_status)
        
        # Planning timer - every 5 seconds
        self.planning_timer = self.create_timer(
            5.0,
            self.planning_timer_callback,
            callback_group=self.timer_group
        )
        
        # Create asyncio event loop for async operations
        # Requirement 10.3
        self.loop = asyncio.new_event_loop()
        
        self.get_logger().info('Exploration node initialized')


    # =========================================================================
    # Task 2.2: Frontier detection functions
    # Requirements: 2.1, 2.2, 2.3, 2.4
    # =========================================================================

    def detect_frontiers(self) -> List[Tuple[float, float]]:
        """
        Detect frontier cells from occupancy grid using vectorized operations.
        
        A frontier cell is a free cell (value >= 0 AND < 50) that is adjacent to 
        at least one unknown cell (value -1) in 8-connected neighborhood.
        
        Uses scipy.ndimage.binary_dilation for 10-50x speedup over nested loops.
        
        Requirements: 2.1, 2.2
        
        Returns:
            List of (world_x, world_y) coordinates for frontier cells
        """
        if self.map is None:
            return []

        map_data = np.array(self.map.data).reshape(
            (self.map.info.height, self.map.info.width)
        )
        
        # Update exploration progress tracking
        self.total_cells = map_data.size
        self.explored_cells = np.sum((map_data >= 0) & (map_data < 50))
        
        # Create masks using vectorized operations
        free_mask = (map_data >= 0) & (map_data < 50)  # Free space
        unknown_mask = (map_data == -1)  # Unknown space
        
        # 8-connected structuring element
        struct = np.array([[1, 1, 1],
                          [1, 1, 1],
                          [1, 1, 1]], dtype=bool)
        
        # Dilate unknown to find cells adjacent to unknown
        unknown_adjacent = binary_dilation(unknown_mask, structure=struct, iterations=1)
        
        # Frontiers = free cells that are adjacent to unknown
        frontier_mask = free_mask & unknown_adjacent
        
        # Get coordinates of frontier cells
        frontier_coords = np.argwhere(frontier_mask)
        
        # Convert grid coordinates to world coordinates (Requirement 2.2)
        world_frontiers = []
        for gy, gx in frontier_coords:  # Note: argwhere returns (row, col) = (y, x)
            world_x = gx * self.map.info.resolution + self.map.info.origin.position.x
            world_y = gy * self.map.info.resolution + self.map.info.origin.position.y
            world_frontiers.append((world_x, world_y))

        return world_frontiers

    def cluster_frontiers(
        self, 
        frontiers: List[Tuple[float, float]]
    ) -> List[Tuple[float, float]]:
        """
        Cluster frontier points using DBSCAN algorithm.
        
        Requirement 2.3
        
        Args:
            frontiers: List of frontier world coordinates
            
        Returns:
            List of cluster centroids as (x, y) tuples
        """
        if len(frontiers) < 2:
            return frontiers

        frontier_array = np.array(frontiers)
        db = DBSCAN(
            eps=self.cluster_tolerance, 
            min_samples=self.min_frontier_size
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
            # Convert to tuple for type consistency
            clustered_frontiers.append((float(centroid[0]), float(centroid[1])))

        return clustered_frontiers

    def filter_obstacles(
        self, 
        frontiers: List[Tuple[float, float]], 
        map_data: npt.NDArray[np.int8]
    ) -> List[Tuple[float, float]]:
        """
        Filter frontiers that are too close to obstacles.
        
        Requirement 2.4
        
        Args:
            frontiers: Clustered frontier centroids
            map_data: 2D numpy array of map values
            
        Returns:
            Filtered list of frontier coordinates
        """
        filtered_frontiers = []
        rejected_count = 0
        
        for frontier in frontiers:
            x, y = frontier[0], frontier[1]
            # Convert world to grid coordinates
            grid_x = int((x - self.map.info.origin.position.x) / self.map.info.resolution)
            grid_y = int((y - self.map.info.origin.position.y) / self.map.info.resolution)

            clearance_cells = int(self.obstacle_clearance / self.map.info.resolution)
            is_clear = True
            obstacle_value = 0
            
            # Check circular area around frontier
            for dx in range(-clearance_cells, clearance_cells + 1):
                for dy in range(-clearance_cells, clearance_cells + 1):
                    # Check if within circular clearance
                    if dx*dx + dy*dy <= clearance_cells*clearance_cells:
                        check_x = grid_x + dx
                        check_y = grid_y + dy
                        # Check bounds
                        if 0 <= check_x < self.map.info.width and 0 <= check_y < self.map.info.height:
                            # Check if obstacle (value > 50)
                            if map_data[check_y, check_x] > 50:
                                obstacle_value = map_data[check_y, check_x]
                                is_clear = False
                                break
                if not is_clear:
                    break
            
            if is_clear:
                filtered_frontiers.append((float(x), float(y)))
            else:
                rejected_count += 1
                self.get_logger().debug(
                    f'[FILTER] Rejected frontier ({x:.2f}, {y:.2f}) - '
                    f'obstacle within {self.obstacle_clearance}m (value={obstacle_value})'
                )
        
        if rejected_count > 0:
            self.get_logger().info(
                f'[FILTER] {rejected_count}/{len(frontiers)} frontiers rejected '
                f'(too close to obstacles, clearance={self.obstacle_clearance}m)'
            )

        return filtered_frontiers

    def filter_obstacles_vectorized(
        self, 
        frontiers: List[Tuple[float, float]], 
        map_data: npt.NDArray[np.int8]
    ) -> List[Tuple[float, float]]:
        """
        Vectorized version of obstacle filtering for better performance on large maps.
        Uses numpy operations instead of nested loops.
        
        Args:
            frontiers: List of frontier coordinates
            map_data: 2D numpy array of map values
            
        Returns:
            Filtered list of frontier coordinates
        """
        if not frontiers:
            return []
        
        clearance_cells = int(self.obstacle_clearance / self.map.info.resolution)
        
        # Pre-compute circular mask
        y, x = np.ogrid[-clearance_cells:clearance_cells+1, -clearance_cells:clearance_cells+1]
        circular_mask = x*x + y*y <= clearance_cells*clearance_cells
        
        filtered_frontiers = []
        rejected_count = 0
        
        for frontier_x, frontier_y in frontiers:
            # Convert to grid coordinates
            grid_x = int((frontier_x - self.map.info.origin.position.x) / self.map.info.resolution)
            grid_y = int((frontier_y - self.map.info.origin.position.y) / self.map.info.resolution)
            
            # Extract region around frontier
            y_min = max(0, grid_y - clearance_cells)
            y_max = min(map_data.shape[0], grid_y + clearance_cells + 1)
            x_min = max(0, grid_x - clearance_cells)
            x_max = min(map_data.shape[1], grid_x + clearance_cells + 1)
            
            region = map_data[y_min:y_max, x_min:x_max]
            
            # Adjust mask for boundary cases
            mask_y_start = max(0, clearance_cells - grid_y)
            mask_x_start = max(0, clearance_cells - grid_x)
            mask_y_end = mask_y_start + (y_max - y_min)
            mask_x_end = mask_x_start + (x_max - x_min)
            
            if (mask_y_end <= circular_mask.shape[0] and 
                mask_x_end <= circular_mask.shape[1] and
                mask_y_start >= 0 and mask_x_start >= 0):
                
                region_mask = circular_mask[mask_y_start:mask_y_end, mask_x_start:mask_x_end]
                
                # Check if any obstacles in masked region
                if not np.any((region > 50) & region_mask):
                    filtered_frontiers.append((frontier_x, frontier_y))
                else:
                    rejected_count += 1
            else:
                # Fallback to original method for edge cases
                rejected_count += 1
        
        if rejected_count > 0:
            self.get_logger().info(
                f'[FILTER_VEC] {rejected_count}/{len(frontiers)} frontiers rejected '
                f'(too close to obstacles, clearance={self.obstacle_clearance}m)'
            )
        
        return filtered_frontiers


    # =========================================================================
    # Task 2.3: Goal selection functions
    # Requirements: 3.1, 3.2, 3.3, 3.4
    # =========================================================================

    def select_best_frontier(
        self, 
        frontiers: List[Tuple[float, float]]
    ) -> Optional[List[Tuple[float, float]]]:
        """
        Select frontiers using utility-based approach: balance distance vs information gain.
        Filter out frontiers closer than min_goal_distance and blacklisted goals.
        
        Requirements: 3.1, 3.2
        
        Args:
            frontiers: List of frontier coordinates
            
        Returns:
            Sorted list of valid frontiers (best utility first), or None if empty
        """
        if not frontiers or self.latest_odom is None:
            return None
            
        robot_pos = np.array([
            self.latest_odom.pose.pose.position.x, 
            self.latest_odom.pose.pose.position.y
        ])
        
        # Debug: count rejection reasons
        too_close_count = 0
        blacklisted_count = 0
        too_close_frontiers = []
        blacklisted_frontiers = []
        
        # Calculate utility for each frontier
        frontier_utilities = []
        for frontier in frontiers:
            distance = np.linalg.norm(robot_pos - np.array(frontier))
            
            # Skip if too close
            if distance < self.min_goal_distance:
                too_close_count += 1
                too_close_frontiers.append((frontier, distance))
                continue
            
            # Skip if blacklisted
            if self.is_goal_blacklisted(frontier):
                blacklisted_count += 1
                blacklisted_frontiers.append(frontier)
                continue
            
            # Estimate information gain (unknown cells in radius around frontier)
            info_gain = self.estimate_information_gain(frontier)
            
            # Utility = info_gain / distance (favor high info gain, penalize distance)
            # Add small constant to avoid division by zero
            utility = info_gain / (distance + 0.1)
            
            frontier_utilities.append((frontier, utility, distance))
        
        if not frontier_utilities:
            # Log detailed rejection reasons
            self.get_logger().warn(
                f'[SELECT] No valid frontiers: {len(frontiers)} total, '
                f'{too_close_count} too close (<{self.min_goal_distance}m), '
                f'{blacklisted_count} blacklisted'
            )
            
            # Log details of rejected frontiers
            if too_close_frontiers:
                for f, d in too_close_frontiers[:3]:  # Show first 3
                    self.get_logger().info(
                        f'[SELECT] Too close: ({f[0]:.2f}, {f[1]:.2f}) dist={d:.2f}m < {self.min_goal_distance}m'
                    )
            if blacklisted_frontiers:
                for f in blacklisted_frontiers[:3]:  # Show first 3
                    self.get_logger().info(
                        f'[SELECT] Blacklisted: ({f[0]:.2f}, {f[1]:.2f})'
                    )
            
            # If most frontiers are blacklisted (>50%), clear old blacklist entries
            if blacklisted_count > 0 and blacklisted_count >= len(frontiers) // 2:
                self.get_logger().info(
                    f'Too many blacklisted ({blacklisted_count}/{len(frontiers)}) - '
                    f'clearing oldest entries'
                )
                # Remove 2/3 of blacklist to allow retry
                remove_count = max(1, (len(self.blacklisted_goals) * 2) // 3)
                self.blacklisted_goals = self.blacklisted_goals[remove_count:]
                # Retry selection with reduced distance as fallback
                return self._select_with_reduced_distance(frontiers, robot_pos)
            
            # If all/most frontiers are too close, try with reduced min_goal_distance
            if too_close_count > 0:
                return self._select_with_reduced_distance(frontiers, robot_pos)
            
            self.get_logger().warn('No frontiers beyond minimum distance')
            return None
        
        # Sort by utility (highest first)
        sorted_frontiers = sorted(frontier_utilities, key=lambda x: x[1], reverse=True)
        
        self.get_logger().debug(
            f'Top frontier: utility={sorted_frontiers[0][1]:.2f}, '
            f'distance={sorted_frontiers[0][2]:.2f}m'
        )
        
        return [f for f, _, _ in sorted_frontiers]

    def _select_with_reduced_distance(
        self,
        frontiers: List[Tuple[float, float]],
        robot_pos: np.ndarray
    ) -> Optional[List[Tuple[float, float]]]:
        """
        Fallback selection with reduced min_goal_distance when all frontiers are too close.
        Uses progressively smaller distances, and finally picks the farthest frontier.
        
        Args:
            frontiers: List of frontier coordinates
            robot_pos: Current robot position
            
        Returns:
            Sorted list of valid frontiers, or None if still empty
        """
        # Try progressively smaller distances
        reduced_distances = [0.2, 0.15, 0.1]
        
        for reduced_min_distance in reduced_distances:
            self.get_logger().info(
                f'Retrying with reduced min_goal_distance: {reduced_min_distance}m'
            )
            
            frontier_utilities = []
            for frontier in frontiers:
                distance = np.linalg.norm(robot_pos - np.array(frontier))
                
                # Use reduced distance threshold
                if distance < reduced_min_distance:
                    continue
                
                # Still check blacklist
                if self.is_goal_blacklisted(frontier):
                    continue
                
                info_gain = self.estimate_information_gain(frontier)
                utility = info_gain / (distance + 0.1)
                frontier_utilities.append((frontier, utility, distance))
            
            if frontier_utilities:
                # Sort by utility (highest first)
                sorted_frontiers = sorted(frontier_utilities, key=lambda x: x[1], reverse=True)
                self.get_logger().info(
                    f'Found {len(sorted_frontiers)} frontiers with reduced distance. '
                    f'Best: distance={sorted_frontiers[0][2]:.2f}m'
                )
                return [f for f, _, _ in sorted_frontiers]
        
        # Last resort: pick the farthest frontier regardless of distance
        self.get_logger().warn('All frontiers very close - selecting farthest one')
        
        farthest_frontier = None
        max_distance = 0.0
        
        for frontier in frontiers:
            distance = np.linalg.norm(robot_pos - np.array(frontier))
            if distance > max_distance and not self.is_goal_blacklisted(frontier):
                max_distance = distance
                farthest_frontier = frontier
        
        if farthest_frontier is not None:
            self.get_logger().info(
                f'Selected farthest frontier at distance={max_distance:.2f}m'
            )
            return [farthest_frontier]
        
        self.get_logger().warn('No valid frontiers found even with fallback')
        return None

    def estimate_information_gain(self, frontier: Tuple[float, float]) -> float:
        """
        Estimate information gain at a frontier by counting unknown cells in radius.
        Uses configurable radius that can scale with map size.
        
        Args:
            frontier: (x, y) world coordinates
            
        Returns:
            Estimated information gain (count of unknown cells)
        """
        if self.map is None:
            return 0.0
        
        # Convert to grid coordinates
        grid_x = int((frontier[0] - self.map.info.origin.position.x) / self.map.info.resolution)
        grid_y = int((frontier[1] - self.map.info.origin.position.y) / self.map.info.resolution)
        
        # Calculate search radius based on configuration
        base_radius = self.information_gain_radius
        
        if self.scale_radius_with_map:
            # Scale based on map size (larger maps = larger search radius)
            map_diagonal = np.sqrt(
                (self.map.info.width * self.map.info.resolution)**2 + 
                (self.map.info.height * self.map.info.resolution)**2
            )
            scale_factor = min(2.0, map_diagonal / 20.0)  # Cap at 2x
            search_radius_m = base_radius * scale_factor
        else:
            search_radius_m = base_radius
        
        search_radius_cells = int(search_radius_m / self.map.info.resolution)
        
        map_data = np.array(self.map.data).reshape(
            (self.map.info.height, self.map.info.width)
        )
        
        # Count unknown cells in circular area
        unknown_count = 0
        for dy in range(-search_radius_cells, search_radius_cells + 1):
            for dx in range(-search_radius_cells, search_radius_cells + 1):
                if dx*dx + dy*dy <= search_radius_cells*search_radius_cells:
                    check_x = grid_x + dx
                    check_y = grid_y + dy
                    if 0 <= check_x < self.map.info.width and 0 <= check_y < self.map.info.height:
                        if map_data[check_y, check_x] == -1:  # Unknown
                            unknown_count += 1
        
        return float(unknown_count)

    async def is_path_free_nav2(
        self, 
        start: Tuple[float, float], 
        goal: Tuple[float, float]
    ) -> bool:
        """
        Check if path from start to goal is valid using Nav2 IsPathValid service.
        
        Requirements: 3.3, 3.4
        
        Args:
            start: Start position (x, y)
            goal: Goal position (x, y)
            
        Returns:
            True if path is valid, False otherwise
        """
        # Create start pose
        start_pose = PoseStamped()
        start_pose.header.frame_id = 'map'
        start_pose.header.stamp = self.get_clock().now().to_msg()
        start_pose.pose.position.x = float(start[0])
        start_pose.pose.position.y = float(start[1])
        start_pose.pose.orientation.w = 1.0

        # Create goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(goal[0])
        goal_pose.pose.position.y = float(goal[1])
        goal_pose.pose.orientation.w = 1.0

        # Create request
        request = IsPathValid.Request()
        request.path.header.frame_id = 'map'
        request.path.header.stamp = self.get_clock().now().to_msg()
        request.path.poses = [start_pose, goal_pose]

        future = self.is_path_valid_client.call_async(request)
        
        try:
            # Wait for the result with timeout using rclpy Future
            # rclpy.task.Future is not compatible with asyncio.wrap_future
            start_time = self.get_clock().now()
            timeout_duration = rclpy.duration.Duration(seconds=1.0)
            
            while not future.done():
                if (self.get_clock().now() - start_time) > timeout_duration:
                    self.get_logger().warn(
                        f'[PATH_CHECK] TIMEOUT checking path to ({goal[0]:.2f}, {goal[1]:.2f})'
                    )
                    return False
                await asyncio.sleep(0.01)  # Small sleep to avoid busy waiting
            
            response = future.result()
            if not response.is_valid:
                self.get_logger().warn(
                    f'[PATH_CHECK] REJECTED by Nav2: goal=({goal[0]:.2f}, {goal[1]:.2f}) - '
                    f'Nav2 planner cannot find valid path'
                )
            return response.is_valid
        except Exception as e:
            self.get_logger().warn(
                f'[PATH_CHECK] SERVICE ERROR for goal=({goal[0]:.2f}, {goal[1]:.2f}): {e}'
            )
            return False


    # =========================================================================
    # Task 2.4: Navigation interface functions
    # Requirements: 4.1, 4.2, 4.3, 4.4
    # =========================================================================

    def send_goal(self, x: float, y: float) -> None:
        """
        Send navigation goal to Nav2.
        
        Requirement 4.1
        
        Args:
            x, y: Goal coordinates in map frame
        """
        self.current_goal = (x, y)
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info('Waiting for NavigateToPose action server...')
        self.navigate_to_pose_client.wait_for_server()
        
        self.get_logger().info(f'Sending goal request to ({x:.2f}, {y:.2f})...')
        self.send_goal_future = self.navigate_to_pose_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        self.navigation_in_progress = True
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future) -> None:
        """
        Handle goal acceptance/rejection.
        
        Requirement 4.2
        """
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            goal_str = f'({self.current_goal[0]:.2f}, {self.current_goal[1]:.2f})' if self.current_goal else 'None'
            self.get_logger().warn(
                f'[NAV2] Goal REJECTED by Nav2 action server: {goal_str} - '
                f'Nav2 refused to accept this navigation goal'
            )
            self.current_goal = None
            self.navigation_in_progress = False
            # Plan next goal
            asyncio.run_coroutine_threadsafe(self.plan_next_goal(), self.loop)
            return

        goal_str = f'({self.current_goal[0]:.2f}, {self.current_goal[1]:.2f})' if self.current_goal else 'None'
        self.get_logger().info(f'[NAV2] Goal ACCEPTED: {goal_str}')
        self.goal_handle = goal_handle

        # Get result asynchronously
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future) -> None:
        """
        Handle navigation success/failure.
        
        Requirements: 4.3, 4.4
        """
        result = future.result().result
        status = future.result().status
        
        goal_str = f'({self.current_goal[0]:.2f}, {self.current_goal[1]:.2f})' if self.current_goal else 'None'
        
        # Status codes: 1=ACCEPTED, 2=EXECUTING, 3=CANCELING, 4=SUCCEEDED, 5=CANCELED, 6=ABORTED
        status_names = {
            1: 'ACCEPTED', 2: 'EXECUTING', 3: 'CANCELING', 
            4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'
        }
        status_name = status_names.get(status, f'UNKNOWN({status})')
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'[NAV2] Navigation SUCCEEDED to {goal_str}')
            self.navigation_in_progress = False
            self.current_goal = None
            # Plan next goal (Requirement 4.3)
            asyncio.run_coroutine_threadsafe(self.plan_next_goal(), self.loop)
        else:
            self.get_logger().warn(
                f'[NAV2] Navigation FAILED to {goal_str} - status={status_name} '
                f'(6=ABORTED means planner/controller failed)'
            )
            # Handle failure (Requirement 4.4)
            self.handle_navigation_failure()

    def feedback_callback(self, feedback_msg) -> None:
        """
        Log navigation feedback.
        
        Requirement 4.2
        """
        feedback = feedback_msg.feedback
        self.get_logger().debug(f'Navigation feedback received: {feedback}')


    # =========================================================================
    # Task 2.5: Visualization functions
    # Requirements: 5.1, 5.2, 5.3
    # =========================================================================

    def visualize_frontiers(self, frontiers: List[Tuple[float, float]]) -> None:
        """
        Create and publish MarkerArray for frontier visualization.
        
        Requirements: 5.1, 5.2, 5.3
        
        Args:
            frontiers: List of frontier coordinates
        """
        marker_array = MarkerArray()
        
        for i, (x, y) in enumerate(frontiers):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "frontiers"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # Position
            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = 0.0
            
            # Scale (0.1 as per design)
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            
            # Color: Green (r=0, g=1, b=0) as per Requirement 5.2
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            
            marker_array.markers.append(marker)
        
        # Publish MarkerArray (Requirement 5.1)
        self.frontier_publisher.publish(marker_array)


    # =========================================================================
    # Task 2.6: Workflow callbacks
    # Requirements: 9.3, 9.4
    # =========================================================================

    def map_callback(self, msg: OccupancyGrid) -> None:
        """
        Handle map updates - trigger initial pose, initialization, or planning.
        
        Requirements: 9.3, 9.4
        """
        self.map = msg
        
        if not self.initial_pose_sent:
            # Send initial pose estimate (Requirement 9.3)
            self.send_initial_pose()
        elif not self.exploration_initialized:
            # Initialize exploration (Requirement 9.4)
            self.initialize_exploration()
        elif self.current_goal is None and not self.navigation_in_progress:
            # Plan next goal
            asyncio.run_coroutine_threadsafe(self.plan_next_goal(), self.loop)

    def odom_callback(self, msg: Odometry) -> None:
        """
        Update latest odometry.
        
        Requirement 9.1
        """
        self.latest_odom = msg
        self.get_logger().debug(
            f'Received odom update. Position: '
            f'({msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f})'
        )

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        """
        Update latest AMCL pose.
        
        Requirement 9.2
        """
        self.latest_amcl_pose = msg
        self.get_logger().debug(
            f'Received AMCL pose update. Position: '
            f'({msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f})'
        )

    def send_initial_pose(self) -> None:
        """
        Publish initial pose estimate.
        
        Requirement 9.3
        """
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.header.frame_id = 'map'

        if self.latest_odom:
            # Use odometry data if available
            initial_pose.pose.pose = self.latest_odom.pose.pose
            self.get_logger().info('Sent initial pose estimate based on odometry')
        else:
            # Set to (0,0,0) if no odometry data
            initial_pose.pose.pose.position.x = 0.0
            initial_pose.pose.pose.position.y = 0.0
            initial_pose.pose.pose.position.z = 0.0
            initial_pose.pose.pose.orientation.x = 0.0
            initial_pose.pose.pose.orientation.y = 0.0
            initial_pose.pose.pose.orientation.z = 0.0
            initial_pose.pose.pose.orientation.w = 1.0
            self.get_logger().info('Sent initial pose estimate at (0, 0, 0)')

        # Set covariance
        initial_pose.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942
        ]

        self.initial_pose_publisher.publish(initial_pose)
        self.initial_pose_sent = True
        
        # Try to initialize exploration if map is available
        if not self.exploration_initialized and self.map is not None:
            self.initialize_exploration()

    def initialize_exploration(self) -> bool:
        """
        Set start_position and trigger first planning.
        
        Requirement 9.4
        
        Returns:
            True if initialization succeeded, False otherwise
        """
        if self.latest_odom is None:
            self.get_logger().warn('No odometry data available. Cannot initialize exploration.')
            return False
        
        self.start_position = (
            self.latest_odom.pose.pose.position.x, 
            self.latest_odom.pose.pose.position.y
        )
        self.exploration_initialized = True
        
        self.get_logger().info(
            f'Exploration initialized. Start position: '
            f'({self.start_position[0]:.2f}, {self.start_position[1]:.2f})'
        )
        
        # Trigger first planning
        asyncio.run_coroutine_threadsafe(self.plan_next_goal(), self.loop)
        return True

    def check_status(self) -> None:
        """
        Periodically log status information and check for stuck condition.
        """
        # Calculate exploration progress
        explored_ratio = self.get_exploration_ratio()
        
        self.get_logger().info(
            f'Status: Map={"Received" if self.map else "Not received"}, '
            f'Goal={self.current_goal if self.current_goal else "None"}, '
            f'Explored={explored_ratio*100:.1f}%, '
            f'Blacklisted={len(self.blacklisted_goals)}'
        )
        
        # Check for stuck condition
        if self.check_stuck():
            self.handle_stuck_condition()
        
        # Adapt parameters based on exploration progress
        self.adapt_parameters()
        
        # Check if exploration is complete
        if self.is_exploration_complete():
            self.get_logger().info('🎉 Exploration complete! Map fully explored.')

    def get_exploration_ratio(self) -> float:
        """Get ratio of explored cells to total cells."""
        if self.total_cells == 0:
            return 0.0
        return self.explored_cells / self.total_cells

    def check_stuck(self) -> bool:
        """
        Detect if robot is stuck (no significant movement for threshold time).
        
        Returns:
            True if robot appears stuck
        """
        if self.latest_odom is None:
            return False
        
        current_pos = np.array([
            self.latest_odom.pose.pose.position.x,
            self.latest_odom.pose.pose.position.y
        ])
        
        if self.last_position is None:
            self.last_position = current_pos
            self.last_position_time = time.time()
            return False
        
        distance_moved = np.linalg.norm(current_pos - self.last_position)
        time_elapsed = time.time() - self.last_position_time
        
        # Update position if moved significantly
        if distance_moved > self.stuck_threshold_distance:
            self.last_position = current_pos
            self.last_position_time = time.time()
            return False
        
        # Check if stuck
        if time_elapsed > self.stuck_threshold_time:
            self.get_logger().warn(
                f'Robot appears stuck! Moved only {distance_moved:.2f}m in {time_elapsed:.0f}s'
            )
            return True
        
        return False

    def handle_stuck_condition(self) -> None:
        """Handle stuck condition by clearing goal and blacklisting."""
        self.get_logger().warn('Handling stuck condition - clearing current goal')
        
        if self.current_goal:
            self.blacklist_goal(self.current_goal)
        
        # Cancel current navigation
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        
        self.current_goal = None
        self.navigation_in_progress = False
        
        # Reset stuck detection
        self.last_position = None
        self.last_position_time = time.time()
        
        # Trigger replanning
        asyncio.run_coroutine_threadsafe(self.plan_next_goal(), self.loop)

    def adapt_parameters(self) -> None:
        """
        Adapt exploration parameters based on progress.
        Reduce min_frontier_size when exploration is nearly complete.
        """
        explored_ratio = self.get_exploration_ratio()
        
        if explored_ratio > self.adaptive_threshold:
            # Late-stage exploration: reduce frontier size threshold
            if self.min_frontier_size > self.min_frontier_size_final:
                self.min_frontier_size = self.min_frontier_size_final
                self.get_logger().info(
                    f'Adapted parameters for late-stage exploration: '
                    f'min_frontier_size={self.min_frontier_size}'
                )

    def is_exploration_complete(self) -> bool:
        """
        Check if exploration is complete (no more valid frontiers).
        
        Returns:
            True if map is fully explored
        """
        if self.exploration_complete:
            return True
        
        if self.map is None:
            return False
        
        # Detect frontiers
        frontiers = self.detect_frontiers()
        if not frontiers:
            self.exploration_complete = True
            return True
        
        # Cluster and filter
        map_data = np.array(self.map.data).reshape(
            (self.map.info.height, self.map.info.width)
        )
        clustered = self.cluster_frontiers(frontiers)
        filtered = self.filter_obstacles(clustered, map_data)
        
        if not filtered:
            self.exploration_complete = True
            return True
        
        return False


    # =========================================================================
    # Task 2.7: Planning workflow
    # Requirement: 4.5
    # =========================================================================

    def planning_timer_callback(self) -> None:
        """
        Periodic planning trigger.
        
        Requirement 4.5
        """
        if not self.exploration_initialized:
            if self.initialize_exploration():
                self.get_logger().info('Exploration initialized in timer callback')
            else:
                self.get_logger().warn('Failed to initialize exploration in timer callback')
            return

        if self.map is None:
            self.get_logger().warn('No map available for planning')
            return
        
        if self.current_goal is None:
            asyncio.run_coroutine_threadsafe(self.plan_next_goal(), self.loop)
        else:
            self.check_and_replan()

    def check_and_replan(self) -> None:
        """
        Check if goal reached or path blocked, trigger replan if needed.
        
        Requirement 4.5
        """
        if self.latest_odom is None or self.current_goal is None:
            return
        
        robot_pos = np.array([
            self.latest_odom.pose.pose.position.x, 
            self.latest_odom.pose.pose.position.y
        ])
        goal_pos = np.array(self.current_goal)
        distance_to_goal = np.linalg.norm(robot_pos - goal_pos)
        
        # Check if within 0.25m of goal (Requirement 4.5)
        if distance_to_goal < 0.25:
            self.get_logger().info('Near current goal. Planning next goal.')
            self.navigation_in_progress = False
            self.current_goal = None
            asyncio.run_coroutine_threadsafe(self.plan_next_goal(), self.loop)
        else:
            # Check if path is still valid
            asyncio.run_coroutine_threadsafe(
                self.check_path_and_replan(robot_pos, goal_pos), 
                self.loop
            )

    async def check_path_and_replan(
        self, 
        robot_pos: np.ndarray, 
        goal_pos: np.ndarray
    ) -> None:
        """
        Async path validation and replan if blocked.
        
        Requirement 4.5
        """
        if not await self.is_path_free_nav2(tuple(robot_pos), tuple(goal_pos)):
            self.get_logger().info('Path to current goal may be obstructed. Replanning.')
            self.current_goal = None
            self.navigation_in_progress = False
            await self.plan_next_goal()

    async def plan_next_goal(self) -> None:
        """
        Full planning pipeline: detect frontiers, cluster, filter, select, navigate.
        
        Requirements: 2.1-2.4, 3.1-3.4, 4.1
        """
        if self.navigation_in_progress and self.current_goal is not None:
            self.get_logger().info('Navigation already in progress, skipping planning')
            return
            
        if self.map is None:
            self.get_logger().warn('No map available for planning')
            return

        # Step 1: Detect frontiers
        frontiers = self.detect_frontiers()
        
        if not frontiers:
            self.get_logger().warn('No frontiers detected')
            return

        # Step 2: Cluster frontiers
        map_data = np.array(self.map.data).reshape(
            (self.map.info.height, self.map.info.width)
        )
        clustered_frontiers = self.cluster_frontiers(frontiers)
        
        # Step 3: Filter obstacles (use vectorized version for large frontier sets)
        if len(clustered_frontiers) > 50:  # Use vectorized for performance
            filtered_frontiers = self.filter_obstacles_vectorized(clustered_frontiers, map_data)
        else:
            filtered_frontiers = self.filter_obstacles(clustered_frontiers, map_data)

        if not filtered_frontiers:
            self.get_logger().warn('No valid frontiers after filtering')
            return
        else:
            self.get_logger().info(f'Detected {len(filtered_frontiers)} filtered frontiers')

        # Step 4: Select best frontier (sort by distance)
        sorted_frontiers = self.select_best_frontier(filtered_frontiers)
        
        if sorted_frontiers is None:
            self.get_logger().warn('No valid frontiers after selection')
            return
            
        # Visualize frontiers
        self.visualize_frontiers(sorted_frontiers)

        if self.latest_odom is None:
            self.get_logger().warn('No odometry data available')
            return

        robot_pos = np.array([
            self.latest_odom.pose.pose.position.x, 
            self.latest_odom.pose.pose.position.y
        ])

        # Step 5: Try each frontier until one is reachable
        rejected_by_nav2 = 0
        self.get_logger().info(
            f'[GOAL_SELECT] Checking {len(sorted_frontiers)} frontiers for valid path...'
        )
        
        for i, frontier in enumerate(sorted_frontiers):
            distance = np.linalg.norm(robot_pos - np.array(frontier))
            self.get_logger().debug(
                f'[GOAL_SELECT] Checking frontier {i+1}/{len(sorted_frontiers)}: '
                f'({frontier[0]:.2f}, {frontier[1]:.2f}), distance={distance:.2f}m'
            )
            
            if await self.is_path_free_nav2(tuple(robot_pos), frontier):
                if not self.navigation_in_progress or self.current_goal is None:
                    self.get_logger().info(
                        f'[GOAL_SELECT] ✓ Selected goal: ({frontier[0]:.2f}, {frontier[1]:.2f}), '
                        f'distance={distance:.2f}m (checked {i+1} frontiers)'
                    )
                    self.current_goal = frontier
                    self.send_goal(frontier[0], frontier[1])
                return
            else:
                rejected_by_nav2 += 1

        self.get_logger().warn(
            f'[GOAL_SELECT] ✗ No reachable frontiers! '
            f'All {rejected_by_nav2} frontiers rejected by Nav2 path planner'
        )
        await self.handle_no_reachable_frontiers()


    # =========================================================================
    # Task 2.8: Recovery handling
    # Requirements: 6.1, 6.2, 6.3
    # =========================================================================

    def handle_navigation_failure(self) -> None:
        """
        Clear goal, blacklist it, and replan after navigation failure.
        
        Requirement 6.3
        """
        self.get_logger().warn(
            'Navigation to current goal failed. Clearing current goal and replanning.'
        )
        
        # Blacklist the failed goal to prevent infinite retry loops
        if self.current_goal:
            self.blacklist_goal(self.current_goal)
        
        self.current_goal = None
        self.navigation_in_progress = False
        asyncio.run_coroutine_threadsafe(self.plan_next_goal(), self.loop)

    def is_goal_blacklisted(self, goal: Tuple[float, float]) -> bool:
        """
        Check if goal is too close to any blacklisted goal.
        
        Args:
            goal: (x, y) coordinates to check
            
        Returns:
            True if goal should be avoided (too close to blacklisted goal)
        """
        goal_array = np.array(goal)
        for blacklisted in self.blacklisted_goals:
            distance = np.linalg.norm(goal_array - np.array(blacklisted))
            if distance < self.blacklist_radius:
                self.get_logger().debug(
                    f'[BLACKLIST] Goal ({goal[0]:.2f}, {goal[1]:.2f}) is {distance:.2f}m from '
                    f'blacklisted ({blacklisted[0]:.2f}, {blacklisted[1]:.2f}) < {self.blacklist_radius}m'
                )
                return True
        return False

    def blacklist_goal(self, goal: Tuple[float, float]) -> None:
        """
        Add goal to blacklist to prevent infinite retry loops.
        
        Args:
            goal: (x, y) coordinates to blacklist
        """
        # Check if already blacklisted (avoid duplicates)
        if self.is_goal_blacklisted(goal):
            return
        
        self.blacklisted_goals.append(goal)
        if len(self.blacklisted_goals) > self.max_blacklist_size:
            self.blacklisted_goals.pop(0)  # Remove oldest
        self.get_logger().info(
            f'Blacklisted goal: ({goal[0]:.2f}, {goal[1]:.2f}) '
            f'[total: {len(self.blacklisted_goals)}]'
        )

    def clear_blacklist(self) -> None:
        """Clear the goal blacklist (e.g., when restarting exploration)."""
        count = len(self.blacklisted_goals)
        self.blacklisted_goals.clear()
        self.get_logger().info(f'Cleared {count} blacklisted goals')

    async def handle_no_reachable_frontiers(self) -> None:
        """
        Return to start position when no frontiers are reachable.
        
        Requirements: 6.1, 6.2
        """
        self.get_logger().warn('No reachable frontiers found. Returning to start position.')
        
        if self.start_position:
            robot_pos = (
                self.latest_odom.pose.pose.position.x, 
                self.latest_odom.pose.pose.position.y
            )
            
            # Check if path to start is valid (Requirement 6.1)
            if await self.is_path_free_nav2(robot_pos, self.start_position):
                self.get_logger().info(
                    f'Returning to start: '
                    f'({self.start_position[0]:.2f}, {self.start_position[1]:.2f})'
                )
                self.send_goal(self.start_position[0], self.start_position[1])
            else:
                # Requirement 6.2: Log warning if path blocked
                self.get_logger().warn(
                    'No path to start position. Waiting for map updates.'
                )
                self.get_logger().info(
                    f'Start position: '
                    f'({self.start_position[0]:.2f}, {self.start_position[1]:.2f})'
                )
        else:
            self.get_logger().error('Start position unknown. Unable to return.')


# =============================================================================
# Task 2.9: Main function
# Requirements: 10.2, 10.3
# =============================================================================

def main(args=None):
    """
    Main entry point for exploration node.
    
    Requirements: 10.2, 10.3
    - Create MultiThreadedExecutor
    - Run executor in separate thread
    - Run asyncio event loop in main thread
    - Handle KeyboardInterrupt for clean shutdown
    """
    rclpy.init(args=args)
    node = ExplorationNode()
    
    # Create MultiThreadedExecutor (Requirement 10.2)
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # Run the executor in a separate thread
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    # Run the asyncio event loop in the main thread (Requirement 10.3)
    try:
        node.loop.run_forever()
    except KeyboardInterrupt:
        pass
    finally:
        node.loop.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        executor_thread.join()


if __name__ == '__main__':
    main()
