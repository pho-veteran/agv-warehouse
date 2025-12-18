-- TurtleBot3 Cartographer SLAM - Industrial AGV Scale Simulation
-- 
-- v3.0 - Scaled for large warehouse (35x50m) with industrial AGV behavior
--
-- SCALE FACTOR: 3x (TurtleBot3 → Industrial AGV simulation)
-- - Robot speed: 0.26 → 0.78 m/s
-- - Effective lidar range: 3.5 → 10.5m (scaled perception)
-- - Motion filter adjusted for faster movement
--
-- Warehouse: tugbot_warehouse.world
-- - Active area: ~35m x 50m
-- - Obstacles: 16 shelves, 5 pallet boxes, 1 cart, 1 walking actor
-- - Aisle width: ~3m
--
-- Based on:
-- - https://google-cartographer-ros.readthedocs.io/en/latest/algo_walkthrough.html
-- - https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/
-- - TurtleBot3 Waffle Pi LDS-02 specs (max range ~12m, 360° scan)
--
-- Original: /opt/ros/jazzy/share/turtlebot3_cartographer/config/turtlebot3_lds_2d.lua

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4

-- =============================================================================
-- TRAJECTORY_BUILDER_2D - Local SLAM
-- =============================================================================

-- Sensor range settings
-- LDS-02 on Waffle Pi: max ~12m, but 8m is more reliable for indoor
TRAJECTORY_BUILDER_2D.min_range = 0.12
TRAJECTORY_BUILDER_2D.max_range = 8.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0

-- IMU: Disable for Gazebo simulation (not reliable in sim)
-- Enable for real robot: set to true
TRAJECTORY_BUILDER_2D.use_imu_data = false

-- Online correlative scan matching: Essential for good local SLAM
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- Correlative scan matcher - SCALED for faster robot
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.3  -- SCALED 2x: larger search for faster movement
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(30.)  -- SCALED: wider angular search
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- Submap parameters
-- Larger submaps = fewer loop closures but more stable
-- Smaller submaps = more loop closures but can drift
-- For large warehouse (>30m): use larger submaps for stability
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 120  -- Increased for large warehouse
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.10  -- 10cm - Larger cells for warehouse scale

-- Motion filter: When to insert new scans
-- Best practice: More frequent scans = less drift accumulation
-- Reduced thresholds to minimize drift in large environments
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 3.    -- Keep: reasonable update rate
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.2 -- Reduced: 0.3 → 0.2 (more scans = less drift)
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.5) -- Reduced: 1.0° → 0.5° (better rotation tracking)

-- Ceres scan matcher weights
-- Best practice: Higher translation_weight reduces accumulated drift
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 20.  -- Increased: 10 → 20 (reduce translation drift)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.

-- Voxel filter for point cloud reduction (performance vs accuracy)
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05  -- Match resolution for warehouse

-- Adaptive voxel filter
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.5
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 200
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 50.

-- =============================================================================
-- POSE_GRAPH - Global SLAM (Loop Closure)
-- =============================================================================

-- Optimize after N nodes (set 0 to disable global SLAM for debugging)
-- Best practice: Balance between performance and drift correction
-- More frequent optimization = earlier drift correction, less "map jumps"
POSE_GRAPH.optimize_every_n_nodes = 90  -- Reduced: 150 → 90 (correct drift earlier, prevent large map shifts)

-- Constraint builder (loop closure detection)
-- Best practice: Higher scores = fewer false positives = more stable map
-- Critical for warehouse with repetitive structures (shelves)
POSE_GRAPH.constraint_builder.min_score = 0.65  -- Increased: 0.60 → 0.65 (stricter matching, prevent wrong loop closures)
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.70  -- Increased: 0.65 → 0.70 (very strict for global matches)

-- How often to search for constraints
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3

-- Loop closure weights
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.1e4
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e5

-- Fast correlative scan matcher for loop closure
-- Best practice: Narrower search window = fewer false matches in repetitive environments
-- Warehouse has many similar shelves → need stricter matching
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 5.0  -- Reduced: 7.0 → 5.0 (prevent false matches between similar shelves)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7

-- Ceres scan matcher for constraint refinement
POSE_GRAPH.constraint_builder.ceres_scan_matcher.occupied_space_weight = 20.
POSE_GRAPH.constraint_builder.ceres_scan_matcher.translation_weight = 10.
POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 1.

-- Optimization problem tuning
-- Best practice: Higher rotation_weight reduces rotation drift (map skew)
POSE_GRAPH.optimization_problem.huber_scale = 1e1
POSE_GRAPH.optimization_problem.acceleration_weight = 1e3
POSE_GRAPH.optimization_problem.rotation_weight = 5e5  -- Increased: 3e5 → 5e5 (reduce rotation drift, prevent map skew)

-- Global constraint search
POSE_GRAPH.global_sampling_ratio = 0.003
POSE_GRAPH.global_constraint_search_after_n_seconds = 10.

-- Maximum number of constraints (memory management)
POSE_GRAPH.max_num_final_iterations = 200

return options

