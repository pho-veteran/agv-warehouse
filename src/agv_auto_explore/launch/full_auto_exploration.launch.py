#!/usr/bin/env python3
"""
Combined launch file for AGV Auto-Exploration in Large Warehouse.

This launch file starts:
1) Gazebo Harmonic simulation with tugbot_warehouse world
2) Robot State Publisher for TurtleBot3
3) Spawns TurtleBot3 Waffle Pi robot
4) Cartographer SLAM
5) Nav2 Navigation Stack (using nav2_bringup directly like Autonomous-Turtlebot)
6) Autonomous exploration node

Usage:
    # Default spawn at center with tuned SLAM config
    ros2 launch agv_auto_explore full_auto_exploration.launch.py
    
    # Spawn at specific location
    ros2 launch agv_auto_explore full_auto_exploration.launch.py x_pose:=12.0 y_pose:=-10.0
    
    # Without GUI (headless)
    ros2 launch agv_auto_explore full_auto_exploration.launch.py gui:=false
    
    # Use default TurtleBot3 SLAM config
    ros2 launch agv_auto_explore full_auto_exploration.launch.py use_tuned_slam:=false

Requirements: 7.1, 7.2, 7.3
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    pkg_share = get_package_share_directory('agv_auto_explore')
    tb3_large_warehouse_share = get_package_share_directory('turtlebot3_large_warehouse_sim')
    tb3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')
    tb3_cartographer_share = get_package_share_directory('turtlebot3_cartographer')
    
    # Asset directories
    worlds_dir = os.path.join(tb3_large_warehouse_share, 'warehouse_assets', 'worlds')
    models_dir = os.path.join(tb3_large_warehouse_share, 'warehouse_assets', 'models')
    tb3_models_dir = os.path.join(tb3_gazebo_share, 'models')
    
    # TurtleBot3 launch files directory
    tb3_launch_dir = os.path.join(tb3_gazebo_share, 'launch')
    
    # Config files
    exploration_config = os.path.join(pkg_share, 'config', 'exploration_params.yaml')
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_exploration_params.yaml')
    
    # SLAM config directories
    tuned_slam_config_dir = '/ros2_ws/config'  # Tuned config in workspace
    default_slam_config_dir = os.path.join(tb3_cartographer_share, 'config')

    # Launch configurations
    gui = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    use_tuned_slam = LaunchConfiguration('use_tuned_slam')
    use_rviz = LaunchConfiguration('use_rviz')

    # Declare launch arguments
    declare_gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start Gazebo GUI (requires X11 forwarding)',
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time',
    )
    declare_x = DeclareLaunchArgument(
        'x_pose', 
        default_value='0.0', 
        description='Robot spawn X position',
    )
    declare_y = DeclareLaunchArgument(
        'y_pose', 
        default_value='0.0', 
        description='Robot spawn Y position',
    )
    declare_z = DeclareLaunchArgument(
        'z_pose',
        default_value='0.1',
        description='Robot spawn Z position',
    )
    declare_use_tuned_slam = DeclareLaunchArgument(
        'use_tuned_slam',
        default_value='true',
        description='Use tuned SLAM config for large warehouse (true) or default TurtleBot3 config (false)',
    )
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization',
    )

    # World file path
    world_path = os.path.join(worlds_dir, 'tugbot_warehouse.world')

    # Setup model paths for Gazebo
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    existing_gz_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    
    model_roots = f"{models_dir}:{tb3_models_dir}"
    gazebo_model_path = f"{model_roots}:{existing_model_path}" if existing_model_path else model_roots
    gz_resource_path = f"{model_roots}:{existing_gz_resource_path}" if existing_gz_resource_path else model_roots

    # Environment variables for Gazebo
    gazebo_env = {
        'GAZEBO_MODEL_PATH': gazebo_model_path,
        'GZ_SIM_RESOURCE_PATH': gz_resource_path,
        'IGN_GAZEBO_RESOURCE_PATH': gz_resource_path,
    }

    # Gazebo server
    gzserver = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', '-v', '4', world_path],
        additional_env=gazebo_env,
        output='screen',
    )

    # Gazebo client (GUI)
    gzclient = ExecuteProcess(
        cmd=['gz', 'sim', '-g'],
        additional_env=gazebo_env,
        condition=IfCondition(gui),
        output='screen',
    )

    # Robot State Publisher (TurtleBot3)
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_launch_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # Spawn TurtleBot3
    spawn_tb3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_launch_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'z_pose': z_pose,
        }.items(),
    )


    # Cartographer SLAM node (tuned config)
    cartographer_node_tuned = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', tuned_slam_config_dir,
            '-configuration_basename', 'turtlebot3_slam_tuned.lua'
        ],
        condition=IfCondition(use_tuned_slam),
    )

    # Cartographer SLAM node (default config)
    cartographer_node_default = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', default_slam_config_dir,
            '-configuration_basename', 'turtlebot3_lds_2d.lua'
        ],
        condition=UnlessCondition(use_tuned_slam),
    )

    # Occupancy grid node for Cartographer
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'resolution': 0.05},
            {'publish_period_sec': 1.0},
        ],
    )

    # Nav2 Navigation Stack - custom launch without AMCL/map_server
    # (Cartographer provides localization and /map topic)
    nav2_exploration = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'nav2_exploration.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true',
            'params_file': nav2_params_file,
        }.items(),
    )

    # RViz for visualization
    rviz_config_dir = os.path.join(tb3_cartographer_share, 'rviz', 'tb3_cartographer.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    # Exploration node - delayed start to allow SLAM and Nav2 to initialize
    # 5 second delay as per requirements
    exploration_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='agv_auto_explore',
                executable='exploration_node',
                name='exploration_node',
                output='screen',
                parameters=[
                    exploration_config,
                    {'use_sim_time': use_sim_time}
                ],
                remappings=[
                    ('map', '/map'),
                    ('odom', '/odom'),
                ],
            )
        ],
    )

    # Build launch description
    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(declare_gui)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_x)
    ld.add_action(declare_y)
    ld.add_action(declare_z)
    ld.add_action(declare_use_tuned_slam)
    ld.add_action(declare_use_rviz)

    # Launch Gazebo
    ld.add_action(gzserver)
    ld.add_action(gzclient)

    # Launch robot
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_tb3)

    # Launch Cartographer SLAM
    ld.add_action(cartographer_node_tuned)
    ld.add_action(cartographer_node_default)
    ld.add_action(occupancy_grid_node)

    # Launch Nav2 Navigation Stack
    ld.add_action(nav2_exploration)

    # Launch RViz
    ld.add_action(rviz_node)

    # Launch exploration node (with delay)
    ld.add_action(exploration_node)

    return ld
