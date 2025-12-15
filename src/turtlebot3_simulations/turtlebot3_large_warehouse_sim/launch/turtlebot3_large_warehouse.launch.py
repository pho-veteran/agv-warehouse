#!/usr/bin/env python3
"""
Launch TurtleBot3 in Large Warehouse World (based on MovAi Tugbot Warehouse).

This launch file starts:
1) Gazebo Harmonic simulation with tugbot_warehouse world
2) Robot State Publisher for TurtleBot3
3) Spawns TurtleBot3 Waffle Pi robot
4) Sets up ROS-Gazebo bridges

The world includes:
- Large warehouse structure from Gazebo Fuel (MovAi)
- Charging station
- Multiple shelves and pallet boxes
- Walking actors for dynamic obstacle testing

Usage:
    # Default spawn at center
    ros2 launch turtlebot3_large_warehouse_sim turtlebot3_large_warehouse.launch.py
    
    # Spawn at specific location
    ros2 launch turtlebot3_large_warehouse_sim turtlebot3_large_warehouse.launch.py spawn_location:=charging
    
    # Custom position
    ros2 launch turtlebot3_large_warehouse_sim turtlebot3_large_warehouse.launch.py x_pose:=12.0 y_pose:=-10.0

Spawn locations:
    - center:   (0, 0)    - Center of warehouse
    - charging: (12, -10) - Near charging station
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution

# Predefined spawn locations
SPAWN_LOCATIONS = {
    'center': {'x': '0.0', 'y': '0.0', 'z': '0.1'},
    'charging': {'x': '12.0', 'y': '-10.0', 'z': '0.1'},
}


def get_spawn_coordinates(context):
    """Get spawn coordinates based on spawn_location or custom x/y/z."""
    spawn_location = LaunchConfiguration('spawn_location').perform(context)
    
    if spawn_location in SPAWN_LOCATIONS:
        return SPAWN_LOCATIONS[spawn_location]
    else:
        # Use custom coordinates
        return {
            'x': LaunchConfiguration('x_pose').perform(context),
            'y': LaunchConfiguration('y_pose').perform(context),
            'z': LaunchConfiguration('z_pose').perform(context),
        }


def generate_launch_description():
    # Package directories
    pkg_share = get_package_share_directory('turtlebot3_large_warehouse_sim')
    tb3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')
    
    # Asset directories
    worlds_dir = os.path.join(pkg_share, 'warehouse_assets', 'worlds')
    models_dir = os.path.join(pkg_share, 'warehouse_assets', 'models')
    tb3_models_dir = os.path.join(tb3_gazebo_share, 'models')
    
    # TurtleBot3 launch files directory
    launch_file_dir = os.path.join(tb3_gazebo_share, 'launch')

    # Launch configurations
    world_name = LaunchConfiguration('world', default='tugbot_warehouse.world')
    gui = LaunchConfiguration('gui', default='true')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    spawn_location = LaunchConfiguration('spawn_location', default='center')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.1')

    # Declare launch arguments
    declare_world = DeclareLaunchArgument(
        'world',
        default_value='tugbot_warehouse.world',
        description='World file name (tugbot_warehouse.world)',
    )
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
    declare_spawn_location = DeclareLaunchArgument(
        'spawn_location',
        default_value='center',
        description='Predefined spawn location: center (0,0) | charging (12,-10)',
    )
    declare_x = DeclareLaunchArgument(
        'x_pose', 
        default_value='0.0', 
        description='Robot spawn X position (overrides spawn_location if set)',
    )
    declare_y = DeclareLaunchArgument(
        'y_pose', 
        default_value='0.0', 
        description='Robot spawn Y position (overrides spawn_location if set)',
    )
    declare_z = DeclareLaunchArgument(
        'z_pose',
        default_value='0.1',
        description='Robot spawn Z position',
    )

    # World file path
    world_path = PathJoinSubstitution([
        TextSubstitution(text=worlds_dir),
        world_name,
    ])

    # Setup model paths for Gazebo
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    existing_gz_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    
    # Include both warehouse models and TurtleBot3 models
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
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # Spawn TurtleBot3
    spawn_tb3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'z_pose': z_pose,
        }.items(),
    )

    # Build launch description
    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(declare_world)
    ld.add_action(declare_gui)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_spawn_location)
    ld.add_action(declare_x)
    ld.add_action(declare_y)
    ld.add_action(declare_z)

    # Launch actions
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_tb3)

    return ld

