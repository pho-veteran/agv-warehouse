#!/usr/bin/env python3
"""
Nav2 Navigation Stack Launch for Exploration with Cartographer SLAM.

This launch file starts Nav2 components needed for autonomous exploration:
- Controller Server (local planner)
- Planner Server (global planner)
- Behavior Server (recovery behaviors)
- BT Navigator (behavior tree navigator)
- Waypoint Follower

Note: AMCL and Map Server are NOT launched because:
- Cartographer provides localization via TF (map -> odom)
- Cartographer publishes /map topic

Based on: Autonomous-Turtlebot/launch/exploration.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get package directories
    pkg_share = get_package_share_directory('agv_auto_explore')
    
    # Config file
    params_file = os.path.join(pkg_share, 'config', 'nav2_exploration_params.yaml')
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file_arg = LaunchConfiguration('params_file')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start Nav2 lifecycle nodes'
    )
    
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='Full path to Nav2 params file'
    )
    
    # Create parameter substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart,
    }
    
    configured_params = RewrittenYaml(
        source_file=params_file_arg,
        param_rewrites=param_substitutions,
        convert_types=True
    )
    
    # Lifecycle manager for Nav2 nodes
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
            ]}
        ]
    )
    
    # Controller Server (local planner - DWB)
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[configured_params],
        remappings=[
            ('cmd_vel', 'cmd_vel'),
        ]
    )
    
    # Planner Server (global planner - NavFn)
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_params],
    )
    
    # Behavior Server (recovery behaviors)
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[configured_params],
    )
    
    # BT Navigator (behavior tree navigator)
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_params],
    )
    
    # Waypoint Follower
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[configured_params],
    )
    
    # Build launch description
    ld = LaunchDescription()
    
    # Declare arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_autostart)
    ld.add_action(declare_params_file)
    
    # Launch Nav2 nodes
    ld.add_action(controller_server)
    ld.add_action(planner_server)
    ld.add_action(behavior_server)
    ld.add_action(bt_navigator)
    ld.add_action(waypoint_follower)
    
    # Launch lifecycle manager last
    ld.add_action(lifecycle_manager)
    
    return ld
