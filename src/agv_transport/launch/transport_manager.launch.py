"""Launch file for AGV Transport Task Manager.

This launch file starts the transport_task_manager_node with configurable
parameters for station configuration and transport behavior.

**Feature: agv-transport-task-manager**
**Validates: Requirements 1.1**
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for transport task manager."""
    
    # Get the package share directory
    pkg_share = get_package_share_directory('agv_transport')
    
    # Default path to the station config file
    default_config_path = os.path.join(pkg_share, 'config', 'warehouse_stations.yaml')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value=default_config_path,
        description='Path to warehouse_stations.yaml configuration file'
    )
    
    status_publish_rate_arg = DeclareLaunchArgument(
        'status_publish_rate',
        default_value='1.0',
        description='Rate (Hz) at which to publish transport status'
    )
    
    loading_timeout_arg = DeclareLaunchArgument(
        'loading_timeout',
        default_value='5.0',
        description='Timeout (seconds) for loading at pickup station'
    )
    
    unloading_timeout_arg = DeclareLaunchArgument(
        'unloading_timeout',
        default_value='5.0',
        description='Timeout (seconds) for unloading at dropoff station'
    )
    
    nav_timeout_arg = DeclareLaunchArgument(
        'nav_timeout',
        default_value='300.0',
        description='Navigation timeout (seconds) before canceling goal'
    )
    
    max_retries_arg = DeclareLaunchArgument(
        'max_retries',
        default_value='3',
        description='Maximum navigation retry attempts before marking order as failed'
    )
    
    max_consecutive_errors_arg = DeclareLaunchArgument(
        'max_consecutive_errors',
        default_value='3',
        description='Maximum consecutive errors before entering ERROR state'
    )
    
    # Launch the transport task manager node
    transport_task_manager_node = Node(
        package='agv_transport',
        executable='transport_task_manager_node',
        name='transport_task_manager',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'config_path': LaunchConfiguration('config_path'),
            'status_publish_rate': LaunchConfiguration('status_publish_rate'),
            'loading_timeout': LaunchConfiguration('loading_timeout'),
            'unloading_timeout': LaunchConfiguration('unloading_timeout'),
            'nav_timeout': LaunchConfiguration('nav_timeout'),
            'max_retries': LaunchConfiguration('max_retries'),
            'max_consecutive_errors': LaunchConfiguration('max_consecutive_errors'),
        }],
    )
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(use_sim_time_arg)
    ld.add_action(config_path_arg)
    ld.add_action(status_publish_rate_arg)
    ld.add_action(loading_timeout_arg)
    ld.add_action(unloading_timeout_arg)
    ld.add_action(nav_timeout_arg)
    ld.add_action(max_retries_arg)
    ld.add_action(max_consecutive_errors_arg)
    
    # Add nodes
    ld.add_action(transport_task_manager_node)
    
    return ld
