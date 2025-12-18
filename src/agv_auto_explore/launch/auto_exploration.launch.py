import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('agv_auto_explore')
    
    # Path to the config file
    config_file = os.path.join(pkg_share, 'config', 'exploration_params.yaml')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Launch the exploration node
    exploration_node = Node(
        package='agv_auto_explore',
        executable='exploration_node',
        name='exploration_node',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('map', '/map'),
            ('odom', '/odom'),
        ]
    )
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(use_sim_time_arg)
    
    # Add nodes
    ld.add_action(exploration_node)
    
    return ld
