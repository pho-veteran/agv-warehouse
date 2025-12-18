#!/usr/bin/env python3
"""
Launch Tugbot in Large Warehouse World.
Follows the same pattern as TurtleBot3 launch files.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    pkg_share = get_package_share_directory('turtlebot3_large_warehouse_sim')
    tb3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')
    
    # Asset directories
    worlds_dir = os.path.join(pkg_share, 'warehouse_assets', 'worlds')
    models_dir = os.path.join(pkg_share, 'warehouse_assets', 'models')
    tb3_models_dir = os.path.join(tb3_gazebo_share, 'models')
    workspace_models_dir = '/ros2_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models'

    # Launch configurations
    world_name = LaunchConfiguration('world', default='tugbot_warehouse.world')
    gui = LaunchConfiguration('gui', default='true')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.05')

    # Declare launch arguments
    declare_world = DeclareLaunchArgument('world', default_value='tugbot_warehouse.world')
    declare_gui = DeclareLaunchArgument('gui', default_value='true')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_x = DeclareLaunchArgument('x_pose', default_value='0.0')
    declare_y = DeclareLaunchArgument('y_pose', default_value='0.0')
    declare_z = DeclareLaunchArgument('z_pose', default_value='0.05')

    # World file path
    world_path = PathJoinSubstitution([TextSubstitution(text=worlds_dir), world_name])

    # Model paths for Gazebo
    model_roots = f"{models_dir}:{tb3_models_dir}:{workspace_models_dir}"
    gazebo_env = {
        'GAZEBO_MODEL_PATH': model_roots,
        'GZ_SIM_RESOURCE_PATH': model_roots,
        'IGN_GAZEBO_RESOURCE_PATH': model_roots,
    }

    # Gazebo server
    gzserver = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', '-v', '4', world_path],
        additional_env=gazebo_env, output='screen')

    # Gazebo client (GUI)
    gzclient = ExecuteProcess(
        cmd=['gz', 'sim', '-g'],
        additional_env=gazebo_env,
        condition=IfCondition(gui), output='screen')

    # Spawn Tugbot model (same as TurtleBot3 spawn)
    spawn_tugbot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'tugbot',
            '-file', os.path.join(workspace_models_dir, 'Tugbot', 'model.sdf'),
            '-x', x_pose, '-y', y_pose, '-z', z_pose],
        output='screen')

    # ROS-Gazebo bridge (same topics as TurtleBot3)
    # Use @ for bidirectional, [ for GZ->ROS, ] for ROS->GZ
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    # Static TF: base_footprint -> base_link (same as TurtleBot3 base_joint)
    static_tf_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--frame-id', 'base_footprint', '--child-frame-id', 'base_link',
                   '--x', '0', '--y', '0', '--z', '0'],
        parameters=[{'use_sim_time': use_sim_time}])

    # Static TF: base_link -> base_scan (lidar position)
    static_tf_scan = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--frame-id', 'base_link', '--child-frame-id', 'base_scan',
                   '--x', '0.221', '--y', '0', '--z', '0.1404'],
        parameters=[{'use_sim_time': use_sim_time}])

    # Static TF: base_link -> imu_link
    static_tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--frame-id', 'base_link', '--child-frame-id', 'imu_link',
                   '--x', '0.14', '--y', '0.02', '--z', '0.25'],
        parameters=[{'use_sim_time': use_sim_time}])

    # Build launch description
    ld = LaunchDescription()
    ld.add_action(declare_world)
    ld.add_action(declare_gui)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_x)
    ld.add_action(declare_y)
    ld.add_action(declare_z)
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(spawn_tugbot)
    ld.add_action(bridge)
    ld.add_action(static_tf_base)
    ld.add_action(static_tf_scan)
    ld.add_action(static_tf_imu)

    return ld
