#!/usr/bin/env python3
"""
Launch TurtleBot3 in AWS RoboMaker Small Warehouse world (assets originally from turtlebot_warehouse).

Asset source priority:
1) Vendored assets inside this package: share/turtlebot3_warehouse_sim/warehouse_assets/{worlds,models}
2) Workspace checkout: /ros2_ws/src/turtlebot_warehouse/aws_robomaker_small_warehouse_world/{worlds,models}

Notes:
- Some turtlebot_warehouse models include legacy URIs like `file://models/...` and
  inertial blocks that Gazebo Harmonic can reject. We patch the model SDFs at runtime
  (text-only change) to improve compatibility.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution


def generate_launch_description():
    # Keep the same flow as turtlebot3_gazebo world launches:
    # 1) start simulator (gzserver + optional gui)
    # 2) robot_state_publisher
    # 3) spawn turtlebot3 (and bridges handled by turtlebot3_gazebo's spawn launch)
    #
    # Resolve asset locations
    pkg_share = get_package_share_directory('turtlebot3_warehouse_sim')
    vendored_worlds_dir = os.path.join(pkg_share, 'warehouse_assets', 'worlds')
    vendored_models_dir = os.path.join(pkg_share, 'warehouse_assets', 'models')

    ws_src = '/ros2_ws/src'
    workspace_worlds_dir = os.path.join(
        ws_src, 'turtlebot_warehouse', 'aws_robomaker_small_warehouse_world', 'worlds'
    )
    workspace_models_dir = os.path.join(
        ws_src, 'turtlebot_warehouse', 'aws_robomaker_small_warehouse_world', 'models'
    )

    warehouse_worlds_dir = vendored_worlds_dir if os.path.isdir(vendored_worlds_dir) else workspace_worlds_dir
    warehouse_models_dir = vendored_models_dir if os.path.isdir(vendored_models_dir) else workspace_models_dir

    # TurtleBot3 Gazebo launch files
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    tb3_models_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models')

    # Args
    world_name = LaunchConfiguration('world', default='no_roof_small_warehouse.world')
    gui = LaunchConfiguration('gui', default='true')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    declare_world = DeclareLaunchArgument(
        'world',
        default_value='no_roof_small_warehouse.world',
        description='Warehouse world filename from turtlebot_warehouse: no_roof_small_warehouse.world | small_warehouse.world | empty_test.world',
    )
    declare_gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start Gazebo GUI (requires working X11 forwarding)',
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use sim time',
    )
    declare_x = DeclareLaunchArgument('x_pose', default_value='0.0', description='Spawn x')
    declare_y = DeclareLaunchArgument('y_pose', default_value='0.0', description='Spawn y')

    world_path = PathJoinSubstitution([
        TextSubstitution(text=warehouse_worlds_dir),
        world_name,
    ])

    # Resource/model paths (pass to gz processes explicitly)
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    existing_gz_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    existing_ign_resource_path = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')

    # Include BOTH warehouse models and turtlebot3_gazebo models so GUI can resolve:
    #   model://turtlebot3_common/...
    model_roots = f"{warehouse_models_dir}:{tb3_models_dir}"
    gazebo_model_path = f"{model_roots}:{existing_model_path}" if existing_model_path else model_roots
    gz_resource_path = f"{model_roots}:{existing_gz_resource_path}" if existing_gz_resource_path else model_roots
    ign_resource_path = f"{model_roots}:{existing_ign_resource_path}" if existing_ign_resource_path else model_roots

    # Patch model SDFs in-place to improve compatibility with Gazebo Harmonic:
    # - file://models/... -> file:///abs/path/to/models/...
    # - remove <inertial> blocks (most of these models are static anyway)
    patch_models = ExecuteProcess(
        cmd=[
            'bash', '-lc',
            f'''set -e
if [ -d "{warehouse_models_dir}" ]; then
  find "{warehouse_models_dir}" -name "*.sdf" -print0 | while IFS= read -r -d '' f; do
    sed -i "s|file://models/|file://{warehouse_models_dir}/|g" "$f"
    perl -0777 -pi -e 's/<inertial>[\\s\\S]*?<\\/inertial>//sg' "$f"
  done
fi
'''
        ],
        output='screen',
    )

    # Gazebo server / client
    gzserver = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', '-v', '4', world_path],
        additional_env={
            'GAZEBO_MODEL_PATH': gazebo_model_path,
            'GZ_SIM_RESOURCE_PATH': gz_resource_path,
            'IGN_GAZEBO_RESOURCE_PATH': ign_resource_path,
        },
        output='screen',
    )

    gzclient = ExecuteProcess(
        cmd=['gz', 'sim', '-g'],
        additional_env={
            'GAZEBO_MODEL_PATH': gazebo_model_path,
            'GZ_SIM_RESOURCE_PATH': gz_resource_path,
            'IGN_GAZEBO_RESOURCE_PATH': ign_resource_path,
        },
        condition=IfCondition(gui),
        output='screen',
    )

    # Use TurtleBot3's standard bringup pieces (robot_state_publisher + spawn/bridges)
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    spawn_tb3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')),
        launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items(),
    )

    ld = LaunchDescription()
    ld.add_action(declare_world)
    ld.add_action(declare_gui)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_x)
    ld.add_action(declare_y)

    ld.add_action(patch_models)
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_tb3)

    return ld


