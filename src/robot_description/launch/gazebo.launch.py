import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('robot_description')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    urdf_path = os.path.join(pkg_share, 'urdf', 'robot_gazebo.urdf')
    bridge_config = os.path.join(pkg_share, 'config', 'bridge.yaml')
    world_src = os.path.join(pkg_share, 'worlds', 'qr_world.sdf')
    texture_dir = os.path.join(pkg_share, 'worlds', 'materials', 'textures')

    # Gazebo does not reliably resolve relative texture paths, which leaves the
    # panels black. Write a copy of the world with absolute texture paths.
    with open(world_src, 'r') as f:
        world_text = f.read().replace('TEXTURE_DIR', texture_dir)
    world_path = '/tmp/qr_world_resolved.sdf'
    with open(world_path, 'w') as f:
        f.write(world_text)

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    # gui:=true -> "-r" (server + GUI), gui:=false -> "-r -s" (headless)
    gz_args = f'-r {world_path}'
    if LaunchConfiguration('gui').perform(context).lower() == 'false':
        gz_args = f'-r -s {world_path}'

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'robot', '-z', '0.05'],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config, 'use_sim_time': True}],
        output='screen',
    )

    # Lets Gazebo resolve model://robot_description/meshes/... (package:// in the URDF)
    share_parent = os.path.dirname(pkg_share)
    env_actions = []
    for var in ('GZ_SIM_RESOURCE_PATH', 'IGN_GAZEBO_RESOURCE_PATH'):
        existing = os.environ.get(var, '')
        value = share_parent + (os.pathsep + existing if existing else '')
        env_actions.append(SetEnvironmentVariable(var, value))

    return env_actions + [gz_sim, robot_state_publisher, spawn_robot, bridge]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Open the Gazebo GUI (false = headless server only)'),
        OpaqueFunction(function=_launch_setup),
    ])
