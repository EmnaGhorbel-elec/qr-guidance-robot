import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_description')
    # same model as in Gazebo (camera at +x on the pan joint)
    urdf_path = os.path.join(pkg_share, 'urdf', 'robot_gazebo.urdf')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # Publishes fake joint states so the wheels have a TF even with no real encoders yet
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
    )

    rviz_config = os.path.join(pkg_share, 'rviz', 'config.rviz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
    ])
