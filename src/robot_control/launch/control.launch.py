from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='robot_control', executable='motor_node',
             name='motor_node', output='screen'),
        Node(package='robot_control', executable='command_node',
             name='command_node', output='screen'),
    ])
