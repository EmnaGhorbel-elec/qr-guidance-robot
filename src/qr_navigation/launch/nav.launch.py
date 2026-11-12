import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('qr_navigation')
    return LaunchDescription([
        Node(package='qr_navigation', executable='localization_node',
             name='localization_node', output='screen'),
        Node(package='qr_navigation', executable='guidance_node',
             name='guidance_node', output='screen',
             parameters=[os.path.join(share, 'config', 'guidance_params.yaml')]),
    ])
