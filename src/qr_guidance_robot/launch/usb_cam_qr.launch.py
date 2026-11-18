import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('qr_guidance_robot')
    params_file = os.path.join(pkg_share, 'config', 'usb_cam_params.yaml')

    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam_node_exe',
        parameters=[params_file],
        output='screen',
    )

    qr_detector_node = Node(
        package='qr_guidance_robot',
        executable='qr_detector_node',
        parameters=[{'image_topic': '/image_raw'}],
        output='screen',
    )

    return LaunchDescription([
        usb_cam_node,
        qr_detector_node,
    ])
