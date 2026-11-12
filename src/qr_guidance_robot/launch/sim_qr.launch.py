"""Camera-only test: Gazebo world + robot + QR detector (no navigation, the robot stays still).

  ros2 launch qr_guidance_robot sim_qr.launch.py
  ros2 topic echo /qr/detections        # WAYPOINT_1 straight ahead at ~1.35 m
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    robot_pkg = get_package_share_directory('robot_description')
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(robot_pkg, 'launch', 'gazebo.launch.py')))
    qr_detector = Node(
        package='qr_guidance_robot', executable='qr_detector_node', output='screen',
        parameters=[{'image_topic': '/camera/image', 'use_sim_time': True}])
    return LaunchDescription([sim, qr_detector])
