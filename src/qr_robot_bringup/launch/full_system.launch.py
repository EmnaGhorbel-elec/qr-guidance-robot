"""Whole robot in Gazebo: same nodes as on the real robot, robot_sim instead of robot_hardware.

  ros2 launch qr_robot_bringup full_system.launch.py [gui:=true] [autostart:=true] [rviz:=false]
                                                      [scan:=true]

Real robot (robot_hardware/robot_hw.launch.py)   |  here
  usb_cam + qr_detector_node                     |  Gazebo camera + qr_detector_node
  arduino_bridge_node (Mega: motors, servo, IMU) |  Gazebo DiffDrive + sim_base_node
  scan_controller_node, motor_node, command_node,|  the same nodes, same parameters
  localization_node, mission_node                |
Every node runs on the simulation clock (/clock from Gazebo).
"""
import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue

SIM = {'use_sim_time': True}
QR_K = 160.6          # f_px x QR side (see qr_navigation/config/mission_params.yaml)


def generate_launch_description():
    mission_params = os.path.join(share('qr_navigation'), 'config', 'mission_params.yaml')
    rviz_cfg = os.path.join(share('robot_description'), 'rviz', 'config.rviz')
    gazebo = os.path.join(share('robot_description'), 'launch', 'gazebo.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo window'),
        DeclareLaunchArgument('autostart', default_value='true',
                              description='false = wait for /mission/start before moving'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('scan', default_value='true',
                              description='camera pan servo: sweep, then follow the QR'),
        SetParameter(name='use_sim_time', value=True),

        # Gazebo world (3 QR panels) + robot + robot_state_publisher + ros_gz_bridge
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gazebo),
                                 launch_arguments={'gui': LaunchConfiguration('gui')}.items()),

        # "Mega" of the simulation: /odom (wheel speed + gyro), /robot/ready, servo protocol
        Node(package='robot_sim', executable='sim_base_node', output='screen', parameters=[SIM]),

        # camera -> QR id, centre, apparent size, capture stamp
        Node(package='qr_guidance_robot', executable='qr_detector_node', output='screen',
             parameters=[SIM, {'image_topic': '/camera/image'}]),

        # camera servo: sweep until the target QR is seen, then keep it centred
        Node(package='qr_guidance_robot', executable='scan_controller_node', output='screen',
             parameters=[SIM, {'autostart': True}],
             condition=IfCondition(LaunchConfiguration('scan'))),

        # /goal -> /cmd_vel_raw -> /cmd_vel (odom feedback, limits, ramps, watchdog)
        Node(package='robot_control', executable='motor_node', name='motor_node',
             output='screen', parameters=[SIM]),
        Node(package='robot_control', executable='command_node', name='command_node',
             output='screen', parameters=[SIM]),

        # QR id -> position fix, only for QRs seen close enough (w_px >= k / 1.0 m)
        Node(package='qr_navigation', executable='localization_node', output='screen',
             parameters=[SIM, {'qr_topic': '/qr/detections', 'publish_tf': False,
                               'min_width_px': QR_K / 1.0,
                               'qr_range_const': QR_K,
                               'camera_offset': 0.15}]),

        # mission: visit the QR checkpoints, correct the odometry drift at each
        Node(package='qr_navigation', executable='mission_node', output='screen',
             parameters=[mission_params, SIM,
                         {'auto_start': ParameterValue(LaunchConfiguration('autostart'),
                                                       value_type=bool)}]),

        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_cfg], parameters=[SIM],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
