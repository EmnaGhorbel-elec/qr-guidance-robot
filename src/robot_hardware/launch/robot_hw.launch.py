"""Real robot on the Pi: camera + QR detector + servo + IMU + motors + QR-command autonomy.

  ros2 launch robot_hardware robot_hw.launch.py                    # waits for /mission/start
  ros2 launch robot_hardware robot_hw.launch.py autostart:=true    # starts by itself
  ros2 launch robot_hardware robot_hw.launch.py mode:=teleop       # keyboard driving only
Args: port (/dev/ttyACM0 or auto), camera (true), scan (true), mode (auto|teleop),
      autostart (false)

POWER-ON: boot/start_robot.sh runs this with autostart:=true port:=auto (see boot/README_BOOT.md).
autostart: autonomy_node waits for /robot/ready (Mega connected + gyro calibrated), then
starts `start_delay` s later. The core nodes respawn if they crash; autonomy_node does not
(a restart would begin the run again from wherever the robot stands).

auto  : camera -> qr_detector -> scan_controller (servo sweep, lock, bearing) -> autonomy_node
        -> /cmd_vel_raw -> motor_node -> /cmd_vel -> arduino_bridge_node -> Mega
        No table, no map: the robot only follows the commands written in the QR codes.
teleop: no autonomy_node, so the keyboard (/cmd_vel_raw) is the only writer.
        (autonomy_node publishes zeros on /cmd_vel_raw all the time and would fight the keys)
"""
import os

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    camera_launch = os.path.join(share('qr_guidance_robot'), 'launch', 'usb_cam_qr.launch.py')
    cfg = os.path.join(share('qr_autonomy'), 'config', 'autonomy_hw.yaml')
    auto = IfCondition(PythonExpression(["'", LaunchConfiguration('mode'), "' == 'auto'"]))

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('scan', default_value='true',
                              description='false = no scan_controller (teleop tests)'),
        DeclareLaunchArgument('mode', default_value='auto', description='auto | teleop'),
        DeclareLaunchArgument('autostart', default_value='false',
                              description='true = the run starts by itself, start_delay s after /robot/ready'),

        # usb_cam + qr_detector_node
        IncludeLaunchDescription(PythonLaunchDescriptionSource(camera_launch),
                                 condition=IfCondition(LaunchConfiguration('camera'))),

        # Mega: motors, servo, MPU-6050
        Node(package='robot_hardware', executable='arduino_bridge_node', output='screen',
             parameters=[{'port': LaunchConfiguration('port')}],
             respawn=True, respawn_delay=2.0),

        # servo: sweep 90 deg right / 90 deg left, lock on a command QR, keep it centred
        Node(package='robot_hardware', executable='scan_controller_node', output='screen',
             parameters=[cfg],
             condition=IfCondition(LaunchConfiguration('scan')),
             respawn=True, respawn_delay=2.0),

        # limits + ramps + watchdog
        Node(package='robot_control', executable='motor_node', name='motor_node',
             output='screen', respawn=True, respawn_delay=2.0),

        # the whole decision logic: sweep, align, approach, read, execute
        Node(package='qr_autonomy', executable='autonomy_node', output='screen',
             parameters=[cfg, {'autostart': ParameterValue(LaunchConfiguration('autostart'),
                                                           value_type=bool)}],
             condition=auto),
    ])
