#!/usr/bin/env python3
"""sim_base_node: what arduino_bridge_node is on the real robot, for Gazebo.

Same ROS interface as robot_hardware/arduino_bridge_node, so every other node runs unchanged:

  /odom (Odometry) + TF odom->base_link
        position = wheel speed integrated, heading = gyro (like the robot: speed + MPU-6050)
  /robot/ready (Bool, 2 Hz)   gyro calibrated and sensors streaming. mission_node waits for it.
  /servo/command (String) --> 'S' sweep | 'H' halt | 'G<angle>'   (the Mega's servo protocol)
  /servo/angle (Int32)    <-- every new servo angle (the Mega's 'A:<angle>')
  /camera_pan_cmd (Float64, rad) -> Gazebo pan joint (via ros_gz_bridge)

Inputs from Gazebo (ros_gz_bridge): /imu/data (gyro), /wheel_odom (DiffDrive, only its speed).
All integration uses the sensor stamps (simulation time), so a slow simulation stays correct.

Parameters:
  linear_source: 'wheels' (DiffDrive speed, default) or 'command' (/cmd_vel speed, which is
                 what the real robot does since it has no encoders)
  pan_dir: +1.0: a higher servo angle turns the camera to the left (as the real robot)
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float64, Int32, String
from tf2_ros import TransformBroadcaster

from robot_sim.base_logic import GyroHeading, ServoSim


def _yaw_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class SimBaseNode(Node):
    def __init__(self):
        super().__init__('sim_base_node')
        self.declare_parameter('linear_source', 'wheels')
        self.declare_parameter('odom_rate_hz', 25.0)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('pan_dir', 1.0)
        self.declare_parameter('pan_center', 90)
        self.declare_parameter('max_speed', 0.35)       # clip, as the real bridge does

        self.gyro = GyroHeading()
        self.servo = ServoSim(self.get_parameter('pan_center').value)
        self.x = self.y = 0.0
        self.v_wheels = 0.0
        self.v_cmd = 0.0
        self.t_last_odom = None
        self.yaw_last_odom = 0.0
        self.t_imu = None
        self.t_wheels = None
        self.was_ready = False

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.ready_pub = self.create_publisher(Bool, 'robot/ready', 10)
        self.angle_pub = self.create_publisher(Int32, 'servo/angle', 10)
        self.pan_pub = self.create_publisher(Float64, 'camera_pan_cmd', 10)
        self.tf = TransformBroadcaster(self)

        self.create_subscription(Imu, 'imu/data', self.on_imu, 50)
        self.create_subscription(Odometry, 'wheel_odom', self.on_wheels, 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        self.create_subscription(String, 'servo/command', self.on_servo_cmd, 10)
        self.create_timer(0.05, self.servo_tick)
        self.create_timer(0.5, self.publish_ready)
        self.create_timer(1.0, self.publish_pan)          # re-send: Gazebo may start late
        self.get_logger().info('sim_base_node: keep the robot still, measuring the gyro bias')

    # ------------------------------------------------------------------ sensors
    def on_wheels(self, msg):
        self.v_wheels = msg.twist.twist.linear.x
        self.t_wheels = self._now_s()

    def on_cmd(self, msg):
        self.v_cmd = msg.linear.x

    def on_imu(self, msg):
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        if t <= 0.0:
            t = self._now_s()
        if self.t_imu is not None and t < self.t_imu:      # simulation reset
            self.gyro = GyroHeading()
            self.x = self.y = 0.0
            self.t_last_odom = None
        self.t_imu = t
        was = self.gyro.ready
        self.gyro.feed(t, msg.angular_velocity.z)
        if self.gyro.ready and not was:
            self.get_logger().info(f'gyro calibrated (bias {self.gyro.bias:+.4f} rad/s)')
        if not self.gyro.ready:
            return
        if self.t_last_odom is None:
            self.t_last_odom, self.yaw_last_odom = t, self.gyro.yaw
            return
        dt = t - self.t_last_odom
        if dt < 1.0 / self.get_parameter('odom_rate_hz').value:
            return
        yaw_prev, self.yaw_last_odom = self.yaw_last_odom, self.gyro.yaw
        self.t_last_odom = t
        if dt > 0.5:
            return                                      # pause / hiccup: do not jump
        v = self.v_cmd if self.get_parameter('linear_source').value == 'command' else self.v_wheels
        vmax = self.get_parameter('max_speed').value
        v = max(-vmax, min(vmax, v))
        mid = yaw_prev + 0.5 * math.atan2(math.sin(self.gyro.yaw - yaw_prev),
                                          math.cos(self.gyro.yaw - yaw_prev))
        self.x += v * math.cos(mid) * dt
        self.y += v * math.sin(mid) * dt
        self.publish_odom(msg.header.stamp, v)

    def publish_odom(self, stamp, v):
        qx, qy, qz, qw = _yaw_quat(self.gyro.yaw)
        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = 'odom'
        o.child_frame_id = 'base_link'
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        o.pose.pose.orientation.x, o.pose.pose.orientation.y = qx, qy
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = qz, qw
        o.twist.twist.linear.x = v
        o.twist.twist.angular.z = self.gyro.rate
        self.odom_pub.publish(o)
        if self.get_parameter('publish_tf').value:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x, t.transform.translation.y = self.x, self.y
            t.transform.rotation.x, t.transform.rotation.y = qx, qy
            t.transform.rotation.z, t.transform.rotation.w = qz, qw
            self.tf.sendTransform(t)

    def publish_ready(self):
        now = self._now_s()
        fresh = self.t_imu is not None and abs(now - self.t_imu) < 1.0
        wheels = self.t_wheels is not None and abs(now - self.t_wheels) < 2.0
        ready = self.gyro.ready and fresh and wheels
        if ready != self.was_ready:
            self.get_logger().info('robot ready' if ready else 'robot NOT ready (sensors stopped?)')
            self.was_ready = ready
        self.ready_pub.publish(Bool(data=ready))

    # ------------------------------------------------------------------ camera servo
    def on_servo_cmd(self, msg):
        self._servo_moved(self.servo.command(msg.data, self._now_s()))

    def servo_tick(self):
        self._servo_moved(self.servo.update(self._now_s()))

    def _servo_moved(self, angle):
        if angle is None:
            return
        self.angle_pub.publish(Int32(data=int(angle)))
        self.publish_pan()

    def publish_pan(self):
        gp = self.get_parameter
        rad = math.radians(gp('pan_dir').value * (self.servo.angle - gp('pan_center').value))
        self.pan_pub.publish(Float64(data=rad))

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = SimBaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
