#!/usr/bin/env python3
"""autonomy_node: ROS shell around qr_autonomy.navigator.Navigator.

  /odom, /scan/lock, /scan/state, /robot/ready  -->  Navigator  -->  /cmd_vel_raw (Twist)
                                                                 -->  /scan/command, /scan/target,
                                                                      /scan/ignore, /autonomy/status

This node is the ONLY writer of /cmd_vel_raw (motor_node adds limits, ramps and watchdog, then
the bridge sends PWM to the Mega). Do not run it together with the keyboard teleop.

  ros2 topic pub --once /mission/start std_msgs/msg/Empty "{}"   # start now (autostart:=false)
  ros2 topic pub --once /mission/stop  std_msgs/msg/Empty "{}"   # stop the wheels, park the servo
"""
import json
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, String

from qr_autonomy.navigator import Navigator, Params


class AutonomyNode(Node):
    def __init__(self):
        super().__init__('autonomy_node')
        defaults = Params()
        for name, value in vars(defaults).items():
            self.declare_parameter(name, value)
        self.declare_parameter('rate_hz', 20.0)
        params = Params(**{name: self.get_parameter(name).value for name in vars(defaults)})
        self.nav = Navigator(params, log=self._log)

        self.pose = None
        self.last_target = None
        self.last_ignore = None
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_raw', 10)
        self.scan_cmd_pub = self.create_publisher(String, 'scan/command', 10)
        self.target_pub = self.create_publisher(String, 'scan/target', 10)
        self.ignore_pub = self.create_publisher(String, 'scan/ignore', 10)
        self.status_pub = self.create_publisher(String, 'autonomy/status', 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_subscription(String, 'scan/lock', self.on_lock, 10)
        self.create_subscription(String, 'scan/state', self.on_scan_state, 10)
        self.create_subscription(Bool, 'robot/ready', self.on_ready, 10)
        self.create_subscription(Empty, 'mission/start', lambda _m: self.nav.request_start(), 10)
        self.create_subscription(Empty, 'mission/stop', lambda _m: self.nav.request_stop(), 10)
        self.create_timer(1.0 / self.get_parameter('rate_hz').value, self.tick)
        self.create_timer(1.0, self.publish_slow)
        self.get_logger().info('autonomy_node ready: no table, no map, the QR commands drive the robot')

    def _log(self, level, msg):
        getattr(self.get_logger(), {'warn': 'warning'}.get(level, level))(msg)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def on_lock(self, msg):
        try:
            d = json.loads(msg.data)
            self.nav.on_lock(float(d['t']), str(d['text']), float(d['bearing']), float(d['w_px']))
        except (ValueError, KeyError, TypeError):
            pass

    def on_scan_state(self, msg):
        self.nav.on_scan_state(msg.data)

    def on_ready(self, msg):
        self.nav.set_ready(msg.data)

    def tick(self):
        cmd = Twist()
        if self.pose is None:
            self.get_logger().warning('waiting for /odom...', throttle_duration_sec=3.0)
            self.cmd_pub.publish(cmd)
            return
        out = self.nav.update(self._now(), *self.pose)
        for text in out.scan_cmds:
            self.scan_cmd_pub.publish(String(data=text))
        cmd.linear.x, cmd.angular.z = float(out.v), float(out.w)
        self.cmd_pub.publish(cmd)
        if self.nav.target != self.last_target:
            self.last_target = self.nav.target
            self.target_pub.publish(String(data=self.nav.target))
        ignore = ','.join(sorted(self.nav.ignore))
        if ignore != self.last_ignore:
            self.last_ignore = ignore
            self.ignore_pub.publish(String(data=ignore))

    def publish_slow(self):
        self.status_pub.publish(String(data=self.nav.status()))
        # re-send in case scan_controller restarted (respawn) and lost them
        self.target_pub.publish(String(data=self.nav.target))
        self.ignore_pub.publish(String(data=','.join(sorted(self.nav.ignore))))


def main(args=None):
    rclpy.init(args=args)
    node = AutonomyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.cmd_pub.publish(Twist())
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
