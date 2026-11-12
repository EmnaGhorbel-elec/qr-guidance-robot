#!/usr/bin/env python3
"""Noeud moteur : interface entre la commande et les moteurs.

  /cmd_vel_raw (Twist)  -->  [limites + rampes + watchdog]  -->  /cmd_vel (Twist)

En simulation, /cmd_vel est lu par le plugin DiffDrive de Gazebo (via le pont).
Avec de vrais moteurs, c'est ici qu'on enverrait les vitesses de roues au driver
(voir le topic informatif /wheel_speeds).
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

from robot_control.control_laws import clip, ramp
from robot_control.graph_guard import WriterGuard


class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')
        self.declare_parameter('max_linear', 0.3)        # m/s
        self.declare_parameter('max_angular', 1.5)       # rad/s
        self.declare_parameter('max_linear_acc', 0.5)    # m/s^2
        self.declare_parameter('max_angular_acc', 2.0)   # rad/s^2
        self.declare_parameter('cmd_timeout', 0.8)       # s sans commande -> arret
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('wheel_radius', 0.033)    # m
        self.declare_parameter('wheel_separation', 0.147)  # m
        # two writers on cmd_vel_raw (command_node + keyboard, or two launches) -> stop
        self.declare_parameter('writer_guard', True)

        self.target_v = 0.0
        self.target_w = 0.0
        self.cur_v = 0.0
        self.cur_w = 0.0
        self.last_cmd_time = None
        self.watchdog_active = False

        self.create_subscription(Twist, 'cmd_vel_raw', self.on_cmd, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.wheels_pub = self.create_publisher(Float64MultiArray, 'wheel_speeds', 10)

        self.dt = 1.0 / self.get_parameter('rate_hz').value
        self.create_timer(self.dt, self.update)
        self.guard = WriterGuard(self, 'cmd_vel_raw')
        self.create_timer(1.0, self.check_graph)
        self.get_logger().info('motor_node pret : /cmd_vel_raw -> /cmd_vel')

    def on_cmd(self, msg):
        max_v = self.get_parameter('max_linear').value
        max_w = self.get_parameter('max_angular').value
        self.target_v = clip(msg.linear.x, -max_v, max_v)
        self.target_w = clip(msg.angular.z, -max_w, max_w)
        self.last_cmd_time = self.get_clock().now()
        if self.watchdog_active:
            self.get_logger().info('Commande reprise')
            self.watchdog_active = False

    def check_graph(self):
        if self.get_parameter('writer_guard').value:
            self.guard.check('commands are mixed, output forced to zero')
        else:
            self.guard.conflict = False

    def update(self):
        timeout = self.get_parameter('cmd_timeout').value
        now = self.get_clock().now()

        lost = (self.last_cmd_time is None or
                (now - self.last_cmd_time).nanoseconds * 1e-9 > timeout)
        if lost or self.guard.conflict:
            tv, tw = 0.0, 0.0
            if self.last_cmd_time is not None and not self.watchdog_active:
                self.get_logger().warn('Plus de commande : arret du robot (watchdog)')
                self.watchdog_active = True
        else:
            tv, tw = self.target_v, self.target_w

        self.cur_v = ramp(self.cur_v, tv, self.get_parameter('max_linear_acc').value * self.dt)
        self.cur_w = ramp(self.cur_w, tw, self.get_parameter('max_angular_acc').value * self.dt)

        out = Twist()
        out.linear.x = self.cur_v
        out.angular.z = self.cur_w
        self.cmd_pub.publish(out)

        # Vitesses de roues (rad/s) : informatif, utile pour de vrais moteurs plus tard
        r = self.get_parameter('wheel_radius').value
        L = self.get_parameter('wheel_separation').value
        wheels = Float64MultiArray()
        wheels.data = [(self.cur_v - self.cur_w * L / 2.0) / r,
                       (self.cur_v + self.cur_w * L / 2.0) / r]
        self.wheels_pub.publish(wheels)


def main():
    rclpy.init()
    node = MotorNode()
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
