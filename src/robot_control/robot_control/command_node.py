#!/usr/bin/env python3
"""Noeud de commande : va vers un point (x, y) du repere odom, avec feedback /odom.

  /goal (Point) + /odom (Odometry)  -->  /cmd_vel_raw (Twist)

Plus tard, le noeud de guidage par QR code publiera simplement sur /goal.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Empty

from robot_control.control_laws import go_to_point, yaw_from_quaternion
from robot_control.graph_guard import WriterGuard


class CommandNode(Node):
    def __init__(self):
        super().__init__('command_node')
        self.declare_parameter('k_lin', 0.8)
        self.declare_parameter('k_ang', 1.5)
        self.declare_parameter('max_v', 0.2)             # m/s
        self.declare_parameter('max_w', 0.8)             # rad/s
        self.declare_parameter('tolerance', 0.05)        # m
        self.declare_parameter('turn_threshold', 0.5)    # rad : au-dela, rotation sur place
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('writer_guard', True)   # arret si 2 sources /odom (ex: simulation)

        self.pose = None   # (x, y, yaw) lu sur /odom
        self.goal = None   # (x, y)

        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_subscription(Point, 'goal', self.on_goal, 10)
        self.create_subscription(Empty, 'stop', self.on_stop, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_raw', 10)
        self.reached_pub = self.create_publisher(Bool, 'goal_reached', 10)

        self.create_timer(1.0 / self.get_parameter('rate_hz').value, self.update)
        self.odom_guard = WriterGuard(self, 'odom')
        self.goal_guard = WriterGuard(self, 'goal')
        self.create_timer(1.0, self.check_graph)
        self.get_logger().info('command_node pret : publie un objectif sur /goal (geometry_msgs/Point)')

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))

    def on_goal(self, msg):
        self.goal = (msg.x, msg.y)
        self.reached_pub.publish(Bool(data=False))
        self.get_logger().info(f'Nouvel objectif : x={msg.x:.2f}, y={msg.y:.2f}')

    def on_stop(self, _msg):
        self.goal = None
        self.get_logger().info('Objectif annule')

    def check_graph(self):
        if not self.get_parameter('writer_guard').value:
            self.odom_guard.conflict = False
            return
        # two /odom sources: the position jumps between two robots -> stop
        self.odom_guard.check('position feedback is mixed, the robot is stopped')
        # two /goal sources (two mission_node): warn only, so a manual /goal test still works
        self.goal_guard.check('goals from several nodes alternate', stop=False)

    def update(self):
        if self.pose is None:
            self.get_logger().warn('En attente de /odom...', throttle_duration_sec=3.0)
            return

        cmd = Twist()
        if self.goal is not None and not self.odom_guard.conflict:
            x, y, yaw = self.pose
            gp = self.get_parameter
            v, w, reached = go_to_point(
                x, y, yaw, self.goal[0], self.goal[1],
                gp('k_lin').value, gp('k_ang').value,
                gp('max_v').value, gp('max_w').value,
                gp('tolerance').value, gp('turn_threshold').value)
            cmd.linear.x = v
            cmd.angular.z = w
            if reached:
                self.get_logger().info(f'Objectif atteint : x={x:.2f}, y={y:.2f}')
                self.goal = None
                self.reached_pub.publish(Bool(data=True))
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = CommandNode()
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
