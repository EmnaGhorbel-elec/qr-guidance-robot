#!/usr/bin/env python3
"""guidance_node: /robot_pose + /goal_pose -> /cmd_vel (proportional control)."""
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, Twist


def yaw_from_quaternion(q):                                              # (1)
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def normalize_angle(a):                                                  # (2)
    return math.atan2(math.sin(a), math.cos(a))


def clamp(value, low, high):                                             # (3)
    return max(low, min(high, value))


class GuidanceNode(Node):

    def __init__(self):
        super().__init__('guidance_node')

        self.control_rate = self._param('control_rate', 10.0)            # (4)
        self.max_linear = self._param('max_linear', 0.20)
        self.max_angular = self._param('max_angular', 0.80)
        self.k_linear = self._param('k_linear', 0.8)
        self.k_angular = self._param('k_angular', 1.5)
        self.dist_tol = self._param('dist_tolerance', 0.10)
        self.yaw_tol = self._param('yaw_tolerance', 0.15)
        self.rotate_first = self._param('rotate_first_threshold', 0.5)
        self.align_final = self._param('align_final_heading', True)
        self.pose_timeout = self._param('pose_timeout', 5.0)

        self.pose = None       # last PoseStamped received                # (5)
        self.goal = None       # current goal (Pose) or None if none/reached

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(PoseStamped, '/robot_pose', self.pose_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.create_timer(1.0 / self.control_rate, self.control_loop)

        self.get_logger().info('guidance_node ready, waiting for pose and goal')

    def _param(self, name, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def pose_callback(self, msg):                                        # (6)
        self.pose = msg

    def goal_callback(self, msg):
        self.goal = msg.pose
        self.get_logger().info(
            f'New goal: x={msg.pose.position.x:.2f} y={msg.pose.position.y:.2f}')

    def control_loop(self):                                              # (7)
        cmd = Twist()                       # all zeros = "stop"

        if self.goal is None or self.pose is None:
            # idle: publish NOTHING. Sending zeros forever made this node a second writer on
            # /cmd_vel next to motor_node on the real robot (alternating stop / move commands).
            # NOTE: this node is the old standalone guidance (QR pose only). The real robot
            # uses mission_node -> command_node -> motor_node -> arduino_bridge_node instead.
            return

        # --- safety: stale pose -> stop                                 # (8)
        age = (self.get_clock().now()
               - Time.from_msg(self.pose.header.stamp)).nanoseconds / 1e9
        if self.pose_timeout > 0.0 and age > self.pose_timeout:
            self.get_logger().warn(
                f'Pose is {age:.1f}s old (> {self.pose_timeout}s): stopping, goal dropped',
                throttle_duration_sec=2.0)
            self.cmd_pub.publish(cmd)
            self.goal = None                # one stop command, then silent (see above)
            return

        # --- errors                                                     # (9)
        p = self.pose.pose
        yaw = yaw_from_quaternion(p.orientation)
        dx = self.goal.position.x - p.position.x
        dy = self.goal.position.y - p.position.y
        dist = math.hypot(dx, dy)

        if dist > self.dist_tol:                                         # (10)
            heading_err = normalize_angle(math.atan2(dy, dx) - yaw)
            if abs(heading_err) > self.rotate_first:
                # phase 1: turn in place
                cmd.angular.z = clamp(self.k_angular * heading_err,
                                      -self.max_angular, self.max_angular)
            else:
                # phase 2: drive and correct heading at the same time
                cmd.linear.x = min(self.k_linear * dist, self.max_linear)
                cmd.angular.z = clamp(self.k_angular * heading_err,
                                      -self.max_angular, self.max_angular)
        else:
            # phase 3: position reached, align to goal orientation
            yaw_err = 0.0
            if self.align_final:
                goal_yaw = yaw_from_quaternion(self.goal.orientation)
                yaw_err = normalize_angle(goal_yaw - yaw)
            if abs(yaw_err) > self.yaw_tol:
                cmd.angular.z = clamp(self.k_angular * yaw_err,
                                      -self.max_angular, self.max_angular)
            else:
                self.get_logger().info('Goal reached, stopping')
                self.goal = None                                         # (11)

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = GuidanceNode()
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
