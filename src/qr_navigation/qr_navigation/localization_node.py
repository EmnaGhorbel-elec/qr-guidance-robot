#!/usr/bin/env python3
"""localization_node: QR id -> known position -> /robot_pose (+ TF map->base_link)."""
import math
import os

import yaml
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from std_msgs.msg import Float64MultiArray
from qr_navigation.qr_utils import close_enough, parse_detection, range_corrected_xy


class LocalizationNode(Node):

    def __init__(self):
        super().__init__('localization_node')                            # (1) """this function registers the node in ROS with that name, the one between paranthesis"""

        default_table = os.path.join(                                    # (2)
            get_package_share_directory('qr_navigation'),
            'config', 'qr_table.yaml')
        self.declare_parameter('qr_table_file', default_table)           #"""recognises the existing setting with a certain default  value which points to a copy of qr_tablle.yaml"""
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        # Integration options (defaults keep the original standalone behaviour):
        #  qr_topic     : '/qr_code_topic' (bare id) or '/qr/detections' (detector JSON)
        #  publish_tf   : False when Gazebo already provides odom->base_link
        #  min_width_px : ignore QRs that look smaller than this (= seen from too far)
        self.declare_parameter('qr_topic', '/qr_code_topic')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('min_width_px', 0.0)
        #  qr_range_const: 0 = plain table position; >0 = refine along the QR axis from the
        #  apparent size (distance ~ k / w_px). camera_offset / standoff / max_range_correction in m.
        self.declare_parameter('qr_range_const', 0.0)
        self.declare_parameter('camera_offset', 0.15)
        self.declare_parameter('standoff', 0.8)
        self.declare_parameter('max_range_correction', 0.25)
        table_file = self.get_parameter('qr_table_file').value
        self.map_frame = self.get_parameter('map_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        qr_topic = self.get_parameter('qr_topic').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.min_width_px = float(self.get_parameter('min_width_px').value)
        self.k = float(self.get_parameter('qr_range_const').value)

        self.qr_table = self.load_table(table_file)                      # (3)
        self.last_pose = None      # (x, y, theta) of last valid QR     # (4) """mémoire du noeud"""

        self.pose_pub = self.create_publisher(                           # (5)
            PoseStamped, '/robot_pose', 10)
        self.create_subscription(
            String, qr_topic, self.qr_callback, 10)
        self.create_subscription(Float64MultiArray, 'qr/calibration', self.on_calibration, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        if self.publish_tf:
            self.create_timer(0.1, self.broadcast_tf)                    # (6)

        self.get_logger().info('localization_node ready')

    def load_table(self, path):                                          # (3) """it turns the yaml table into a dictionary"""
        try:
            with open(path, 'r') as f:
                table = yaml.safe_load(f)['qr_codes']
        except (OSError, yaml.YAMLError, KeyError, TypeError) as err:
            self.get_logger().error(f'Cannot load QR table "{path}": {err}')
            return {}
        self.get_logger().info(f'Loaded {len(table)} QR codes from {path}')
        return table

    def on_calibration(self, msg):
        """The mission measured the real QR size: adopt it (also rescales the distance gate)."""
        if not msg.data or msg.data[0] <= 0.0 or abs(msg.data[0] - self.k) < 1e-6:
            return
        old_k, self.k = self.k, float(msg.data[0])
        if self.min_width_px > 0.0:       # gate was enabled: keep it at ~standoff + 0.15 m
            standoff = float(self.get_parameter('standoff').value)
            offset = float(self.get_parameter('camera_offset').value)
            self.min_width_px = self.k / (standoff + 0.15 - offset)
        self.get_logger().info(
            f'QR size calibration received: k {old_k:.1f} -> {self.k:.1f}, '
            f'min_width_px now {self.min_width_px:.0f}')

    def qr_callback(self, msg):                                          # (7)
        qr_id, w_px = parse_detection(msg.data)
        if qr_id == '' or qr_id == 'NONE':
            return                       # nothing detected: keep last pose
        if not close_enough(w_px, self.min_width_px):
            self.get_logger().info(
                f'{qr_id} seen but too far (w={w_px:.0f}px < {self.min_width_px:.0f}px), '
                'no position fix', throttle_duration_sec=3.0)
            return
        if qr_id not in self.qr_table:
            self.get_logger().warn(
                f'Unknown QR code "{qr_id}", ignored', throttle_duration_sec=2.0)
            return

        entry = self.qr_table[qr_id]
        x, y, theta = float(entry['x']), float(entry['y']), float(entry['theta'])
        x, y = range_corrected_xy(
            x, y, theta, w_px, self.k,
            float(self.get_parameter('camera_offset').value),
            float(self.get_parameter('standoff').value),
            float(self.get_parameter('max_range_correction').value))
        self.last_pose = (x, y, theta)

        pose = PoseStamped()                                             # (8)
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(theta / 2.0)
        pose.pose.orientation.w = math.cos(theta / 2.0)
        self.pose_pub.publish(pose)
        self.get_logger().info(
            f'{qr_id} detected -> pose x={x:.2f} y={y:.2f} theta={theta:.2f}',
            throttle_duration_sec=1.0)

    def broadcast_tf(self):                                              # (9)
        if self.last_pose is None:
            return
        x, y, theta = self.last_pose
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.robot_frame
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation.z = math.sin(theta / 2.0)
        t.transform.rotation.w = math.cos(theta / 2.0)
        self.tf_broadcaster.sendTransform(t)


def main(args=None):                                                     # (10)
    rclpy.init(args=args)
    node = LocalizationNode()
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
