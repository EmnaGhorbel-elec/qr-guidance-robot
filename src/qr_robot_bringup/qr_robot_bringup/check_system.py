#!/usr/bin/env python3
"""check_system: watches the running system and prints a PASS/FAIL report.

  ros2 run qr_robot_bringup check_system [--timeout 180]

Exits when the mission reports DONE/FAILED, or after --timeout seconds.
"""
import argparse
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int32, String

NODES = ['sim_base_node', 'qr_detector_node', 'scan_controller_node', 'localization_node',
         'mission_node', 'command_node', 'motor_node']


class Checker(Node):
    def __init__(self):
        super().__init__('check_system')
        self.count = {}
        self.ids, self.status, self.t0 = set(), '', time.time()
        for name, typ, topic in [
                ('camera', Image, '/camera/image'), ('odom', Odometry, '/odom'),
                ('goal', Point, '/goal'), ('cmd_vel_raw', Twist, '/cmd_vel_raw'),
                ('cmd_vel', Twist, '/cmd_vel'), ('robot_pose', PoseStamped, '/robot_pose')]:
            self.count[name] = 0
            self.create_subscription(typ, topic, lambda m, n=name: self._hit(n), 10)
        self.count['qr_detections'] = 0
        self.create_subscription(String, '/qr/detections', self._det, 10)
        self.create_subscription(String, '/mission/status', self._status, 10)
        self.ready = False
        self.create_subscription(Bool, '/robot/ready', self._ready, 10)
        self.count['servo_angle'] = 0
        self.create_subscription(Int32, '/servo/angle', lambda m: self._hit('servo_angle'), 10)

    def _hit(self, n):
        self.count[n] += 1

    def _det(self, m):
        self.count['qr_detections'] += 1
        self.ids.add(m.data.split('"id": "')[-1].split('"')[0] if '"id"' in m.data else m.data)

    def _ready(self, m):
        if m.data and not self.ready:
            print(f'  [{time.time() - self.t0:5.1f}s] robot ready (gyro calibrated)', flush=True)
        self.ready = self.ready or bool(m.data)

    def _status(self, m):
        if m.data != self.status:
            self.status = m.data
            print(f'  [{time.time() - self.t0:5.1f}s] mission: {m.data}', flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--timeout', type=float, default=180.0)
    args = ap.parse_args()
    rclpy.init()
    node = Checker()
    print('Watching the system (Ctrl+C to stop early)...')
    try:
        while rclpy.ok() and time.time() - node.t0 < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.status.startswith(('DONE', 'FAILED')):
                for _ in range(10):
                    rclpy.spin_once(node, timeout_sec=0.1)
                break
    except KeyboardInterrupt:
        pass

    alive = set(node.get_node_names())
    results = [(f'node {n} running', n in alive) for n in NODES]
    c = node.count
    results += [
        ('camera images received', c['camera'] > 0),
        ('/odom received', c['odom'] > 0),
        ('/robot/ready became true', node.ready),
        ('camera servo moved (/servo/angle)', c['servo_angle'] > 0),
        ('QR detected by camera', c['qr_detections'] > 0),
        ('QR position fix published', c['robot_pose'] > 0),
        ('goals sent by mission', c['goal'] > 0),
        ('velocity commands to Gazebo', c['cmd_vel'] > 0),
        ('mission DONE', node.status.startswith('DONE')),
        ('all QR confirmed (no "missed")', node.status.startswith('DONE') and 'missed=[]' in node.status),
    ]
    print('\n=== REPORT ===')
    for label, ok in results:
        print(f'  {"PASS" if ok else "FAIL"}  {label}')
    print(f'\n  message counts: {c}')
    print(f'  QR ids seen: {sorted(node.ids)}')
    print(f'  final status: {node.status or "(none received)"}')
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if all(ok for _, ok in results) else 1)


if __name__ == '__main__':
    main()
