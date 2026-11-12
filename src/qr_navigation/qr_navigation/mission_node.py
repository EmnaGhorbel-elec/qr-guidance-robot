#!/usr/bin/env python3
"""mission_node: guidance layer. Visits the QR checkpoints in order.

  QR table (world) --> /goal (Point, odom frame) --> robot_control (command + motor)
  /robot_pose (QR fix from localization_node) --> drift correction of world->odom
  /mission/status (String) reports progress.

Per QR: drive to an approach point, drive to the checkpoint, wait for a position fix
from that QR, correct the world->odom transform, go on. If the fix never comes the
mission continues on odometry alone and the QR is reported as 'missed'.

Timing: a detection describes the frame the camera took 50-300 ms earlier (decode time on
the Pi). Every goal and fix computed from a detection uses the odometry pose and the camera
pan AT THE CAPTURE TIME (short histories below), so the robot turning in the meantime does
not make it overshoot the QR.
"""
import math
import os
from collections import deque

import yaml
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Empty, Float64MultiArray, Int32, String

from robot_control.control_laws import yaw_from_quaternion
from robot_control.frames import estimate_world_to_odom, odom_to_world, world_to_odom
from qr_navigation.mission_logic import (build_legs, calibrate_k, fix_matches,
                                         measured_robot_xy, pixel_bearing, pose_at,
                                         qr_world_xy, servo_goal, slew_value_at,
                                         update_transform)
from qr_navigation.qr_utils import parse_detection_full

WAIT_ODOM, WAIT_START, SEND_GOAL, MOVING, CALIB, SERVO, WAIT_FIX, DONE, FAILED = (
    'WAIT_ODOM', 'WAIT_START', 'SEND_GOAL', 'MOVING', 'CALIB', 'SERVO', 'WAIT_FIX', 'DONE',
    'FAILED')


class MissionNode(Node):

    def __init__(self):
        super().__init__('mission_node')
        default_table = os.path.join(
            get_package_share_directory('qr_navigation'), 'config', 'qr_table.yaml')
        self.declare_parameter('qr_table_file', default_table)
        self.declare_parameter('mission', ['WAYPOINT_1', 'WAYPOINT_2', 'WAYPOINT_3'])
        self.declare_parameter('approach_distance', 0.5)
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_yaw', 0.0)
        self.declare_parameter('auto_start', True)
        self.declare_parameter('start_delay', 5.0)         # s after the robot is ready (auto_start)
        self.declare_parameter('require_ready', False)     # real robot: wait for /robot/ready
        self.declare_parameter('fix_timeout', 8.0)
        self.declare_parameter('leg_timeout', 90.0)
        self.declare_parameter('fix_tolerance', 0.3)
        self.declare_parameter('correct_yaw', False)
        self.declare_parameter('fix_retries', 2)
        # Final approach by visual servoing on the QR (falls back to a blind drive if unseen)
        self.declare_parameter('servo', True)
        self.declare_parameter('servo_k', 160.3)           # distance ~ k / w_px (README)
        self.declare_parameter('servo_fov', 1.047)         # camera horizontal fov, rad
        self.declare_parameter('camera_offset', 0.15)      # camera ahead of base_link, m
        self.declare_parameter('standoff', 0.8)            # base-to-QR distance to stop at, m
        self.declare_parameter('servo_range_tol', 0.12)    # m
        self.declare_parameter('servo_bearing_tol', 0.10)  # rad
        self.declare_parameter('servo_period', 0.5)        # s between servo goals
        # Pan servo carrying the camera (real robot): bearing = pixel bearing + servo pan
        self.declare_parameter('use_pan', True)            # no /servo/angle messages = pan 0
        self.declare_parameter('pan_center', 90)           # servo angle that points straight ahead
        self.declare_parameter('pan_sign', 1.0)            # +1: higher angle pans the camera left
        self.declare_parameter('servo_blind_wait', 3.0)    # s without the QR before driving blind
        self.declare_parameter('servo_timeout', 30.0)      # s max in servo
        # Measure k at the first approach point (robot stationary at a known distance from the QR)
        self.declare_parameter('auto_calibrate', True)
        self.declare_parameter('calib_timeout', 5.0)       # s max to collect samples
        # Detections: max age to steer on, and lag between the 'A:<angle>' echo (sent when the
        # servo is commanded) and the camera actually being there (SG90 ~0.1 s per 60 deg)
        self.declare_parameter('det_max_age', 1.0)         # s
        self.declare_parameter('pan_lag', 0.0)             # s extra delay before the servo moves
        self.declare_parameter('servo_speed_dps', 344.0)   # servo slew rate (Gazebo 6 rad/s, SG90 ~500)
        # Fix from the measured range + bearing of the QR (needs a measured servo_k).
        # Falls back to "robot is at the table checkpoint" when not possible.
        self.declare_parameter('measured_fix', True)
        self.declare_parameter('fix_max_jump', 1.0)        # m: larger correction = bad measure
        gp = lambda n: self.get_parameter(n).value  # noqa: E731

        with open(gp('qr_table_file'), 'r') as f:
            table = yaml.safe_load(f)['qr_codes']
        self.order = list(gp('mission'))
        self.legs = build_legs(table, self.order, float(gp('approach_distance')))
        self.table = table

        self.odom = None            # (x, y, yaw) from /odom
        self.w2o = None             # world->odom transform, set when the mission starts
        self.state = WAIT_ODOM
        self.idx = 0
        self.seen_false = False     # saw /goal_reached=False after sending the goal
        self.reached = False
        self.start_requested = False
        self.robot_ready = False    # /robot/ready from arduino_bridge_node (Mega + gyro OK)
        self._waiting_logged = False
        self.t_state = self.get_clock().now()
        self.arrival = None
        self.last_det = None        # latest detection, see on_det()
        self.det_seq = 0            # counts detections, so one frame is never used twice
        self.servo_seq = 0          # det_seq of the last detection the servo looked at
        self.t_target_seen = None   # receipt Time of the last detection of the current QR
        self.odom_hist = deque(maxlen=80)   # (t s, (x, y, yaw)) ~4 s at 20 Hz
        self.pan_hist = deque(maxlen=80)    # (t s, pan rad)
        self.servo_ok = False       # servo converged: robot is facing the QR
        self.servo_stable = 0
        self.blind_sent = False
        self._t_servo_goal = None
        self.confirmed, self.missed = [], []
        self.retries = {}           # qr id -> retries used
        self.k = float(self.get_parameter('servo_k').value)   # size constant, may be recalibrated
        self.calibrated = False
        self.pan = 0.0              # camera pan relative to the robot, rad (left positive)
        self.calib_widths = []
        self._t_calib_pub = 0.0
        self._last_status = None
        self._last_status_t = 0.0

        self.goal_pub = self.create_publisher(Point, 'goal', 10)
        self.stop_pub = self.create_publisher(Empty, 'stop', 10)
        self.status_pub = self.create_publisher(String, 'mission/status', 10)
        self.calib_pub = self.create_publisher(Float64MultiArray, 'qr/calibration', 10)
        # which QR the camera servo should lock on (scan_controller_node ignores the others)
        self.target_pub = self.create_publisher(String, 'scan/target', 10)
        self._target = None
        self._t_target_pub = 0.0
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_subscription(Bool, 'goal_reached', self.on_reached, 10)
        self.create_subscription(PoseStamped, 'robot_pose', self.on_fix, 10)
        self.create_subscription(Empty, 'mission/start', self.on_start, 10)
        self.create_subscription(Bool, 'robot/ready', self.on_ready, 10)
        self.create_subscription(Int32, 'servo/angle', self.on_servo_angle, 10)
        self.create_subscription(String, '/qr/detections', self.on_det, 10)
        self.create_timer(0.1, self.tick)
        self.get_logger().info(
            f'mission_node ready: {len(self.order)} QR, {len(self.legs)} legs '
            f'({" -> ".join(self.order)}); waiting for /odom')

    # ---------------------------------------------------------------- helpers
    def _now(self):
        return self.get_clock().now()

    def _elapsed(self):
        return (self._now() - self.t_state).nanoseconds / 1e9

    def _set_state(self, state):
        self.state = state
        self.t_state = self._now()
        self._publish_status(force=True)

    def _publish_status(self, force=False):
        leg = self.legs[self.idx] if self.idx < len(self.legs) else None
        text = f'{self.state}'
        if leg is not None and self.state in (SEND_GOAL, MOVING, SERVO, WAIT_FIX):
            text += f' leg={self.idx + 1}/{len(self.legs)} {leg["qr"]}:{leg["kind"]}'
        if self.state in (DONE, FAILED):
            text += (f' confirmed=[{",".join(self.confirmed)}]'
                     f' missed=[{",".join(self.missed)}]')
        t = self._now().nanoseconds / 1e9
        if force or text != self._last_status or t - self._last_status_t >= 1.0:
            self.status_pub.publish(String(data=text))
            self._last_status, self._last_status_t = text, t

    # -------------------------------------------------------------- callbacks
    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.odom = (p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))
        t = Time.from_msg(msg.header.stamp).nanoseconds / 1e9
        if t <= 0.0:
            t = self._now().nanoseconds / 1e9
        if self.odom_hist and t < self.odom_hist[-1][0]:
            self.odom_hist.clear()                   # clock jumped (bridge restarted)
        self.odom_hist.append((t, self.odom))

    def on_det(self, msg):
        d = parse_detection_full(msg.data)
        if d is None:
            return
        now = self._now()
        t_now = now.nanoseconds / 1e9
        # capture time of the frame; reception time if missing or not on our clock
        t_img = d['stamp'] if d['stamp'] is not None and 0.0 <= t_now - d['stamp'] < 2.0 else t_now
        pose = pose_at(self.odom_hist, t_img)
        if pose is None:
            return
        pan = 0.0
        if self.get_parameter('use_pan').value:
            p = slew_value_at(self.pan_hist, t_img - self.get_parameter('pan_lag').value,
                              math.radians(self.get_parameter('servo_speed_dps').value))
            pan = self.pan if p is None else p
        self.det_seq += 1
        self.last_det = {'d': d, 't': now, 'pose': pose, 'pan': pan, 'seq': self.det_seq}
        leg = self.legs[self.idx] if self.idx < len(self.legs) else None
        if leg is not None and d['id'] == leg['qr'] and d['w_px'] > 0:
            self.t_target_seen = now
        if (self.state == CALIB and d['w_px'] > 0 and self._elapsed() >= 0.5
                and leg is not None and d['id'] == leg['qr']):
            self.calib_widths.append(d['w_px'])

    def on_servo_angle(self, msg):
        gp = self.get_parameter
        if gp('use_pan').value:
            self.pan = math.radians(gp('pan_sign').value * (msg.data - gp('pan_center').value))
            self.pan_hist.append((self._now().nanoseconds / 1e9, self.pan))

    def _fresh_det(self, qr_id):
        """Latest detection of qr_id with a usable size, if younger than det_max_age."""
        det = self.last_det
        if det is None or det['d']['id'] != qr_id or det['d']['w_px'] <= 0:
            return None
        age = (self._now() - det['t']).nanoseconds / 1e9
        return det if age < self.get_parameter('det_max_age').value else None

    def on_start(self, _msg):
        self.start_requested = True

    def on_ready(self, msg):
        self.robot_ready = bool(msg.data)

    def on_reached(self, msg):
        if self.state not in (MOVING, SERVO):
            return
        if not msg.data:
            self.seen_false = True
        elif self.seen_false:
            self.reached = True

    def on_fix(self, msg):
        if self.state != WAIT_FIX or self.odom is None:
            return
        if Time.from_msg(msg.header.stamp) < self.arrival:
            return                                   # fix from before we arrived
        leg = self.legs[self.idx]
        fx, fy = msg.pose.position.x, msg.pose.position.y
        if not fix_matches((fx, fy), (leg['x'], leg['y']), self.get_parameter('fix_tolerance').value):
            self.get_logger().warn(
                f'Fix ({fx:.2f}, {fy:.2f}) is not the expected {leg["qr"]}, ignored',
                throttle_duration_sec=2.0)
            return
        q = msg.pose.orientation
        fix_pose = (fx, fy, yaw_from_quaternion(q.x, q.y, q.z, q.w))
        odom_ref = self.odom
        use_yaw = bool(self.get_parameter('correct_yaw').value) and self.servo_ok
        source = 'table checkpoint'
        measured = self._measured_fix(leg['qr'])
        if measured is not None:
            fix_pose, odom_ref = measured
            use_yaw, source = False, 'measured range+bearing'   # heading stays the gyro's
        self.w2o, drift_m, drift_deg = update_transform(
            self.w2o, fix_pose, odom_ref, use_yaw)
        self.get_logger().info(
            f'{leg["qr"]} fix OK ({source}): robot at ({fix_pose[0]:.2f}, {fix_pose[1]:.2f}), '
            f'odometry drift removed = {drift_m:.3f} m, {drift_deg:.1f} deg'
            f'{" (heading corrected)" if use_yaw else ""}')
        self.confirmed.append(leg['qr'])
        self._next_leg()

    def _measured_fix(self, qr_id):
        """((x, y, yaw) world, odom pose at the same instant) from the latest sighting, or None.

        The table fix only says "the robot is at the checkpoint", which is false whenever
        the robot stopped beside the QR axis or at the wrong distance. Here the QR's known
        position, its measured distance (k / w_px) and its bearing (pixel + camera pan),
        and the gyro heading give the real position of the robot.
        """
        gp = self.get_parameter
        det = self._fresh_det(qr_id)
        if not gp('measured_fix').value or det is None or self.k <= 0.0 or self.w2o is None:
            return None
        e = self.table[qr_id]
        qr_xy = qr_world_xy(float(e['x']), float(e['y']), float(e['theta']),
                            gp('standoff').value)
        d, pose = det['d'], det['pose']
        cam_dist = self.k / d['w_px']
        if not 0.15 <= cam_dist <= 3.0:
            return None
        heading = pose[2] + self.w2o[2]
        bearing = pixel_bearing(d['cx'], d['img_w'], gp('servo_fov').value) + det['pan']
        rx, ry = measured_robot_xy(qr_xy, heading, bearing, cam_dist, gp('camera_offset').value)
        bx, by = odom_to_world((pose[0], pose[1]), self.w2o)
        jump = math.hypot(rx - bx, ry - by)
        if jump > gp('fix_max_jump').value:
            self.get_logger().warn(
                f'{qr_id}: measured fix ({rx:.2f}, {ry:.2f}) is {jump:.2f} m from where odometry '
                f'puts the robot: rejected (check servo_k={self.k:.0f}, pan_sign, the QR table); '
                'using the table checkpoint instead')
            return None
        return (rx, ry, heading), pose

    def _last_seen_text(self, qr_id):
        if self.last_det is None:
            return 'no QR ever seen by the detector'
        d, t = self.last_det['d'], self.last_det['t']
        age = (self._now() - t).nanoseconds / 1e9
        return (f'last detection: {d["id"]} {age:.1f} s ago, w={d["w_px"]:.0f}px, '
                f'cx={d["cx"]:.0f}/{d["img_w"]:.0f}')

    def _calib_step(self):
        gp = self.get_parameter
        n, elapsed = len(self.calib_widths), self._elapsed()
        if n < 8 and elapsed < gp('calib_timeout').value:
            return
        leg = self.legs[self.idx]
        base = gp('approach_distance').value + gp('standoff').value
        k = calibrate_k(self.calib_widths, base, gp('camera_offset').value) if n >= 3 else None
        k0 = float(gp('servo_k').value)
        if k is not None and 0.4 * k0 <= k <= 2.5 * k0:
            self.k, self.calibrated = k, True
            self.get_logger().info(
                f'QR size calibrated: k={k:.1f} (default {k0:.1f}) from {n} frames at '
                f'{base:.2f} m, median width {sorted(self.calib_widths)[n // 2]:.0f}px')
        else:
            self.get_logger().warn(
                f'Calibration failed ({n} frames{"" if k is None else f", k={k:.1f} implausible"}); '
                f'keeping k={self.k:.1f}')
        self._t_servo_goal = None
        self.get_logger().info(f'{leg["qr"]}: final approach by visual servoing')
        self._set_state(SERVO)

    def _servo_step(self):
        gp = self.get_parameter
        leg = self.legs[self.idx]
        now = self._now()
        det = self._fresh_det(leg['qr'])
        elapsed = self._elapsed()
        if elapsed > gp('servo_timeout').value:
            self.stop_pub.publish(Empty())
            self.get_logger().warn(f'{leg["qr"]}: servo timed out, waiting for a fix anyway')
            self.arrival = now
            self._set_state(WAIT_FIX)
            return
        if det is None:
            self.servo_stable = 0
            # time WITHOUT the QR (not time in SERVO): otherwise, after servo_blind_wait, every
            # gap between two detections switched the goal from the QR to the table checkpoint
            # and back, and the robot zig-zagged between two different targets
            since = self.t_state
            if self.t_target_seen is not None and self.t_target_seen > since:
                since = self.t_target_seen
            unseen = (now - since).nanoseconds / 1e9
            if self.blind_sent:
                if self.reached:                       # blind drive done, still nothing
                    self.arrival = now
                    self._set_state(WAIT_FIX)
            elif unseen >= gp('servo_blind_wait').value:
                gx, gy = world_to_odom((leg['x'], leg['y']), self.w2o)
                self.seen_false, self.reached, self.blind_sent = False, False, True
                self.get_logger().warn(
                    f'{leg["qr"]} not in view for {unseen:.1f} s: driving blind to the table '
                    f'checkpoint ({self._last_seen_text(leg["qr"])})')
                self.goal_pub.publish(Point(x=gx, y=gy, z=0.0))
            return
        if det['seq'] == self.servo_seq:
            return                                     # same frame as last tick: nothing new
        self.servo_seq = det['seq']
        if self.blind_sent:
            self.get_logger().info(f'{leg["qr"]} back in view: steering on it again')
        self.blind_sent = False
        d = det['d']
        # pose and pan at the CAPTURE time: the goal is a fixed odom point, valid even if the
        # robot has turned since the frame was taken
        gx, gy, bearing, remaining = servo_goal(
            det['pose'], d['cx'], d['img_w'], d['w_px'], self.k,
            gp('servo_fov').value, gp('camera_offset').value, gp('standoff').value,
            pan=det['pan'])
        # one-sided on purpose: being closer than the stand-off is fine, never drive on
        if remaining <= gp('servo_range_tol').value and abs(bearing) <= gp('servo_bearing_tol').value:
            self.servo_stable += 1                     # counts distinct frames, not ticks
            if self.servo_stable >= 2:
                self.stop_pub.publish(Empty())
                self.servo_ok = True
                self.arrival = now
                self.get_logger().info(
                    f'{leg["qr"]} centred: bearing {math.degrees(bearing):.1f} deg, '
                    f'{remaining:+.2f} m from stand-off')
                self._set_state(WAIT_FIX)
            return
        self.servo_stable = 0
        last = self._t_servo_goal
        if last is None or (now - last).nanoseconds / 1e9 >= gp('servo_period').value:
            self._t_servo_goal = now
            self.goal_pub.publish(Point(x=gx, y=gy, z=0.0))
            self.get_logger().info(
                f'servo {leg["qr"]}: bearing {math.degrees(bearing):+.1f} deg, '
                f'{remaining:+.2f} m to stand-off (w={d["w_px"]:.0f}px, k={self.k:.0f})',
                throttle_duration_sec=1.0)

    # ---------------------------------------------------------- state machine
    def _next_leg(self):
        self.idx += 1
        if self.idx >= len(self.legs):
            self.get_logger().info(
                f'Mission complete. confirmed={self.confirmed} missed={self.missed}')
            self._set_state(DONE)
        else:
            self._set_state(SEND_GOAL)

    def tick(self):
        gp = self.get_parameter
        if self.state == WAIT_ODOM:
            if self.odom is not None:
                self._set_state(WAIT_START)
                if gp('auto_start').value:
                    self.get_logger().info(
                        f'/odom OK, starting {gp("start_delay").value:.0f} s after the robot is ready')
                else:
                    self.get_logger().info(
                        'Waiting for: ros2 topic pub --once /mission/start std_msgs/msg/Empty "{}"')

        elif self.state == WAIT_START:
            if gp('require_ready').value and not self.robot_ready:
                # Mega not connected or gyro not calibrated yet: the start delay only counts
                # from the moment the robot is ready
                self.t_state = self._now()
                if not self._waiting_logged:
                    self._waiting_logged = True
                    self.get_logger().info('waiting for /robot/ready (Mega connected, gyro calibrated)')
                self._publish_status()
                return
            go = self.start_requested or (
                gp('auto_start').value and self._elapsed() >= gp('start_delay').value)
            if go:
                start = (gp('start_x').value, gp('start_y').value, gp('start_yaw').value)
                self.w2o = estimate_world_to_odom(start, self.odom)
                self.get_logger().info(f'Mission start, world pose assumed {start}')
                self._set_state(SEND_GOAL)

        elif self.state == SEND_GOAL:
            leg = self.legs[self.idx]
            if leg['kind'] == 'checkpoint' and gp('servo').value:
                self.servo_ok, self.servo_stable, self.blind_sent = False, 0, False
                self._t_servo_goal = None
                prev = self.legs[self.idx - 1] if self.idx > 0 else None
                if (gp('auto_calibrate').value and not self.calibrated and prev is not None
                        and prev['qr'] == leg['qr'] and prev['kind'] == 'approach'):
                    self.calib_widths = []
                    self.get_logger().info(
                        f'{leg["qr"]}: calibrating QR size from the approach point '
                        f'({gp("approach_distance").value + gp("standoff").value:.2f} m from the QR)')
                    self._set_state(CALIB)
                else:
                    self.get_logger().info(f'{leg["qr"]}: final approach by visual servoing')
                    self._set_state(SERVO)
                return
            gx, gy = world_to_odom((leg['x'], leg['y']), self.w2o)
            self.seen_false = False
            self.reached = False
            self.get_logger().info(
                f'Leg {self.idx + 1}/{len(self.legs)} {leg["qr"]} {leg["kind"]}: '
                f'world ({leg["x"]:.2f}, {leg["y"]:.2f}) -> odom goal ({gx:.2f}, {gy:.2f})')
            self._set_state(MOVING)          # before publishing: the reply may come back at once
            self.goal_pub.publish(Point(x=gx, y=gy, z=0.0))

        elif self.state == MOVING:
            leg = self.legs[self.idx]
            if self.reached:
                if leg['kind'] == 'checkpoint':
                    self.arrival = self._now()
                    self._set_state(WAIT_FIX)
                else:
                    self._next_leg()
            elif self._elapsed() > gp('leg_timeout').value:
                self.stop_pub.publish(Empty())
                self.get_logger().error(f'Leg {self.idx + 1} timed out, mission aborted')
                self._set_state(FAILED)

        elif self.state == CALIB:
            self._calib_step()

        elif self.state == SERVO:
            self._servo_step()

        elif self.state == WAIT_FIX:
            if self._elapsed() > gp('fix_timeout').value:
                leg = self.legs[self.idx]
                qr = leg['qr']
                self.get_logger().warn(
                    f'No fix from {qr} after {gp("fix_timeout").value:.0f} s; {self._last_seen_text(qr)} '
                    '(gate: see localization min_width_px)')
                prev = self.legs[self.idx - 1] if self.idx > 0 else None
                can_retry = (prev is not None and prev['qr'] == qr and prev['kind'] == 'approach'
                             and self.retries.get(qr, 0) < gp('fix_retries').value)
                if can_retry:
                    self.retries[qr] = self.retries.get(qr, 0) + 1
                    self.get_logger().warn(
                        f'Retry {self.retries[qr]}/{gp("fix_retries").value} for {qr}: '
                        'backing off to the approach point and re-approaching along the QR axis')
                    self.idx -= 1
                    self._set_state(SEND_GOAL)
                else:
                    self.missed.append(qr)
                    self._next_leg()

        if self.calibrated:
            t = self._now().nanoseconds / 1e9
            if t - self._t_calib_pub >= 1.0:
                self.calib_pub.publish(Float64MultiArray(data=[float(self.k)]))
                self._t_calib_pub = t
        self._publish_target()
        self._publish_status()

    def _publish_target(self):
        """Tell the camera servo which QR to lock on ('' = any), on change and every second."""
        active = self.state in (SEND_GOAL, MOVING, CALIB, SERVO, WAIT_FIX)
        target = self.legs[self.idx]['qr'] if active and self.idx < len(self.legs) else ''
        t = self._now().nanoseconds / 1e9
        if target != self._target or t - self._t_target_pub >= 1.0:
            self.target_pub.publish(String(data=target))
            self._target, self._t_target_pub = target, t


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
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
