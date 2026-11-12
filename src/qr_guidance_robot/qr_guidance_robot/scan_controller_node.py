#!/usr/bin/env python3
"""scan_controller_node: the camera servo searches, then follows the QR.

No serial here: it only publishes S / H / G<angle> on /servo/command. On the robot
arduino_bridge_node delivers them to the Mega, in Gazebo robot_sim/sim_base_node does.

  SCANNING --QR seen--> LOCKED (servo follows the QR so it stays centred in the image)
  LOCKED --no QR for lost_timeout s--> SCANNING again   (resume_after_lost)

  /qr/detections --> servo tracking
  /scan/command  --> 'rescan' (search again) | 'center' (servo to 90 deg, stop tracking)
  /scan/target   --> id of the QR the mission is heading for ('' = any). Other QRs are
                     ignored, so the camera no longer stays locked on the previous QR while
                     the mission looks for the next one.
  /scan/state    <-- SCANNING | LOCKED | IDLE

Tracking uses the servo angle at the time the frame was TAKEN (detection 'stamp'), not the
current one: decoding takes 50-300 ms, and stepping from the current angle made the camera
overshoot and hunt around the QR.

World-stabilised tracking (stabilize:=true, needs /odom): each detection gives the QR's
bearing in the odom frame (robot heading + servo pan + pixel bearing, all at capture time).
Between detections the servo is driven at 20 Hz to keep pointing at that bearing using the
odometry heading, so when the robot turns toward the QR the camera counter-rotates at once.
Without it the camera only reacted to new detections: a robot turning at ~50 deg/s pushed
the QR out of the frame before the next one, and the camera stayed pointing at the wall.
mission_node adds /servo/angle to the QR bearing, so it knows where the QR really is.
"""
import json
import math
from collections import deque

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32, String

from qr_guidance_robot.scan_logic import slew_value_at, track_step, value_at


class ScanControllerNode(Node):
    def __init__(self):
        super().__init__('scan_controller_node')
        self.declare_parameter('autostart', True)
        self.declare_parameter('start_delay', 4.0)   # s: the bridge needs ~2 s to boot the Mega
        self.declare_parameter('center_angle', 90)
        self.declare_parameter('min_angle', 20)
        self.declare_parameter('max_angle', 160)
        self.declare_parameter('servo_sign', 1.0)    # +1: higher angle pans the camera LEFT
        self.declare_parameter('track', True)
        self.declare_parameter('hfov', 1.047)        # camera horizontal field of view, rad
        self.declare_parameter('track_gain', 0.6)
        self.declare_parameter('track_deadband_deg', 1.5)
        self.declare_parameter('track_max_step_deg', 8.0)
        self.declare_parameter('track_period', 0.15)   # s between servo moves
        self.declare_parameter('resume_after_lost', True)
        self.declare_parameter('lost_timeout', 3.0)
        self.declare_parameter('rescan_resend_s', 5.0)   # re-send 'S' while scanning (see check_lost)
        self.declare_parameter('pan_lag', 0.0)           # s: extra delay before the servo starts moving
        self.declare_parameter('servo_speed_dps', 344.0)  # servo slew rate (Gazebo joint: 6 rad/s; SG90 ~500)
        self.declare_parameter('stabilize', True)        # keep pointing at the QR while the robot turns
        self.declare_parameter('stabilize_rate_hz', 20.0)
        self.declare_parameter('stabilize_deadband_deg', 1.0)

        self.state = 'IDLE'
        self.angle = None
        self.last_det = None
        self.last_move = None
        self.last_scan_cmd = None
        self.target = ''              # QR to lock on ('' = any)
        self.locked_id = None
        self.angle_hist = deque(maxlen=60)   # (t s, angle)
        self.yaw = None                      # robot heading from /odom, rad
        self.yaw_hist = deque(maxlen=100)    # (t s, yaw)
        self.qr_bearing = None               # QR direction in the odom frame, rad
        self.cmd_angle = None                # last angle sent by the stabiliser
        self.cmd_pub = self.create_publisher(String, 'servo/command', 10)
        self.state_pub = self.create_publisher(String, 'scan/state', 10)
        self.create_subscription(String, '/qr/detections', self.on_detection, 10)
        self.create_subscription(String, 'scan/command', self.on_command, 10)
        self.create_subscription(Int32, 'servo/angle', self.on_angle, 10)
        self.create_subscription(String, 'scan/target', self.on_target, 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_timer(0.5, self.check_lost)
        self.create_timer(1.0 / self.get_parameter('stabilize_rate_hz').value, self.stabilize_tick)

        if self.get_parameter('autostart').value:
            self._start_timer = self.create_timer(
                self.get_parameter('start_delay').value, self._autostart)
        self.get_logger().info('scan_controller_node ready')

    def _autostart(self):
        self._start_timer.cancel()
        self.start_scan()

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def set_state(self, state):
        self.state = state
        self.state_pub.publish(String(data=state))

    def start_scan(self, from_angle=None):
        self.qr_bearing = None
        self.cmd_angle = None
        self.last_scan_cmd = self._now()
        if from_angle is not None:
            self.cmd_pub.publish(String(data=f'G{from_angle}'))   # the sweep starts from here
        self.cmd_pub.publish(String(data='S'))
        self.set_state('SCANNING')
        self.get_logger().info('Scanning')

    def on_angle(self, msg):
        self.angle = msg.data
        self.angle_hist.append((self._now(), msg.data))

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.yaw_hist.append((t if t > 0.0 else self._now(), self.yaw))

    def _stabilizing(self):
        return bool(self.get_parameter('stabilize').value) and self.yaw is not None

    def _update_bearing(self, cx, img_w, t_img):
        """QR direction in the odom frame from one detection (capture-time heading and pan)."""
        gp = self.get_parameter
        angle = slew_value_at(self.angle_hist, t_img - gp('pan_lag').value,
                              gp('servo_speed_dps').value)
        yaw = value_at(self.yaw_hist, t_img)
        if angle is None:
            angle = self.angle
        if angle is None or yaw is None or cx < 0.0 or img_w <= 0.0:
            return
        fx = (img_w / 2.0) / math.tan(gp('hfov').value / 2.0)
        pixel = -math.atan2(cx - img_w / 2.0, fx)
        pan = math.radians(gp('servo_sign').value * (angle - gp('center_angle').value))
        b = yaw + pan + pixel
        self.qr_bearing = math.atan2(math.sin(b), math.cos(b))

    def _predicted_angle(self):
        """Servo angle pointing at the last known QR bearing, or None."""
        gp = self.get_parameter
        if self.qr_bearing is None or self.yaw is None:
            return None
        pan = self.qr_bearing - self.yaw
        pan = math.degrees(math.atan2(math.sin(pan), math.cos(pan)))
        angle = int(round(gp('center_angle').value + gp('servo_sign').value * pan))
        return max(gp('min_angle').value, min(gp('max_angle').value, angle))

    def stabilize_tick(self):
        gp = self.get_parameter
        if (self.state != 'LOCKED' or not gp('track').value or not self._stabilizing()
                or self.qr_bearing is None):
            return
        angle = self._predicted_angle()
        if self.cmd_angle is not None and abs(angle - self.cmd_angle) < gp('stabilize_deadband_deg').value:
            return
        self.cmd_angle = angle
        self.last_move = self._now()
        self.cmd_pub.publish(String(data=f'G{angle}'))

    def on_target(self, msg):
        target = msg.data.strip()
        if target == self.target:
            return
        self.target = target
        self.get_logger().info(f'target QR: {target or "any"}')
        if self.state == 'LOCKED' and target and self.locked_id != target:
            self.get_logger().info(f'locked on {self.locked_id}, not the target: scanning again')
            self.start_scan()

    def on_detection(self, msg):
        gp = self.get_parameter
        try:
            d = json.loads(msg.data)
            qr_id = str(d.get('id', '')).strip()
            cx, img_w = float(d['cx']), float(d['img_w'])
            stamp = d.get('stamp')
        except (ValueError, KeyError, TypeError, AttributeError):
            return
        if self.target and qr_id != self.target:
            return                    # not the QR the mission wants: do not lock / track it
        now = self._now()
        self.last_det = now
        # capture time of the frame (fall back to now)
        t_img = float(stamp) if stamp is not None and 0.0 <= now - float(stamp) < 2.0 else now
        if self.state == 'SCANNING':
            self.cmd_pub.publish(String(data='H'))
            self.locked_id = qr_id
            self.set_state('LOCKED')
            self.get_logger().info(f'QR {qr_id} found at servo angle {self.angle}: tracking it')
            if self._stabilizing():
                self._update_bearing(cx, img_w, t_img)
            return
        if self.state != 'LOCKED' or not gp('track').value or self.angle is None:
            return
        if self._stabilizing():
            self._update_bearing(cx, img_w, t_img)   # stabilize_tick moves the servo
            return
        # no odometry: step toward the QR from the angle at capture time
        if self.last_move is not None and now - self.last_move < gp('track_period').value:
            return
        if cx < 0.0 or img_w <= 0.0:
            return
        seen_at = slew_value_at(self.angle_hist, t_img - gp('pan_lag').value,
                                gp('servo_speed_dps').value)
        if seen_at is None:
            seen_at = self.angle
        new = track_step(cx, img_w, gp('hfov').value, seen_at, gp('servo_sign').value,
                         gp('track_gain').value, gp('track_deadband_deg').value,
                         gp('track_max_step_deg').value, gp('min_angle').value,
                         gp('max_angle').value)
        if new != self.angle:
            self.last_move = now
            self.angle = new
            self.cmd_pub.publish(String(data=f'G{new}'))

    def check_lost(self):
        gp = self.get_parameter
        # At power-on the Mega may not be connected yet (the first 'S' is dropped), and a
        # Mega reset forgets the sweep. 'S' is harmless when repeated, so keep sending it.
        if (self.state == 'SCANNING' and self.last_scan_cmd is not None
                and self._now() - self.last_scan_cmd >= gp('rescan_resend_s').value):
            self.last_scan_cmd = self._now()
            self.cmd_pub.publish(String(data='S'))
        if (self.state == 'LOCKED' and gp('resume_after_lost').value and
                self.last_det is not None and self._now() - self.last_det > gp('lost_timeout').value):
            predicted = self._predicted_angle()
            self.start_scan(from_angle=predicted)
            self.get_logger().info('QR lost: scanning again' + (
                f' from where odometry says it is (servo {predicted})' if predicted else ''))

    def on_command(self, msg):
        if msg.data == 'rescan' and self.state in ('LOCKED', 'IDLE'):
            self.start_scan()
        elif msg.data == 'center':
            angle = self.get_parameter('center_angle').value
            self.cmd_pub.publish(String(data=f'G{angle}'))
            self.set_state('IDLE')
            self.get_logger().info(f'Servo centred at {angle}')


def main(args=None):
    rclpy.init(args=args)
    node = ScanControllerNode()
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
