#!/usr/bin/env python3
"""scan_controller_node: the camera servo sweeps, locks on a command QR and keeps it centred.

No serial here: it only publishes G<angle> on /servo/command (arduino_bridge_node delivers
them to the Mega). The sweep is driven from the Pi, step by step, so its extent, speed and
pause are ROS parameters (the Mega's own S sweep, 20-160 deg, is no longer used).

  IDLE      servo parked (after 'center'), nothing tracked
  SWEEPING  centre -> `sweep_extent` deg RIGHT -> `sweep_extent` deg LEFT -> centre, one step of
            `sweep_step` deg every `sweep_dwell` s so the camera has time to take and decode a frame
            --QR seen--> LOCKED     --end of the sweep, nothing seen--> NOT_FOUND
  LOCKED    the servo keeps pointing at the QR, also while the robot turns (see below)
            --no detection for lost_timeout s--> LOST (still points where the QR should be)
  NOT_FOUND servo back at the centre

  /scan/command  --> 'sweep' | 'center'
  /scan/target   --> command id the robot is heading for ('' = any). Other QRs are ignored.
  /scan/ignore   --> comma separated ids already executed: never locked again
  /qr/detections --> JSON from qr_detector_node
  /scan/state    <-- IDLE | SWEEPING | LOCKED | LOST | NOT_FOUND
  /scan/lock     <-- JSON {"text", "bearing", "w_px", "t"} on every detection of the locked QR.
                     bearing = direction of the QR in the ODOM frame (rad, counter-clockwise),
                     the sum of robot heading + servo pan + pixel offset, all taken at the
                     instant the frame was captured. THE SERVO ANGLE AT THE LOCK IS THE ANGLE
                     THE ROBOT MUST TURN TO.

World-stabilised tracking: between detections the servo is driven at 20 Hz to keep pointing at
that odom bearing using the live heading, so when the robot turns toward the QR the camera
counter-rotates at once and the QR stays in the image.

Only text matching `accept_regex` (default: a command QR, 'C07|...') and at least
`min_width_px` wide is locked (the width filter drops QRs too far away to be read reliably).
"""
import json
import math
import re
from collections import deque

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Int32, String

from robot_hardware.kinematics import slew_value_at, sweep_angles, value_at


class ScanControllerNode(Node):
    def __init__(self):
        super().__init__('scan_controller_node')
        self.declare_parameter('center_angle', 90)
        self.declare_parameter('min_angle', 0)
        self.declare_parameter('max_angle', 180)
        self.declare_parameter('servo_sign', 1.0)       # +1: higher angle pans the camera LEFT
        self.declare_parameter('hfov', 1.047)           # camera horizontal field of view, rad
        self.declare_parameter('sweep_extent', 90.0)    # deg each side of the centre
        self.declare_parameter('sweep_step', 10.0)      # deg per step (< hfov so frames overlap)
        self.declare_parameter('sweep_dwell', 0.45)     # s per step: capture + decode a frame
        self.declare_parameter('sweep_settle', 0.8)     # s at the centre before the first step
        self.declare_parameter('accept_regex', r'^C\d+\|')
        self.declare_parameter('min_width_px', 30.0)    # only to START a lock (QR not too far)
        self.declare_parameter('lost_timeout', 1.5)
        self.declare_parameter('pan_lag', 0.0)
        self.declare_parameter('servo_speed_dps', 450.0)
        self.declare_parameter('stabilize_rate_hz', 20.0)
        self.declare_parameter('stabilize_deadband_deg', 1.0)

        self.state = 'IDLE'
        self.angle = None
        self.last_det = None
        self.target = ''
        self.ignore = set()
        self.locked_id = None
        self.angle_hist = deque(maxlen=60)    # (t s, angle)
        self.yaw = None
        self.yaw_hist = deque(maxlen=100)     # (t s, yaw)
        self.qr_bearing = None
        self.cmd_angle = None
        self.queue = []
        self.next_step_t = 0.0
        self.regex = re.compile(self.get_parameter('accept_regex').value)

        self.cmd_pub = self.create_publisher(String, 'servo/command', 10)
        self.state_pub = self.create_publisher(String, 'scan/state', 10)
        self.lock_pub = self.create_publisher(String, 'scan/lock', 10)
        self.create_subscription(String, '/qr/detections', self.on_detection, 10)
        self.create_subscription(String, 'scan/command', self.on_command, 10)
        self.create_subscription(String, 'scan/target', self.on_target, 10)
        self.create_subscription(String, 'scan/ignore', self.on_ignore, 10)
        self.create_subscription(Int32, 'servo/angle', self.on_angle, 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_timer(0.05, self.sweep_tick)
        self.create_timer(0.5, self.check_lost)
        self.create_timer(1.0 / self.get_parameter('stabilize_rate_hz').value, self.stabilize_tick)
        self.create_timer(1.0, lambda: self.state_pub.publish(String(data=self.state)))
        self.get_logger().info('scan_controller_node ready (waiting for /scan/command)')

    # ---------------------------------------------------------------- helpers
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _clip_angle(self, a):
        gp = self.get_parameter
        return int(max(gp('min_angle').value, min(gp('max_angle').value, a)))

    def _goto(self, angle):
        self.cmd_pub.publish(String(data=f'G{self._clip_angle(angle)}'))

    def set_state(self, state):
        if state != self.state:
            self.get_logger().info(f'{self.state} -> {state}')
        self.state = state
        self.state_pub.publish(String(data=state))

    # ---------------------------------------------------------------- inputs
    def on_angle(self, msg):
        self.angle = msg.data
        self.angle_hist.append((self._now(), msg.data))

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.yaw_hist.append((t if t > 0.0 else self._now(), self.yaw))

    def on_target(self, msg):
        self.target = msg.data.strip()

    def on_ignore(self, msg):
        self.ignore = {s.strip() for s in msg.data.split(',') if s.strip()}

    def on_command(self, msg):
        cmd = msg.data.strip().lower()
        gp = self.get_parameter
        if cmd in ('sweep', 'rescan'):
            self.start_sweep()
        elif cmd == 'center':
            self.queue = []
            self.qr_bearing = self.cmd_angle = None
            self.locked_id = None
            self._goto(gp('center_angle').value)
            self.set_state('IDLE')
        else:
            self.get_logger().warn(f'unknown /scan/command {msg.data!r} (use sweep or center)')

    # ---------------------------------------------------------------- sweep
    def start_sweep(self):
        gp = self.get_parameter
        self.qr_bearing = self.cmd_angle = None
        self.locked_id = None
        self.last_det = None
        self.queue = sweep_angles(gp('center_angle').value, gp('sweep_extent').value,
                                  gp('sweep_step').value, gp('servo_sign').value,
                                  gp('min_angle').value, gp('max_angle').value)
        self._goto(gp('center_angle').value)              # first look: straight ahead
        self.next_step_t = self._now() + gp('sweep_settle').value
        self.set_state('SWEEPING')

    def sweep_tick(self):
        if self.state != 'SWEEPING' or self._now() < self.next_step_t:
            return
        if self.queue:
            self._goto(self.queue.pop(0))
            self.next_step_t = self._now() + self.get_parameter('sweep_dwell').value
        else:
            self.get_logger().info('sweep finished: no command QR in view')
            self.set_state('NOT_FOUND')

    # ---------------------------------------------------------------- detections
    def _update_bearing(self, cx, img_w, t_img):
        """QR direction in the odom frame from one detection (capture-time heading and pan)."""
        gp = self.get_parameter
        angle = slew_value_at(self.angle_hist, t_img - gp('pan_lag').value,
                              gp('servo_speed_dps').value)
        yaw = value_at(self.yaw_hist, t_img)
        if angle is None:
            angle = self.angle
        if angle is None or yaw is None or cx < 0.0 or img_w <= 0.0:
            return False
        fx = (img_w / 2.0) / math.tan(gp('hfov').value / 2.0)
        pixel = -math.atan2(cx - img_w / 2.0, fx)
        pan = math.radians(gp('servo_sign').value * (angle - gp('center_angle').value))
        b = yaw + pan + pixel
        self.qr_bearing = math.atan2(math.sin(b), math.cos(b))
        return True

    def _predicted_angle(self):
        """Servo angle that points at the last known QR bearing, or None."""
        gp = self.get_parameter
        if self.qr_bearing is None or self.yaw is None:
            return None
        pan = self.qr_bearing - self.yaw
        pan = math.degrees(math.atan2(math.sin(pan), math.cos(pan)))
        return self._clip_angle(round(gp('center_angle').value + gp('servo_sign').value * pan))

    def on_detection(self, msg):
        gp = self.get_parameter
        try:
            d = json.loads(msg.data)
            text = str(d.get('id', '')).strip()
            cx, img_w = float(d['cx']), float(d['img_w'])
            w_px = float(d.get('w_px', -1.0))
            stamp = d.get('stamp')
        except (ValueError, KeyError, TypeError, AttributeError):
            return
        if self.state not in ('SWEEPING', 'LOCKED', 'LOST') or not self.regex.match(text):
            return
        cid = text.split('|', 1)[0].strip()
        if cid in self.ignore or (self.target and cid != self.target):
            return
        if self.state != 'SWEEPING' and cid != self.locked_id:
            return                                   # stay on the QR we locked, do not hop
        if self.state == 'SWEEPING' and 0.0 <= w_px < gp('min_width_px').value:
            return                                   # too far to be read reliably: not yet
        now = self._now()
        t_img = float(stamp) if stamp is not None and 0.0 <= now - float(stamp) < 2.0 else now
        if not self._update_bearing(cx, img_w, t_img):
            return
        self.last_det = now
        if self.state == 'SWEEPING':
            self.queue = []
            self.locked_id = cid
            self.get_logger().info(
                f'QR {cid} found at servo angle {self.angle}: bearing '
                f'{math.degrees(self.qr_bearing):.0f} deg (odom), tracking it')
        if self.state != 'LOCKED':
            self.set_state('LOCKED')
        self.lock_pub.publish(String(data=json.dumps(
            {'text': text, 'bearing': self.qr_bearing, 'w_px': w_px, 't': now})))

    # ---------------------------------------------------------------- tracking
    def stabilize_tick(self):
        gp = self.get_parameter
        if self.state not in ('LOCKED', 'LOST') or self.qr_bearing is None or self.yaw is None:
            return
        angle = self._predicted_angle()
        if (self.cmd_angle is not None
                and abs(angle - self.cmd_angle) < gp('stabilize_deadband_deg').value):
            return
        self.cmd_angle = angle
        self._goto(angle)

    def check_lost(self):
        if (self.state == 'LOCKED' and self.last_det is not None
                and self._now() - self.last_det > self.get_parameter('lost_timeout').value):
            self.get_logger().warn(f'QR {self.locked_id} lost: servo keeps pointing where it should be')
            self.set_state('LOST')


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
