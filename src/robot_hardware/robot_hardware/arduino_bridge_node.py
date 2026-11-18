#!/usr/bin/env python3
"""arduino_bridge_node: the ONLY node that talks to the Arduino Mega.

  /cmd_vel (Twist) ------------> "M<a>,<b>" (PWM) --USB serial--> Mega --> motors
  /servo/command (String) -----> "S" | "H" | "G<angle>" ---------> Mega --> camera servo
  Mega "A:<angle>" ------------> /servo/angle (Int32)
  Mega "I:<ms>,<gz>" (MPU-6050 gyro Z, 25 Hz, 9600 baud link) --> /imu/data (Imu)
  /odom (Odometry): position = commanded speed integrated (no encoders), heading = gyro
                    (falls back to the commanded turn rate if the IMU is not sending)

  /robot/ready (Bool, 2 Hz): Mega connected + gyro calibrated. autonomy_node waits for it.

One serial port = one owner: the port is opened with an exclusive lock, so a second
bridge (robot.service + a manual launch) cannot write to the Mega at the same time.

Safety on real hardware (both stop the wheels and say why in the log):
  - more than one publisher on /cmd_vel (motor_node must be the only one),
  - the robot turns the OPPOSITE way to angular.z (gyro sign / motor side error, see
    kinematics.TurnCheck): latched until the node restarts.
Power-on: the Mega may not be enumerated yet when ROS starts, and it may drop out later
(brownout, cable). The bridge keeps trying to (re)open the port; it never exits.
"""
import glob
import math

import rclpy
import serial
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Int32, String

from robot_control.graph_guard import WriterGuard
from robot_hardware.kinematics import (
    GyroYaw, TurnCheck, clip, imu_raw_to_si, integrate_pose, integrate_position,
    parse_imu, parse_servo_angle, pick_port, robot_ready, twist_to_pwm)


class ArduinoBridgeNode(Node):
    def __init__(self):
        super().__init__('arduino_bridge_node')
        self.declare_parameter('port', '/dev/ttyACM0')     # or 'auto': first /dev/ttyACM* or ttyUSB*
        self.declare_parameter('baud', 9600)
        self.declare_parameter('wheel_separation', 0.147)  # m (skid steer: tune, see README)
        self.declare_parameter('max_speed', 0.35)          # m/s at PWM 255  <-- placeholder
        self.declare_parameter('min_pwm', 70)              # PWM below which the wheels stall
        self.declare_parameter('swap_ab', False)           # True if motor A is the RIGHT side
        self.declare_parameter('invert_a', False)
        self.declare_parameter('invert_b', False)
        self.declare_parameter('cmd_timeout', 0.5)         # s without /cmd_vel -> stop
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('odom_linear_scale', 1.0)   # real distance / commanded distance
        self.declare_parameter('odom_angular_scale', 1.0)  # used only without the IMU
        # IMU
        self.declare_parameter('use_imu', True)
        self.declare_parameter('imu_yaw_sign', 1.0)        # -1 if the chip's Z axis points down
        self.declare_parameter('gyro_calib_samples', 50)   # 2 s at 25 Hz, robot must be still
        self.declare_parameter('gyro_deadband', 0.004)     # rad/s: below this = standing still
        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('require_imu', True)        # no IMU -> /robot/ready stays False
        self.declare_parameter('mega_boot_s', 2.5)         # the Mega resets when the port opens
        # safety checks (see the module docstring)
        self.declare_parameter('turn_check', True)
        self.declare_parameter('writer_guard', True)

        gp = self.get_parameter
        self.ser = None
        self.connected = False          # port open AND the Mega has finished booting
        self.boot_done_at = None
        self.connect_warned = False
        self.rx = ''

        self.v = 0.0
        self.w = 0.0
        self.last_cmd = None
        self.x = self.y = self.yaw = 0.0
        self.last_t = self.get_clock().now()
        self.timeout_warned = False

        self.gyro = GyroYaw(gp('gyro_calib_samples').value, gp('gyro_deadband').value)
        self.imu_last = None            # Pi time of the last IMU sample
        self.imu_seen = False
        self.gyro_yaw_prev = 0.0
        self.calib_restarts_warned = 0
        self.t_start = self.get_clock().now()
        self.imu_missing_warned = False
        self.was_ready = False
        self.turn_check = TurnCheck()
        self.turn_fault = False         # latched: the robot turns the wrong way
        self.cmd_guard = WriterGuard(self, 'cmd_vel')

        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(String, 'servo/command', self.on_servo_cmd, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.angle_pub = self.create_publisher(Int32, 'servo/angle', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 20)
        self.ready_pub = self.create_publisher(Bool, 'robot/ready', 10)

        self.create_timer(1.0 / gp('rate_hz').value, self.tick)
        self.create_timer(0.01, self.read_serial)       # 100 Hz: keep up with the 25 Hz IMU
        self.create_timer(1.0, self.connect)            # (re)open the port until it works
        self.create_timer(0.5, self.publish_ready)
        self.create_timer(1.0, self.check_graph)
        self.connect()
        self.get_logger().info(f"arduino_bridge_node started (port: {gp('port').value})")

    # ---------- serial ----------
    def connect(self):
        """Open the port if it is closed; mark the Mega as booted a moment later."""
        now = self.get_clock().now()
        if self.ser is not None:
            if (not self.connected and self.boot_done_at is not None
                    and now >= self.boot_done_at):
                self.ser.reset_input_buffer()
                self.rx = ''
                self.connected = True
                self.get_logger().info('Mega connected')
            return
        gp = self.get_parameter
        port = pick_port(gp('port').value,
                         glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*'))
        try:
            if port is None:
                raise OSError('no /dev/ttyACM* or /dev/ttyUSB* yet')
            # exclusive: a second process (another launch, the Arduino IDE serial monitor)
            # can no longer open the port and write motor commands next to this node
            self.ser = serial.Serial(port, gp('baud').value, timeout=0, exclusive=True)
        except (serial.SerialException, OSError) as exc:
            if not self.connect_warned:
                busy = 'lock' in str(exc).lower() or 'busy' in str(exc).lower()
                self.get_logger().warn(
                    f'waiting for the Mega ({exc}); will keep trying'
                    + ('. The port is held by ANOTHER process: is robot.service running '
                       '(sudo systemctl stop robot) or a second launch?' if busy else ''))
                self.connect_warned = True
            return
        self.connect_warned = False
        self.boot_done_at = now + Duration(seconds=gp('mega_boot_s').value)
        # a fresh Mega run: its millisecond clock restarts, so the gyro must calibrate again
        # (the heading is kept: tick() copies self.yaw into the gyro while it is not ready)
        self.gyro.reset()
        self.calib_restarts_warned = 0
        self.imu_last = None
        self.get_logger().info(f'opened {port}, waiting {gp("mega_boot_s").value:.1f} s for the Mega')

    def lost(self, what, exc):
        self.get_logger().error(f'serial {what} failed: {exc}; reconnecting',
                                throttle_duration_sec=2.0)
        try:
            self.ser.close()
        except Exception:  # noqa: BLE001
            pass
        self.ser = None
        self.connected = False

    def send(self, text):
        if not self.connected:
            return
        try:
            self.ser.write((text + '\n').encode())
        except (serial.SerialException, OSError) as exc:
            self.lost('write', exc)

    def read_serial(self):
        if self.ser is None:
            return
        try:
            n = self.ser.in_waiting
            if n:
                data = self.ser.read(n).decode(errors='ignore')
                if self.connected:          # while the Mega boots, its output is discarded
                    self.rx += data
        except (serial.SerialException, OSError) as exc:
            self.lost('read', exc)
            return
        while '\n' in self.rx:
            line, self.rx = self.rx.split('\n', 1)
            line = line.strip()
            if not line:
                continue
            if line.startswith('I:'):
                self.on_imu_line(line)
                continue
            angle = parse_servo_angle(line)
            if angle is not None:
                self.angle_pub.publish(Int32(data=angle))
            elif line.startswith('E:'):
                self.get_logger().warn(f'Arduino: {line}')
            elif line in ('OK:SCAN', 'OK:HALT'):
                self.get_logger().debug(f'Arduino: {line}')    # repeated on purpose
            else:
                self.get_logger().info(f'Arduino: {line}')
        if len(self.rx) > 256:          # garbage without newline: do not grow forever
            self.rx = ''

    # ---------- IMU ----------
    def on_imu_line(self, line):
        raw = parse_imu(line)
        if raw is None:
            return
        gp = self.get_parameter
        ms, ax, ay, az, gx, gy, gz = raw
        acc, gyr = imu_raw_to_si(ax, ay, az, gx, gy, gz)
        if not self.imu_seen:
            self.imu_seen = True
            self.get_logger().info('IMU streaming. Keep the robot still while the gyro calibrates')
        self.imu_last = self.get_clock().now()

        sign = gp('imu_yaw_sign').value
        was_ready = self.gyro.ready
        if gp('use_imu').value:
            self.gyro.update(ms, sign * gyr[2])
        if self.gyro.restarts != self.calib_restarts_warned:
            self.calib_restarts_warned = self.gyro.restarts
            self.get_logger().warn('gyro calibration restarted: the robot was moving')
        if self.gyro.ready and not was_ready:
            self.get_logger().info(f'gyro calibrated, bias {math.degrees(self.gyro.bias):.2f} deg/s')

        msg = Imu()
        msg.header.stamp = self.imu_last.to_msg()
        msg.header.frame_id = gp('imu_frame').value
        msg.orientation_covariance[0] = -1.0            # no orientation estimate
        full = line.count(',') == 6                     # 9600 baud: the Mega sends only gz
        msg.angular_velocity.x, msg.angular_velocity.y = gyr[0], gyr[1]
        msg.angular_velocity.z = sign * gyr[2] - (self.gyro.bias if self.gyro.ready else 0.0)
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = acc
        for i in (0, 4, 8):
            msg.angular_velocity_covariance[i] = 0.02 ** 2
            msg.linear_acceleration_covariance[i] = 0.3 ** 2
        if not full:                                    # REP-145: -1 = this axis is unknown
            msg.angular_velocity_covariance[0] = msg.angular_velocity_covariance[4] = -1.0
            msg.linear_acceleration_covariance[0] = -1.0
        self.imu_pub.publish(msg)

    def imu_ok(self):
        """True while the gyro is calibrated and samples keep arriving."""
        if not (self.get_parameter('use_imu').value and self.gyro.ready and self.imu_last):
            return False
        return (self.get_clock().now() - self.imu_last).nanoseconds * 1e-9 < 0.3

    # ---------- ROS inputs ----------
    def on_cmd_vel(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z
        self.last_cmd = self.get_clock().now()
        self.timeout_warned = False

    def on_servo_cmd(self, msg):
        cmd = msg.data.strip()
        if cmd and cmd[0] in 'SHG':
            self.send(cmd)
        else:
            self.get_logger().warn(f'ignored servo command {msg.data!r} (use S, H or G<angle>)')

    # ---------- 20 Hz loop ----------
    def tick(self):
        gp = self.get_parameter
        now = self.get_clock().now()
        dt = (now - self.last_t).nanoseconds * 1e-9
        self.last_t = now

        stale = (self.last_cmd is None or
                 (now - self.last_cmd).nanoseconds * 1e-9 > gp('cmd_timeout').value)
        v, w = (0.0, 0.0) if stale else (self.v, self.w)
        if stale and self.last_cmd is not None and not self.timeout_warned:
            self.get_logger().warn('no /cmd_vel: motors stopped')
            self.timeout_warned = True

        if not self.connected:          # nothing is driving the wheels: odometry must not move
            v, w = 0.0, 0.0
        # The gyro bias is measured at the start: do not drive until it is done
        # (only when the IMU is actually sending, so a missing IMU never blocks the robot).
        if gp('use_imu').value and self.imu_seen and not self.gyro.ready:
            v, w = 0.0, 0.0
        if (gp('use_imu').value and self.connected and not self.imu_seen
                and not self.imu_missing_warned
                and (now - self.t_start).nanoseconds * 1e-9 > 6.0):
            self.imu_missing_warned = True
            self.get_logger().warn(
                'no IMU data from the Mega: heading comes from commands only'
                + (' and /robot/ready stays False, so the mission will not start by itself '
                   '(set require_imu:=false to allow it)' if gp('require_imu').value else ''))

        if self.cmd_guard.conflict or self.turn_fault:
            v, w = 0.0, 0.0
        self.check_turn(w, dt)
        if self.turn_fault:
            v, w = 0.0, 0.0

        # heartbeat: sent every tick, also when stopped (the Arduino has its own watchdog)
        a, b = twist_to_pwm(v, w, gp('wheel_separation').value, gp('max_speed').value,
                            gp('min_pwm').value, gp('swap_ab').value,
                            gp('invert_a').value, gp('invert_b').value)
        self.send(f'M{a},{b}')

        # odometry: position from commanded speed, heading from the gyro when we have it
        vmax = gp('max_speed').value
        vs = clip(v, -vmax, vmax) * gp('odom_linear_scale').value
        if self.imu_ok():
            yaw_prev = self.yaw
            self.yaw = self.gyro.yaw
            self.x, self.y = integrate_position(self.x, self.y, yaw_prev, self.yaw, vs, dt)
            ws = self.gyro.rate
        else:
            ws = w * gp('odom_angular_scale').value
            self.x, self.y, self.yaw = integrate_pose(self.x, self.y, self.yaw, vs, ws, dt)
            self.gyro.yaw = self.yaw       # resume from here if the IMU comes back
        self.publish_odom(now, vs, ws)

    def check_turn(self, w, dt):
        """Latch a fault when the gyro says the robot turns against the command."""
        if self.turn_fault or not self.get_parameter('turn_check').value or not self.imu_ok():
            return
        result = self.turn_check.update(w, self.gyro.rate, dt)
        if result == 'mismatch':
            self.turn_fault = True
            self.get_logger().error(
                f'TURN DIRECTION ERROR: commanded angular.z={w:+.2f} rad/s but the gyro measures '
                f'{self.gyro.rate:+.2f} rad/s. The heading loop would push the robot further '
                'off course, so the wheels are stopped until this node restarts. Fix: with '
                'teleop, command a LEFT turn (angular.z > 0) and look from above. Robot turns '
                'counter-clockwise -> the gyro sign is wrong: imu_yaw_sign:=-1.0. Robot turns '
                'clockwise -> left/right motors are swapped: swap_ab:=true (or rewire).')
        elif result == 'stall':
            self.get_logger().warn(
                f'angular.z={w:+.2f} rad/s commanded for 2.5 s but the robot does not rotate: '
                'if it DRIVES STRAIGHT instead, one side is reversed (all 4 wheels forward -> '
                'invert_a:=true, all 4 backward -> invert_b:=true). If it stays still, a skid-'
                'steer robot needs more torque to pivot: raise min_pwm, check the battery '
                '(the L298N drops ~2 V).', throttle_duration_sec=10.0)

    def check_graph(self):
        if self.get_parameter('writer_guard').value:
            self.cmd_guard.check('the wheels are stopped until only one is left')
        else:
            self.cmd_guard.conflict = False

    def publish_ready(self):
        gp = self.get_parameter
        ready = robot_ready(self.connected, gp('use_imu').value, gp('require_imu').value,
                            self.imu_seen, self.gyro.ready, self.imu_ok())
        ready = ready and not self.turn_fault
        self.ready_pub.publish(Bool(data=bool(ready)))
        if ready and not self.was_ready:
            self.get_logger().info('robot ready (Mega connected, gyro calibrated)')
        elif self.was_ready and not ready:
            self.get_logger().warn('robot NOT ready (Mega or IMU lost): wheels stopped')
        self.was_ready = ready

    def publish_odom(self, stamp, v, w):
        msg = Odometry()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = w
        self.odom_pub.publish(msg)

    def shutdown(self):
        for cmd in ('X', 'H'):
            self.send(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
