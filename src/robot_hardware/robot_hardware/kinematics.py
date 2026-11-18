"""Pure helpers (no ROS, no serial): easy to test on their own.

Convention (same as motor_node): v = forward m/s, w = rad/s, positive w = turn left.
"""
import math


def clip(x, lo, hi):
    return max(lo, min(hi, x))


def twist_to_wheel_speeds(v, w, wheel_separation):
    """Body velocity -> (left, right) wheel linear speeds in m/s."""
    return (v - w * wheel_separation / 2.0,
            v + w * wheel_separation / 2.0)


def speed_to_pwm(speed, max_speed, min_pwm):
    """Wheel speed (m/s) -> signed PWM -255..255.

    DC motors do not turn below a certain PWM (stiction), so any non-zero request
    is lifted to `min_pwm`; the rest of the range is spread linearly up to 255.
    """
    if abs(speed) < 0.005:
        return 0
    frac = min(abs(speed) / max_speed, 1.0)
    pwm = min_pwm + (255 - min_pwm) * frac
    return int(round(math.copysign(pwm, speed)))


def twist_to_pwm(v, w, wheel_separation, max_speed, min_pwm,
                 swap_ab=False, invert_a=False, invert_b=False):
    """Body velocity -> (pwm_a, pwm_b) for the Arduino.

    Default: motor A = left side, motor B = right side.
    """
    left, right = twist_to_wheel_speeds(v, w, wheel_separation)
    pwm_l = speed_to_pwm(left, max_speed, min_pwm)
    pwm_r = speed_to_pwm(right, max_speed, min_pwm)
    a, b = (pwm_r, pwm_l) if swap_ab else (pwm_l, pwm_r)
    return (-a if invert_a else a), (-b if invert_b else b)


def integrate_pose(x, y, yaw, v, w, dt):
    """Dead reckoning step (midpoint rule). Returns the new (x, y, yaw)."""
    mid = yaw + w * dt / 2.0
    return (x + v * math.cos(mid) * dt,
            y + v * math.sin(mid) * dt,
            math.atan2(math.sin(yaw + w * dt), math.cos(yaw + w * dt)))


def parse_servo_angle(line):
    """'A:95' -> 95, anything else -> None (never raises)."""
    if not line.startswith('A:'):
        return None
    try:
        return int(line[2:])
    except ValueError:
        return None


# ---------------------------------------------------------------- IMU (MPU-6050)
ACCEL_LSB_PER_G = 16384.0       # +-2 g range (firmware sets it)
GYRO_LSB_PER_DPS = 65.5         # +-500 deg/s range (firmware sets it)
G = 9.80665


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def parse_imu(line):
    """'I:<ms>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>' -> (ms, ax, ay, az, gx, gy, gz) ints.

    Also accepts the short line the firmware sends at 9600 baud, 'I:<ms>,<gz>', which
    returns (ms, 0, 0, 0, 0, 0, gz). Raw sensor counts. Anything malformed -> None.
    """
    if not line.startswith('I:'):
        return None
    parts = line[2:].split(',')
    if len(parts) not in (2, 7):
        return None
    try:
        v = tuple(int(p) for p in parts)
    except ValueError:
        return None
    return (v[0], 0, 0, 0, 0, 0, v[1]) if len(v) == 2 else v


def imu_raw_to_si(ax, ay, az, gx, gy, gz):
    """Raw counts -> ((ax, ay, az) in m/s^2, (gx, gy, gz) in rad/s)."""
    acc = tuple(v / ACCEL_LSB_PER_G * G for v in (ax, ay, az))
    gyr = tuple(math.radians(v / GYRO_LSB_PER_DPS) for v in (gx, gy, gz))
    return acc, gyr


class GyroYaw:
    """Heading from the gyro Z axis.

    The first `calib_samples` readings (robot standing still) give the gyro bias.
    If the robot is moving during that time (spread above `max_spread` rad/s) the
    calibration restarts. After that: yaw += (gz - bias) * dt, with dt taken from the
    Mega's own millisecond timestamp (serial reads arrive in bunches, so the Pi's
    clock would give a wrong dt).
    """

    def __init__(self, calib_samples=100, deadband=0.004, max_spread=0.1):
        self.calib_samples = calib_samples
        self.deadband = deadband
        self.max_spread = max_spread
        self.reset()

    def reset(self):
        self.samples = []
        self.bias = 0.0
        self.ready = False
        self.yaw = 0.0
        self.rate = 0.0
        self.last_ms = None
        self.restarts = 0

    def update(self, ms, gz):
        """gz in rad/s (sign already applied). Returns True when `ready`."""
        if not self.ready:
            self.samples.append(gz)
            if max(self.samples) - min(self.samples) > self.max_spread:
                self.samples = [gz]
                self.restarts += 1
            elif len(self.samples) >= self.calib_samples:
                self.bias = sum(self.samples) / len(self.samples)
                self.ready = True
                self.last_ms = ms
            return self.ready
        dt = ((ms - self.last_ms) & 0xFFFFFFFF) / 1000.0
        self.last_ms = ms
        if dt <= 0.0 or dt > 0.5:          # lost samples: skip, do not integrate a gap
            return True
        w = gz - self.bias
        if abs(w) < self.deadband:
            w = 0.0
        self.rate = w
        self.yaw = wrap_angle(self.yaw + w * dt)
        return True


def integrate_position(x, y, yaw_start, yaw_end, v, dt):
    """Advance x, y along a heading that went from yaw_start to yaw_end (gyro)."""
    mid = yaw_start + wrap_angle(yaw_end - yaw_start) / 2.0
    return x + v * math.cos(mid) * dt, y + v * math.sin(mid) * dt


# ---------------------------------------------------------------- servo tracking
def track_step(cx, img_w, hfov, angle, servo_sign=1.0, gain=0.6, deadband_deg=1.5,
               max_step_deg=8.0, lo=20, hi=160):
    """New servo angle (int degrees) that moves the QR toward the image centre.

    servo_sign = +1 when a higher angle pans the camera to the LEFT (counter-clockwise
    seen from above), -1 otherwise. Returns the same angle when already centred.
    """
    fx = (img_w / 2.0) / math.tan(hfov / 2.0)
    bearing_deg = math.degrees(-math.atan2(cx - img_w / 2.0, fx))   # QR left of centre > 0
    if abs(bearing_deg) <= deadband_deg:
        return int(angle)
    step = clip(servo_sign * bearing_deg * gain, -max_step_deg, max_step_deg)
    return int(round(clip(angle + step, lo, hi)))


def pan_offset(angle, center=90, servo_sign=1.0):
    """Camera pan relative to the robot's forward axis, rad, left positive."""
    return math.radians(servo_sign * (angle - center))


# ---------------------------------------------------------------- power-on helpers
def pick_port(port, candidates):
    """Serial port to open. 'auto' -> first of `candidates` (sorted), else `port` itself.

    Returns None when 'auto' finds nothing (the Mega is not enumerated yet at boot).
    """
    if port != 'auto':
        return port
    found = sorted(candidates)
    return found[0] if found else None


def robot_ready(connected, use_imu, require_imu, imu_seen, gyro_ready, imu_fresh):
    """True when the robot may start its mission by itself.

    connected: serial open and the Mega finished booting.
    The gyro must be calibrated and still streaming, unless the IMU is switched off
    (use_imu False) or not required (require_imu False and no sample ever arrived).
    """
    if not connected:
        return False
    if not use_imu:
        return True
    if gyro_ready and imu_fresh:
        return True
    return (not require_imu) and (not imu_seen)


# ---------------------------------------------------------------- turn direction check
class TurnCheck:
    """Compares the commanded turn rate with the gyro, to catch sign errors on real hardware.

    The closed loop (autonomy_node steers on the gyro heading) only works if a positive
    angular.z really turns the robot to the LEFT (counter-clockwise seen from above) AND the
    gyro reports that as positive. If one of the two is inverted (left/right motors swapped,
    IMU mounted upside down), every heading correction makes the error bigger: the robot
    spins or wanders "randomly" although each part works on its own.

    update() returns:
      'mismatch' : the robot turned the opposite way for `mismatch_time` s (latch and stop)
      'stall'    : a turn was commanded for `stall_time` s but the robot did not rotate
                   (skid steer needs more torque to turn in place: min_pwm too low)
      'ok'       : otherwise
    """

    def __init__(self, min_cmd=0.3, min_rate=0.15, mismatch_time=0.6,
                 stall_rate=0.05, stall_time=2.5):
        self.min_cmd, self.min_rate, self.mismatch_time = min_cmd, min_rate, mismatch_time
        self.stall_rate, self.stall_time = stall_rate, stall_time
        self.t_mismatch = 0.0
        self.t_stall = 0.0

    def update(self, w_cmd, rate, dt):
        if abs(w_cmd) < self.min_cmd or dt <= 0.0 or dt > 0.5:
            self.t_mismatch = self.t_stall = 0.0
            return 'ok'
        opposite = w_cmd * rate < 0.0 and abs(rate) >= self.min_rate
        self.t_mismatch = self.t_mismatch + dt if opposite else 0.0
        self.t_stall = self.t_stall + dt if abs(rate) < self.stall_rate else 0.0
        if self.t_mismatch >= self.mismatch_time:
            return 'mismatch'
        if self.t_stall >= self.stall_time:
            self.t_stall = 0.0                  # report again only after another full period
            return 'stall'
        return 'ok'


def value_at(history, t):
    """Latest value of a [(t, value), ...] history (oldest first) recorded at or before t.

    Falls back to the oldest entry when t is older than the whole history, None if empty.
    """
    if not history:
        return None
    best = history[0][1]
    for ti, v in history:
        if ti > t:
            break
        best = v
    return best


def slew_value_at(history, t, speed):
    """Position at time t of a servo that moves toward each commanded value at `speed`/s.

    history: [(t, commanded value), ...] oldest first (a hobby servo reaches a new angle
    after |change| / speed s, not instantly and not after a fixed delay).
    """
    if not history:
        return None
    t0, pos = history[0]
    target = pos
    for ti, v in history:
        if ti > t:
            break
        dt = max(0.0, ti - t0)
        pos += clip(target - pos, -speed * dt, speed * dt)
        t0, target = ti, v
    dt = max(0.0, t - t0)
    return pos + clip(target - pos, -speed * dt, speed * dt)


# ---------------------------------------------------------------- servo sweep
def sweep_angles(center=90, extent=90, step=10, servo_sign=1.0, lo=0, hi=180):
    """Servo angles (int degrees) of one search sweep, in the order they are visited.

    Starts at the centre (camera straight ahead), goes `extent` deg to the RIGHT,
    then `extent` deg to the LEFT, then back to the centre.
    servo_sign = +1 when a higher angle pans the camera LEFT, so right = centre - extent.
    The list does not repeat the starting centre. Angles are clipped to [lo, hi].
    """
    right = int(clip(center - servo_sign * extent, lo, hi))
    left = int(clip(center + servo_sign * extent, lo, hi))
    step = max(1, int(step))

    def run(a, b):
        n = max(1, int(math.ceil(abs(b - a) / step)))
        return [int(round(a + (b - a) * i / n)) for i in range(1, n + 1)]

    return run(center, right) + run(right, left) + run(left, center)
