"""Pure helpers of scan_controller_node (same as robot_hardware.kinematics on the robot)."""
import math


def clip(x, lo, hi):
    return max(lo, min(hi, x))


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


def value_at(history, t):
    """Latest value of a [(t, value), ...] history (oldest first) recorded at or before t."""
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
