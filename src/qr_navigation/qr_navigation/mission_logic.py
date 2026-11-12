"""Pure mission logic (no ROS): leg planning and world<->odom bookkeeping.

World frame = frame of the QR table / Gazebo world. odom = wheel odometry frame.
A transform is (tx, ty, dyaw): pose of the odom frame expressed in the world frame
(same convention as robot_control.frames).
"""
import math

from robot_control.frames import estimate_world_to_odom, odom_to_world, wrap


def build_legs(table, order, approach_distance):
    """Turn a list of QR ids into driving legs, in the world frame.

    table[id] = {x, y, theta} is the CHECKPOINT pose: where the robot stands when it
    reads that QR (looking at it). For each QR, one 'approach' leg is placed
    `approach_distance` metres behind the checkpoint along its heading, so the robot
    arrives on the checkpoint already facing the QR, then one 'checkpoint' leg.
    """
    legs = []
    for qr_id in order:
        if qr_id not in table:
            raise KeyError(f'QR "{qr_id}" is in the mission but not in the QR table')
        e = table[qr_id]
        x, y, th = float(e['x']), float(e['y']), float(e['theta'])
        if approach_distance > 0.0:
            legs.append({'qr': qr_id, 'kind': 'approach',
                         'x': x - approach_distance * math.cos(th),
                         'y': y - approach_distance * math.sin(th)})
        legs.append({'qr': qr_id, 'kind': 'checkpoint', 'x': x, 'y': y})
    return legs


def fix_matches(fix_xy, expected_xy, tol=0.01):
    """The localization fix must be the one of the QR we are standing at."""
    return math.hypot(fix_xy[0] - expected_xy[0], fix_xy[1] - expected_xy[1]) <= tol


def update_transform(old_tf, fix_pose, odom_pose, correct_yaw=False):
    """New world->odom transform after a QR fix.

    fix_pose  : (x, y, yaw) of the robot in the world, from the QR table.
    odom_pose : (x, y, yaw) of the robot in odom, at the same instant.
    correct_yaw=False keeps the previous rotation and only fixes the translation
    (the robot is not guaranteed to be exactly aligned when it reads the QR).
    Returns (new_tf, drift_m, drift_deg): the position/heading error that was removed.
    """
    believed = odom_to_world((odom_pose[0], odom_pose[1]), old_tf)
    believed_yaw = wrap(odom_pose[2] + old_tf[2])
    drift_m = math.hypot(fix_pose[0] - believed[0], fix_pose[1] - believed[1])
    drift_deg = math.degrees(wrap(fix_pose[2] - believed_yaw))
    if correct_yaw:
        return estimate_world_to_odom(fix_pose, odom_pose), drift_m, drift_deg
    dth = old_tf[2]
    c, s = math.cos(dth), math.sin(dth)
    tx = fix_pose[0] - (c * odom_pose[0] - s * odom_pose[1])
    ty = fix_pose[1] - (s * odom_pose[0] + c * odom_pose[1])
    return (tx, ty, dth), drift_m, drift_deg


def servo_goal(odom_pose, cx, img_w, w_px, k, fov, camera_offset, standoff, min_step=0.15,
               pan=0.0):
    """Visual-servo goal toward a QR, in the odom frame.

    bearing  : angle of the QR relative to the robot heading (left positive),
               from the pixel offset of its centre (pinhole camera, horizontal fov in rad)
               plus `pan`, the camera servo angle relative to straight ahead (rad, left
               positive; 0 for a fixed camera)
    distance : base-to-QR distance ~ k / w_px + camera_offset
    Returns (gx, gy, bearing, remaining) where remaining = distance - standoff.
    The goal is placed on the line of sight, at least `min_step` ahead so the
    controller keeps steering while the bearing error is being removed.
    """
    bearing = pixel_bearing(cx, img_w, fov) + pan
    distance = k / w_px + camera_offset
    remaining = distance - standoff
    # near or beyond the stand-off: keep at least min_step (the servo stops on its own at the
    # stand-off); clearly too close (> 0.10 m inside): never push forward, tiny step to steer only
    r = max(remaining, min_step) if remaining > -0.10 else 0.05
    heading = odom_pose[2] + bearing
    return (odom_pose[0] + r * math.cos(heading),
            odom_pose[1] + r * math.sin(heading), bearing, remaining)


def calibrate_k(widths, base_dist, camera_offset):
    """Size constant k (distance ~ k / w_px) from QR widths seen at a KNOWN base distance.

    Uses the median width so a bad frame does not matter. Returns None without samples.
    """
    ws = sorted(w for w in widths if w and w > 0.0)
    if not ws:
        return None
    return ws[len(ws) // 2] * (base_dist - camera_offset)


def pixel_bearing(cx, img_w, fov):
    """Angle of a pixel column from the camera axis (pinhole, rad, left positive)."""
    fx = (img_w / 2.0) / math.tan(fov / 2.0)
    return -math.atan2(cx - img_w / 2.0, fx)


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


def pose_at(history, t):
    """Odometry pose (x, y, yaw) at time t, linearly interpolated in a [(t, pose)] history.

    The QR detector needs 50-300 ms per frame on the Pi: a goal or a fix computed from a
    detection must use where the robot was when the frame was taken, not where it is now.
    Clamped to the ends of the history. None if the history is empty.
    """
    if not history:
        return None
    prev = None
    for t1, p1 in history:                 # works on a list or a deque (no slicing)
        if t <= t1:
            if prev is None:
                return p1
            t0, p0 = prev
            a = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
            yaw = wrap(p0[2] + a * wrap(p1[2] - p0[2]))
            return (p0[0] + a * (p1[0] - p0[0]), p0[1] + a * (p1[1] - p0[1]), yaw)
        prev = (t1, p1)
    return prev[1]


def qr_world_xy(checkpoint_x, checkpoint_y, theta, standoff):
    """QR position from its table entry (checkpoint = QR - standoff along theta)."""
    return (checkpoint_x + standoff * math.cos(theta),
            checkpoint_y + standoff * math.sin(theta))


def measured_robot_xy(qr_xy, heading, bearing_cam, cam_dist, camera_offset):
    """Robot (base_link) position in the world from ONE sighting of a QR of known position.

    heading     : robot heading in the world (gyro heading + world->odom rotation), rad
    bearing_cam : direction of the QR seen from the camera, relative to the robot heading
                  (pixel bearing + camera pan), rad, left positive
    cam_dist    : camera-to-QR distance (k / w_px), m
    camera_offset: the camera (pan axis) is this far ahead of base_link, m

    This is a real measurement: unlike the table pose it also captures a sideways offset
    when the robot did not arrive exactly on the QR axis.
    """
    a = heading + bearing_cam
    cam_x = qr_xy[0] - cam_dist * math.cos(a)
    cam_y = qr_xy[1] - cam_dist * math.sin(a)
    return (cam_x - camera_offset * math.cos(heading),
            cam_y - camera_offset * math.sin(heading))


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
        pos += max(-speed * dt, min(speed * dt, target - pos))
        t0, target = ti, v
    dt = max(0.0, t - t0)
    return pos + max(-speed * dt, min(speed * dt, target - pos))
