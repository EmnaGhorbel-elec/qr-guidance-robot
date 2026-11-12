import math

from qr_navigation.mission_logic import (build_legs, calibrate_k, fix_matches, servo_goal,
                                         update_transform)
from qr_navigation.qr_utils import close_enough, parse_detection, range_corrected_xy
from robot_control.frames import estimate_world_to_odom, odom_to_world

TABLE = {'A': {'x': 0.7, 'y': 0.0, 'theta': 0.0},
         'B': {'x': 2.2, 'y': 0.8, 'theta': math.pi / 2}}


def test_legs_order_and_approach_point():
    legs = build_legs(TABLE, ['A', 'B'], 0.5)
    assert [(l['qr'], l['kind']) for l in legs] == [
        ('A', 'approach'), ('A', 'checkpoint'), ('B', 'approach'), ('B', 'checkpoint')]
    assert math.isclose(legs[0]['x'], 0.2) and math.isclose(legs[0]['y'], 0.0)
    # B faces +y, so its approach point is 0.5 m behind it, on -y
    assert math.isclose(legs[2]['x'], 2.2, abs_tol=1e-9) and math.isclose(legs[2]['y'], 0.3)


def test_no_approach_leg_when_distance_zero():
    assert [l['kind'] for l in build_legs(TABLE, ['A'], 0.0)] == ['checkpoint']


def test_unknown_qr_raises():
    try:
        build_legs(TABLE, ['Z'], 0.5)
        assert False
    except KeyError:
        pass


def test_fix_matches():
    assert fix_matches((0.7, 0.0), (0.7, 0.0))
    assert not fix_matches((2.2, 0.8), (0.7, 0.0))


def test_update_transform_removes_drift_translation_only():
    tf = (0.0, 0.0, 0.0)                    # robot started at the world origin
    odom = (0.75, 0.05, 0.1)                # odometry says...
    fix = (0.7, 0.0, 0.0)                   # ...QR says the robot is at the checkpoint
    new_tf, drift, _ = update_transform(tf, fix, odom, correct_yaw=False)
    assert math.isclose(drift, math.hypot(0.05, 0.05))
    assert new_tf[2] == tf[2]               # rotation kept
    wx, wy = odom_to_world((odom[0], odom[1]), new_tf)
    assert math.isclose(wx, 0.7, abs_tol=1e-9) and math.isclose(wy, 0.0, abs_tol=1e-9)


def test_update_transform_with_yaw_correction_matches_frames():
    old = estimate_world_to_odom((0, 0, 0), (0, 0, 0))
    new_tf, _, deg = update_transform(old, (0.7, 0.0, 0.0), (0.75, 0.05, 0.1), correct_yaw=True)
    assert math.isclose(deg, math.degrees(-0.1))
    assert math.isclose(new_tf[2], -0.1)


def test_parse_detection_json_and_bare():
    assert parse_detection('{"id": "WAYPOINT_1", "w_px": 246.0}') == ('WAYPOINT_1', 246.0)
    assert parse_detection('{"id": "WAYPOINT_1"}') == ('WAYPOINT_1', None)
    assert parse_detection(' WAYPOINT_2 ') == ('WAYPOINT_2', None)
    assert parse_detection('{broken') == ('', None)
    assert parse_detection('') == ('', None)


def test_close_enough_gate():
    assert close_enough(250.0, 200.0)
    assert not close_enough(80.0, 200.0)
    assert close_enough(80.0, 0.0)          # gate off
    assert close_enough(None, 200.0)        # unknown width (bare id): not gated


def test_range_correction_nominal_and_shifts():
    k, off, so = 160.3, 0.15, 0.8
    w_nominal = k / (so - off)                      # QR looks exactly as at the standoff
    assert range_corrected_xy(0.7, 0.0, 0.0, w_nominal, k, off, so, 0.25) == (0.7, 0.0) or \
        math.isclose(range_corrected_xy(0.7, 0.0, 0.0, w_nominal, k, off, so, 0.25)[0], 0.7)
    x, _ = range_corrected_xy(0.7, 0.0, 0.0, w_nominal * 0.9, k, off, so, 0.25)
    assert x < 0.7                                   # smaller QR -> robot is further back
    x, _ = range_corrected_xy(0.7, 0.0, 0.0, w_nominal * 1.1, k, off, so, 0.25)
    assert x > 0.7                                   # bigger QR -> robot is further forward


def test_range_correction_clamped_and_disabled():
    x, _ = range_corrected_xy(0.7, 0.0, 0.0, 1.0, 160.3, 0.15, 0.8, 0.25)   # absurd width
    assert math.isclose(x, 0.7 - 0.25)
    assert range_corrected_xy(0.7, 0.0, 0.0, 250.0, 0.0, 0.15, 0.8, 0.25) == (0.7, 0.0)
    assert range_corrected_xy(0.7, 0.0, 0.0, None, 160.3, 0.15, 0.8, 0.25) == (0.7, 0.0)


def test_servo_goal_centred_and_offset():
    from qr_navigation.mission_logic import servo_goal
    # QR centred, at nominal stand-off: bearing 0, remaining ~0, goal min_step ahead
    w_nom = 160.3 / (0.8 - 0.15)
    gx, gy, b, rem = servo_goal((1.0, 2.0, 0.0), 320.0, 640.0, w_nom, 160.3, 1.047, 0.15, 0.8)
    assert abs(b) < 1e-9 and abs(rem) < 1e-9
    assert math.isclose(gx, 1.15) and math.isclose(gy, 2.0)
    # QR on the right half of the image -> robot must turn right (negative bearing)
    _, _, b, _ = servo_goal((0, 0, 0.0), 500.0, 640.0, w_nom, 160.3, 1.047, 0.15, 0.8)
    assert b < 0
    # QR small (far): goal is `remaining` ahead along the line of sight
    _, _, _, rem = servo_goal((0, 0, 0.0), 320.0, 640.0, 100.0, 160.3, 1.047, 0.15, 0.8)
    assert math.isclose(rem, 160.3 / 100.0 + 0.15 - 0.8)


def test_parse_detection_full():
    from qr_navigation.qr_utils import parse_detection_full
    d = parse_detection_full('{"id": "W", "cx": 300, "cy": 200, "img_w": 640, "img_h": 480, "w_px": 120.5}')
    assert d == {'id': 'W', 'cx': 300.0, 'img_w': 640.0, 'w_px': 120.5, 'stamp': None}
    d = parse_detection_full('{"id": "W", "cx": 1, "img_w": 640, "w_px": 2, "stamp": 12.5}')
    assert d['stamp'] == 12.5
    assert parse_detection_full('W') is None
    assert parse_detection_full('{"id": "W"}') is None


def test_servo_goal_never_pushes_forward_when_too_close():
    # base distance = 160/300 + 0.15 = 0.68 m < standoff 0.8 -> remaining < 0
    gx, gy, bearing, remaining = servo_goal((0.0, 0.0, 0.0), 320, 640, 300.0, 160.0, 1.047, 0.15, 0.8)
    assert remaining < 0
    assert math.hypot(gx, gy) <= 0.05 + 1e-9          # tiny step: steer only, no advance


def test_servo_goal_far_moves_at_least_min_step():
    gx, gy, _, remaining = servo_goal((0.0, 0.0, 0.0), 320, 640, 100.0, 160.0, 1.047, 0.15, 0.8)
    assert remaining > 0.15 and math.isclose(math.hypot(gx, gy), remaining)


def test_calibrate_k_median_and_none():
    # robot 1.3 m from the QR, camera 0.15 m ahead of base -> camera distance 1.15 m
    assert math.isclose(calibrate_k([138.0, 139.0, 500.0, 0.0], 1.3, 0.15), 139.0 * 1.15)
    assert calibrate_k([], 1.3, 0.15) is None
    assert calibrate_k([0.0, None], 1.3, 0.15) is None


def test_servo_goal_adds_camera_pan_to_bearing():
    # QR centred in the image but the camera is panned 30 deg left: the QR is 30 deg left of the robot
    _, _, b, _ = servo_goal((0, 0, 0.0), 320.0, 640.0, 100.0, 160.3, 1.047, 0.15, 0.8,
                            pan=math.radians(30))
    assert abs(b - math.radians(30)) < 1e-9
    # default (fixed camera) is unchanged
    _, _, b0, _ = servo_goal((0, 0, 0.0), 320.0, 640.0, 100.0, 160.3, 1.047, 0.15, 0.8)
    assert abs(b0) < 1e-9


# ---------------------------------------------------------------- fixes added on the real robot
from qr_navigation.mission_logic import (measured_robot_xy, pixel_bearing, pose_at,  # noqa: E402
                                         qr_world_xy, value_at)


def test_parse_detection_unknown_width_is_none():
    assert parse_detection('{"id": "W", "w_px": -1}') == ('W', None)


def test_pose_at_interpolates_and_clamps():
    from collections import deque
    h = deque([(0.0, (0.0, 0.0, 0.0)), (1.0, (1.0, 0.0, 0.4))])
    x, y, yaw = pose_at(h, 0.5)
    assert math.isclose(x, 0.5) and math.isclose(yaw, 0.2)
    assert pose_at(h, -5.0) == h[0][1] and pose_at(h, 9.0) == h[-1][1]
    assert pose_at([], 1.0) is None
    # yaw interpolated the short way across +-pi
    h2 = [(0.0, (0, 0, math.pi - 0.1)), (1.0, (0, 0, -math.pi + 0.1))]
    assert abs(abs(pose_at(h2, 0.5)[2]) - math.pi) < 1e-9


def test_value_at_takes_latest_before():
    h = [(0.0, 'a'), (1.0, 'b'), (2.0, 'c')]
    assert value_at(h, 1.5) == 'b' and value_at(h, 2.0) == 'c' and value_at(h, -1) == 'a'
    assert value_at([], 0.0) is None


def test_measured_fix_recovers_true_position():
    """Simulate a sighting from a known robot pose and check the pose comes back."""
    qr = qr_world_xy(0.7, 0.0, 0.0, 0.8)                 # QR at (1.5, 0)
    assert qr == (1.5, 0.0)
    robot, heading, off = (0.65, 0.25, 0.2), 0.2, 0.10  # beside the axis, turned 0.2 rad
    cam = (robot[0] + off * math.cos(heading), robot[1] + off * math.sin(heading))
    d = math.hypot(qr[0] - cam[0], qr[1] - cam[1])
    bearing_cam = math.atan2(qr[1] - cam[1], qr[0] - cam[0]) - heading
    x, y = measured_robot_xy(qr, heading, bearing_cam, d, off)
    assert math.isclose(x, robot[0], abs_tol=1e-9) and math.isclose(y, robot[1], abs_tol=1e-9)


def test_pixel_bearing_matches_servo_goal():
    assert pixel_bearing(320, 640, 1.047) == 0.0
    assert pixel_bearing(600, 640, 1.047) < 0 < pixel_bearing(40, 640, 1.047)
    _, _, b, _ = servo_goal((0, 0, 0), 600, 640, 100.0, 80.0, 1.047, 0.1, 0.8, pan=0.3)
    assert math.isclose(b, pixel_bearing(600, 640, 1.047) + 0.3)
