import math

from robot_hardware.kinematics import (
    GyroYaw, imu_raw_to_si, integrate_position, pan_offset, parse_imu, track_step)


def test_parse_imu_ok_and_garbage():
    assert parse_imu('I:1000,1,-2,16384,10,-20,655') == (1000, 1, -2, 16384, 10, -20, 655)
    assert parse_imu('I:1000,655') == (1000, 0, 0, 0, 0, 0, 655)       # 9600 baud short line
    for bad in ('A:90', 'I:1,2,3', 'I:a,b,c,d,e,f,g', 'I:', '', 'I:1,2,3,4,5,6,7,8', 'I:1000,x'):
        assert parse_imu(bad) is None


def test_raw_to_si_scale():
    acc, gyr = imu_raw_to_si(0, 0, 16384, 0, 0, 655)
    assert math.isclose(acc[2], 9.80665)
    assert math.isclose(gyr[2], math.radians(10.0))


def _feed(g, n, gz, start_ms=0, period=20):
    ms = start_ms
    for _ in range(n):
        g.update(ms, gz)
        ms += period
    return ms


def test_gyro_calibrates_then_integrates_quarter_turn():
    g = GyroYaw(calib_samples=50)
    ms = _feed(g, 50, 0.02)                       # bias 0.02 rad/s while still
    assert g.ready and math.isclose(g.bias, 0.02)
    ms = _feed(g, 100, 0.02 + 0.5, start_ms=ms)   # 2 s at 0.5 rad/s
    assert math.isclose(g.yaw, 1.0, rel_tol=0.03)


def test_gyro_deadband_stops_drift_when_still():
    g = GyroYaw(calib_samples=10)
    ms = _feed(g, 10, 0.0)
    _feed(g, 500, 0.002, start_ms=ms)             # tiny noise under the deadband
    assert g.yaw == 0.0


def test_gyro_restarts_calibration_if_robot_moves():
    g = GyroYaw(calib_samples=10)
    ms = _feed(g, 5, 0.0)
    _feed(g, 1, 1.0, start_ms=ms)                 # a spike: someone moved it
    assert not g.ready and g.restarts == 1


def test_gyro_millisecond_wraparound_and_gap():
    g = GyroYaw(calib_samples=3)
    _feed(g, 3, 0.0, start_ms=0xFFFFFFFF - 40)
    g.update((0xFFFFFFFF - 40 + 60) & 0xFFFFFFFF, 1.0)   # wraps, dt = 20 ms
    assert math.isclose(g.yaw, 0.02, rel_tol=0.01)
    g.update(5000, 1.0)                                  # 5 s gap: ignored
    assert math.isclose(g.yaw, 0.02, rel_tol=0.01)


def test_integrate_position_follows_mid_heading():
    x, y = integrate_position(0.0, 0.0, 0.0, math.pi / 2, 1.0, 1.0)
    assert math.isclose(x, math.cos(math.pi / 4)) and math.isclose(y, math.sin(math.pi / 4))


def test_track_step_directions_and_limits():
    fov = 1.047
    assert track_step(320, 640, fov, 90) == 90               # centred: stay
    assert track_step(100, 640, fov, 90) > 90                # QR on the left: pan left (+)
    assert track_step(100, 640, fov, 90, servo_sign=-1) < 90
    assert track_step(600, 640, fov, 90) < 90                # QR on the right
    assert track_step(0, 640, fov, 158) <= 160               # clamped to the mechanical range
    assert abs(track_step(0, 640, fov, 90) - 90) <= 8        # max step per update


def test_pan_offset_sign():
    assert math.isclose(pan_offset(120), math.radians(30))
    assert math.isclose(pan_offset(120, servo_sign=-1), math.radians(-30))
    assert pan_offset(90) == 0.0


# ---------------------------------------------------------------- real-robot safety checks
from robot_hardware.kinematics import TurnCheck, value_at  # noqa: E402


def _run(check, w_cmd, rate, seconds, dt=0.05):
    out = 'ok'
    for _ in range(int(seconds / dt)):
        r = check.update(w_cmd, rate, dt)
        out = r if r != 'ok' else out
    return out


def test_turn_check_flags_opposite_rotation():
    assert _run(TurnCheck(), 0.8, -0.8, 1.0) == 'mismatch'      # gyro sign or motor sides wrong
    assert _run(TurnCheck(), 0.8, 0.7, 5.0) == 'ok'             # turning the right way
    assert _run(TurnCheck(), 0.8, -0.8, 0.3) == 'ok'            # short transient: tolerated
    assert _run(TurnCheck(), 0.1, -0.8, 5.0) == 'ok'            # tiny command: not judged


def test_turn_check_reports_stall():
    assert _run(TurnCheck(), 0.8, 0.0, 3.0) == 'stall'          # min_pwm too low to pivot


def test_value_at():
    h = [(0.0, 90), (1.0, 95)]
    assert value_at(h, 0.5) == 90 and value_at(h, 1.0) == 95 and value_at([], 0) is None
