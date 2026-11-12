import math

from qr_guidance_robot.scan_logic import slew_value_at, track_step, value_at


def test_slew_model_reaches_target_at_servo_speed():
    h = [(0.0, 90.0), (1.0, 130.0)]           # command 40 deg away at t=1 s
    assert slew_value_at(h, 0.5, 400.0) == 90.0
    assert math.isclose(slew_value_at(h, 1.05, 400.0), 110.0)
    assert slew_value_at(h, 1.2, 400.0) == 130.0
    assert slew_value_at([], 1.0, 400.0) is None


def test_track_step_and_value_at():
    assert track_step(320, 640, 1.047, 90) == 90           # centred
    assert track_step(100, 640, 1.047, 90) > 90             # QR on the left: pan left
    assert value_at([(0.0, 1), (1.0, 2)], 0.5) == 1
