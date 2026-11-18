from robot_hardware.kinematics import sweep_angles


def test_sweep_goes_right_then_left_then_centre():
    a = sweep_angles(90, 90, 10, 1.0)
    assert a[0] == 80 and a[-1] == 90
    assert min(a) == 0 and max(a) == 180
    assert a.index(0) < a.index(180)            # right end first (servo_sign +1: right = lower angle)
    assert all(abs(b - c) <= 10 for b, c in zip(a, a[1:]))


def test_sweep_mirrored_for_negative_sign():
    a = sweep_angles(90, 90, 10, -1.0)
    assert a[0] == 100 and a.index(180) < a.index(0)


def test_sweep_clipped_and_odd_step():
    a = sweep_angles(90, 90, 25, 1.0, lo=10, hi=170)
    assert min(a) == 10 and max(a) == 170 and a[-1] == 90
