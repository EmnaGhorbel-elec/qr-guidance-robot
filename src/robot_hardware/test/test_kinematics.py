import math

from robot_hardware.kinematics import (
    integrate_pose, parse_servo_angle, speed_to_pwm, twist_to_pwm,
    twist_to_wheel_speeds)

SEP, VMAX, MINPWM = 0.147, 0.35, 60


def test_straight_gives_equal_wheels():
    l, r = twist_to_wheel_speeds(0.2, 0.0, SEP)
    assert l == r == 0.2


def test_turn_left_in_place_right_wheel_faster():
    l, r = twist_to_wheel_speeds(0.0, 1.0, SEP)
    assert l < 0 < r and math.isclose(l, -r)


def test_zero_and_tiny_speed_stop():
    assert speed_to_pwm(0.0, VMAX, MINPWM) == 0
    assert speed_to_pwm(0.004, VMAX, MINPWM) == 0


def test_small_speed_is_lifted_to_min_pwm_and_full_speed_saturates():
    assert speed_to_pwm(0.01, VMAX, MINPWM) >= MINPWM
    assert speed_to_pwm(VMAX, VMAX, MINPWM) == 255
    assert speed_to_pwm(5.0, VMAX, MINPWM) == 255
    assert speed_to_pwm(-5.0, VMAX, MINPWM) == -255


def test_pwm_is_monotonic():
    values = [speed_to_pwm(s / 100.0, VMAX, MINPWM) for s in range(1, 36)]
    assert values == sorted(values)


def test_forward_both_positive_and_inversion_swap():
    a, b = twist_to_pwm(0.2, 0.0, SEP, VMAX, MINPWM)
    assert a == b > 0
    a2, b2 = twist_to_pwm(0.2, 0.0, SEP, VMAX, MINPWM, invert_a=True)
    assert a2 == -a and b2 == b
    # spin left: left wheel back, right wheel forward; swap_ab exchanges the motors
    a3, b3 = twist_to_pwm(0.0, 1.0, SEP, VMAX, MINPWM)
    a4, b4 = twist_to_pwm(0.0, 1.0, SEP, VMAX, MINPWM, swap_ab=True)
    assert a3 < 0 < b3 and (a4, b4) == (b3, a3)


def test_integrate_straight_one_metre():
    x = y = yaw = 0.0
    for _ in range(100):
        x, y, yaw = integrate_pose(x, y, yaw, 0.2, 0.0, 0.05)
    assert math.isclose(x, 1.0, abs_tol=1e-9) and abs(y) < 1e-9


def test_integrate_quarter_turn_in_place():
    x = y = yaw = 0.0
    for _ in range(100):
        x, y, yaw = integrate_pose(x, y, yaw, 0.0, math.pi / 2, 0.01)
    assert math.isclose(yaw, math.pi / 2, abs_tol=1e-9) and abs(x) < 1e-9


def test_parse_servo_angle_never_raises():
    assert parse_servo_angle('A:95') == 95
    assert parse_servo_angle('A:') is None
    assert parse_servo_angle('OK:SCAN') is None
    assert parse_servo_angle('A:9x') is None
