import math

from robot_sim.base_logic import GyroHeading, ServoSim


def test_gyro_calibrates_then_integrates_without_bias():
    g = GyroHeading(settle=1.0, calib=1.5)
    t = 0.0
    while not g.ready:                       # still robot, constant bias
        g.feed(t, 0.01)
        t += 0.02
        assert t < 5.0
    assert math.isclose(g.bias, 0.01, abs_tol=1e-9)
    for _ in range(100):                     # turn at 0.5 rad/s for 2 s
        t += 0.02
        g.feed(t, 0.51)
    assert math.isclose(g.yaw, 1.0, abs_tol=0.02)


def test_gyro_recalibrates_if_robot_moves():
    g = GyroHeading(settle=0.0, calib=1.0, max_spread=0.05)
    t = 0.0
    for i in range(60):                      # shaking: spread 0.4 rad/s
        g.feed(t, 0.2 if i % 2 else -0.2)
        t += 0.02
    assert not g.ready


def test_servo_protocol_like_the_mega():
    s = ServoSim()
    assert s.command('G120', 0.0) == 120 and not s.scanning
    assert s.command('G500', 0.0) == 160                 # clamped like the firmware
    s.command('S', 0.0)
    assert s.update(0.3) is None                          # dwell 0.7 s
    assert s.update(0.8) == 160                           # at the top (same as the firmware)
    assert s.update(1.6) == 155                           # then sweeps down
    s.command('H', 2.0)
    assert s.update(5.0) is None
