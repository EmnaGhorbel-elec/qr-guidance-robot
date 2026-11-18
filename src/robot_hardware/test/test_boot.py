from robot_hardware.kinematics import pick_port, robot_ready


def test_pick_port_fixed_and_auto():
    assert pick_port('/dev/ttyACM0', []) == '/dev/ttyACM0'
    assert pick_port('auto', []) is None
    assert pick_port('auto', ['/dev/ttyUSB0', '/dev/ttyACM1']) == '/dev/ttyACM1'


def test_not_ready_until_connected_and_gyro_done():
    args = dict(use_imu=True, require_imu=True)
    assert not robot_ready(False, imu_seen=True, gyro_ready=True, imu_fresh=True, **args)
    assert not robot_ready(True, imu_seen=False, gyro_ready=False, imu_fresh=False, **args)
    assert not robot_ready(True, imu_seen=True, gyro_ready=False, imu_fresh=False, **args)
    assert not robot_ready(True, imu_seen=True, gyro_ready=True, imu_fresh=False, **args)  # stalled
    assert robot_ready(True, imu_seen=True, gyro_ready=True, imu_fresh=True, **args)


def test_missing_imu_blocks_unless_allowed():
    assert not robot_ready(True, True, True, False, False, False)
    assert robot_ready(True, True, False, False, False, False)
    # an IMU that was seen and then calibrated-lost never counts as "missing"
    assert not robot_ready(True, True, False, True, True, False)
    assert robot_ready(True, False, True, False, False, False)     # IMU switched off
