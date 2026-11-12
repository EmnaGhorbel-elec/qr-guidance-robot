import math

from qr_navigation.mission_logic import slew_value_at


def test_slew_in_radians():
    h = [(0.0, 0.0), (1.0, 1.0)]
    assert math.isclose(slew_value_at(h, 1.1, 6.0), 0.6)
    assert slew_value_at(h, 2.0, 6.0) == 1.0
