"""
Tests: forward limit based on the ultrasonic sensor.
"""

from mobile_base.speed_limits import GuardParams, limit_forward

P = GuardParams(slow_distance_m=0.30, stop_distance_m=0.15, slow_factor=0.5)


def test_free_path_keeps_speed():
    assert limit_forward(0.1, 1.0, P) == 0.1


def test_half_speed_in_slow_zone():
    assert limit_forward(0.1, 0.2, P) == 0.05


def test_stops_when_close():
    assert limit_forward(0.1, 0.1, P) == 0.0


def test_no_range_no_forward():
    assert limit_forward(0.1, None, P) == 0.0


def test_backwards_always_allowed():
    assert limit_forward(-0.1, 0.05, P) == -0.1
    assert limit_forward(-0.1, None, P) == -0.1
