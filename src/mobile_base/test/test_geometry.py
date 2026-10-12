import math

import pytest

from mobile_base.core.geometry import distance, relative_bearing, wrap_angle


@pytest.mark.parametrize(
    "angle, expected",
    [(0.0, 0.0), (math.pi / 2, math.pi / 2), (3 * math.pi, -math.pi), (-3 * math.pi / 2, math.pi / 2)],
)
def test_wrap_angle(angle, expected):
    assert wrap_angle(angle) == pytest.approx(expected)


def test_distance():
    assert distance(0.0, 0.0, 3.0, 4.0) == pytest.approx(5.0)


def test_bearing_of_point_ahead_is_zero():
    assert relative_bearing(0.0, 0.0, 0.0, 1.0, 0.0) == pytest.approx(0.0)


def test_bearing_of_point_on_the_left_is_positive():
    assert relative_bearing(0.0, 0.0, 0.0, 0.0, 1.0) == pytest.approx(math.pi / 2)


def test_bearing_accounts_for_heading():
    # Robot facing +y: a point at +x is on its right.
    assert relative_bearing(0.0, 0.0, math.pi / 2, 1.0, 0.0) == pytest.approx(-math.pi / 2)
