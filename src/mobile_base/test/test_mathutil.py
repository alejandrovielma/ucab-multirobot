"""
Tests for the shared math helpers.
"""

import math

import pytest

from mobile_base.mathutil import clamp, wrap_angle


def test_clamp_limits_both_ends():
    assert clamp(5.0, -1.0, 1.0) == 1.0
    assert clamp(-5.0, -1.0, 1.0) == -1.0
    assert clamp(0.25, -1.0, 1.0) == 0.25


def test_wrap_angle_stays_in_range():
    assert wrap_angle(0.0) == pytest.approx(0.0)
    assert wrap_angle(3 * math.pi / 2) == pytest.approx(-math.pi / 2)
    assert wrap_angle(-3 * math.pi / 2) == pytest.approx(math.pi / 2)
