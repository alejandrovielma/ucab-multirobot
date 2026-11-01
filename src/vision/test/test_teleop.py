"""
Tests: web viewer manual control (keys -> velocities).
"""

import math

import pytest

from vision.teleop import TeleopLimits, teleop_velocity

LIMITS = TeleopLimits(max_linear_m_s=0.2, max_angular_rad_s=1.0)


def test_forward_half_speed():
    assert teleop_velocity({"vx": 1, "speed": 0.5}, LIMITS) == (0.1, 0.0, 0.0)


def test_strafe_and_turn_signs():
    vx, vy, wz = teleop_velocity({"vy": -1, "wz": 1, "speed": 1}, LIMITS)
    assert (vx, vy, wz) == (0.0, -0.2, 1.0)


def test_values_are_clamped():
    vx, vy, wz = teleop_velocity(
        {"vx": 5, "vy": -5, "wz": 3, "speed": 9}, LIMITS
    )
    assert (vx, vy, wz) == (0.2, -0.2, 1.0)


def test_negative_speed_stops():
    assert teleop_velocity({"vx": 1, "speed": -1}, LIMITS) == (0.0, 0.0, 0.0)


def test_missing_fields_stop():
    assert teleop_velocity({}, LIMITS) == (0.0, 0.0, 0.0)


@pytest.mark.parametrize("bad", [math.nan, math.inf, "x", None])
def test_bad_values_rejected(bad):
    with pytest.raises((ValueError, TypeError)):
        teleop_velocity({"vx": bad, "speed": 1}, LIMITS)
