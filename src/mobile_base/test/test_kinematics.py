"""
Tests: mecanum kinematics (forward, strafe, turn).
"""

import pytest

from mobile_base.kinematics import (
    MecanumGeometry,
    scale_to_limit,
    wheel_ratios,
    wheel_speeds,
)
from mobile_base.motor_hardware import (
    FakeMotors,
    MotorHardwareParams,
    to_duty,
)

GEO = MecanumGeometry(
    wheel_radius_m=0.0325, half_wheelbase_m=0.0605, half_track_m=0.0655
)


def test_forward_all_wheels_same():
    s = wheel_speeds(0.1, 0.0, 0.0, GEO)
    assert all(v == pytest.approx(0.1 / 0.0325) for v in s.values())


def test_strafe_left_diagonal_pairs():
    s = wheel_speeds(0.0, 0.1, 0.0, GEO)
    assert s["fl"] < 0 and s["rr"] < 0
    assert s["fr"] > 0 and s["rl"] > 0
    assert s["fl"] == pytest.approx(-s["fr"])


def test_turn_left_right_side_forward():
    s = wheel_speeds(0.0, 0.0, 1.0, GEO)
    assert s["fr"] > 0 and s["rr"] > 0
    assert s["fl"] < 0 and s["rl"] < 0
    assert s["fr"] == pytest.approx((0.0605 + 0.0655) / 0.0325)


def test_scale_keeps_ratios():
    s = scale_to_limit({"fl": 10.0, "fr": -5.0, "rl": 2.0, "rr": 0.0}, 5.0)
    assert s == pytest.approx({"fl": 5.0, "fr": -2.5, "rl": 1.0, "rr": 0.0})


def test_scale_does_nothing_below_limit():
    s = {"fl": 1.0, "fr": -1.0, "rl": 0.5, "rr": 0.0}
    assert scale_to_limit(s, 5.0) == s


def test_wheel_ratios_are_speed_over_max():
    r = wheel_ratios(0.1, 0.0, 0.0, GEO, max_wheel_rad_s=10.0)
    assert all(v == pytest.approx(0.1 / 0.0325 / 10.0) for v in r.values())


def test_wheel_ratios_saturate_together():
    r = wheel_ratios(1.0, 0.5, 0.0, GEO, max_wheel_rad_s=1.0)
    assert max(abs(v) for v in r.values()) == pytest.approx(1.0)
    assert r["fl"] == pytest.approx(r["rr"])


def test_duty_is_capped_and_inverted():
    assert to_duty(1.0, inverted=False, max_duty_ratio=0.9) == 0.9
    assert to_duty(0.5, inverted=True, max_duty_ratio=0.9) == -0.45
    assert to_duty(-3.0, inverted=False, max_duty_ratio=0.9) == -0.9


def test_fake_motors_apply_inversion_per_wheel():
    motors = FakeMotors(MotorHardwareParams(max_duty_ratio=1.0))
    motors.set({"fl": 0.5, "fr": 0.5, "rl": 0.5, "rr": 0.5})
    # fl and rl are inverted by default.
    assert motors.duties == {"fl": -0.5, "fr": 0.5, "rl": -0.5, "rr": 0.5}
    motors.stop()
    assert all(d == 0.0 for d in motors.duties.values())
