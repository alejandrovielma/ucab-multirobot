import pytest

from mobile_base.core.kinematics import (
    MecanumGeometry,
    duty_cap,
    normalize,
    to_duty,
    wheel_speeds,
)

GEOM = MecanumGeometry()


def test_default_geometry_matches_webots_model():
    assert GEOM.wheel_radius == pytest.approx(0.0325)
    assert GEOM.lx == pytest.approx(0.0605)
    assert GEOM.ly == pytest.approx(0.0655)


def test_pure_forward_gives_equal_speeds():
    fl, fr, rl, rr = wheel_speeds(0.1, 0.0, 0.0, GEOM)
    assert fl == pytest.approx(0.1 / GEOM.wheel_radius)
    assert fl == pytest.approx(fr) == pytest.approx(rl) == pytest.approx(rr)


def test_pure_left_strafe_pattern():
    fl, fr, rl, rr = wheel_speeds(0.0, 0.1, 0.0, GEOM)
    assert fl < 0 and rr < 0
    assert fr > 0 and rl > 0
    assert abs(fl) == pytest.approx(abs(fr))


def test_pure_ccw_rotation_left_back_right_forward():
    fl, fr, rl, rr = wheel_speeds(0.0, 0.0, 1.0, GEOM)
    assert fl < 0 and rl < 0
    assert fr > 0 and rr > 0
    assert fr == pytest.approx((GEOM.lx + GEOM.ly) / GEOM.wheel_radius)


def test_normalize_keeps_speeds_under_limit_untouched():
    assert normalize((1.0, -2.0, 3.0, 4.0), 10.0) == (1.0, -2.0, 3.0, 4.0)


def test_normalize_preserves_ratios():
    speeds = (5.0, -10.0, 20.0, 2.0)
    scaled = normalize(speeds, 10.0)
    assert max(abs(s) for s in scaled) == pytest.approx(10.0)
    for original, new in zip(speeds, scaled):
        assert new / original == pytest.approx(0.5)


def test_duty_cap():
    assert duty_cap(8.4) == pytest.approx(6.0 / 8.4)
    assert duty_cap(5.0) == pytest.approx(1.0)
    assert duty_cap(12.0, v_motor_max=3.0) == pytest.approx(0.25)
    with pytest.raises(ValueError):
        duty_cap(0.0)


def test_to_duty_maps_and_clamps():
    duties = to_duty((10.5, -5.25, 0.0, 30.0), max_wheel_speed=10.5, duty_cap=0.7)
    assert duties[0] == pytest.approx(0.7)
    assert duties[1] == pytest.approx(-0.35)
    assert duties[2] == pytest.approx(0.0)
    assert duties[3] == pytest.approx(0.7)
