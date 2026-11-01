"""
Mecanum inverse kinematics (X-roller layout). Pure logic, no rclpy.

REP 103 convention: vx > 0 forward, vy > 0 left, wz > 0 counter-clockwise.
Wheel order: fl, fr, rl, rr.
"""

from dataclasses import dataclass

WHEELS = ("fl", "fr", "rl", "rr")
# A wheel ratio of 1 is the wheel at its top speed.
FULL_RATIO = 1.0


@dataclass(frozen=True)
class MecanumGeometry:
    """
    Chassis dimensions needed by the inverse kinematics.
    """

    wheel_radius_m: float
    half_wheelbase_m: float  # half the wheelbase (front-back)
    half_track_m: float  # half the track width (left-right)


def wheel_speeds(
    vx: float,
    vy: float,
    wz: float,
    geo: MecanumGeometry,
) -> dict[str, float]:
    """
    Angular speed of each wheel (rad/s) for a motion (m/s, m/s, rad/s).
    """
    k = geo.half_wheelbase_m + geo.half_track_m
    r = geo.wheel_radius_m
    return {
        "fl": (vx - vy - k * wz) / r,
        "fr": (vx + vy + k * wz) / r,
        "rl": (vx + vy - k * wz) / r,
        "rr": (vx - vy + k * wz) / r,
    }


def scale_to_limit(
    speeds: dict[str, float],
    max_abs: float,
) -> dict[str, float]:
    """
    If any wheel exceeds `max_abs`, scale all of them equally so the
    direction of motion is preserved.
    """
    peak = max(abs(v) for v in speeds.values())
    if peak <= max_abs or peak == 0.0:
        return dict(speeds)
    factor = max_abs / peak
    return {name: v * factor for name, v in speeds.items()}


def wheel_ratios(
    vx: float,
    vy: float,
    wz: float,
    geo: MecanumGeometry,
    max_wheel_rad_s: float,
) -> dict[str, float]:
    """
    Wheel commands in [-1, 1] (1 = `max_wheel_rad_s`) for a motion (m/s,
    m/s, rad/s).

    When one wheel would saturate, all of them are scaled down together so
    the direction of motion is kept.
    """
    speeds = wheel_speeds(vx, vy, wz, geo)
    ratios = {name: v / max_wheel_rad_s for name, v in speeds.items()}
    return scale_to_limit(ratios, FULL_RATIO)
