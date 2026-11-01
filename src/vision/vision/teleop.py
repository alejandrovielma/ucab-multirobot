"""
Manual control from the web viewer: keys -> velocities. Pure logic, no rclpy.

The page sends each axis in [-1, 1] (W/S, A/D, Q/E) and a speed in [0, 1];
here they are converted to m/s and rad/s using the `teleop_max_*` caps.
REP 103 convention: vx > 0 forward, vy > 0 left, wz > 0 counter-clockwise.
"""

import math
from dataclasses import dataclass

AXIS_MIN = -1.0
AXIS_MAX = 1.0
SPEED_MIN = 0.0
SPEED_MAX = 1.0


@dataclass(frozen=True)
class TeleopLimits:
    """
    Velocity caps reached at 100 % speed.
    """

    max_linear_m_s: float
    max_angular_rad_s: float


def _clamp_finite(value, low: float, high: float) -> float:
    """
    `value` as a float limited to [low, high]; ValueError if not finite.
    """
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("non-finite value")
    return max(low, min(high, number))


def teleop_velocity(
    data: dict,
    limits: TeleopLimits,
) -> tuple[float, float, float]:
    """
    (vx m/s, vy m/s, wz rad/s) from the page's {"vx", "vy", "wz", "speed"}.
    """
    speed = _clamp_finite(data.get("speed", 0.0), SPEED_MIN, SPEED_MAX)
    vx = _clamp_finite(data.get("vx", 0.0), AXIS_MIN, AXIS_MAX)
    vy = _clamp_finite(data.get("vy", 0.0), AXIS_MIN, AXIS_MAX)
    wz = _clamp_finite(data.get("wz", 0.0), AXIS_MIN, AXIS_MAX)
    return (
        vx * speed * limits.max_linear_m_s,
        vy * speed * limits.max_linear_m_s,
        wz * speed * limits.max_angular_rad_s,
    )
