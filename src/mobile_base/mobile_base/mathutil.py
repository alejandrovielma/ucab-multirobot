"""
Small math helpers shared by the pure-logic modules. No rclpy.
"""

import math

FULL_TURN_RAD = 2.0 * math.pi


def clamp(value: float, low: float, high: float) -> float:
    """
    Limit `value` to the closed interval [low, high].
    """
    return max(low, min(high, value))


def wrap_angle(angle_rad: float) -> float:
    """
    Wrap an angle to the interval [-pi, pi).
    """
    return (angle_rad + math.pi) % FULL_TURN_RAD - math.pi
