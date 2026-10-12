"""Small planar geometry helpers shared by the safety gate and the behavior."""

import math


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def distance(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.hypot(x2 - x1, y2 - y1)


def relative_bearing(x: float, y: float, theta: float, px: float, py: float) -> float:
    """Bearing of point (px, py) seen from pose (x, y, theta); positive = to the left."""
    return wrap_angle(math.atan2(py - y, px - x) - theta)
