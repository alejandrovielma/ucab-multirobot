"""
Mecanum kinematics for the browser tool. Pure logic, no hardware.

Ratios are normalised to [-1, 1] (a fraction of the duty cycle), unlike
src/mobile_base/mobile_base/kinematics.py which works in rad/s for ROS.
"""

WHEELS = ("fl", "fr", "rl", "rr")

RATIO_LIMIT = 1.0


def clamp(
    value: float,
    low: float = -RATIO_LIMIT,
    high: float = RATIO_LIMIT,
) -> float:
    """
    Limit `value` to the closed interval [low, high].
    """
    return max(low, min(high, value))


def mecanum_wheel_ratios(
    vx: float,
    vy: float,
    wz: float,
) -> dict[str, float]:
    """
    Mecanum inverse kinematics (X-pattern rollers), normalised to [-1, 1].

    vx > 0 forward, vy > 0 left, wz > 0 counter-clockwise (REP 103), each in
    [-1, 1]. If any wheel goes past 1, all are scaled down to keep the
    direction.
    """
    ratios = {
        "fl": vx - vy - wz,
        "fr": vx + vy + wz,
        "rl": vx + vy - wz,
        "rr": vx - vy + wz,
    }
    peak = max(RATIO_LIMIT, *(abs(r) for r in ratios.values()))
    return {wheel: ratio / peak for wheel, ratio in ratios.items()}
