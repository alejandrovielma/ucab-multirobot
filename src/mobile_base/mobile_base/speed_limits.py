"""
Speed limit based on the ultrasonic distance. Pure logic, no rclpy.

Levels from the plan: beyond `slow_distance_m` normal speed, between
`stop_distance_m` and `slow_distance_m` reduced speed, below
`stop_distance_m` stopped. Only forward motion is limited (vx > 0):
reversing, strafing and turning are allowed so the robot can get out.
"""

from dataclasses import dataclass

# --- Defaults (the real values live in config/collector.yaml) -----------------
DEFAULT_SLOW_DISTANCE_M = 0.30
DEFAULT_STOP_DISTANCE_M = 0.15
DEFAULT_SLOW_FACTOR = 0.5


@dataclass(frozen=True)
class GuardParams:
    """
    Distance thresholds and speed factor of the ultrasonic guard.

    Field names double as ROS parameter names (see recolector_common.params).
    """

    slow_distance_m: float = DEFAULT_SLOW_DISTANCE_M
    stop_distance_m: float = DEFAULT_STOP_DISTANCE_M
    slow_factor: float = DEFAULT_SLOW_FACTOR


def limit_forward(vx: float, range_m: float | None, p: GuardParams) -> float:
    """
    Allowed forward speed for the requested `vx`.

    `range_m` None means there is no recent reading: no forward motion.
    """
    if vx <= 0.0:
        return vx
    if range_m is None or range_m < p.stop_distance_m:
        return 0.0
    if range_m < p.slow_distance_m:
        return vx * p.slow_factor
    return vx
