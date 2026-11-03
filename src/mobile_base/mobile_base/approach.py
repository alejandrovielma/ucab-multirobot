"""
Approach a cube of the target colours and stop at a given distance.

This is the first piece of the state machine
(SEARCH -> APPROACH -> ARRIVED); collector.py adds the scoop and the
delivery on top. Pure logic, no rclpy: the behavior_fsm node passes in the
cubes it sees and publishes the command.
"""

import math
from dataclasses import dataclass, field

from mobile_base.mathutil import clamp
from mobile_base.states import State

# --- Defaults (the real values live in config/collector.yaml) -----------------
DEFAULT_TARGET_COLORS = ("red", "orange", "yellow")
# Stops when the cube is this far from the camera.
DEFAULT_STOP_DISTANCE_M = 0.17
# If the cube moves further than stop + margin, approach again.
DEFAULT_RESUME_MARGIN_M = 0.04
# Start slowing down from this distance.
DEFAULT_SLOW_DOWN_DISTANCE_M = 0.45
DEFAULT_MAX_FORWARD_M_S = 0.12
DEFAULT_MIN_FORWARD_M_S = 0.03
# rad/s of turn per rad of bearing error.
DEFAULT_TURN_GAIN = 1.5
DEFAULT_MAX_TURN_RAD_S = 0.8
# With more bearing error than this, turn in place without moving forward.
DEFAULT_ALIGN_TOLERANCE_RAD = 0.35
DEFAULT_SEARCH_TURN_RAD_S = 0.5
# Time without seeing the cube before searching again.
DEFAULT_LOST_TIMEOUT_S = 1.0

# --- Controller constants -----------------------------------------------------
# Smallest allowed slow-down span, avoids dividing by zero.
MIN_SLOW_DOWN_SPAN_M = 1e-6
# Fraction of the forward speed removed when the bearing error is exactly at
# the alignment tolerance (the worse the alignment, the slower).
MAX_ALIGN_SLOWDOWN_FRACTION = 0.5
# Search direction: +1 turns left (counter-clockwise), -1 turns right.
SEARCH_DIR_LEFT = 1.0
SEARCH_DIR_RIGHT = -1.0


@dataclass(frozen=True)
class Cube:
    """
    A cube seen by the camera, in robot polar coordinates.
    """

    color: str
    bearing_rad: float  # positive = to the left
    distance_m: float


@dataclass
class ApproachParams:
    """
    Tunable parameters of `ApproachController`.

    Field names double as ROS parameter names (see recolector_common.params).
    """

    target_colors: list[str] = field(
        default_factory=lambda: list(DEFAULT_TARGET_COLORS),
    )
    stop_distance_m: float = DEFAULT_STOP_DISTANCE_M
    resume_margin_m: float = DEFAULT_RESUME_MARGIN_M
    slow_down_distance_m: float = DEFAULT_SLOW_DOWN_DISTANCE_M
    max_forward_m_s: float = DEFAULT_MAX_FORWARD_M_S
    min_forward_m_s: float = DEFAULT_MIN_FORWARD_M_S
    turn_gain: float = DEFAULT_TURN_GAIN
    max_turn_rad_s: float = DEFAULT_MAX_TURN_RAD_S
    align_tolerance_rad: float = DEFAULT_ALIGN_TOLERANCE_RAD
    search_turn_rad_s: float = DEFAULT_SEARCH_TURN_RAD_S
    lost_timeout_s: float = DEFAULT_LOST_TIMEOUT_S


@dataclass(frozen=True)
class Command:
    """
    Velocity command: forward speed (m/s) and turn rate (rad/s).
    """

    vx: float = 0.0
    wz: float = 0.0


class ApproachController:
    """
    SEARCH -> APPROACH -> ARRIVED controller for one target cube.
    """

    def __init__(self, params: ApproachParams | None = None):
        """
        Start searching and enabled, with `params` (defaults when None).
        """
        self.params = params or ApproachParams()
        self.state = State.SEARCH
        self.enabled = True
        self.target: Cube | None = None
        self._last_seen_s: float | None = None
        self._search_dir = SEARCH_DIR_LEFT

    def reset(self) -> None:
        """
        Forget the last target and search again (after a catch or a
        delivery), keeping the search direction.
        """
        self.state = State.SEARCH
        self.target = None
        self._last_seen_s = None

    def pick_target(self, cubes: list[Cube]) -> Cube | None:
        """
        The nearest cube of the target colours.
        """
        wanted = set(self.params.target_colors)
        candidates = [c for c in cubes if c.color in wanted]
        return min(candidates, key=lambda c: c.distance_m, default=None)

    def update(self, cubes: list[Cube], now_s: float) -> Command:
        """
        Run one control step and return the velocity command.
        """
        if not self.enabled:
            self.state = State.PAUSED
            self.target = None
            return Command()
        if self.state == State.PAUSED:
            self.state = State.SEARCH

        cube = self.pick_target(cubes)
        if cube is not None:
            return self._on_cube_seen(cube, now_s)
        return self._on_cube_missing(now_s)

    def _on_cube_seen(self, cube: Cube, now_s: float) -> Command:
        """
        Handle a step where a target cube is visible.
        """
        p = self.params
        self.target = cube
        self._last_seen_s = now_s
        self._search_dir = (
            SEARCH_DIR_LEFT if cube.bearing_rad >= 0 else SEARCH_DIR_RIGHT
        )
        if self.state == State.ARRIVED:
            if cube.distance_m <= p.stop_distance_m + p.resume_margin_m:
                return Command()
        elif cube.distance_m <= p.stop_distance_m:
            self.state = State.ARRIVED
            return Command()
        self.state = State.APPROACH
        return self._drive_to(cube)

    def _on_cube_missing(self, now_s: float) -> Command:
        """
        Handle a step where no target cube is visible.
        """
        p = self.params
        if self._last_seen_s is None:
            lost_for_s = math.inf
        else:
            lost_for_s = now_s - self._last_seen_s
        recently_seen = lost_for_s < p.lost_timeout_s
        if recently_seen and self.state in (State.APPROACH, State.ARRIVED):
            # May be a bad frame: wait in place for a moment.
            return Command()
        self.state = State.SEARCH
        self.target = None
        return Command(0.0, self._search_dir * p.search_turn_rad_s)

    def _drive_to(self, cube: Cube) -> Command:
        """
        Turn toward the cube and drive at a speed that fades with distance
        and with the alignment error.
        """
        return drive_toward(
            cube.bearing_rad,
            cube.distance_m,
            self.params.stop_distance_m,
            self.params,
        )


def turn_toward(bearing_rad: float, p: ApproachParams) -> float:
    """
    Turn rate (rad/s) toward something at `bearing_rad` (positive = left).
    """
    return clamp(
        p.turn_gain * bearing_rad,
        -p.max_turn_rad_s,
        p.max_turn_rad_s,
    )


def drive_toward(
    bearing_rad: float,
    distance_m: float,
    stop_distance_m: float,
    p: ApproachParams,
) -> Command:
    """
    Turn toward a target and drive at a speed that fades with the distance
    left to `stop_distance_m` and with the alignment error; turn in place
    while the error is above `align_tolerance_rad`.
    """
    wz = turn_toward(bearing_rad, p)
    if abs(bearing_rad) > p.align_tolerance_rad:
        return Command(0.0, wz)
    span_m = max(
        MIN_SLOW_DOWN_SPAN_M,
        p.slow_down_distance_m - stop_distance_m,
    )
    fraction = clamp((distance_m - stop_distance_m) / span_m, 0.0, 1.0)
    vx = p.min_forward_m_s + fraction * (p.max_forward_m_s - p.min_forward_m_s)
    alignment_error = abs(bearing_rad) / p.align_tolerance_rad
    vx *= 1.0 - alignment_error * MAX_ALIGN_SLOWDOWN_FRACTION
    return Command(vx, wz)
