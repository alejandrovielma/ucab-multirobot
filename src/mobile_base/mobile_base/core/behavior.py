"""Behavior finite state machine of a collector robot (pure logic, time passed in).

EXPLORE  -> drive forward; on the border line turn in place for a random time.
APPROACH -> steer toward the largest (closest) cube of my colors.
CAPTURE  -> push forward for a fixed time so the cube enters the fixed scoop.
CARRY    -> transport and delivery: TODO for the team (see BehaviorFSM._carry).
STOP     -> all zeros.

The output is the DESIRED command; the SafetyGate filters it before the motors.
"""

import random
from collections.abc import Sequence
from dataclasses import dataclass
from enum import Enum

from mobile_base.core.colors import is_mine

Command = tuple[float, float, float]
ZERO: Command = (0.0, 0.0, 0.0)


class State(str, Enum):
    EXPLORE = "EXPLORE"
    APPROACH = "APPROACH"
    CAPTURE = "CAPTURE"
    CARRY = "CARRY"
    STOP = "STOP"


@dataclass(frozen=True)
class Cube:
    color: str
    cx_norm: float  # [-1, 1], left negative
    cy_norm: float  # [0, 1], 0 = top of the image
    area_px: float


@dataclass(frozen=True)
class BehaviorParams:
    my_colors: Sequence[str] = ()
    explore_speed: float = 0.12  # m/s
    turn_speed: float = 1.2  # rad/s
    turn_min: float = 0.8  # s
    turn_max: float = 2.0  # s
    k_turn: float = 1.5  # rad/s per unit of cx_norm
    approach_speed: float = 0.10  # m/s
    approach_min_scale: float = 0.3  # forward scale when the cube is at the image edge
    lost_timeout: float = 1.0  # s
    capture_cy: float = 0.85  # cube this low in the image is right in front of the scoop
    capture_speed: float = 0.08  # m/s
    capture_time: float = 1.0  # s


@dataclass(frozen=True)
class BehaviorInput:
    now: float
    cubes: Sequence[Cube]
    line_front: bool
    pose: tuple[float, float, float] | None = None


class BehaviorFSM:
    def __init__(self, params: BehaviorParams, rng: random.Random | None = None):
        self.params = params
        self._rng = rng or random.Random()
        self.state = State.EXPLORE
        self._turn_until: float | None = None
        self._turn_dir = 1.0
        self._last_seen = 0.0
        self._capture_until = 0.0

    @property
    def carrying(self) -> bool:
        return self.state is State.CARRY

    def stop(self) -> None:
        self.state = State.STOP

    def step(self, inp: BehaviorInput) -> Command:
        handler = {
            State.EXPLORE: self._explore,
            State.APPROACH: self._approach,
            State.CAPTURE: self._capture,
            State.CARRY: self._carry,
            State.STOP: lambda _inp: ZERO,
        }[self.state]
        return handler(inp)

    def _target(self, cubes: Sequence[Cube]) -> Cube | None:
        mine = [c for c in cubes if is_mine(c.color, self.params.my_colors)]
        return max(mine, key=lambda c: c.area_px, default=None)

    def _start_turn(self, now: float) -> None:
        p = self.params
        self._turn_until = now + self._rng.uniform(p.turn_min, p.turn_max)
        self._turn_dir = self._rng.choice((-1.0, 1.0))

    def _explore(self, inp: BehaviorInput) -> Command:
        p = self.params
        if self._turn_until is not None and inp.now >= self._turn_until:
            self._turn_until = None
        if self._turn_until is None:
            if inp.line_front:
                self._start_turn(inp.now)
            elif self._target(inp.cubes) is not None:
                self.state = State.APPROACH
                self._last_seen = inp.now
                return self._approach(inp)
        if self._turn_until is not None:
            return (0.0, 0.0, self._turn_dir * p.turn_speed)
        return (p.explore_speed, 0.0, 0.0)

    def _approach(self, inp: BehaviorInput) -> Command:
        p = self.params
        target = self._target(inp.cubes)
        if target is None:
            if inp.now - self._last_seen > p.lost_timeout:
                self.state = State.EXPLORE
                return self._explore(inp)
            return ZERO
        self._last_seen = inp.now
        if target.cy_norm > p.capture_cy:
            self.state = State.CAPTURE
            self._capture_until = inp.now + p.capture_time
            return self._capture(inp)
        if inp.line_front:
            # Cube beyond the border: give up and turn away from the line.
            self.state = State.EXPLORE
            self._start_turn(inp.now)
            return self._explore(inp)
        scale = max(p.approach_min_scale, 1.0 - abs(target.cx_norm))
        return (p.approach_speed * scale, 0.0, -p.k_turn * target.cx_norm)

    def _capture(self, inp: BehaviorInput) -> Command:
        if inp.now >= self._capture_until:
            self.state = State.CARRY
            return self._carry(inp)
        return (self.params.capture_speed, 0.0, 0.0)

    def _carry(self, inp: BehaviorInput) -> Command:
        """Transport the captured cube to the delivery square. TODO(team).

        Acceptance criteria for the team implementation:
        - Find the delivery square (ArUco id 10 next to it) using the overhead pose and/or the camera.
        - Call /zone/request and wait until granted before entering; keep a safe distance while denied.
        - Enter the square from the north side and push the cube inside.
        - Reverse out ONLY after the cube is inside the square (the fixed scoop drops it when reversing).
        - Call /zone/release with delivered=true and the cube color, then return to EXPLORE.
        - vx must NEVER be negative while carrying (enforced by tests).
        """
        return ZERO
