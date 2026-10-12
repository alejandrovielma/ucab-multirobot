"""Behavior finite state machine of a collector robot (pure logic, time passed in).

EXPLORE  -> drive forward; on the border line turn in place for a random time.
APPROACH -> steer toward the largest (closest) cube of my colors.
CAPTURE  -> push forward for a fixed time so the cube enters the fixed scoop.
CARRY    -> drive in arcs to the entry point north of the delivery square (overhead pose),
            ask for the zone lease when close and wait there until it is granted.
DELIVER  -> with the lease: drive south across the square's tape until the cube is inside,
            then reverse out (the U is open at the front, so the cube stays) and release the lease.
STOP     -> all zeros.

The output is the DESIRED command; the SafetyGate filters it before the motors. Requests to the
zone arbiter are returned as ZoneAction items (take_actions) so this module stays free of ROS.
"""

import math
import random
from collections.abc import Sequence
from dataclasses import dataclass
from enum import Enum

from mobile_base.core.colors import is_mine
from mobile_base.core.delivery import DeliveryZone, zone_from
from mobile_base.core.geometry import distance, relative_bearing

Command = tuple[float, float, float]
ZERO: Command = (0.0, 0.0, 0.0)


class State(str, Enum):
    EXPLORE = "EXPLORE"
    APPROACH = "APPROACH"
    CAPTURE = "CAPTURE"
    CARRY = "CARRY"
    DELIVER = "DELIVER"
    STOP = "STOP"


@dataclass(frozen=True)
class Cube:
    color: str
    cx_norm: float  # [-1, 1], left negative
    cy_norm: float  # [0, 1], 0 = top of the image
    area_px: float


@dataclass(frozen=True)
class ZoneAction:
    kind: str  # "request" or "release"
    delivered: bool = False
    color: str = ""


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
    # Transport: arcs only (the gate enforces the same limits).
    carry_speed: float = 0.08  # m/s
    carry_min_speed: float = 0.05  # m/s
    carry_max_turn: float = 0.6  # rad/s
    k_heading: float = 1.5  # rad/s per rad of heading error
    arrive_radius: float = 0.10  # m, entry point reached (> turning radius, or the robot orbits it)
    request_radius: float = 0.40  # m, ask for the lease when this close to the entry point
    request_period: float = 1.0  # s, retry while denied
    enter_speed: float = 0.05  # m/s
    enter_lookahead: float = 0.10  # m, carrot distance along the square axis
    reverse_speed: float = 0.04  # m/s
    drop_timeout: float = 6.0  # s, give up reversing (the cube is already inside)
    # Delivery square (see core/delivery.py); flat so each one is a ROS parameter.
    zone_x: float = DeliveryZone.x
    zone_square_y: float = DeliveryZone.square_y
    zone_entry_y: float = DeliveryZone.entry_y
    zone_drop_y: float = DeliveryZone.drop_y
    zone_exit_y: float = DeliveryZone.exit_y
    zone_corridor_half_width: float = DeliveryZone.corridor_half_width

    @property
    def zone(self) -> DeliveryZone:
        return zone_from(self)


@dataclass(frozen=True)
class BehaviorInput:
    now: float
    cubes: Sequence[Cube]
    line_front: bool
    pose: tuple[float, float, float] | None = None
    holds_zone: bool = False  # /zone/occupant == this robot


class BehaviorFSM:
    def __init__(self, params: BehaviorParams, rng: random.Random | None = None):
        self.params = params
        self._zone = params.zone
        self._rng = rng or random.Random()
        self.state = State.EXPLORE
        self._turn_until: float | None = None
        self._turn_dir = 1.0
        self._last_seen = 0.0
        self._capture_until = 0.0
        self._target_color = ""
        self._carried_color = ""
        self._dropping = False
        self._drop_until = 0.0
        self._last_request: float | None = None
        self._actions: list[ZoneAction] = []

    @property
    def carrying(self) -> bool:
        return self.state is State.CARRY or (self.state is State.DELIVER and not self._dropping)

    def stop(self) -> None:
        self.state = State.STOP

    def take_actions(self) -> list[ZoneAction]:
        """Zone arbiter calls requested since the last call (the node performs them)."""
        actions, self._actions = self._actions, []
        return actions

    def step(self, inp: BehaviorInput) -> Command:
        handler = {
            State.EXPLORE: self._explore,
            State.APPROACH: self._approach,
            State.CAPTURE: self._capture,
            State.CARRY: self._carry,
            State.DELIVER: self._deliver,
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
        self._target_color = target.color
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
            self._carried_color = self._target_color
            self._last_request = None
            return self._carry(inp)
        return (self.params.capture_speed, 0.0, 0.0)

    def _steer(self, pose, gx: float, gy: float, speed: float) -> Command:
        """Forward-only arc toward (gx, gy): never an in-place spin (it ejects the cube)."""
        p = self.params
        x, y, theta = pose
        err = relative_bearing(x, y, theta, gx, gy)
        wz = max(-p.carry_max_turn, min(p.carry_max_turn, p.k_heading * err))
        vx = max(p.carry_min_speed, speed * math.cos(err))
        return (vx, 0.0, wz)

    def _ask_lease(self, inp: BehaviorInput) -> None:
        if inp.holds_zone:
            return
        if self._last_request is None or inp.now - self._last_request >= self.params.request_period:
            self._actions.append(ZoneAction("request"))
            self._last_request = inp.now

    def _carry(self, inp: BehaviorInput) -> Command:
        if inp.pose is None:
            return ZERO  # without the overhead pose the robot cannot find the square
        z = self._zone
        x, y, _ = inp.pose
        d = distance(x, y, z.x, z.entry_y)
        if d <= self.params.request_radius:
            self._ask_lease(inp)
        if d > self.params.arrive_radius:
            return self._steer(inp.pose, z.x, z.entry_y, self.params.carry_speed)
        if not inp.holds_zone:
            return ZERO  # wait at the entry point; the other robot is using the square
        self.state = State.DELIVER
        self._dropping = False
        return self._deliver(inp)

    def _deliver(self, inp: BehaviorInput) -> Command:
        p = self.params
        z = self._zone
        if inp.pose is None:
            return ZERO
        x, y, _ = inp.pose
        if not inp.holds_zone:
            self._ask_lease(inp)  # lease expired mid-maneuver: the gate blocks the tape and the reverse
        if not self._dropping:
            if y > z.drop_y:
                # The square's north tape is crossed on purpose here: line_front is ignored.
                carrot_y = max(y - p.enter_lookahead, z.drop_y - p.enter_lookahead)
                _, _, wz = self._steer(inp.pose, z.x, carrot_y, p.enter_speed)
                return (p.enter_speed, 0.0, wz)
            self._dropping = True
            self._drop_until = inp.now + p.drop_timeout
        if y < z.exit_y and inp.now < self._drop_until:
            return (-p.reverse_speed, 0.0, 0.0)
        self._actions.append(ZoneAction("release", delivered=True, color=self._carried_color))
        self._dropping = False
        self._carried_color = ""
        self.state = State.EXPLORE
        self._start_turn(inp.now)  # facing the square's tape: turn away before exploring
        return self._explore(inp)
