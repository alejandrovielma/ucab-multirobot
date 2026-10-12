"""Behavior finite state machine of a collector robot (pure logic, time passed in).

EXPLORE  -> drive forward; on the border line turn in place a random 135-225 deg (measured with the
            overhead pose; a random time when there is no pose).
APPROACH -> steer toward the largest (closest) cube of my colors.
CAPTURE  -> push forward for a fixed time so the cube enters the fixed scoop; it counts only if the
            camera then sees a cube of my color inside the U (scoop_color), otherwise APPROACH again.
CARRY    -> ask for the zone lease; with it, drive in arcs to the entry point north of the delivery
            square (overhead pose); without it, wait at this robot's own waiting point, away from
            the entry so the robot holding the lease can get through. Reaching the border line
            ends the transport: the turn away from it drops the cube (back to EXPLORE).
DELIVER  -> with the lease: drive south across the square's tape until the cube is inside,
            then reverse out (the U is open at the front, so the cube stays) and release the lease.
STOP     -> all zeros.

While carrying, a cube not seen in the scoop for scoop_lost_timeout ends the transport (lease
released with delivered=false), and a delivery counts only if the cube left the scoop after reversing.

Geofence (with the overhead pose and a track polygon): the front point outside the track counts as
the border line in every state but DELIVER, and with the center already outside the robot drops
whatever it was doing and drives back toward the track centroid.

The output is the DESIRED command; the SafetyGate filters it before the motors. Requests to the
zone arbiter are returned as ZoneAction items (take_actions) so this module stays free of ROS.
"""

import dataclasses
import math
import random
from collections.abc import Sequence
from dataclasses import dataclass
from enum import Enum

from mobile_base.core.colors import is_mine
from mobile_base.core.delivery import DeliveryZone, zone_from
from mobile_base.core.geometry import distance, relative_bearing, wrap_angle
from mobile_base.core.track import Track

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
    turn_angle_min: float = math.radians(135.0)  # border turn with pose: face back into the track
    turn_angle_max: float = math.radians(225.0)
    k_turn: float = 1.5  # rad/s per unit of cx_norm
    approach_speed: float = 0.10  # m/s
    approach_min_scale: float = 0.3  # forward scale when the cube is at the image edge
    lost_timeout: float = 1.0  # s
    capture_cy: float = 0.62  # cube this low in the image is at the scoop mouth (IMX219-120, measured 0.66)
    capture_speed: float = 0.08  # m/s
    capture_time: float = 1.0  # s
    scoop_lost_timeout: float = 1.0  # s without seeing the cube in the scoop while carrying
    # Transport: arcs only (the gate enforces the same limits).
    carry_speed: float = 0.08  # m/s
    carry_min_speed: float = 0.05  # m/s
    carry_max_turn: float = 0.6  # rad/s
    k_heading: float = 1.5  # rad/s per rad of heading error
    arrive_radius: float = 0.10  # m, waiting point reached (> turning radius, or the robot orbits it)
    lead_in: float = 0.10  # m, aim this far south of the entry when close to the square axis
    lead_in_width: float = 0.20  # m, "close to the axis"
    wait_x: float = -0.45  # m, waiting point while the other robot holds the lease (per robot)
    wait_y: float = 0.05
    request_period: float = 1.0  # s, retry while denied
    enter_speed: float = 0.05  # m/s
    enter_lookahead: float = 0.10  # m, carrot distance along the square axis
    reverse_speed: float = 0.04  # m/s
    drop_timeout: float = 6.0  # s, give up reversing (the cube is already inside)
    track_lookahead: float = 0.10  # m, geofence point ahead of the center (keep equal to the gate)
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
    scoop_color: str = ""  # color seen inside the scoop by the camera ("" = empty or unknown)


class BehaviorFSM:
    def __init__(self, params: BehaviorParams, rng: random.Random | None = None, track: Track | None = None):
        self.params = params
        self._track = track or Track(())
        self._zone = params.zone
        self._rng = rng or random.Random()
        self.state = State.EXPLORE
        self._turn_until: float | None = None
        self._turn_dir = 1.0
        self._turn_goal: float | None = None  # rad still to turn (pose mode)
        self._turn_prev_theta = 0.0
        self._last_seen = 0.0
        self._capture_until = 0.0
        self._target_color = ""
        self._carried_color = ""
        self._dropping = False
        self._drop_until = 0.0
        self._last_request: float | None = None
        self._scoop_seen = 0.0
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
        if self.state not in (State.DELIVER, State.STOP) and self._track.enabled and inp.pose is not None:
            x, y, theta = inp.pose
            if not self._track.contains(x, y):
                return self._return_to_track(inp)
            if not self._track.front_inside(x, y, theta, self.params.track_lookahead):
                inp = dataclasses.replace(inp, line_front=True)
        handler = {
            State.EXPLORE: self._explore,
            State.APPROACH: self._approach,
            State.CAPTURE: self._capture,
            State.CARRY: self._carry,
            State.DELIVER: self._deliver,
            State.STOP: lambda _inp: ZERO,
        }[self.state]
        return handler(inp)

    def _return_to_track(self, inp: BehaviorInput) -> Command:
        if self.state is State.CARRY:
            self._give_up_cube(inp)
        self.state = State.EXPLORE
        self._turn_until = None
        x, y, theta = inp.pose
        cx, cy = self._track.centroid
        err = relative_bearing(x, y, theta, cx, cy)
        if abs(err) > 0.3:
            return (0.0, 0.0, math.copysign(self.params.turn_speed, err))
        return (self.params.explore_speed, 0.0, 0.0)

    def _still_holding(self, inp: BehaviorInput) -> bool:
        if is_mine(inp.scoop_color, self.params.my_colors):
            self._scoop_seen = inp.now
        return inp.now - self._scoop_seen <= self.params.scoop_lost_timeout

    def _abandon_transport(self, inp: BehaviorInput) -> Command:
        self._give_up_cube(inp)
        self.state = State.EXPLORE
        self._start_turn(inp.now, inp.pose)
        return self._explore(inp)

    def _give_up_cube(self, inp: BehaviorInput) -> None:
        if inp.holds_zone:
            self._actions.append(ZoneAction("release", delivered=False, color=self._carried_color))
        self._carried_color = ""

    def _target(self, cubes: Sequence[Cube]) -> Cube | None:
        mine = [c for c in cubes if is_mine(c.color, self.params.my_colors)]
        return max(mine, key=lambda c: c.area_px, default=None)

    def _start_turn(self, now: float, pose) -> None:
        p = self.params
        self._turn_dir = self._rng.choice((-1.0, 1.0))
        if pose is None:
            self._turn_goal = None
            self._turn_until = now + self._rng.uniform(p.turn_min, p.turn_max)
            return
        # A timed turn can stop with the front sensor already beyond the tape (it then reads white
        # and the robot drives off the track): turn a measured angle that points back inside.
        self._turn_goal = self._rng.uniform(p.turn_angle_min, p.turn_angle_max)
        self._turn_prev_theta = pose[2]
        self._turn_until = now + 2.0 * self._turn_goal / p.turn_speed  # guard if the pose freezes

    def _turn_done(self, inp: BehaviorInput) -> bool:
        if self._turn_goal is not None and inp.pose is not None:
            self._turn_goal -= abs(wrap_angle(inp.pose[2] - self._turn_prev_theta))
            self._turn_prev_theta = inp.pose[2]
            if self._turn_goal <= 0.0:
                return True
        return inp.now >= self._turn_until

    def _explore(self, inp: BehaviorInput) -> Command:
        p = self.params
        if self._turn_until is not None and self._turn_done(inp):
            self._turn_until = None
        if self._turn_until is None:
            if inp.line_front:
                self._start_turn(inp.now, inp.pose)
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
        if inp.line_front:
            # Cube on or beyond the border: give up and turn away. Checked before the capture
            # (Webots run 8: a cube just past the edge looped APPROACH <-> CAPTURE for minutes).
            self.state = State.EXPLORE
            self._start_turn(inp.now, inp.pose)
            return self._explore(inp)
        if target.cy_norm > p.capture_cy:
            self.state = State.CAPTURE
            self._capture_until = inp.now + p.capture_time
            return self._capture(inp)
        scale = max(p.approach_min_scale, 1.0 - abs(target.cx_norm))
        return (p.approach_speed * scale, 0.0, -p.k_turn * target.cx_norm)

    def _capture(self, inp: BehaviorInput) -> Command:
        if inp.line_front:
            self.state = State.EXPLORE
            self._start_turn(inp.now, inp.pose)
            return self._explore(inp)
        if inp.now >= self._capture_until:
            if not is_mine(inp.scoop_color, self.params.my_colors):
                # Pushed but nothing of mine in the U: the cube slipped aside. Look for it again.
                self.state = State.APPROACH
                self._last_seen = inp.now
                return self._approach(inp)
            self.state = State.CARRY
            self._carried_color = inp.scoop_color
            self._scoop_seen = inp.now
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
        p = self.params
        z = self._zone
        if inp.line_front:
            # The gate turns in place at the line and the turn drops the cube: stop carrying.
            return self._abandon_transport(inp)
        if not self._still_holding(inp):
            return self._abandon_transport(inp)  # the camera no longer sees the cube in the U
        x, y, theta = inp.pose
        self._ask_lease(inp)
        if not inp.holds_zone:
            if distance(x, y, p.wait_x, p.wait_y) <= p.arrive_radius:
                return ZERO  # the other robot is using the square
            return self._steer(inp.pose, p.wait_x, p.wait_y, p.carry_speed)
        if z.in_corridor(x, y) and z.aligned(theta):
            self.state = State.DELIVER
            self._dropping = False
            return self._deliver(inp)
        if abs(x - z.x) <= p.lead_in_width:
            # Near the axis: aim a bit south of the entry so the arc ends facing the square
            # (Webots run 8: arriving at the entry facing north and entering anyway went wrong).
            return self._steer(inp.pose, z.x, z.entry_y - p.lead_in, p.carry_speed)
        return self._steer(inp.pose, z.x, z.entry_y, p.carry_speed)

    def _deliver(self, inp: BehaviorInput) -> Command:
        p = self.params
        z = self._zone
        if inp.pose is None:
            return ZERO
        x, y, _ = inp.pose
        if not inp.holds_zone:
            self._ask_lease(inp)  # lease expired mid-maneuver: the gate blocks the tape and the reverse
        if not self._dropping:
            if not self._still_holding(inp):
                return self._abandon_transport(inp)
            if not z.in_corridor(x, y):
                self.state = State.CARRY  # drifted out of the corridor: line up again
                return self._carry(inp)
            if y > z.drop_y:
                # The square's north tape is crossed on purpose here: line_front is ignored.
                carrot_y = max(y - p.enter_lookahead, z.drop_y - p.enter_lookahead)
                _, _, wz = self._steer(inp.pose, z.x, carrot_y, p.enter_speed)
                return (p.enter_speed, 0.0, wz)
            self._dropping = True
            self._drop_until = inp.now + p.drop_timeout
        if y < z.exit_y and inp.now < self._drop_until:
            return (-p.reverse_speed, 0.0, 0.0)
        # Counted only if the cube stayed in the square: it must have left the scoop.
        left_behind = not is_mine(inp.scoop_color, self.params.my_colors)
        self._actions.append(ZoneAction("release", delivered=left_behind, color=self._carried_color))
        self._dropping = False
        self._carried_color = ""
        self.state = State.EXPLORE
        self._start_turn(inp.now, inp.pose)  # facing the square's tape: turn away before exploring
        return self._explore(inp)
