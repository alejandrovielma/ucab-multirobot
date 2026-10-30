"""Behavior finite state machine of a collector robot (pure logic, time passed in).

Motion rule (team, 2026-10-27): the robot translates in any direction or spins in place, never both;
every change of heading is a spin. The safety gate enforces it; this module only asks for such moves.

Mission: target colors to collect (set_mission, or my_colors at start) and colors whose touch costs
points. Without targets the robot stays still (IDLE). Only target cubes are captured and carried;
forbidden and neutral cubes may be moved on the track but never into the square.

IDLE     -> all zeros until a mission with at least one target color arrives.
EXPLORE  -> drive forward; on the border line back off line_backoff and spin a random 135-225 deg
            (measured with the overhead pose; a random time when there is no pose).
APPROACH -> spin in place until the largest (closest) target cube is centered, then drive straight.
CAPTURE  -> push forward for a fixed time so the cube enters the fixed scoop; it counts only if the
            camera then sees a target cube inside the U (scoop_color), otherwise APPROACH again.
CARRY    -> ask for the zone lease; with it, go to the entry point in front of the delivery square
            with straights and slow spins (overhead pose; core/delivery.py, approach heading west on
            the real track); without it, wait at this robot's own waiting point, away from the entry
            so the robot holding the lease can get through. Reaching the border line ends the
            transport: the spin away from it drops the cube (back to EXPLORE).
DELIVER  -> with the lease: face the square and translate along its axis (sideways corrections, no
            steering) until the cube is inside, then reverse out (the U is open at the front, so the
            cube stays) and release the lease.
STOP     -> all zeros.

Scoop gate (has_gate): a servo door at the mouth of the U. Open while exploring and capturing; it
closes once the camera confirms the capture (the robot waits gate_close_time), and with it closed the
cube is retained, so the carry plan may spin fast and reverse and the border line no longer drops the
cube. At the drop point it opens (waits gate_open_time) and the robot reverses out.

While carrying, a cube not seen in the scoop for scoop_lost_timeout ends the transport (lease
released with delivered=false): a slow spin can still throw it out, and the camera then finds it
again right in front and recaptures it. A delivery counts only if the cube left the scoop after
reversing.

Non-target cubes. While exploring, one in the scoop's path (below avoid_cy, within avoid_cx of the
image center) is spun away from, for at most avoid_timeout; while approaching, one in the path closer
than the target makes the robot skip the target for skip_time. A target that keeps hiding behind one
(clear_after skips within clear_window) is reached by pushing through for clear_time. One seen inside
the U for shed_confirm (scoop_color, not carrying) is backed off shed_distance (at least line_backoff
on the line) before turning away, or spun out of the scoop (clear_turn) while clearing the way.

Without the overhead pose (team decision 2026-10-27: only the robot's own camera) CARRY looks for the
blue square: slow spins until the camera sees it (a short drive forward every square_search_time
without it), spins to center it, then drives straight until its centroid is at square_reach_cy.
DELIVER then pushes the scoop in for deliver_push_time (sideways corrections from the camera) and
backs out for deliver_back_time. Without use_lease there is no zone arbiter: the robot acts as if it
always held the square and sends no requests.

Corners: corner_hits line events within corner_window without a pose make the next border reaction a
doubled backoff and a half turn (random 135-225 deg turns kept pointing at the other tape).

Body geofence (overhead pose + track polygon; same model as the safety gate): the scoop may step on
the tape, the body may not. Without a cube, a forward move that leaves no room to spin counts as the
border line; a spin with no room tries the other way or backs off first. With a cube the robot follows
a plan from core/planner.py (straights and slow in-place spins that keep the body inside) to the
entry, or to its waiting point without the lease; it replans when it drifts, and with no plan it drops
the cube by backing off. A cube is captured only if a plan to the entry exists from the pose after the
capture push (otherwise it is skipped for skip_time). With the center already outside, the robot spins
toward the track centroid and drives back.

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
from mobile_base.core.planner import Plan, plan_arcs
from mobile_base.core.track import TOLERANCE, Footprint, Track, path_inside, penetration, predict

Command = tuple[float, float, float]
ZERO: Command = (0.0, 0.0, 0.0)


class State(str, Enum):
    IDLE = "IDLE"
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
    approach_turn_min: float = 0.4  # rad/s, slowest spin while centering a cube (slower stalls the motors)
    approach_align: float = 0.15  # |cx_norm| below which the cube counts as straight ahead
    lost_timeout: float = 1.0  # s
    # Far cubes flicker at the detector's area threshold (Webots): smaller targets are left to the
    # exploration (and the cartographer's memory) instead of being approached. 0 = any size.
    approach_min_area: float = 0.0  # px
    # The U is barely twice a cube wide: once the cube is close (cy above approach_strafe_cy) the last
    # centering is a sideways slide, not a spin (a spin swings the scoop's mouth). 0 = off.
    approach_strafe_cy: float = 0.0
    approach_strafe_tol: float = 0.04  # |cx| that counts as centered on the U
    approach_strafe_speed: float = 0.03  # m/s
    # Centering by spinning that lasts longer than this means the target turns WITH the robot (a cube
    # dragged by the outside of a scoop wall, seen in Webots): back off approach_backoff_time to leave
    # it and ignore it for skip_time. 0 = off.
    approach_spin_timeout: float = 0.0  # s
    approach_backoff_time: float = 1.5  # s
    capture_cy: float = 0.62  # cube this low in the image is at the scoop mouth (IMX219-120, measured 0.66)
    capture_speed: float = 0.08  # m/s
    capture_time: float = 1.0  # s
    has_gate: bool = False  # servo gate at the mouth of the U (see the module docstring)
    gate_close_time: float = 0.4  # s, SG90 needs ~0.25 s for 134 deg at 4.8 V
    gate_open_time: float = 0.4  # s
    scoop_lost_timeout: float = 1.0  # s without seeing the cube in the scoop while carrying
    # Transport: straights and slow in-place spins (no arcs: team rule 2026-10-27).
    carry_speed: float = 0.08  # m/s
    carry_max_turn: float = 0.4  # rad/s, spin with a cube in the fixed scoop (keep <= the gate's)
    heading_tolerance: float = 0.1  # rad, heading error that makes the robot spin before translating
    arrive_radius: float = 0.10  # m, waiting point reached
    wait_x: float = 0.0  # m, waiting point while the other robot holds the lease (per robot)
    wait_y: float = 0.22
    request_period: float = 1.0  # s, retry while denied
    enter_speed: float = 0.05  # m/s
    enter_lookahead: float = 0.10  # m, carrot distance along the square axis
    square_align_gain: float = 0.8  # sideways speed (fraction of enter_speed) per unit of square_cx while entering
    reverse_speed: float = 0.04  # m/s
    drop_timeout: float = 6.0  # s, give up reversing (the cube is already inside)
    # The safety gate predicts 0.3 s ahead (1.5 cm at enter_speed) and stops the robot that close to a
    # drop point placed 3 mm from the boundary: start the drop within this distance of it.
    drop_tolerance: float = 0.02  # m
    # Body geofence (keep equal to the gate): see core/track.py.
    track_horizon: float = 0.3  # s
    body_front: float = 0.095  # m
    body_rear: float = 0.095
    body_half_width: float = 0.082
    plan_replan_distance: float = 0.04  # m, drift from the planned pose that triggers a replan
    plan_replan_heading: float = 0.3  # rad
    plan_max_expansions: int = 30000
    plan_retry: float = 2.0  # s before planning again after a failure
    plan_give_up: float = 6.0  # s of failed planning (holding still) before dropping the cube
    # Entry pose the planner lines up to (spins are always allowed now, so the entry is precise).
    gate_entry_across_tolerance: float = 0.02  # m off the axis
    gate_entry_along_band: float = 0.05  # m around zone_entry_offset
    gate_entry_heading_tolerance: float = math.radians(10.0)
    skip_time: float = 3.0  # s, ignore cubes after skipping one at the border
    backoff_time: float = 3.0  # s, longest reverse looking for room to spin
    # m reversed (timed, at reverse_speed) before turning away from the line while exploring or
    # approaching: with the line sensor at the robot center the body front is already past the tape.
    line_backoff: float = 0.0
    # Cubes of other colors (see the module docstring); calibrate avoid_* with the real camera.
    # Guided exploration (core/cartographer.py, estimated pose): head for explore_goal, spinning in place
    # until within explore_heading_tol; stop and turn when the estimated tape is closer than explore_tape_stop.
    explore_tape_stop: float = 0.18  # m ahead of the robot center (body front 0.095 + margin)
    explore_heading_tol: float = 0.3  # rad
    avoid_cy: float = 0.45  # a cube lower in the image than this is about to enter the scoop
    avoid_cx: float = 0.3  # |cx_norm| of the scoop's path at that distance
    avoid_timeout: float = 3.0  # s turning away (~200 deg at turn_speed) before ignoring it for skip_time
    shed_distance: float = 0.08  # m reversed so a cube in the U (48 mm deep) ends past the scoop tips
    shed_confirm: float = 0.3  # s the foreign color must stay in the scoop before backing off
    clear_after: int = 2  # skips of a target behind the same kind of obstacle before pushing through
    clear_window: float = 20.0  # s
    clear_time: float = 5.0  # s pushing through (a cube caught meanwhile is spun out of the scoop)
    clear_turn: float = math.pi / 2  # rad, spin that throws the caught cube aside
    corner_hits: int = 3  # line events within corner_window (no pose): next reaction is a half turn
    corner_window: float = 10.0  # s
    use_lease: bool = True  # zone arbiter (Perry); False: no requests, the square is taken as free
    # Delivery with the robot's camera only (no pose); calibrate square_* and deliver_* on the track.
    square_search_time: float = 15.0  # s spinning without seeing the square before a short drive forward
    square_probe_time: float = 1.5  # s of that drive forward
    square_align: float = 0.15  # |square_cx| below which the square counts as straight ahead
    square_reach_cy: float = 0.6  # square centroid this low in the image: the scoop is at its edge
    # Delivery along an axis (no arcs, team decision 2026-10-30): the robot lines itself up on the
    # perpendicular through the midpoint of the green and red rectangles (centered on them, both at the
    # same apparent height), then drives and pushes straight. False = aim at the blue square only.
    axis_delivery: bool = False
    axis_align: float = 0.1  # |refs_cx| below which the midpoint counts as straight ahead
    axis_skew_tol: float = 0.06  # |refs_skew| below which the robot counts as on the axis
    axis_strafe_speed: float = 0.04  # m/s sideways to get onto the axis (the gate caps it at max_side_speed)
    # Sharing the square without the zone arbiter: the robot with yield_to_peer waits while the other
    # robot's marker is within peer_yield_distance (m) and gives the cube up after yield_timeout s.
    yield_to_peer: bool = False
    peer_yield_distance: float = 0.5
    yield_timeout: float = 10.0
    # Non-target cubes in the way while carrying: strafe around them, give up after dodge_limit tries.
    dodge_limit: int = 3
    # While pushing into the square, a non-target cube this wide in the image (|cx|) and this low (cy) is
    # in front of the scoop's walls too: the push is called off (the cube would go in with it).
    deliver_guard_cx: float = 0.6
    deliver_guard_cy: float = 0.5
    deliver_push_time: float = 2.0  # s pushing in at enter_speed (10 cm)
    deliver_back_time: float = 2.5  # s backing out at reverse_speed (10 cm)
    # The line sensor reads the blue square as white and its track side has no tape: while exploring
    # or approaching a cube (never carrying or delivering), a blue square seen with square_cy at or
    # above this value is reacted to as the line (line_backoff reverse, then spin). 0 disables it.
    square_border_cy: float = 0.0
    open_probe_distance: float = 0.5  # m, straight free distance probed per heading after the border
    # Delivery square (see core/delivery.py); flat so each one is a ROS parameter.
    zone_x: float = DeliveryZone.x
    zone_y: float = DeliveryZone.y
    zone_heading: float = DeliveryZone.heading
    zone_square_depth: float = DeliveryZone.square_depth
    zone_entry_offset: float = DeliveryZone.entry_offset
    zone_drop_offset: float = DeliveryZone.drop_offset
    zone_exit_offset: float = DeliveryZone.exit_offset
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
    # Blue delivery square seen by the robot camera (vision.detect_square); never reported as a cube.
    square_visible: bool = False
    square_cx: float = 0.0  # [-1, 1], left negative
    square_cy: float = 0.0  # [0, 1], 0 = top
    square_area: float = 0.0  # px
    # Cartographer (particle filter + memory), only while its estimate is trusted; None otherwise.
    est_pose: tuple[float, float, float] | None = None
    explore_goal: tuple[float, float] | None = None
    tape_ahead: float | None = None  # m along the heading to the tape, from the estimated pose
    peer_seen: tuple[float, float] | None = None  # the other robot's marker: (bearing rad, distance m)
    refs_visible: bool = False  # green and red rectangles beside the square (vision.detect_refs)
    refs_cx: float = 0.0  # [-1, 1], midpoint between them
    refs_skew: float = 0.0  # left cy - right cy: > 0 means the robot is left of the axis


class BehaviorFSM:
    def __init__(self, params: BehaviorParams, rng: random.Random | None = None, track: Track | None = None):
        self.params = params
        self._track = track or Track(())
        self._body = Footprint(params.body_front, params.body_rear, params.body_half_width)
        self._ignore_until = 0.0
        self._backoff_until: float | None = None
        self._line_backoff_until = 0.0  # timed reverse after the line (works without a pose)
        self.plan: Plan | None = None
        self._plan_t0 = 0.0
        self._plan_kind = ""
        self._plan_fail_until = 0.0
        self._plan_failing_since: float | None = None
        self._zone = params.zone
        self._rng = rng or random.Random()
        self.targets: tuple[str, ...] = tuple(params.my_colors)
        self.avoid: tuple[str, ...] = ()
        self.forbidden_touches = 0  # forbidden cubes caught in the scoop (each one costs points)
        self.state = State.EXPLORE if self.targets else State.IDLE
        self.note = ""  # why the last transition happened (logged by the node)
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
        self.gate_closed = False
        self._gate_ready_at = 0.0
        self._carry_backoff_until = 0.0
        self._foreign_since: float | None = None  # foreign cube seen in the scoop since (not yet confirmed)
        self._avoid_since: float | None = None  # turning away from a foreign cube since
        self._avoid_ignore_until = 0.0
        self._aligning = False  # APPROACH: spinning to center the target (hysteresis)
        self._skips: list[float] = []  # times a target was skipped behind another cube
        self._clear_until = 0.0  # pushing through to a target until then
        self._line_hits: list[float] = []  # border reactions, for the corner guard
        self._search_since: float | None = None  # camera carry: spinning without the square since
        self._probe_until = 0.0  # camera carry: driving forward to look from elsewhere until
        self._push_until = 0.0  # camera deliver: pushing in until
        self._guide_turning = False  # guided exploration: spinning toward the goal
        self._align_since: float | None = None  # APPROACH: spinning to center the target since
        self._yield_since: float | None = None  # camera carry: waiting for the other robot since
        self._dodges = 0  # camera carry: sidesteps around non-target cubes in this transport
        self._dodging = False
        self._on_axis = False  # camera carry: lined up on the delivery axis (the rectangles may leave the view)
        self.delivered = 0  # deliveries counted by this robot (the arbiter keeps the official count)
        self._actions: list[ZoneAction] = []

    @property
    def carrying(self) -> bool:
        return self.state is State.CARRY or (self.state is State.DELIVER and not self._dropping)

    def stop(self) -> None:
        self.state = State.STOP

    def set_mission(self, targets: Sequence[str], avoid: Sequence[str] = ()) -> None:
        """Colors to collect and colors whose touch costs points; no targets = stay still (IDLE).

        A cube already carried that is no longer a target stops counting as held, so the transport
        ends after scoop_lost_timeout (lease released, nothing delivered). A color cannot be both."""
        both = set(targets) & set(avoid)
        if both:
            raise ValueError(f"color(s) {sorted(both)} both target and forbidden")
        self.targets = tuple(targets)
        self.avoid = tuple(avoid)

    def take_actions(self) -> list[ZoneAction]:
        """Zone arbiter calls requested since the last call (the node performs them)."""
        actions, self._actions = self._actions, []
        return actions

    def step(self, inp: BehaviorInput) -> Command:
        if self.state is not State.STOP:
            if not self.targets:
                if self.state is not State.IDLE:
                    if self.state in (State.CARRY, State.DELIVER):
                        self._give_up_cube(inp)
                        self._dropping = False
                    self.state = State.IDLE
                    self.note = "no mission"
                    self._last_request = inp.now
                elif inp.holds_zone and inp.now - (self._last_request or 0.0) >= self.params.request_period:
                    # A lease granted after the mission was cleared (request in flight): give it back.
                    self._zone_action(ZoneAction("release", delivered=False))
                    self._last_request = inp.now
                return ZERO
            if self.state is State.IDLE:
                self.state = State.EXPLORE
                self.note = "mission received"
        if not self.params.use_lease:
            inp = dataclasses.replace(inp, holds_zone=True)  # no arbiter: the square is always ours
        if self.state not in (State.DELIVER, State.STOP) and self._track.enabled and inp.pose is not None:
            x, y, theta = inp.pose
            if not self._track.contains(x, y):
                return self._return_to_track(inp)
            if self.state in (State.EXPLORE, State.CAPTURE) and not self._fits(
                inp.pose, self.params.explore_speed, 0.0, carrying=False
            ):
                # No room ahead for the body: same as the border, except that a visible cube is still
                # approached (APPROACH handles the geofence itself; the scoop reaches past the body).
                edge_only = not inp.line_front
                inp = dataclasses.replace(inp, line_front=True)
                if edge_only and self.state is State.EXPLORE and self._target(inp.cubes, inp.now) is not None:
                    inp = dataclasses.replace(inp, line_front=False)
        p = self.params
        foreign_in_scoop = (
            self.state in (State.EXPLORE, State.APPROACH, State.CAPTURE)
            and self._turn_until is None  # not while already backing off and turning away from it
            and inp.scoop_color
            and not is_mine(inp.scoop_color, self.targets)
        )
        if not foreign_in_scoop:
            self._foreign_since = None
        else:
            # Confirmed over shed_confirm: red and orange are neighbor hues, one misread frame must not abort a capture.
            self._foreign_since = inp.now if self._foreign_since is None else self._foreign_since
            if inp.now - self._foreign_since >= p.shed_confirm:
                self._foreign_since = None
                if inp.scoop_color in self.avoid:
                    self.forbidden_touches += 1
                self.state = State.EXPLORE
                if inp.now < self._clear_until and not inp.line_front:
                    # Clearing the way to a target: spin the cube out of the scoop, off the path.
                    self._clear_until = 0.0
                    self.note = f"{inp.scoop_color} cube pushed aside"
                    self._start_fixed_turn(inp.now, inp.pose, p.clear_turn)
                else:
                    self.note = f"{inp.scoop_color} cube in the scoop: back off"
                    # On the line the body must also get back inside before the spin (line_backoff).
                    backoff = max(p.shed_distance, p.line_backoff) if inp.line_front else p.shed_distance
                    self._start_line_turn(inp.now, inp.pose, backoff)
        handler = {
            State.EXPLORE: self._explore,
            State.APPROACH: self._approach,
            State.CAPTURE: self._capture,
            State.CARRY: self._carry,
            State.DELIVER: self._deliver,
            State.STOP: lambda _inp: ZERO,
            State.IDLE: lambda _inp: ZERO,
        }[self.state]
        return handler(inp)

    def _fits(self, pose, vx: float, wz: float, carrying: bool, vy: float = 0.0) -> bool:
        """Same test as SafetyGate._body_fits (carrying kept for symmetry with the gate)."""
        if pose is None or not self._track.enabled:
            return True
        return path_inside(self._track, self._body, pose, vx, wz, self.params.track_horizon, vy=vy)

    def _at_entry(self, pose) -> bool:
        p = self.params
        z = self._zone
        error = abs(math.remainder(pose[2] - z.heading, 2.0 * math.pi))
        if self._track.enabled:
            # The plan may spin in place, so it lines up precisely: the square is only 22 cm wide
            # inside and the body may not touch its tape (Webots run 19: 5 cm and 27 deg off the
            # axis, a body corner on the tape, stuck for 7 minutes).
            along, across = z.along_across(pose[0], pose[1])
            return (
                abs(across) <= p.gate_entry_across_tolerance
                and abs(-along - z.entry_offset) <= p.gate_entry_along_band
                and error <= p.gate_entry_heading_tolerance
            )
        return z.in_corridor(pose[0], pose[1]) and error <= p.gate_entry_heading_tolerance

    def _plan_to(self, kind: str, pose) -> Plan | None:
        p = self.params
        z = self._zone
        if kind == "entry":
            goal, target = self._at_entry, z.entry_point
        else:
            def goal(q):
                return distance(q[0], q[1], p.wait_x, p.wait_y) <= p.arrive_radius

            target = (p.wait_x, p.wait_y)
        return plan_arcs(
            self._track, self._body, pose, goal, target,
            straight_speed=p.carry_speed, arc_speed=None, turn=p.carry_max_turn,
            max_expansions=p.plan_max_expansions,
            # With the gate the cube is retained: fast spins and reverse; without it, slow spins only.
            spin_speed=p.turn_speed if p.has_gate else p.carry_max_turn,
            reverse_speed=p.reverse_speed if p.has_gate else None,
        )

    def _deliverable_from(self, pose) -> bool:
        """After the capture push, is there a way to the entry with the body inside the track?"""
        p = self.params
        if pose is None or not self._track.enabled:
            return True
        after = predict(pose, p.capture_speed, 0.0, p.capture_time)
        return self._plan_to("entry", after) is not None

    def _follow_plan(self, inp: BehaviorInput, kind: str) -> Command | None:
        """Command from the current plan; replan when drifting, finished or for a new goal."""
        p = self.params
        if self.plan is not None and self._plan_kind == kind:
            t = inp.now - self._plan_t0
            expected = self.plan.expected(t)
            cmd = self.plan.command(t)
            drift = distance(expected[0], expected[1], inp.pose[0], inp.pose[1])
            heading = abs(math.remainder(expected[2] - inp.pose[2], 2.0 * math.pi))
            if cmd is not None and drift <= p.plan_replan_distance and heading <= p.plan_replan_heading:
                return (cmd[0], 0.0, cmd[1])
        if inp.now < self._plan_fail_until and self._plan_kind == kind:
            return None
        self._plan_kind = kind
        self.plan = self._plan_to(kind, inp.pose)
        if self.plan is None:
            self._plan_fail_until = inp.now + p.plan_retry
            return None
        self._plan_t0 = inp.now
        cmd = self.plan.command(0.0)
        if cmd is None:  # already there
            return ZERO
        return (cmd[0], 0.0, cmd[1])

    def _return_to_track(self, inp: BehaviorInput) -> Command:
        if self.state is State.CARRY:
            self._give_up_cube(inp)
        self.state = State.EXPLORE
        self._turn_until = None
        self._line_backoff_until = 0.0
        x, y, theta = inp.pose
        cx, cy = self._track.centroid
        err = relative_bearing(x, y, theta, cx, cy)
        if abs(err) > 0.3:
            return (0.0, 0.0, math.copysign(self.params.turn_speed, err))
        return (self.params.explore_speed, 0.0, 0.0)

    def _still_holding(self, inp: BehaviorInput) -> bool:
        if is_mine(inp.scoop_color, self.targets):
            self._scoop_seen = inp.now
        return inp.now - self._scoop_seen <= self.params.scoop_lost_timeout

    def _abandon_transport(self, inp: BehaviorInput, why: str) -> Command:
        self.note = why
        self.plan = None
        self._on_axis = False
        self._yield_since, self._dodges, self._dodging = None, 0, False
        self._give_up_cube(inp)
        self.state = State.EXPLORE
        self._start_turn(inp.now, inp.pose)
        return self._explore(inp)

    def _give_up_cube(self, inp: BehaviorInput) -> None:
        self.gate_closed = False  # backing off must leave the cube behind
        if inp.holds_zone:
            self._zone_action(ZoneAction("release", delivered=False, color=self._carried_color))
        self._carried_color = ""

    def _zone_action(self, action: ZoneAction) -> None:
        if self.params.use_lease:
            self._actions.append(action)

    def _target(self, cubes: Sequence[Cube], now: float) -> Cube | None:
        if now < self._ignore_until:
            return None
        mine = [c for c in cubes if is_mine(c.color, self.targets) and c.area_px >= self.params.approach_min_area]
        return max(mine, key=lambda c: c.area_px, default=None)

    def _blocking(self, cubes: Sequence[Cube]) -> Cube | None:
        """Closest cube of another color in the scoop's path."""
        p = self.params
        ahead = [
            c for c in cubes
            if not is_mine(c.color, self.targets) and c.cy_norm > p.avoid_cy and abs(c.cx_norm) < p.avoid_cx
        ]
        return max(ahead, key=lambda c: c.cy_norm, default=None)

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
        open_heading = self._open_heading(pose)
        if open_heading is not None:
            # With the track polygon, aim at open space: a random angle in the narrow waist often
            # pointed at the tape again (kinematic harness: 16 s spinning in place).
            diff = math.remainder(open_heading - pose[2], 2.0 * math.pi)
            self._turn_dir = 1.0 if diff >= 0.0 else -1.0
            self._turn_goal = max(abs(diff), 0.05)
        self._turn_prev_theta = pose[2]
        self._turn_until = now + 2.0 * self._turn_goal / p.turn_speed  # guard if the pose freezes
        self._check_turn_room(now, pose)

    def _start_fixed_turn(self, now: float, pose, angle: float) -> None:
        """Spin `angle` rad either way (measured with the pose, timed without it)."""
        p = self.params
        self._turn_dir = self._rng.choice((-1.0, 1.0))
        self._turn_goal = angle if pose is not None else None
        self._turn_prev_theta = pose[2] if pose is not None else 0.0
        self._turn_until = now + (2.0 if pose is not None else 1.0) * angle / p.turn_speed

    def _start_line_turn(self, now: float, pose, backoff: float | None = None) -> None:
        """Turn away, reversing backoff (line_backoff by default) first (timed, so it also works with pose=None).

        A border reaction (backoff None) also feeds the corner guard: without a pose, corner_hits of
        them within corner_window make this one a doubled backoff and a half turn. The gate stops the
        longer reverse if it reaches the tape behind."""
        p = self.params
        if backoff is None:
            backoff = p.line_backoff
            self._line_hits = [t for t in self._line_hits if now - t <= p.corner_window] + [now]
            if pose is None and len(self._line_hits) >= p.corner_hits:
                self._line_hits = []
                self.note = "corner: back off further and turn around"
                self._start_fixed_turn(now, pose, math.pi)
                backoff *= 2.0
            else:
                self._start_turn(now, pose)
        else:
            self._start_turn(now, pose)
        if backoff > 0.0:
            reverse = backoff / p.reverse_speed
            self._line_backoff_until = now + reverse
            self._turn_until += reverse  # the turn (and its timeout) starts after the reverse

    def _free_distance(self, pose, heading: float) -> float:
        """How far the body can go straight along heading and stay inside the track."""
        p = self.params
        x, y, _ = pose
        free = 0.0
        while free < p.open_probe_distance:
            nxt = free + 0.02
            probe = (x + nxt * math.cos(heading), y + nxt * math.sin(heading), heading)
            if penetration(self._track, self._body, probe) > TOLERANCE:
                break
            free = nxt
        return free

    def _open_heading(self, pose) -> float | None:
        """A heading (random among the most open of 12) to explore after the border, or None."""
        if pose is None or not self._track.enabled:
            return None
        scored = [(self._free_distance(pose, k * math.pi / 6.0), k * math.pi / 6.0) for k in range(12)]
        best = max(free for free, _ in scored)
        if best <= 0.0:
            return None
        return self._rng.choice([h for free, h in scored if free >= 0.6 * best])

    def _room_to_spin(self, pose) -> bool:
        """The whole 135-225 deg turn fits: the corners sweep a circle of sweep_radius.

        Checking only the next 0.3 s let a turn start and get stuck halfway (Webots run 12: Phineas
        rocked left and right in a corner for a minute)."""
        if pose is None or not self._track.enabled:
            return True
        return self._track.distance_to_edge(pose[0], pose[1]) >= self._body.sweep_radius

    def _check_turn_room(self, now: float, pose) -> None:
        """No room for the full spin: back off (pose-checked reverse) until there is, at most backoff_time."""
        p = self.params
        if self._room_to_spin(pose):
            return
        self._backoff_until = now + p.backoff_time
        self._turn_until = max(self._turn_until, self._backoff_until + 2.0 * (self._turn_goal or 0.0) / p.turn_speed)

    def _turn_done(self, inp: BehaviorInput) -> bool:
        if self._turn_goal is not None and inp.pose is not None:
            self._turn_goal -= abs(wrap_angle(inp.pose[2] - self._turn_prev_theta))
            self._turn_prev_theta = inp.pose[2]
            if self._turn_goal <= 0.0:
                return True
        return inp.now >= self._turn_until

    def _square_ahead(self, inp: BehaviorInput) -> bool:
        """The blue square is close in the camera: a border for the body (only the scoop may enter)."""
        cy = self.params.square_border_cy
        return cy > 0.0 and inp.square_visible and inp.square_cy >= cy

    def _explore(self, inp: BehaviorInput) -> Command:
        p = self.params
        if self._backoff_until is not None and (inp.now >= self._backoff_until or self._room_to_spin(inp.pose)):
            self._backoff_until = None  # room found (or gave up backing off): spin now
            if inp.pose is not None:
                self._turn_prev_theta = inp.pose[2]
        if self._turn_until is not None and self._turn_done(inp):
            self._turn_until = None
        if self._turn_until is None:
            if inp.line_front or self._square_ahead(inp):
                self._start_line_turn(inp.now, inp.pose)
            elif self._target(inp.cubes, inp.now) is not None:
                self.state = State.APPROACH
                self._last_seen = inp.now
                self._aligning = False
                return self._approach(inp)
        if inp.now < self._line_backoff_until:
            return (-p.reverse_speed, 0.0, 0.0)  # body back inside the tape before spinning
        if self._backoff_until is not None:
            return (-p.reverse_speed, 0.0, 0.0)  # no room to spin: back off first (leaves any cube)
        if self._turn_until is not None:
            return (0.0, 0.0, self._turn_dir * p.turn_speed)
        obstacle = self._blocking(inp.cubes) if inp.now >= self._avoid_ignore_until else None
        if obstacle is None:
            self._avoid_since = None
        else:
            self._avoid_since = inp.now if self._avoid_since is None else self._avoid_since
            if inp.now - self._avoid_since <= p.avoid_timeout:
                # Turn away from it (cube on the right, cx > 0: counterclockwise) until it leaves the path.
                return (0.0, 0.0, math.copysign(p.turn_speed, obstacle.cx_norm))
            # Still ahead after turning that long: it turns with the robot (stuck in the scoop, or a
            # misread part of the robot), so dodging it never ends. Ignore it for a while.
            self._avoid_ignore_until = inp.now + p.skip_time
            self._avoid_since = None
        if inp.est_pose is not None:
            guided = self._guided(inp)
            if guided is not None:
                return guided
        return (p.explore_speed, 0.0, 0.0)

    def _guided(self, inp: BehaviorInput) -> Command | None:
        """Star motion toward the cartographer's goal: spin in place to face it, then straight. The tape
        is anticipated from the estimate (the line sensor still stops the robot if the estimate is off)."""
        p = self.params
        x, y, th = inp.est_pose
        gx, gy = inp.explore_goal if inp.explore_goal is not None else (0.0, 0.0)  # no goal: the center
        err = math.remainder(math.atan2(gy - y, gx - x) - th, 2.0 * math.pi)
        near_tape = inp.tape_ahead is not None and inp.tape_ahead < p.explore_tape_stop
        if near_tape and abs(err) < p.explore_heading_tol:
            err = math.pi / 2.0  # the goal is past the tape (estimate off): turn away anyway
        if near_tape or abs(err) > p.explore_heading_tol or (self._guide_turning and abs(err) > p.explore_heading_tol / 2.0):
            self._guide_turning = True
            if near_tape:
                self.note = "tape ahead (estimated): turn before touching it"
            return (0.0, 0.0, math.copysign(p.turn_speed, err))
        self._guide_turning = False
        return None

    def _approach(self, inp: BehaviorInput) -> Command:
        p = self.params
        target = self._target(inp.cubes, inp.now)
        if target is None:
            if inp.now - self._last_seen > p.lost_timeout:
                self.state = State.EXPLORE
                self.note = "target lost"
                return self._explore(inp)
            return ZERO
        self._last_seen = inp.now
        self._target_color = target.color
        square = self._square_ahead(inp)
        if inp.line_front or square:
            # Cube on or beyond the border: give up and turn away. Checked before the capture
            # (Webots run 8: a cube just past the edge looped APPROACH <-> CAPTURE for minutes).
            self.state = State.EXPLORE
            self.note = "border before the cube" if inp.line_front else "blue square ahead"
            self._start_line_turn(inp.now, inp.pose)
            return self._explore(inp)
        obstacle = self._blocking(inp.cubes)
        if obstacle is not None and obstacle.cy_norm > target.cy_norm and inp.now >= self._clear_until:
            recent = [t for t in self._skips if inp.now - t <= p.clear_window]
            if len(recent) >= p.clear_after:
                # The target keeps hiding behind other cubes: they may be moved, so push through.
                self._skips = []
                self._clear_until = inp.now + p.clear_time
                self.note = f"{obstacle.color} cube in the way again: push it aside"
            else:
                # Pushing on would take the other cube along: skip the target and go around.
                self._skips = recent + [inp.now]
                self._ignore_until = inp.now + p.skip_time
                self.state = State.EXPLORE
                self.note = f"{obstacle.color} cube in the way"
                return self._explore(inp)
        if abs(target.cx_norm) > p.approach_align:
            self._aligning = True
        elif abs(target.cx_norm) < p.approach_align / 2.0:
            self._aligning = False  # hysteresis: no spin/drive chatter around approach_align
        if not self._aligning:
            self._align_since = None
        elif p.approach_spin_timeout > 0.0:
            self._align_since = inp.now if self._align_since is None else self._align_since
            if inp.now - self._align_since > p.approach_spin_timeout:
                self._align_since = None
                self._ignore_until = inp.now + p.skip_time
                self._line_backoff_until = inp.now + p.approach_backoff_time  # EXPLORE reverses first
                self.state = State.EXPLORE
                self.note = f"{target.color} cube turns with the robot: back off and leave it"
                return (-p.reverse_speed, 0.0, 0.0)
        if self._aligning:
            turn = -math.copysign(max(p.k_turn * abs(target.cx_norm), p.approach_turn_min), target.cx_norm)
            if inp.pose is None or self._fits(inp.pose, 0.0, turn, carrying=False):
                return (0.0, 0.0, turn)
        if (p.approach_strafe_cy > 0.0 and not self._aligning and target.cy_norm > p.approach_strafe_cy
                and abs(target.cx_norm) > p.approach_strafe_tol):
            self.note = "centering the cube on the scoop"
            return (0.0, -math.copysign(p.approach_strafe_speed, target.cx_norm), 0.0)  # cube right: slide right
        if target.cy_norm > p.capture_cy and not self._aligning:
            if not self._deliverable_from(inp.pose):
                # Captured here, the robot would face the tape with no move that keeps its body in.
                self._ignore_until = inp.now + p.skip_time
                self.state = State.EXPLORE
                self.note = "capture skipped: no plan to the square from there"
                self._start_turn(inp.now, inp.pose)
                return self._explore(inp)
            self.state = State.CAPTURE
            self._capture_until = inp.now + p.capture_time
            return self._capture(inp)
        cmd = (p.approach_speed, 0.0, 0.0)
        if self._aligning or (inp.pose is not None and not self._fits(inp.pose, cmd[0], 0.0, carrying=False)):
            # No room to spin toward it, or the body cannot advance with the cube centered: give up.
            self._ignore_until = inp.now + p.skip_time
            self.state = State.EXPLORE
            self.note = "border before the cube"
            self._start_turn(inp.now, inp.pose)
            return self._explore(inp)
        return cmd

    def _capture(self, inp: BehaviorInput) -> Command:
        if inp.line_front:
            self.state = State.EXPLORE
            self.note = "border during capture"
            self._start_turn(inp.now, inp.pose)
            return self._explore(inp)
        if inp.now >= self._capture_until:
            if not is_mine(inp.scoop_color, self.targets):
                # Pushed but nothing of mine in the U: the cube slipped aside. Look for it again.
                self.state = State.APPROACH
                self.note = "capture failed: scoop empty"
                self._last_seen = inp.now
                return self._approach(inp)
            self.state = State.CARRY
            self._carried_color = inp.scoop_color
            self._scoop_seen = inp.now
            self._last_request = None
            if self.params.has_gate:
                self.gate_closed = True
                self._gate_ready_at = inp.now + self.params.gate_close_time
            return self._carry(inp)
        return (self.params.capture_speed, 0.0, 0.0)

    def _steer(self, pose, gx: float, gy: float, speed: float) -> Command:
        """Spin in place (slowly: a cube may be in the scoop) until facing (gx, gy), then go straight."""
        face = self._face(pose, math.atan2(gy - pose[1], gx - pose[0]))
        return face if face != ZERO else (speed, 0.0, 0.0)

    def _face(self, pose, heading: float) -> Command:
        """Slow spin toward heading; ZERO once within heading_tolerance."""
        p = self.params
        err = math.remainder(heading - pose[2], 2.0 * math.pi)
        if abs(err) <= p.heading_tolerance:
            return ZERO
        return (0.0, 0.0, math.copysign(p.carry_max_turn, err))

    def _ask_lease(self, inp: BehaviorInput) -> None:
        if inp.holds_zone:
            return
        if self._last_request is None or inp.now - self._last_request >= self.params.request_period:
            self._zone_action(ZoneAction("request"))
            self._last_request = inp.now

    def _carry(self, inp: BehaviorInput) -> Command:
        p = self.params
        z = self._zone
        if inp.line_front and not self.gate_closed:
            # The gate turns in place at the line and the turn drops the cube: stop carrying.
            return self._abandon_transport(inp, "line while carrying")
        if not self._still_holding(inp):
            return self._abandon_transport(inp, "cube lost from the scoop")
        if inp.now < self._gate_ready_at:
            return ZERO  # the gate is still closing
        if inp.now < self._carry_backoff_until:
            return (-p.reverse_speed, 0.0, 0.0)  # cube retained by the gate
        if inp.pose is None:
            return self._carry_by_camera(inp)
        x, y, theta = inp.pose
        self._ask_lease(inp)
        if not inp.holds_zone and distance(x, y, p.wait_x, p.wait_y) <= p.arrive_radius:
            return ZERO  # the other robot is using the square
        if inp.holds_zone and self._at_entry(inp.pose):
            # Same heading tolerance the planner aims for, not the corridor's 60 deg: entering
            # diagonally put a body corner on the square's tape (Webots run 17).
            self.state = State.DELIVER
            self._dropping = False
            return self._deliver(inp)
        if self._track.enabled:
            kind = "entry" if inp.holds_zone else "wait"
            cmd = self._follow_plan(inp, kind)
            if cmd is not None:
                self._plan_failing_since = None
                return cmd
            if kind == "wait":
                return ZERO  # cannot reach the waiting point: wait here
            if self._plan_failing_since is None:
                self._plan_failing_since = inp.now
            if inp.now - self._plan_failing_since < p.plan_give_up:
                return ZERO  # hold still with the cube and retry (drift, pose noise, the other robot)
            self._plan_failing_since = None
            return self._abandon_transport(inp, "no plan to the square: cube dropped")
        if not inp.holds_zone:
            return self._steer(inp.pose, p.wait_x, p.wait_y, p.carry_speed)
        if z.in_corridor(x, y):
            return self._face(inp.pose, z.heading)  # in front of the square: face it, then DELIVER
        return self._steer(inp.pose, *z.entry_point, p.carry_speed)

    def _carry_by_camera(self, inp: BehaviorInput) -> Command:
        """No pose: find the blue square with the camera, center it, drive to it (slow spins only)."""
        p = self.params
        turn = p.turn_speed if self.gate_closed else p.carry_max_turn
        if inp.now < self._probe_until:
            return (p.carry_speed, 0.0, 0.0)  # looking for the square from somewhere else
        if (p.yield_to_peer and inp.peer_seen is not None and inp.peer_seen[1] <= p.peer_yield_distance):
            self._yield_since = inp.now if self._yield_since is None else self._yield_since
            if inp.now - self._yield_since > p.yield_timeout:
                return self._abandon_transport(inp, "the other robot blocks the way to the square")
            self.note = "waiting for the other robot"
            return ZERO
        self._yield_since = None
        axis = p.axis_delivery
        # Close to the square the rectangles leave the view: once on the axis, the square is enough.
        seen = inp.refs_visible or (self._on_axis and inp.square_visible) if axis else inp.square_visible
        if not seen:
            self._on_axis = False
            self._search_since = inp.now if self._search_since is None else self._search_since
            if inp.now - self._search_since >= p.square_search_time:
                self._search_since = None
                self._probe_until = inp.now + p.square_probe_time
                self.note = "square not seen: drive forward and look again"
                return (p.carry_speed, 0.0, 0.0)
            return (0.0, 0.0, turn)
        self._search_since = None
        if axis and inp.refs_visible:
            # Two decoupled corrections, no arcs: spin to center the midpoint, strafe to even the rectangles.
            if abs(inp.refs_cx) > p.axis_align:
                self._on_axis = False
                return (0.0, 0.0, -math.copysign(turn, inp.refs_cx))
            if abs(inp.refs_skew) > p.axis_skew_tol:
                self._on_axis = False
                self.note = "getting onto the delivery axis"
                return (0.0, -math.copysign(p.axis_strafe_speed, inp.refs_skew), 0.0)
            self._on_axis = True
        elif not axis and abs(inp.square_cx) > p.square_align:
            return (0.0, 0.0, -math.copysign(turn, inp.square_cx))  # square on the right: spin right
        obstacle = self._blocking(inp.cubes)
        if obstacle is not None:
            if not self._dodging:
                self._dodges += 1
            self._dodging = True
            if self._dodges > p.dodge_limit:
                return self._abandon_transport(inp, "a non-target cube keeps blocking the way")
            self.note = f"{obstacle.color} cube ahead: sidestep"
            return (0.0, math.copysign(p.axis_strafe_speed, obstacle.cx_norm if obstacle.cx_norm else 1.0), 0.0)
        self._dodging = False
        if not (inp.square_visible and inp.square_cy >= p.square_reach_cy):
            return (p.carry_speed, 0.0, 0.0)
        self._ask_lease(inp)
        if not inp.holds_zone:
            return ZERO  # the other robot is using the square
        self.state = State.DELIVER
        self._dropping = False
        self._push_until = inp.now + p.deliver_push_time
        self.note = "at the blue square: push the cube in"
        return self._deliver(inp)

    def _deliver_by_camera(self, inp: BehaviorInput) -> Command:
        """No pose: push in for deliver_push_time (centering the square sideways), back out, count."""
        p = self.params
        if not self._dropping:
            if not self._still_holding(inp):
                return self._abandon_transport(inp, "cube lost while entering")
            if inp.now < self._push_until:
                guard = [c for c in inp.cubes if not is_mine(c.color, self.targets)
                         and c.cy_norm > p.deliver_guard_cy and abs(c.cx_norm) < p.deliver_guard_cx]
                if guard:  # pushing on would take it into the square
                    return self._abandon_transport(inp, "non-target cube in the push path")
                if inp.scoop_color and not is_mine(inp.scoop_color, self.targets):
                    # No grace period here (unlike CARRY): one more push puts the foreign cube in the square.
                    return self._abandon_transport(inp, f"{inp.scoop_color} cube in the scoop while entering")
                vy = 0.0
                if inp.square_visible and not p.axis_delivery:  # on the axis the push is straight
                    vy = max(-p.enter_speed, min(p.enter_speed, -p.square_align_gain * p.enter_speed * inp.square_cx))
                return (p.enter_speed, vy, 0.0)
            self._dropping = True
            self._drop_until = inp.now + p.deliver_back_time
            if self.gate_closed:
                self.gate_closed = False  # open the door, then reverse out leaving the cube
                self._gate_ready_at = inp.now + p.gate_open_time
                self._drop_until += p.gate_open_time
        if inp.now < self._gate_ready_at:
            return ZERO  # the gate is still opening
        if inp.now < self._drop_until:
            return (-p.reverse_speed, 0.0, 0.0)
        return self._finish_delivery(inp)

    def _finish_delivery(self, inp: BehaviorInput) -> Command:
        # Counted only if the cube stayed in the square: it must have left the scoop.
        left_behind = not is_mine(inp.scoop_color, self.targets)
        self.delivered += left_behind
        self._zone_action(ZoneAction("release", delivered=left_behind, color=self._carried_color))
        self._dropping = False
        self._on_axis = False
        self._yield_since, self._dodges, self._dodging = None, 0, False
        self._carried_color = ""
        self.state = State.EXPLORE
        self.note = "delivered" if left_behind else "delivery failed: the cube is still in the scoop"
        self._start_turn(inp.now, inp.pose)  # facing the square: turn away before exploring
        return self._explore(inp)

    def _deliver(self, inp: BehaviorInput) -> Command:
        p = self.params
        z = self._zone
        if inp.pose is None:
            return self._deliver_by_camera(inp)
        x, y, theta = inp.pose
        if not inp.holds_zone:
            self._ask_lease(inp)  # lease expired mid-maneuver: the gate blocks the tape and the reverse
        if not self._dropping:
            if not self._still_holding(inp):
                return self._abandon_transport(inp, "cube lost while entering")
            if not z.in_corridor(x, y):
                self.state = State.CARRY  # drifted out of the corridor: line up again
                return self._carry(inp)
            before = z.before(x, y)
            if before > z.drop_offset + p.drop_tolerance:
                # The square's tape is crossed on purpose here (by the scoop): line_front is ignored.
                face = self._face(inp.pose, z.heading)
                if face != ZERO:
                    return face  # heading drifted off the axis: spin back before going on
                carrot = max(before - p.enter_lookahead, z.drop_offset - p.enter_lookahead)
                bearing = relative_bearing(x, y, theta, *z.point(carrot))
                vx, vy = p.enter_speed * math.cos(bearing), p.enter_speed * math.sin(bearing)
                if inp.square_visible:
                    # The camera sees the blue square: center it sideways (pose error across the axis).
                    vy -= p.square_align_gain * p.enter_speed * inp.square_cx
                    vy = max(-p.enter_speed, min(p.enter_speed, vy))
                if self.gate_closed and not self._fits(inp.pose, vx, 0.0, carrying=False, vy=vy):
                    # The body would touch the square's tape: back off with the retained cube, replan.
                    self.state = State.CARRY
                    self.note = "entry blocked: back off and replan"
                    self.plan = None
                    self._carry_backoff_until = inp.now + p.backoff_time / 3.0
                    return (-p.reverse_speed, 0.0, 0.0)
                return (vx, vy, 0.0)
            self._dropping = True
            self._drop_until = inp.now + p.drop_timeout
            if self.gate_closed:
                self.gate_closed = False  # open the door, then reverse out leaving the cube
                self._gate_ready_at = inp.now + p.gate_open_time
                self._drop_until += p.gate_open_time
        if inp.now < self._gate_ready_at:
            return ZERO  # the gate is still opening
        if z.before(x, y) < z.exit_offset and inp.now < self._drop_until:
            return (-p.reverse_speed, 0.0, 0.0)
        return self._finish_delivery(inp)
