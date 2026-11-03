"""
The collector's full state machine on top of the approach controller. Pure
logic, no rclpy: the behavior_fsm node passes in what it sees (`Inputs`)
and publishes what comes out (`Output`).

SEARCH / APPROACH / ARRIVED (approach.py)
  -> CATCH: blind push into the scoop (the cube hides under it), then
     optional confirmation by the scoop sensors
  -> CARRY: drive to the delivery area (zone/seen), asking for the zone
     turn when close
  -> WAIT_ZONE: stopped outside the area until the turn is granted
  -> DELIVER: blind push into the area; the count goes up
  -> RELEASE: scoop up, back up leaving the cube, turn, scoop down
  -> SEARCH

LINE_ESCAPE overrides everything: on black tape the robot backs up (while
the sensor still sees black, up to a limit), turns, and resumes what it was
doing. The delivery area is a gap in the tape, so delivering never needs to
cross it; touching black there means the robot was misaligned and retries.
The escape always backs up: the sensor is at the front, so the robot only
reaches the tape driving forward.
"""

from dataclasses import dataclass, field

from mobile_base.approach import (
    SEARCH_DIR_LEFT,
    SEARCH_DIR_RIGHT,
    ApproachController,
    Command,
    Cube,
    drive_toward,
    turn_toward,
)
from mobile_base.states import State

# --- Defaults (the real values live in config/collector.yaml) -----------------
# Catch only when the cube is this straight ahead (the scoop is 1 cube wide).
DEFAULT_CATCH_ALIGN_TOLERANCE_RAD = 0.2
# Blind push from stop_distance_m until the cube is in the scoop
# (tools/teleop_web.yaml: scoop_push_s).
DEFAULT_CATCH_PUSH_S = 0.65
DEFAULT_CATCH_SPEED_M_S = 0.08
# How long to wait for the scoop sensors after the push.
DEFAULT_CONFIRM_TIMEOUT_S = 1.0
# Ask for the zone turn this far before the arrival point.
DEFAULT_ZONE_REQUEST_MARGIN_M = 0.45
# Without the turn, wait this far before the arrival point (rule: more
# than 30 cm from the area).
DEFAULT_ZONE_WAIT_MARGIN_M = 0.30
# Area paper in view: arrived when its near edge is this close or this low
# in the image (~18.5 cm, where the scoop starts hiding things).
DEFAULT_FLOOR_STOP_DISTANCE_M = 0.19
DEFAULT_ZONE_ARRIVE_V_PX = 445
# Only the landmarks in view: arrived at this distance from them
# (tools/teleop_web.yaml: zone_stop_distance_m).
DEFAULT_GATE_STOP_DISTANCE_M = 0.42
DEFAULT_DELIVER_PUSH_S = 1.4
DEFAULT_DELIVER_SPEED_M_S = 0.08
DEFAULT_RELEASE_BACKUP_S = 1.6
DEFAULT_RELEASE_TURN_S = 2.0
DEFAULT_BACKUP_SPEED_M_S = 0.08
DEFAULT_LINE_BACKUP_S = 0.5
DEFAULT_LINE_BACKUP_MAX_S = 1.5
DEFAULT_LINE_TURN_S = 1.2
DEFAULT_ESCAPE_TURN_RAD_S = 0.6

# Name of the area estimate made from the area paper (vision/zone.py).
ZONE_SOURCE_FLOOR = "floor"

# --- Catch verdicts -----------------------------------------------------------
VERDICT_WAIT = "wait"
VERDICT_CARRY = "carry"
VERDICT_REJECT = "reject"
VERDICT_MISSED = "missed"

# States that steer by the camera (they stop when vision is stale).
VISION_STATES = (State.SEARCH, State.APPROACH, State.ARRIVED, State.CARRY)


@dataclass
class CollectorParams:
    """
    Tunable parameters of `CollectorFsm` (the approach has its own).

    Field names double as ROS parameter names (see recolector_common.params).
    """

    catch_align_tolerance_rad: float = DEFAULT_CATCH_ALIGN_TOLERANCE_RAD
    catch_push_s: float = DEFAULT_CATCH_PUSH_S
    catch_speed_m_s: float = DEFAULT_CATCH_SPEED_M_S
    confirm_timeout_s: float = DEFAULT_CONFIRM_TIMEOUT_S
    # FC-51 wired: a catch counts only if it sees the cube.
    require_has_cube: bool = False
    # TCS34725 wired: a cube of another colour is left behind.
    confirm_color: bool = False
    zone_request_margin_m: float = DEFAULT_ZONE_REQUEST_MARGIN_M
    zone_wait_margin_m: float = DEFAULT_ZONE_WAIT_MARGIN_M
    floor_stop_distance_m: float = DEFAULT_FLOOR_STOP_DISTANCE_M
    zone_arrive_v_px: int = DEFAULT_ZONE_ARRIVE_V_PX
    gate_stop_distance_m: float = DEFAULT_GATE_STOP_DISTANCE_M
    # Without Perry's zone_arbiter (simulation), deliver without a turn.
    require_zone_grant: bool = True
    deliver_push_s: float = DEFAULT_DELIVER_PUSH_S
    deliver_speed_m_s: float = DEFAULT_DELIVER_SPEED_M_S
    release_backup_s: float = DEFAULT_RELEASE_BACKUP_S
    release_turn_s: float = DEFAULT_RELEASE_TURN_S
    backup_speed_m_s: float = DEFAULT_BACKUP_SPEED_M_S
    line_backup_s: float = DEFAULT_LINE_BACKUP_S
    line_backup_max_s: float = DEFAULT_LINE_BACKUP_MAX_S
    line_turn_s: float = DEFAULT_LINE_TURN_S
    escape_turn_rad_s: float = DEFAULT_ESCAPE_TURN_RAD_S


@dataclass(frozen=True)
class Zone:
    """
    The delivery area as seen by the camera (see vision/zone.py).
    """

    source: str  # "floor" (area paper) or "gate" (landmarks)
    bearing_rad: float  # positive = to the left
    distance_m: float
    v_px: int  # image row of the area's near edge


@dataclass
class Inputs:
    """
    What the robot knows at one control step. None = no recent reading.
    """

    cubes: list[Cube] = field(default_factory=list)
    zone: Zone | None = None
    vision_ok: bool = True
    line_black: bool = False
    zone_granted: bool = False
    has_cube: bool | None = None
    scoop_color: str | None = None


@dataclass(frozen=True)
class Output:
    """
    What to publish after one control step.
    """

    cmd: Command
    scoop_lower: bool
    zone_want: bool
    # Colour of the cube delivered on this very step, else None.
    delivered_color: str | None = None


class CollectorFsm:
    """
    Search, catch, carry, deliver and count, one cube at a time.
    """

    def __init__(
        self,
        approach: ApproachController,
        params: CollectorParams | None = None,
    ):
        """
        Start searching, with nothing in the scoop and nothing delivered.
        """
        self.approach = approach
        self.params = params or CollectorParams()
        self.state = State.SEARCH
        self.count = 0
        self.carrying_color: str | None = None
        self._catch_color = ""
        self._want = False
        self._phase_start_s = 0.0
        self._line_last_black_s = 0.0
        self._resume = State.SEARCH
        self._search_dir = SEARCH_DIR_LEFT

    @property
    def needs_vision(self) -> bool:
        """
        True while the current state steers by the camera.
        """
        return self.state in VISION_STATES

    # ---------- main step ----------
    def step(self, now: float, inp: Inputs) -> Output:
        """
        Run one control step.
        """
        if not self.approach.enabled:
            self.state = State.PAUSED
            return self._out(Command())
        if self.state == State.PAUSED:
            self._go(self._idle_state(), now)
        if inp.line_black and self.state != State.LINE_ESCAPE:
            self._start_line_escape(now)
        handlers = {
            State.CATCH: self._catch,
            State.CARRY: self._carry,
            State.WAIT_ZONE: self._wait_zone,
            State.DELIVER: self._deliver,
            State.RELEASE: self._release,
            State.LINE_ESCAPE: self._line_escape,
        }
        return handlers.get(self.state, self._search)(now, inp)

    def _go(self, state: State, now: float) -> None:
        """
        Enter `state`, starting its timed phase at `now`.
        """
        self.state = state
        self._phase_start_s = now
        if state == State.SEARCH:
            self.approach.reset()

    def _idle_state(self) -> State:
        """
        Where to go when nothing else is happening.
        """
        return State.SEARCH if self.carrying_color is None else State.CARRY

    def _elapsed(self, now: float) -> float:
        """
        Time spent in the current timed phase (s).
        """
        return now - self._phase_start_s

    def _out(
        self,
        cmd: Command,
        delivered_color: str | None = None,
    ) -> Output:
        """
        The output for this step; the scoop is up only while releasing.
        """
        return Output(
            cmd=cmd,
            scoop_lower=self.state != State.RELEASE,
            zone_want=self._want,
            delivered_color=delivered_color,
        )

    # ---------- search and approach ----------
    def _search(self, now: float, inp: Inputs) -> Output:
        """
        SEARCH / APPROACH / ARRIVED: the approach controller drives; when
        it arrives straight in front of a cube, the catch starts.
        """
        if not inp.vision_ok:
            self.approach.target = None
            return self._out(Command())
        cmd = self.approach.update(inp.cubes, now)
        self.state = self.approach.state
        target = self.approach.target
        if self.state != State.ARRIVED or target is None:
            return self._out(cmd)
        if abs(target.bearing_rad) > self.params.catch_align_tolerance_rad:
            # Close but not centred on the scoop: turn in place.
            wz = turn_toward(target.bearing_rad, self.approach.params)
            return self._out(Command(0.0, wz))
        self._catch_color = target.color
        self._go(State.CATCH, now)
        return self._catch(now, inp)

    # ---------- catch ----------
    def _catch(self, now: float, inp: Inputs) -> Output:
        """
        Push blind into the scoop, then wait for the scoop sensors.
        """
        p = self.params
        if self._elapsed(now) < p.catch_push_s:
            return self._out(Command(p.catch_speed_m_s, 0.0))
        verdict = self._catch_verdict(now, inp)
        if verdict == VERDICT_CARRY:
            self.carrying_color = self._confirmed_color(inp)
            self._go(State.CARRY, now)
        elif verdict == VERDICT_REJECT:
            # Not our colour: leave it where it is.
            self._go(State.RELEASE, now)
        elif verdict == VERDICT_MISSED:
            self._go(State.SEARCH, now)
        return self._out(Command())

    def _catch_verdict(self, now: float, inp: Inputs) -> str:
        """
        Decide what the scoop sensors say about the catch.
        """
        p = self.params
        color_known = p.confirm_color and bool(inp.scoop_color)
        targets = self.approach.params.target_colors
        if color_known and inp.scoop_color not in targets:
            return VERDICT_REJECT
        cube_ok = inp.has_cube is True or not p.require_has_cube
        color_ok = color_known or not p.confirm_color
        if cube_ok and color_ok:
            return VERDICT_CARRY
        if self._elapsed(now) >= p.catch_push_s + p.confirm_timeout_s:
            # No colour reading in time: trust the camera.
            return VERDICT_CARRY if cube_ok else VERDICT_MISSED
        return VERDICT_WAIT

    def _confirmed_color(self, inp: Inputs) -> str:
        """
        Colour of the caught cube: the scoop sensor's when it has one.
        """
        if self.params.confirm_color and inp.scoop_color:
            return inp.scoop_color
        return self._catch_color

    # ---------- carry to the area ----------
    def _carry(self, now: float, inp: Inputs) -> Output:
        """
        Drive to the delivery area; ask for the turn when close, wait
        outside without it, and push in once arrived and aligned.
        """
        p = self.params
        zone = inp.zone
        if not inp.vision_ok:
            return self._out(Command())
        if zone is None:
            # Turn in place until the landmarks come into view.
            spin = self._search_dir * self.approach.params.search_turn_rad_s
            return self._out(Command(0.0, spin))
        self._search_dir = (
            SEARCH_DIR_LEFT if zone.bearing_rad >= 0 else SEARCH_DIR_RIGHT
        )
        to_stop_m = zone.distance_m - self._stop_distance(zone)
        if to_stop_m <= p.zone_request_margin_m:
            self._want = True
        if self._must_wait(zone, inp.zone_granted):
            self._go(State.WAIT_ZONE, now)
            return self._out(Command())
        aligned = abs(zone.bearing_rad) <= p.catch_align_tolerance_rad
        if self._at_zone(zone) and aligned:
            self._go(State.DELIVER, now)
            return self._deliver(now, inp)
        return self._out(
            drive_toward(
                zone.bearing_rad,
                zone.distance_m,
                self._stop_distance(zone),
                self.approach.params,
            )
        )

    def _must_wait(self, zone: Zone, granted: bool) -> bool:
        """
        True when the robot is at the waiting distance without the turn.
        """
        p = self.params
        to_stop_m = zone.distance_m - self._stop_distance(zone)
        close = to_stop_m <= p.zone_wait_margin_m
        return p.require_zone_grant and not granted and close

    def _at_zone(self, zone: Zone) -> bool:
        """
        True when the area is right in front of the scoop.
        """
        if zone.source == ZONE_SOURCE_FLOOR:
            return (
                zone.distance_m <= self.params.floor_stop_distance_m
                or zone.v_px >= self.params.zone_arrive_v_px
            )
        return zone.distance_m <= self.params.gate_stop_distance_m

    def _stop_distance(self, zone: Zone) -> float:
        """
        Distance at which the approach to the area slows to a stop.
        """
        if zone.source == ZONE_SOURCE_FLOOR:
            return self.params.floor_stop_distance_m
        return self.params.gate_stop_distance_m

    def _wait_zone(self, now: float, inp: Inputs) -> Output:
        """
        Stay still outside the area until the turn is granted.
        """
        self._want = True
        if inp.zone_granted or not self.params.require_zone_grant:
            self._go(State.CARRY, now)
            return self._carry(now, inp)
        return self._out(Command())

    # ---------- deliver and release ----------
    def _deliver(self, now: float, inp: Inputs) -> Output:
        """
        Push blind into the area; at the end the cube counts.
        """
        p = self.params
        if self._elapsed(now) < p.deliver_push_s:
            return self._out(Command(p.deliver_speed_m_s, 0.0))
        color = self.carrying_color or self._catch_color
        self.carrying_color = None
        self.count += 1
        self._go(State.RELEASE, now)
        return self._out(Command(), delivered_color=color)

    def _release(self, now: float, inp: Inputs) -> Output:
        """
        Scoop up, back up leaving the cube, turn, then search again.
        """
        p = self.params
        elapsed = self._elapsed(now)
        if elapsed < p.release_backup_s:
            return self._out(Command(-p.backup_speed_m_s, 0.0))
        if elapsed < p.release_backup_s + p.release_turn_s:
            spin = self._search_dir * p.escape_turn_rad_s
            return self._out(Command(0.0, spin))
        self._want = False
        self._go(State.SEARCH, now)
        return self._out(Command())

    # ---------- black line ----------
    def _start_line_escape(self, now: float) -> None:
        """
        Remember what to resume and start backing away from the tape.
        """
        if self.carrying_color is None:
            # A cube left in the area (RELEASE) frees the zone too.
            self._want = False
        self._resume = self._idle_state()
        self._line_last_black_s = now
        self._go(State.LINE_ESCAPE, now)

    def _line_escape(self, now: float, inp: Inputs) -> Output:
        """
        Back up while on black (up to a limit) plus a margin, turn, resume.
        """
        p = self.params
        if inp.line_black and self._elapsed(now) < p.line_backup_max_s:
            self._line_last_black_s = now
        since_black_s = now - self._line_last_black_s
        if since_black_s < p.line_backup_s:
            return self._out(Command(-p.backup_speed_m_s, 0.0))
        if since_black_s < p.line_backup_s + p.line_turn_s:
            spin = self._search_dir * p.escape_turn_rad_s
            return self._out(Command(0.0, spin))
        self._go(self._resume, now)
        return self._out(Command())
