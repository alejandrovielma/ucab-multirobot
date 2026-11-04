"""
Auto mode state machine for the browser tool. Pure logic: no hardware, no
threads. Time is passed in as a float in seconds.

Auto mode: find a target-colour cube, push it into the scoop, carry it to
the delivery area (blue and green paper in a gap of the line, between the
green and red landmarks), leave it there and count it. The area comes from
vision/zone.py: from the landmarks far away ("gate") or from the paper
itself up close ("floor"), each with its own arrival thresholds. The
bottom line sensor stops the robot at the black border (it backs up and
turns inward).

Axes: on the physical robot, `DriveCommand` maps to:
- `vy > 0` = forward (W), `vy < 0` = backward/reverse (S)
- `vx < 0` = strafe left (A), `vx > 0` = strafe right (D)
- `wz > 0` = spin counter-clockwise (Q), `wz < 0` = spin clockwise (E)
`_forward`, `_backward`, `_strafe` and `_turn` hide that so the state machine
can think in plain directions.
"""

import logging
import math
import time
from typing import NamedTuple, Protocol

from teleop_web_lib.config import AutoConfig
from teleop_web_lib.kinematics import clamp

log = logging.getLogger("teleop_web")

# --- Machine states -----------------------------------------------------------
STATE_PAUSED = "PAUSED"
STATE_MANUAL = "MANUAL"
STATE_SEARCH = "SEARCH"
STATE_APPROACH = "APPROACH"
STATE_ALIGN = "ALIGN"
STATE_CATCH = "CATCH"
# Stopped after the blind push, checking the cube is not still in front.
STATE_CATCH_CHECK = "CATCH (check)"
STATE_CARRY = "CARRY"
STATE_DROP = "DROP"
STATE_EXPLORE = "EXPLORE"
STATE_LINE_STOP = "LINE (stop)"
STATE_LINE_BACKUP = "LINE (back up)"
STATE_LINE_TURN = "LINE (turn)"
STATE_DROP_BACKUP = "DROP (back up)"
STATE_DROP_TURN = "DROP (turn)"
# The paper's near edge went under the scoop: the last stretch of the push.
STATE_DROP_BLIND = "DROP (blind)"
STATE_EXPLORE_BACKUP = "EXPLORE (back up)"
STATE_EXPLORE_AVOID = "EXPLORE (avoid gate)"
STATE_EXPLORE_REACQUIRE = "EXPLORE (reacquire)"
# States in which the robot is going for a cube or pushing it.
CHASE_STATES = (STATE_APPROACH, STATE_ALIGN, STATE_CATCH, STATE_CATCH_CHECK)
# Prefixes used to recognise a family of states.
PREFIX_LINE = "LINE"
PREFIX_DROP = "DROP"
PREFIX_DONE = "DONE"
PREFIX_NO_CUBES = "NO CUBES"
STATE_NO_CUBES = f"{PREFIX_NO_CUBES} (stopped)"
# Suffixes appended to a base state while it has sub-phases.
SUFFIX_FORWARD = " (forward)"
SUFFIX_TURN = " (turn)"
SUFFIX_ALIGN = " (align)"
# Delivery approach (docs/auto-search.md): face the pink entry line squarely,
# slide in front of the area's centre, or wait when the line is not in view.
STATE_CARRY_SQUARE = f"{STATE_CARRY} (square)"
STATE_CARRY_CENTER = f"{STATE_CARRY} (center)"
STATE_CARRY_NO_ENTRY = f"{STATE_CARRY} (no entry line)"
# Squaring up at the paper: committed to delivering from there.
COMMITTED_CARRY_STATES = (
    STATE_CARRY_SQUARE,
    STATE_CARRY_CENTER,
    STATE_CARRY_NO_ENTRY,
)

# --- Direction helpers --------------------------------------------------------
# Direction of a turn: +1 toward the left, -1 toward the right.
DIR_LEFT = 1.0
DIR_RIGHT = -1.0
FULL_AXIS = 1.0

# --- Target descriptions shown in the page ------------------------------------
TARGET_LOOKING_FOR_ZONE = "looking for zone"
CM_PER_M = 100
# Area estimate made from the area paper (vision/zone.py).
ZONE_SOURCE_FLOOR = "floor"
# A push speed measured outside this range (m/s) is noise: use the setting.
MIN_PUSH_SPEED_MPS = 0.05
MAX_PUSH_SPEED_MPS = 0.6


class DriveCommand(NamedTuple):
    """
    What the drive thread applies: three axes in [-1, 1] and a speed in
    [0, 1] (see the module docstring for the axis meaning).
    """

    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    speed: float = 0.0


STOP = DriveCommand()


class Vision(Protocol):
    """
    What the autopilot needs from the camera pipeline.
    """

    def latest_detection(self, now: float, timeout_s: float):
        """
        The best cube seen less than `timeout_s` ago, or None.
        """

    def latest_foreign(self, now: float, timeout_s: float):
        """
        The nearest cube the mission says to avoid seen less than `timeout_s`
        ago, or None.
        """

    def latest_zone(self, now: float, timeout_s: float):
        """
        The delivery gate seen less than `timeout_s` ago, or None.
        """


def _forward(speed: float) -> DriveCommand:
    """
    Drive straight ahead.
    """
    return DriveCommand(0.0, FULL_AXIS, 0.0, speed)


def _backward(speed: float) -> DriveCommand:
    """
    Drive straight back.
    """
    return DriveCommand(0.0, -FULL_AXIS, 0.0, speed)


def _turn(direction: float, speed: float) -> DriveCommand:
    """
    Spin in place; `direction` is DIR_LEFT (+1, Q/CCW) or DIR_RIGHT (-1, E/CW).
    """
    return DriveCommand(0.0, 0.0, direction, speed)


def _strafe(direction: float, speed: float) -> DriveCommand:
    """
    Move sideways; `direction` is DIR_LEFT (+1, A) or DIR_RIGHT (-1, D).
    """
    return DriveCommand(-direction, 0.0, 0.0, speed)


def _side_of(bearing_rad: float) -> float:
    """
    Turn direction toward something at `bearing_rad` (positive = left).
    """
    return DIR_LEFT if bearing_rad >= 0 else DIR_RIGHT


class AutoPilot:
    """
    Decides the next `DriveCommand` while Auto mode is on.

    Not thread-safe by itself: the caller (Drive) serialises access.
    """

    def __init__(self, cfg: AutoConfig, vision: Vision | None = None):
        """
        Start paused with `cfg`; `vision` supplies the detections.
        """
        self.cfg = cfg
        self.vision = vision
        self.enabled = cfg.enabled
        self.state = STATE_PAUSED
        self.target_info: str | None = None
        self.collected_count = 0  # cubes pushed into the scoop
        self.delivered_count = 0  # cubes left on the delivery paper
        self.carrying = False
        self._phase_until = 0.0
        # Delivery push: the paper's near edge (time, distance) when the push
        # started and when last seen, the far edge when last seen (None if
        # not yet), and whether the blind part began.
        self._drop_edge_first = (0.0, 0.0)
        self._drop_edge_last = (0.0, 0.0)
        self._drop_far_last: tuple[float, float] | None = None
        self._drop_blind = False
        # When and where (bearing) the area paper was last in view while
        # carrying.
        self._last_paper_s = float("-inf")
        self._last_paper_bearing_rad = 0.0
        self._line_turn_until = 0.0
        self._line_stop_until = 0.0
        self._line_black = False
        self._escape = STOP
        self._last_motion: DriveCommand | None = None
        self._last_motion_s = 0.0
        self._resume_state = STATE_SEARCH
        self._last_seen_s: float | None = None
        self._search_start_s = time.monotonic()
        self._search_dir = DIR_LEFT
        # A cube was seen and has not been caught: if it vanishes from view
        # it may be hidden under the scoop, so back up once to uncover it.
        self._reveal_pending = False
        self._reveal_until = 0.0
        self._last_bearing_rad = 0.0
        # Cubes are ignored until then: the last one led to the border line.
        self._ignore_cubes_until = 0.0
        # Turn direction while avoiding the area, fixed for the whole turn.
        self._avoid_dir = DIR_RIGHT
        self._last_zone_s = -math.inf
        # Slide direction around a cube to avoid, fixed until it is clear.
        self._dodge_dir: float | None = None
        # When the robot last had something to do (start, a cube seen, a
        # delivery): the search gives up no_cubes_stop_s after it.
        self._idle_start_s = self._search_start_s

    # ---------- mode control ----------
    def configure(
        self,
        now: float,
        enabled: bool | None = None,
        reset: bool = False,
    ) -> None:
        """
        Switch Auto on or off and/or reset the counters.
        """
        if enabled is not None:
            self.enabled = bool(enabled)
        restartable = self.state.startswith(
            (PREFIX_DONE, PREFIX_NO_CUBES, STATE_PAUSED)
        )
        if (reset or self.enabled) and (restartable or reset):
            self.state = (
                STATE_CARRY if self.carrying and not reset else STATE_SEARCH
            )
            self._search_start_s = now
            self._idle_start_s = now
            if reset:
                self.collected_count = 0
                self.delivered_count = 0
                self.carrying = False
        if not self.enabled:
            self.state = STATE_PAUSED

    def mark_manual(self) -> None:
        """
        The operator is driving by hand.
        """
        self.state = STATE_MANUAL

    def mark_paused(self) -> None:
        """
        Auto is off and nobody is driving.
        """
        self.state = STATE_PAUSED

    # ---------- main step ----------
    def step(self, now: float, line_black: bool = False) -> DriveCommand:
        """
        The command for time `now`; `line_black` is the line sensor reading.
        """
        if self.state.startswith((PREFIX_DONE, PREFIX_NO_CUBES)):
            return STOP
        if self.enabled and self.state in (STATE_MANUAL, STATE_PAUSED):
            # Back from WASD or pause: if it had a cube, keep carrying it.
            self._resume_after_pause(now)
        self._line_black = line_black
        if line_black and not self.state.startswith(PREFIX_LINE):
            self._start_line_recovery(now)
        for phase in (self._line_phase, self._catch_phase, self._drop_phase):
            command = phase(now)
            if command is not None:
                return command
        if not self.enabled:
            self.state = STATE_PAUSED
            return STOP
        if self.state.startswith(STATE_CARRY):
            command = self._carry_step(now)
        else:
            command = self._search_step(now)
        return self._dodge_foreign(now, command)

    def _dodge_foreign(self, now: float, command: DriveCommand) -> DriveCommand:
        """
        Never push a cube the mission says to avoid, with or without a cube
        in the scoop: a plain forward move toward one that is in the way
        becomes a slide away from it, including right before a catch or
        delivery push starts. Once a catch or delivery push has started, it
        runs in its own phase above; turns and slides do not run it over.
        """
        cube = self._foreign_in_way(now, command)
        if cube is None:
            self._dodge_dir = None
            return command
        if self._dodge_dir is None:
            self._dodge_dir = -_side_of(cube.bearing_rad)
        if self.state == STATE_CATCH:
            self.state = STATE_APPROACH
        elif self.state.startswith(PREFIX_DROP):
            self.state = STATE_CARRY_CENTER
        self.target_info = (
            f"avoiding {cube.color} {cube.distance_m * CM_PER_M:.0f}cm"
        )
        return _strafe(self._dodge_dir, self.cfg.strafe_speed)

    def _foreign_in_way(self, now: float, command: DriveCommand):
        """
        The cube to avoid blocking a plain forward move, or None when the
        path ahead is clear or the command is a turn/slide.
        """
        cfg = self.cfg
        if self.vision is None or command.vy <= 0:
            return None
        if command.vx or command.wz:
            return None
        cube = self.vision.latest_foreign(now, cfg.lost_timeout_s)
        if cube is None or cube.distance_m > cfg.foreign_avoid_distance_m:
            return None
        lateral_m = cube.distance_m * math.sin(abs(cube.bearing_rad))
        limit_m = cfg.foreign_clearance_m
        if self._dodge_dir is not None:
            limit_m += cfg.foreign_exit_margin_m
        return cube if lateral_m <= limit_m else None

    def _resume_after_pause(self, now: float) -> None:
        """
        Continue carrying if a cube is in the scoop, else search (backing
        up first: a cube may be sitting in the scoop's blind spot).
        """
        self.state = STATE_CARRY if self.carrying else STATE_SEARCH
        self._search_start_s = now

    # ---------- timed phases (return None when not active) ----------
    def _start_line_recovery(self, now: float) -> None:
        """
        Black line underneath: brake, move away from the line (opposite to
        the way it was going) until the sensor sees white again plus a
        margin, turn inward (unless it backed into the line and escaped
        forward, already facing the arena), then resume what it was doing.
        """
        cfg = self.cfg
        self._ban_cubes_if_chasing_one(now)
        self._settle_cube_on_line()
        if not self.enabled:
            self._resume_state = STATE_PAUSED
        elif self.carrying:
            self._resume_state = STATE_CARRY
        else:
            self._resume_state = STATE_SEARCH
        self._escape = self._escape_command(now)
        self.state = STATE_LINE_STOP
        self._line_stop_until = now + cfg.line_stop_s
        self._phase_until = self._line_stop_until + cfg.line_backup_s
        # Spin inward only when we hit the line going forward in Auto mode.
        needs_turn = (
            self.enabled and self._escape.vy < 0 and self._escape.vx == 0
        )
        turn_s = cfg.line_turn_s if needs_turn else 0.0
        self._line_turn_until = self._phase_until + turn_s
        self.target_info = None

    def _ban_cubes_if_chasing_one(self, now: float) -> None:
        """
        A line hit while going for a cube, pushing one or carrying one means
        that cube is at the border, where the scoop cannot bring it back:
        ignore cubes for a while so the robot does not hit the line again.
        """
        chasing = self.state in CHASE_STATES or (
            self.carrying and not self.state.startswith(PREFIX_DROP)
        )
        if not chasing:
            return
        self._ignore_cubes_until = now + self.cfg.line_cube_ban_s
        self._last_seen_s = None
        self._reveal_pending = False

    def _settle_cube_on_line(self) -> None:
        """
        A line hit while carrying: during the delivery push the cube is
        already in the area (count it); anywhere else it was pushed over
        the border and stays behind, so the robot is no longer carrying.
        """
        if self.state.startswith(PREFIX_DROP):
            self._finish_drop()
        else:
            self.carrying = False

    def _finish_drop(self) -> None:
        """
        The cube is on the delivery area: stop carrying it and count it.
        """
        if self.carrying:
            self.carrying = False
            self.delivered_count += 1

    def note_motion(self, now: float, command: DriveCommand) -> None:
        """
        Remember the translation the robot is doing (manual or automatic:
        forward/back on `vy` or lateral strafe on `vx`), so the line recovery
        can undo it. Ignored while recovering, so the escape itself does not
        overwrite the direction it escapes from.
        """
        if self.state.startswith(PREFIX_LINE) or command.speed <= 0.0:
            return
        if command.vx != 0.0 or command.vy != 0.0:
            self._last_motion = command
            self._last_motion_s = now

    def _escape_command(self, now: float) -> DriveCommand:
        """
        Straight move away from the line: opposite to the main axis of the
        last translation (forward W -> back S, back S -> forward W, lateral
        left A -> right D, lateral right D -> left A). Backs up when there
        is no recent motion to undo.
        """
        cfg = self.cfg
        last = self._last_motion
        if last is None or now - self._last_motion_s > cfg.line_motion_memory_s:
            return _backward(cfg.line_speed)
        if abs(last.vx) > abs(last.vy):
            direction = DIR_RIGHT if last.vx < 0 else DIR_LEFT
            return _strafe(direction, cfg.line_speed)
        if last.vy < 0:
            return _forward(cfg.line_speed)
        return _backward(cfg.line_speed)

    def _line_phase(self, now: float) -> DriveCommand | None:
        """
        Stop, move away from the line, then turn, then hand control back to
        the resumed state.
        """
        if not self.state.startswith(PREFIX_LINE):
            return None
        cfg = self.cfg
        if now < self._line_stop_until:
            self.state = STATE_LINE_STOP
            return STOP
        if now < self._phase_until:
            self.state = STATE_LINE_BACKUP
            # Still on black: keep moving away (up to a limit), so it always
            # leaves the border before turning.
            backing_s = now - self._line_stop_until
            if self._line_black and backing_s < cfg.line_backup_max_s:
                needs_turn = (
                    self.enabled
                    and self._escape.vy < 0
                    and self._escape.vx == 0
                )
                turn_s = cfg.line_turn_s if needs_turn else 0.0
                self._phase_until = now + cfg.line_backup_s
                self._line_turn_until = self._phase_until + turn_s
            return self._escape
        if now < self._line_turn_until:
            self.state = STATE_LINE_TURN
            return _turn(self._search_dir, cfg.turn_speed)
        self._last_motion = None
        self.state = self._resume_state
        self._search_start_s = now
        return None

    def _catch_phase(self, now: float) -> DriveCommand | None:
        """
        Blind push into the scoop, a short stop, then count the cube as
        picked unless it is still in view in front: a push that fell short
        left it outside the paddles and the robot carried an empty scoop.
        """
        if self.state not in (STATE_CATCH, STATE_CATCH_CHECK):
            return None
        cfg = self.cfg
        if now < self._phase_until:
            return _forward(cfg.forward_speed)
        if now < self._phase_until + cfg.catch_check_s:
            self.state = STATE_CATCH_CHECK
            return STOP
        in_scoop = self._cube_ahead(now, cfg.assume_in_scoop_distance_m)
        # A cube seen this close is in the scoop, however badly the camera sees it.
        if self._cube_ahead(now, cfg.catch_check_distance_m) and not in_scoop:
            # Not in the scoop: go for it again.
            self.state = STATE_SEARCH
            self._search_start_s = now
            return STOP
        self.collected_count += 1
        self.carrying = True
        self._reveal_pending = False
        self.state = STATE_CARRY
        self._search_start_s = now
        return STOP

    def _drop_phase(self, now: float) -> DriveCommand | None:
        """
        Delivery: push the cube onto the paper, back up leaving it there and
        turn around.
        """
        if not self.state.startswith(PREFIX_DROP):
            return None
        cfg = self.cfg
        if not self._drop_blind:
            self._guided_push_step(now)
        if now < self._phase_until:
            return _forward(cfg.forward_speed)
        self._finish_drop()
        backup_until = self._phase_until + cfg.floor_deliver_backup_s
        if now < backup_until:
            self.state = STATE_DROP_BACKUP
            return _backward(cfg.line_speed)
        if self.state == STATE_DROP_BACKUP and self._cube_left_out(now):
            # Backed up and the cube is still right in front: it stayed
            # outside. Only then: checked while turning, any other cube on
            # the table near the area undid deliveries that were fine.
            self.delivered_count -= 1
            log.info("delivery: cube left outside, not counted")
            self._end_drop(now)
            return None
        if now < backup_until + cfg.deliver_turn_s:
            self.state = STATE_DROP_TURN
            return _turn(self._search_dir, cfg.turn_speed)
        self._end_drop(now)
        return None

    def _guided_push_step(self, now: float) -> None:
        """
        Delivery push. With the area's far edge in view, drive until it is
        deliver_far_stop_m away: the cube is then over the middle of the
        area, however fast the robot goes (two pushes timed blind carried the
        cube right across the area). Without it, drive while the near edge is
        in view and farther than deliver_blind_edge_m (below ~10 cm the scoop
        hides it), then push blind the rest of the way, timed with the speed
        measured from the edge coming closer and never past the far edge.
        """
        cfg = self.cfg
        zone = None
        if self.vision is not None:
            zone = self.vision.latest_zone(now, cfg.deliver_edge_timeout_s)
        seen = zone is not None and zone.color == ZONE_SOURCE_FLOOR
        far_m = (
            self._usable_far_m(zone, getattr(zone, "far_distance_m", None))
            if seen
            else None
        )
        if seen:
            self._drop_edge_last = (now, zone.distance_m)
            if far_m is not None and cfg.deliver_far_stop_m > 0:
                self._drop_far_last = (now, far_m)
                if far_m <= cfg.deliver_far_stop_m:
                    log.info(
                        "delivery: far edge %.3f m <= %.3f m, stop pushing (near %.3f m)",
                        far_m, cfg.deliver_far_stop_m, zone.distance_m,
                    )
                    self._drop_blind = True
                    self._phase_until = now  # there: stop pushing
                    return
                if now < self._phase_until:
                    return
            elif (
                zone.distance_m > cfg.deliver_blind_edge_m
                and now < self._phase_until
            ):
                return
        edge_s, edge_m = self._drop_edge_last
        speed = self._measured_push_speed()
        # The edge kept coming closer since it was last seen.
        edge_m = max(0.0, edge_m - speed * (now - edge_s))
        blind_m = max(0.0, edge_m + cfg.deliver_insert_m - cfg.cube_ahead_m)
        if self._drop_far_last is not None:
            far_s, far_m = self._drop_far_last
            far_m -= speed * (now - far_s)
            blind_m = min(blind_m, max(0.0, far_m - cfg.deliver_far_stop_m))
        self._drop_blind = True
        self.state = STATE_DROP_BLIND
        self._phase_until = now + blind_m / speed
        log.info(
            "delivery: blind push, near edge %.3f m, speed %.2f m/s, %.3f m = %.2f s",
            edge_m, speed, blind_m, blind_m / speed,
        )

    def _usable_far_m(self, zone, far_m):
        """
        The area's far edge, or None when it is not credible: the paper is
        ~13.8 cm deep, and a far edge closer than deliver_far_min_span_m
        behind the near one is the blob cut short by the cube or the scoop.
        """
        min_span_m = self.cfg.deliver_far_min_span_m
        if far_m is None or not min_span_m:
            return far_m
        return far_m if far_m - zone.distance_m >= min_span_m else None

    def _measured_push_speed(self) -> float:
        """
        Forward speed (m/s) from the paper's near edge coming closer during
        the push; forward_speed_mps when it was not seen long enough.
        """
        cfg = self.cfg
        (t0, d0), (t1, d1) = self._drop_edge_first, self._drop_edge_last
        if t1 - t0 >= cfg.deliver_speed_min_s:
            speed = (d0 - d1) / (t1 - t0)
            if MIN_PUSH_SPEED_MPS <= speed <= MAX_PUSH_SPEED_MPS:
                return speed
        return cfg.forward_speed_mps

    def _end_drop(self, now: float) -> None:
        """
        Back to looking for cubes, with a fresh search.
        """
        self.state = STATE_SEARCH
        self._search_start_s = now
        self._idle_start_s = now

    def _cube_left_out(self, now: float) -> bool:
        """
        True when a cube (not one inside the area) is in view this close and
        straight ahead right after backing up from a delivery: where a cube
        the push did not carry in is left. Another cube off to a side is not
        the delivered one.
        """
        cfg = self.cfg
        if not self._cube_ahead(
            now,
            cfg.drop_verify_distance_m,
            cfg.lost_timeout_s,
        ):
            return False
        det = self.vision.latest_detection(now, cfg.lost_timeout_s)
        return abs(det.bearing_rad) <= cfg.drop_verify_bearing_rad

    def _cube_ahead(
        self,
        now: float,
        distance_m: float,
        timeout_s: float | None = None,
    ) -> bool:
        """
        True when a cube was seen less than `timeout_s` ago (default
        catch_check_s: only frames taken after the push) this close.
        """
        if self.vision is None:
            return False
        if timeout_s is None:
            timeout_s = self.cfg.catch_check_s
        det = self.vision.latest_detection(now, timeout_s)
        return det is not None and det.distance_m <= distance_m

    # ---------- looking for cubes ----------
    def _search_step(self, now: float) -> DriveCommand:
        """
        Look for a cube: approach it, catch it, or roam when none is seen.
        """
        cfg = self.cfg
        det = None
        if self.vision is not None and now >= self._ignore_cubes_until:
            det = self.vision.latest_detection(now, cfg.lost_timeout_s)
        if det is None:
            return self._no_cube_step(now)

        self.target_info = f"{det.color} {det.distance_m * CM_PER_M:.0f}cm"
        self._last_seen_s = now
        self._last_bearing_rad = det.bearing_rad
        self._reveal_pending = det.distance_m <= cfg.reveal_max_distance_m
        self._search_start_s = now
        self._idle_start_s = now
        self._search_dir = _side_of(det.bearing_rad)

        # Cube at the scoop threshold (~18.5 cm or image bottom) and aligned.
        at_scoop = (
            det.distance_m <= cfg.stop_distance_m
            or det.v >= cfg.scoop_arrive_v_px
        )
        if at_scoop and self._is_aligned(det.bearing_rad):
            self.state = STATE_CATCH
            self._phase_until = now + cfg.scoop_push_s
            return _forward(cfg.forward_speed)
        return self._steer_to(
            det.bearing_rad,
            det.distance_m,
            cfg.stop_distance_m,
            STATE_APPROACH,
        )

    def _no_cube_step(self, now: float) -> DriveCommand:
        """
        No cube in view: wait briefly, give up after a while, else roam.
        """
        cfg = self.cfg
        self.target_info = None
        # If it lost sight of a cube a moment ago, wait briefly before turning.
        recently_seen = (
            self._last_seen_s is not None
            and (now - self._last_seen_s) < cfg.lost_timeout_s
        )
        if recently_seen:
            return STOP
        # Hard limit: searching this long without seeing any cube, stop.
        if (now - self._idle_start_s) >= cfg.no_cubes_stop_s:
            return self._give_up()
        return self._scan_step(now)

    def _give_up(self) -> DriveCommand:
        """
        No cube left to find: stop for good.
        """
        if self.delivered_count > 0:
            self.state = f"{PREFIX_DONE} ({self.delivered_count} delivered)"
        else:
            self.state = STATE_NO_CUBES
        return STOP

    def _scan_step(self, now: float) -> DriveCommand:
        """
        Nothing in view: back up once to uncover a cube hidden under the
        scoop, never drive toward the delivery gate without a cube (past it
        there is no border line to stop at), else roam.
        """
        cfg = self.cfg
        if self._reveal_pending:
            self._reveal_pending = False
            self._reveal_until = now + cfg.reveal_backup_s
        if now < self._reveal_until:
            self.state = STATE_EXPLORE_BACKUP
            return _backward(cfg.line_speed)
        zone = self._zone_in_view(now)
        if self._must_avoid(zone):
            return self._avoid_step(zone)
        return self._reacquire_step(now) or self._roam(now, STATE_EXPLORE)

    def _reacquire_step(self, now: float) -> DriveCommand | None:
        """
        A cube was seen a moment ago and is gone: keep heading where it was
        (turn toward its side, or creep forward if it was straight ahead)
        instead of wandering off. None once the memory has expired.
        """
        cfg = self.cfg
        seen_s = self._last_seen_s
        if seen_s is None or now - seen_s >= cfg.reacquire_s:
            return None
        self.state = STATE_EXPLORE_REACQUIRE
        if abs(self._last_bearing_rad) <= cfg.align_tolerance_rad:
            return _forward(cfg.roam_forward_speed)
        return _turn(_side_of(self._last_bearing_rad), cfg.search_turn_speed)

    def _must_avoid(self, zone) -> bool:
        """
        True when the delivery area or its legos are this close and no cube
        is in view: roaming forward there ran the robot over the legos and
        along the area's black tape.
        """
        return (
            zone is not None
            and zone.distance_m <= self.cfg.gate_avoid_distance_m
        )

    def _avoid_step(self, zone) -> DriveCommand:
        """
        Turn away from the area. The side is chosen once, when the avoidance
        starts, and kept: choosing it every tick flips with the bearing noise
        and the robot never turns.
        """
        if self.state != STATE_EXPLORE_AVOID:
            self._avoid_dir = -_side_of(zone.bearing_rad)
            # The scan carries on this way: turning back toward the area (the
            # side it delivered on) wobbled in front of it until it gave up.
            self._search_dir = self._avoid_dir
        self.state = STATE_EXPLORE_AVOID
        return _turn(self._avoid_dir, self.cfg.search_turn_speed)

    def _zone_in_view(self, now: float):
        """
        The delivery gate seen less than `lost_timeout_s` ago, or None.
        """
        if self.vision is None:
            return None
        return self.vision.latest_zone(now, self.cfg.lost_timeout_s)

    # ---------- carrying to the delivery gate ----------
    def _carry_step(self, now: float) -> DriveCommand:
        """
        With a cube in the scoop: head for the area (the landmarks lead from
        afar) and deliver only once the area paper itself is in view: a push
        started from the landmarks left the cube ~20 cm short of the paper.
        If the area is never found, assume the cube was lost.

        Once squaring up at the paper it stays committed: with cubes already
        on the paper its distance jittered around the commit distance and
        the blob dropped out now and then, and the robot danced between
        squaring up and turning to the landmarks. If it stays out and was
        last seen off to a side, slide that way to bring it back.
        """
        cfg = self.cfg
        zone = self._zone_in_view(now)
        committed = self.state in COMMITTED_CARRY_STATES
        paper = zone is not None and zone.color == ZONE_SOURCE_FLOOR
        if paper:
            self._last_paper_s = now
            self._last_paper_bearing_rad = zone.bearing_rad
        elif committed:
            lost_s = now - self._last_paper_s
            if lost_s < cfg.carry_paper_hold_s:
                # The paper flickered out: wait for it, do not turn away.
                return STOP
            slide_until_s = cfg.carry_paper_hold_s + cfg.carry_paper_slide_s
            off_side = (
                abs(self._last_paper_bearing_rad)
                > cfg.drop_align_tolerance_rad
            )
            if off_side and lost_s < slide_until_s:
                # Squaring up pushed it out of view: slide back toward it.
                self.state = STATE_CARRY_CENTER
                side = _side_of(self._last_paper_bearing_rad)
                return _strafe(side, cfg.strafe_speed)
        if zone is None:
            if now - self._last_zone_s < cfg.carry_zone_hold_s:
                return STOP
            if now - self._search_start_s >= cfg.carry_zone_timeout_s:
                return self._give_up_carrying(now)
            self.target_info = TARGET_LOOKING_FOR_ZONE
            return self._roam(now, STATE_CARRY)

        self.target_info = (
            f"zone {zone.color} {zone.distance_m * CM_PER_M:.0f}cm"
        )
        self._search_start_s = now
        self._last_zone_s = now
        self._search_dir = _side_of(zone.bearing_rad)
        # The landmarks only lead there (slowing down near them): the push
        # starts once the paper is in view, from in front of the entry line.
        commit_m = cfg.deliver_commit_distance_m
        if committed:
            commit_m += cfg.deliver_commit_hysteresis_m
        at_paper = paper and (
            zone.distance_m <= commit_m or zone.v >= cfg.floor_arrive_v_px
        )
        if at_paper:
            return self._square_and_drop(now, zone)
        stop_distance_m = (
            cfg.deliver_commit_distance_m if paper else cfg.zone_stop_distance_m
        )
        return self._steer_to(
            zone.bearing_rad,
            zone.distance_m,
            stop_distance_m,
            STATE_CARRY,
        )

    def _square_and_drop(self, now: float, zone) -> DriveCommand:
        """
        At the paper: turn in place until facing the pink entry line
        squarely, slide sideways (Mecanum, keeps the heading) until the
        area's centre is straight ahead, then push the cube blind straight
        in. Without the pink line in view it waits instead of pushing
        crooked (a crooked push knocks the legos down), unless the paper is
        so close that the scoop hides the line: backing up to see it would
        leave the cube behind, and that close the robot faces it already.

        Arriving diagonally, the paper sits off to a side: turning in place
        pushed it out of view, so with its centre beyond
        `square_max_bearing_rad` it slides toward it first.
        """
        cfg = self.cfg
        side_tol = cfg.square_max_bearing_rad
        if self.state == STATE_CARRY_CENTER:
            side_tol *= cfg.align_exit_fraction
        entry_angle_rad = zone.entry_angle_rad
        hidden = zone.distance_m <= cfg.entry_hidden_distance_m
        needs_square = entry_angle_rad is not None and (
            abs(entry_angle_rad) > cfg.entry_square_tolerance_rad
        )
        must_face = needs_square or (entry_angle_rad is None and not hidden)
        if must_face and abs(zone.bearing_rad) > side_tol:
            self.state = STATE_CARRY_CENTER
            return _strafe(_side_of(zone.bearing_rad), cfg.strafe_speed)
        if entry_angle_rad is None:
            if not hidden:
                self.state = STATE_CARRY_NO_ENTRY
                return STOP
            entry_angle_rad = 0.0
        # Once turning (or sliding), finish it: do not stop at the edge.
        square_tol = cfg.entry_square_tolerance_rad
        if self.state == STATE_CARRY_SQUARE:
            square_tol *= cfg.align_exit_fraction
        if abs(entry_angle_rad) > square_tol:
            self.state = STATE_CARRY_SQUARE
            return _turn(_side_of(entry_angle_rad), cfg.search_turn_speed)
        center_tol = cfg.drop_align_tolerance_rad
        if self.state == STATE_CARRY_CENTER:
            center_tol *= cfg.align_exit_fraction
        if self._center_blocked_by_foreign(now, zone.bearing_rad):
            center_tol = cfg.square_max_bearing_rad
        if abs(zone.bearing_rad) > center_tol:
            self.state = STATE_CARRY_CENTER
            return _strafe(_side_of(zone.bearing_rad), cfg.strafe_speed)
        # Push straight in, guided by the area's edges (see _guided_push_step).
        self.state = STATE_DROP
        self._drop_edge_first = (now, zone.distance_m)
        self._drop_edge_last = (now, zone.distance_m)
        far_m = self._usable_far_m(zone, getattr(zone, "far_distance_m", None))
        self._drop_far_last = None if far_m is None else (now, far_m)
        self._phase_until = now + cfg.deliver_push_max_s
        self._drop_blind = False
        return _forward(cfg.forward_speed)

    def _center_blocked_by_foreign(
        self,
        now: float,
        zone_bearing_rad: float,
    ) -> bool:
        """
        True when sliding toward the area's centre would move into or back
        behind a cube to avoid in front of the robot.
        """
        if self.vision is None:
            return False
        cube = self.vision.latest_foreign(now, self.cfg.lost_timeout_s)
        if cube is None or cube.distance_m > self.cfg.foreign_avoid_distance_m:
            return False
        return self._dodge_dir is not None or _side_of(
            cube.bearing_rad
        ) == _side_of(zone_bearing_rad)

    def _give_up_carrying(self, now: float) -> DriveCommand:
        """
        The gate was not found in time: the cube slipped out of the scoop,
        so go back to looking for cubes.
        """
        self.carrying = False
        self.state = STATE_SEARCH
        self.target_info = None
        self._search_start_s = now
        return STOP

    # ---------- motion primitives ----------
    def _is_aligned(self, bearing_rad: float) -> bool:
        """
        True when the target is straight enough ahead to push into it.
        """
        cfg = self.cfg
        limit_rad = cfg.align_tolerance_rad * cfg.catch_align_factor
        return abs(bearing_rad) <= limit_rad

    def _roam(self, now: float, label: str) -> DriveCommand:
        """
        Free roaming: turn to scan, then drive forward to another spot at
        the end of each period (a spin that saw nothing is not the whole
        arena).
        """
        cfg = self.cfg
        # Not counted from the last sighting: a glimpse of the area on every
        # spin would keep it from ever driving forward.
        elapsed_s = now % cfg.roam_period_s
        if elapsed_s >= cfg.roam_period_s - cfg.roam_forward_s:
            self.state = f"{label}{SUFFIX_FORWARD}"
            return _forward(cfg.roam_forward_speed)
        self.state = f"{label}{SUFFIX_TURN}"
        return _turn(self._search_dir, cfg.search_turn_speed)

    def _steer_to(
        self,
        bearing: float,
        distance: float,
        stop_distance: float,
        label: str,
    ) -> DriveCommand:
        """
        Turn in place if off-heading; otherwise drive straight forward
        (star motion: never mix rotation and translation on Mecanum wheels,
        which causes sideways drift).
        """
        cfg = self.cfg
        tolerance_rad = cfg.align_tolerance_rad
        if distance <= cfg.close_align_distance_m:
            tolerance_rad *= cfg.close_align_factor
        if self.state.endswith(SUFFIX_ALIGN) or self.state == STATE_ALIGN:
            # Already turning: finish the turn, do not stop at the edge.
            tolerance_rad *= cfg.align_exit_fraction
        if abs(bearing) > tolerance_rad:
            if label == STATE_APPROACH:
                self.state = STATE_ALIGN
            else:
                self.state = f"{label}{SUFFIX_ALIGN}"
            return _turn(_side_of(bearing), cfg.turn_speed)
        self.state = label
        span_m = max(
            cfg.min_slow_span_m,
            cfg.slow_down_distance_m - stop_distance,
        )
        fraction = clamp(
            (distance - stop_distance) / span_m,
            cfg.min_speed_fraction,
            FULL_AXIS,
        )
        return _forward(cfg.forward_speed * fraction)
