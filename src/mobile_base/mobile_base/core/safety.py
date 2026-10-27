"""SafetyGate: last filter between the behavior command and the motors.

Motion rule of the team (2026-10-27): the robot translates in any direction (forward, back, sideways,
diagonals: a "star") or spins in place, never both at once. Every change of heading is a spin.

Rules, applied in order every tick:
0. Emergency stop (the estop topic, from the phone interface) -> full stop until it is cleared.
1. Coordinator heartbeat lost (when required) -> full stop.
2. Own pose stale (when required) -> full stop.
3. Star motion: a command that both translates and spins keeps only the spin. Any translation that is
   not straight ahead (sideways, diagonal, backward) is limited to max_side_speed: the only line
   sensor is at the robot center, so it sees the tape once the body is already 8-9.5 cm past it.
4. Line sensor on black -> no translation toward the tape. The tape is taken to lie in the direction
   the robot was translating when the line appeared (forward if it had not translated yet), so
   backing away the way it came is allowed and pushing on (or out "through another side") is not.
   Spinning is allowed, and that direction turns with the robot (the tape stays where it is): with
   the pose by its heading change, without it by integrating the spin the gate let through. A line
   that comes back less than line_clear_time after it went white is the same tape (the TCRT5000
   flickers on its edge), so the direction is not taken again.
5. Peer robot in the travel direction (overhead-camera poses, or without them the bearing and
   distance of its marker seen by this robot's camera, peer_seen) -> slow down, stop, and spin away
   from it. If both robots stay blocked facing each other for deadlock_time, the robot WITHOUT
   priority rotates away (or holds still when it carries a cube in the open scoop).
6. Carrying a cube in the open scoop -> spins at most carry_max_turn (a fast spin throws the cube
   out of the fixed U; the behavior watches the scoop with the camera and recaptures it).

Body geofence (last rule, needs the overhead pose and a track polygon). Rule of the team: the
scoop may step on the tape, the robot body may never cross it. The final command is simulated for
track_horizon seconds and every corner of the body rectangle (wheels included, scoop excluded) must
stay inside the polygon: a spin that would swing a corner over the tape tries the other direction,
else stops; a translation that would push the body over it stops. With the robot center already
outside, translation is allowed only toward the track centroid, even over "black" (off the board
the floor reads as tape).

Scoop gate: when the robot reports gate_closed the cube is retained by the servo door, so the carry
restrictions (slow spins, hold still in a deadlock) do not apply.

Delivery exception (rule 4): while this robot holds the zone lease (/zone/occupant) and its own
overhead pose is inside the delivery corridor facing the square, it may cross the square's tape.
Both conditions come from inputs the gate reads itself, so a faulty behavior cannot unlock it.
"""

import math
from dataclasses import dataclass

from mobile_base.core.delivery import DeliveryZone, zone_from
from mobile_base.core.geometry import distance, relative_bearing
from mobile_base.core.track import Footprint, Track, path_inside

Command = tuple[float, float, float]
STOP: Command = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    theta: float
    stamp: float  # seconds, same clock as SafetyInput.now


@dataclass(frozen=True)
class SafetyParams:
    require_heartbeat: bool = True
    heartbeat_timeout: float = 2.0
    require_pose: bool = True
    pose_timeout: float = 0.5
    front_cone_deg: float = 60.0
    # Center-to-center distances. Scoop tips touch at ~0.38 m when facing each other.
    stop_distance: float = 0.40
    slow_distance: float = 0.60
    avoid_distance: float = 0.60
    avoid_turn: float = 0.6  # rad/s, turning away from the peer
    has_priority: bool = False
    deadlock_time: float = 3.0
    escape_turn: float = 0.8  # rad/s, deadlock escape rotation of the robot without priority
    escape_time: float = 1.5
    # The fixed scoop only retains a cube when it is pushed forward: a fast spin throws it out of
    # the U (observed in Webots), so with a cube the spins are slow.
    carry_max_turn: float = 0.4  # rad/s, spin-rate limit with a cube
    max_side_speed: float = 0.05  # m/s, any translation that is not straight ahead (rule 3)
    # Mecanum wheels turn and translate at once (one command spread over the four wheels): arcs are
    # allowed when allow_arcs, with both rates capped. False = star motion (spin wins).
    allow_arcs: bool = False
    max_arc_speed: float = 0.08  # m/s, translation while spinning
    max_arc_turn: float = 0.6  # rad/s, spin while translating
    # Slew limits (0 = off): hard starts make the rollers slip and stall the motors; stopping is never limited.
    max_accel: float = 0.0  # m/s^2
    max_alpha: float = 0.0  # rad/s^2
    line_clear_time: float = 0.3  # s of white before a new black reading counts as a new tape (rule 4)
    track_horizon: float = 0.3  # s, body geofence prediction
    body_front: float = 0.095  # m, body rectangle without the scoop (wheels reach 0.093 / 0.0805)
    body_rear: float = 0.095
    body_half_width: float = 0.082
    track_return_cone: float = 45.0  # deg, outside the track: forward only within this of the centroid
    # Delivery square (see core/delivery.py); keep equal to the behavior's zone_* parameters.
    zone_x: float = DeliveryZone.x
    zone_y: float = DeliveryZone.y
    zone_heading: float = DeliveryZone.heading
    zone_square_depth: float = DeliveryZone.square_depth
    zone_entry_offset: float = DeliveryZone.entry_offset
    zone_drop_offset: float = DeliveryZone.drop_offset
    zone_exit_offset: float = DeliveryZone.exit_offset
    zone_corridor_half_width: float = DeliveryZone.corridor_half_width


@dataclass(frozen=True)
class SafetyInput:
    cmd: Command
    line_front: bool
    own_pose: Pose2D | None
    peer_pose: Pose2D | None
    now: float
    heartbeat_age: float | None  # None = never received
    carrying: bool = False
    holds_zone: bool = False  # /zone/occupant == this robot
    gate_closed: bool = False  # scoop gate closed: the cube is retained
    estop: bool = False  # emergency stop requested (latched by the node until cleared)
    # Without overhead poses: the peer's marker seen by this robot's camera, (bearing rad, distance m).
    peer_seen: tuple[float, float] | None = None


class SafetyGate:
    def __init__(self, params: SafetyParams, track: Track | None = None):
        self.params = params
        self._zone = zone_from(params)
        self._track = track or Track(())
        self._body = Footprint(params.body_front, params.body_rear, params.body_half_width)
        self._blocked_since: float | None = None
        self._escape_until: float | None = None
        self._escape_cmd: Command = STOP
        self._escape_reason = ""
        self._travel_dir = (1.0, 0.0)  # unit translation direction (robot frame) of the last move
        self._line_dir = (1.0, 0.0)  # where the tape is taken to be while the line is seen
        self._line_seen = False
        self._line_clear_since = -math.inf  # when the sensor last went from black to white
        self._last_now: float | None = None
        self._last_theta: float | None = None  # own heading at the previous tick (fresh pose only)
        self._last_wz = 0.0  # spin let through at the previous tick
        self._out: Command = STOP  # what the previous tick let through, for the slew limit
        self._out_at: float | None = None

    def filter(self, inp: SafetyInput) -> tuple[Command, str]:
        self._follow_rotation(inp)
        cmd, reason = self._filter(inp)
        cmd = self._slew(cmd, inp.now)
        self._last_wz = cmd[2]
        return cmd, reason

    def _slew(self, cmd: Command, now: float) -> Command:
        p = self.params
        dt = 0.1 if self._out_at is None else min(max(now - self._out_at, 0.0), 0.5)
        self._out_at = now
        limits = (p.max_accel, p.max_accel, p.max_alpha)
        out = []
        for new, old, rate in zip(cmd, self._out, limits):
            step = rate * dt
            # Only speeding up (or reversing) is ramped; slowing down or stopping goes through at once.
            out.append(new if rate <= 0.0 or abs(new) <= abs(old) and new * old >= 0.0
                       else old + max(-step, min(step, new - old)))
        self._out = (out[0], out[1], out[2])
        return self._out

    def _follow_rotation(self, inp: SafetyInput) -> None:
        """Turn the robot-frame directions of rule 4 by the robot's own spin since the last tick."""
        fresh = self._is_fresh(inp.own_pose, inp.now)
        if fresh and self._last_theta is not None:
            dtheta = math.remainder(inp.own_pose.theta - self._last_theta, 2.0 * math.pi)
        elif self._last_now is not None:
            dtheta = self._last_wz * max(0.0, inp.now - self._last_now)
        else:
            dtheta = 0.0
        self._last_theta = inp.own_pose.theta if fresh else None
        self._last_now = inp.now
        if dtheta:
            self._line_dir = _rotate(self._line_dir, -dtheta)
            self._travel_dir = _rotate(self._travel_dir, -dtheta)

    def _filter(self, inp: SafetyInput) -> tuple[Command, str]:
        p = self.params
        if inp.estop:
            # A deadlock escape must not resume when the stop is cleared: the deadlock is timed afresh.
            self._escape_until = None
            self._blocked_since = None
            return STOP, "stop: emergency stop"
        if p.require_heartbeat and (inp.heartbeat_age is None or inp.heartbeat_age > p.heartbeat_timeout):
            return STOP, "stop: coordinator heartbeat lost"
        own_fresh = self._is_fresh(inp.own_pose, inp.now)
        if p.require_pose and not own_fresh:
            return STOP, "stop: own pose stale"

        if self._escape_until is not None:
            if inp.now < self._escape_until:
                return self._escape_cmd, self._escape_reason
            self._escape_until = None
            self._blocked_since = None

        vx, vy, wz = inp.cmd
        reasons = []
        # Only a cube in the OPEN fixed scoop restricts the motion; a closed gate retains it.
        loose_cube = inp.carrying and not inp.gate_closed
        delivering = own_fresh and inp.holds_zone and self._in_delivery_corridor(inp.own_pose)
        on_track = own_fresh and self._track.enabled and self._track.contains(inp.own_pose.x, inp.own_pose.y)
        vx, vy, wz = self._star(vx, vy, wz, reasons)
        if vy != 0.0 or vx < 0.0:
            vx, vy = self._cap(vx, vy, p.max_side_speed, "slow: not straight ahead", reasons)
        returning, off_track = self._outside_track(inp, own_fresh, vx, vy, reasons)
        if inp.line_front and not self._line_seen and inp.now - self._line_clear_since >= p.line_clear_time:
            self._line_dir = self._travel_dir  # a new tape: it lies where the robot was heading for
        if self._line_seen and not inp.line_front:
            self._line_clear_since = inp.now
        self._line_seen = inp.line_front
        line_blocks = inp.line_front and not delivering and not returning
        if (line_blocks and _toward(vx, vy, self._line_dir)) or (off_track and (vx or vy)):
            vx = vy = 0.0
            reasons.append("line ahead" if line_blocks else "no translation away from the track")

        own, peer = inp.own_pose, (inp.peer_pose if self._is_fresh(inp.peer_pose, inp.now) else None)
        if (not own_fresh or peer is None) and inp.peer_seen is not None:
            # No overhead poses: put the peer the camera sees in the robot's own frame. Its heading is
            # unknown: taken as facing us (the worst case, so the deadlock rule still applies).
            bearing, dist = inp.peer_seen
            own = Pose2D(0.0, 0.0, 0.0, inp.now)
            peer = Pose2D(dist * math.cos(bearing), dist * math.sin(bearing), math.pi + bearing, inp.now)
        blocked = False
        if own is not None and peer is not None and (own_fresh or inp.peer_seen is not None):
            # The lease holder has right of way near the square: only slow down / stop, no turn-away.
            # Turning away froze a carrying robot in the corridor for 8 minutes (Webots run 17) and kept
            # a lease holder replanning next to the waiting robot for 10 minutes (run 20).
            vx, vy, wz, blocked, peer_reason = self._avoid_peer(vx, vy, wz, own, peer, turn=not inp.holds_zone)
            if peer_reason:
                reasons.append(peer_reason)

        escape = self._update_deadlock(blocked, inp, own, peer)
        if escape is not None:
            return escape
        if loose_cube and abs(wz) > p.carry_max_turn:
            wz = math.copysign(p.carry_max_turn, wz)
            reasons.append("carrying: slow spin")
        vx, vy, wz = self._star(vx, vy, wz, reasons)  # the peer rule may have added a spin
        if on_track:
            vx, vy, wz = self._body_geofence(inp.own_pose, vx, vy, wz, reasons)
        if (vx or vy) and not inp.line_front:
            norm = math.hypot(vx, vy)
            self._travel_dir = (vx / norm, vy / norm)
        return (vx, vy, wz), ", ".join(reasons) or "ok"

    def _star(self, vx: float, vy: float, wz: float, reasons: list[str]) -> Command:
        """Without arcs: translate or spin in place, never both (the spin wins). With arcs: cap both rates."""
        if wz != 0.0 and (vx != 0.0 or vy != 0.0) and self.params.allow_arcs:
            p = self.params
            vx, vy = self._cap(vx, vy, p.max_arc_speed, "arc: slow", reasons)
            if abs(wz) > p.max_arc_turn:
                wz = math.copysign(p.max_arc_turn, wz)
                reasons.append("arc: slow turn")
            return vx, vy, wz
        if wz != 0.0 and (vx != 0.0 or vy != 0.0):
            if "spin in place only" not in reasons:
                reasons.append("spin in place only")
            return 0.0, 0.0, wz
        return vx, vy, wz

    @staticmethod
    def _cap(vx: float, vy: float, limit: float, why: str, reasons: list[str]) -> tuple[float, float]:
        """Scale the translation down to `limit` m/s keeping its direction."""
        norm = math.hypot(vx, vy)
        if norm <= limit:
            return vx, vy
        reasons.append(why)
        return vx * limit / norm, vy * limit / norm

    def _outside_track(self, inp: SafetyInput, own_fresh: bool, vx: float, vy: float,
                       reasons: list[str]) -> tuple[bool, bool]:
        """Center outside the polygon: return (translating back in, translation must be blocked)."""
        if not own_fresh or not self._track.enabled:
            return False, False
        own = inp.own_pose
        if self._track.contains(own.x, own.y):
            return False, False
        cx, cy = self._track.centroid
        travel = own.theta + math.atan2(vy, vx) if (vx or vy) else own.theta
        returning = abs(relative_bearing(own.x, own.y, travel, cx, cy)) <= math.radians(self.params.track_return_cone)
        reasons.append("outside track: " + ("back in" if returning else "turn back"))
        return returning, not returning

    def _body_fits(self, pose: Pose2D, vx: float, vy: float, wz: float) -> bool:
        """Every body corner stays inside the track along the predicted motion."""
        start = (pose.x, pose.y, pose.theta)
        return path_inside(self._track, self._body, start, vx, wz, self.params.track_horizon, vy=vy)

    def _body_geofence(self, pose: Pose2D, vx: float, vy: float, wz: float, reasons: list[str]) -> Command:
        if (vx == 0.0 and vy == 0.0 and wz == 0.0) or self._body_fits(pose, vx, vy, wz):
            return vx, vy, wz
        if wz != 0.0:  # a spin (commands are pure by now): the other direction may fit
            if self._body_fits(pose, 0.0, 0.0, -wz):
                reasons.append("turn the other way (body geofence)")
                return 0.0, 0.0, -wz
            reasons.append("body would cross the tape: no room to turn")
            return STOP
        reasons.append("track edge in the way (pose)")
        return STOP

    def _in_delivery_corridor(self, pose: Pose2D) -> bool:
        return self._zone.in_corridor(pose.x, pose.y) and self._zone.aligned(pose.theta)

    def _is_fresh(self, pose: Pose2D | None, now: float) -> bool:
        return pose is not None and now - pose.stamp <= self.params.pose_timeout

    def _avoid_peer(self, vx: float, vy: float, wz: float, own: Pose2D, peer: Pose2D, turn: bool = True):
        """Slow down or stop a translation toward the peer, and spin away from it when it is close.

        The cone is around the travel direction (the heading while spinning or still), so moving
        away from the peer, sideways or backward, is never held back."""
        p = self.params
        moving = vx != 0.0 or vy != 0.0
        travel = own.theta + math.atan2(vy, vx) if moving else own.theta
        if abs(relative_bearing(own.x, own.y, travel, peer.x, peer.y)) >= math.radians(p.front_cone_deg):
            return vx, vy, wz, False, ""
        d = distance(own.x, own.y, peer.x, peer.y)
        blocked = False
        reason = ""
        if d < p.stop_distance:
            blocked = moving
            vx = vy = 0.0
            reason = f"peer ahead at {d:.2f} m: stop"
        elif d < p.slow_distance and moving:
            k = (d - p.stop_distance) / (p.slow_distance - p.stop_distance)
            vx, vy = vx * k, vy * k
            reason = f"peer ahead at {d:.2f} m: slow"
        bearing = relative_bearing(own.x, own.y, own.theta, peer.x, peer.y)
        if d < p.avoid_distance and turn and abs(bearing) < math.radians(p.front_cone_deg):
            # Spin away from a peer in front; dead ahead counts as "left" so both robots break right.
            # One behind (in the cone only because the robot backs toward it) is not spun away from:
            # that would swing the rear into it.
            wz = -p.avoid_turn if bearing >= 0.0 else p.avoid_turn
            reason = reason or f"peer ahead at {d:.2f} m"
            reason += ", turn away"
        return vx, vy, wz, blocked, reason

    def _update_deadlock(self, blocked: bool, inp: SafetyInput, own: Pose2D | None, peer: Pose2D | None):
        p = self.params
        if not blocked or not self._peer_faces_us(own, peer):
            self._blocked_since = None
            return None
        if self._blocked_since is None:
            self._blocked_since = inp.now
            return None
        if p.has_priority or inp.now - self._blocked_since < p.deadlock_time:
            return None
        if inp.carrying and not inp.gate_closed:
            # A spin could throw the cube out: hold still and let the priority robot go around.
            self._escape_cmd = STOP
            self._escape_reason = "deadlock: hold still (carrying)"
        else:
            bearing = relative_bearing(own.x, own.y, own.theta, peer.x, peer.y)
            turn = -p.escape_turn if bearing >= 0.0 else p.escape_turn
            self._escape_cmd = (0.0, 0.0, turn)
            self._escape_reason = "deadlock: rotate away"
        self._escape_until = inp.now + p.escape_time
        return self._escape_cmd, self._escape_reason

    def _peer_faces_us(self, own: Pose2D, peer: Pose2D) -> bool:
        bearing = relative_bearing(peer.x, peer.y, peer.theta, own.x, own.y)
        return abs(bearing) < math.radians(self.params.front_cone_deg)


def _toward(vx: float, vy: float, direction: tuple[float, float]) -> bool:
    """The translation has a component along direction (robot frame)."""
    return vx * direction[0] + vy * direction[1] > 1e-9


def _rotate(v: tuple[float, float], angle: float) -> tuple[float, float]:
    c, s = math.cos(angle), math.sin(angle)
    return c * v[0] - s * v[1], s * v[0] + c * v[1]
