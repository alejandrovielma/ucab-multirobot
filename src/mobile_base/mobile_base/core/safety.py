"""SafetyGate: last filter between the behavior command and the motors.

Rules, applied in order every tick:
1. Coordinator heartbeat lost (when required) -> full stop.
2. Own pose stale (when required) -> full stop.
3. Forward-only translation: the robot has a single FRONT line sensor, so strafing or
   reversing is blind to the track edge (in Webots, strafing drove both robots off the board).
   Turning in place is allowed; vy and negative vx are removed.
4. Front line sensor on black -> no forward motion (turning allowed).
5. Peer robot ahead (overhead-camera poses) -> slow down, stop, and turn away from it.
   If both robots stay blocked facing each other for deadlock_time, the robot
   WITHOUT priority rotates away (or holds still when it carries a cube, since any
   maneuver would drop it from the fixed scoop).
6. Carrying a cube -> turns only in arcs (see _carry_limits).
"""

import math
from dataclasses import dataclass

from mobile_base.core.geometry import distance, relative_bearing

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
    # The fixed scoop only retains a cube when it is pushed forward: in-place spins and strafes
    # eject it (observed in Webots), and reversing leaves it behind.
    carry_min_forward: float = 0.05  # m/s, forward speed forced while turning with a cube
    carry_max_turn: float = 0.6  # rad/s, turn-rate limit with a cube


@dataclass(frozen=True)
class SafetyInput:
    cmd: Command
    line_front: bool
    own_pose: Pose2D | None
    peer_pose: Pose2D | None
    now: float
    heartbeat_age: float | None  # None = never received
    carrying: bool = False


class SafetyGate:
    def __init__(self, params: SafetyParams):
        self.params = params
        self._blocked_since: float | None = None
        self._escape_until: float | None = None
        self._escape_cmd: Command = STOP
        self._escape_reason = ""

    def filter(self, inp: SafetyInput) -> tuple[Command, str]:
        p = self.params
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
        if vy != 0.0 or vx < 0.0:
            vx, vy = max(vx, 0.0), 0.0
            reasons.append("forward only")
        if inp.line_front and vx > 0.0:
            vx = 0.0
            reasons.append("line ahead")

        peer = inp.peer_pose if self._is_fresh(inp.peer_pose, inp.now) else None
        blocked = False
        if own_fresh and peer is not None:
            vx, wz, blocked, peer_reason = self._avoid_peer(vx, wz, inp.own_pose, peer)
            if peer_reason:
                reasons.append(peer_reason)

        escape = self._update_deadlock(blocked, inp, peer)
        if escape is not None:
            return escape
        if inp.carrying:
            if inp.line_front and "line ahead" not in reasons:
                reasons.append("line ahead")
            vx, vy, wz = self._carry_limits(vx, wz, blocked or inp.line_front, reasons)
        return (vx, vy, wz), ", ".join(reasons) or "ok"

    def _carry_limits(self, vx: float, wz: float, forward_blocked: bool, reasons: list[str]) -> Command:
        p = self.params
        vx = max(vx, 0.0)
        wz = max(-p.carry_max_turn, min(p.carry_max_turn, wz))
        if wz != 0.0:
            if forward_blocked:
                reasons.append("carrying: no in-place turn")
                return 0.0, 0.0, 0.0
            if vx < p.carry_min_forward:
                vx = p.carry_min_forward
                reasons.append("carrying: turn in arc")
        return vx, 0.0, wz

    def _is_fresh(self, pose: Pose2D | None, now: float) -> bool:
        return pose is not None and now - pose.stamp <= self.params.pose_timeout

    def _avoid_peer(self, vx: float, wz: float, own: Pose2D, peer: Pose2D):
        p = self.params
        bearing = relative_bearing(own.x, own.y, own.theta, peer.x, peer.y)
        if abs(bearing) >= math.radians(p.front_cone_deg):
            return vx, wz, False, ""
        d = distance(own.x, own.y, peer.x, peer.y)
        blocked = False
        reason = ""
        if d < p.stop_distance:
            blocked = vx > 0.0
            vx = min(vx, 0.0)
            reason = f"peer ahead at {d:.2f} m: stop"
        elif d < p.slow_distance and vx > 0.0:
            vx *= (d - p.stop_distance) / (p.slow_distance - p.stop_distance)
            reason = f"peer ahead at {d:.2f} m: slow"
        if d < p.avoid_distance:
            # Turn away from the peer; dead ahead counts as "left" so both robots break right.
            wz = -p.avoid_turn if bearing >= 0.0 else p.avoid_turn
            reason = reason or f"peer ahead at {d:.2f} m"
            reason += ", turn away"
        return vx, wz, blocked, reason

    def _update_deadlock(self, blocked: bool, inp: SafetyInput, peer: Pose2D | None):
        p = self.params
        if not blocked or not self._peer_faces_us(inp.own_pose, peer):
            self._blocked_since = None
            return None
        if self._blocked_since is None:
            self._blocked_since = inp.now
            return None
        if p.has_priority or inp.now - self._blocked_since < p.deadlock_time:
            return None
        if inp.carrying:
            # Any escape maneuver would drop the cube: hold still and let the priority robot go around.
            self._escape_cmd = STOP
            self._escape_reason = "deadlock: hold still (carrying)"
        else:
            bearing = relative_bearing(inp.own_pose.x, inp.own_pose.y, inp.own_pose.theta, peer.x, peer.y)
            turn = -p.escape_turn if bearing >= 0.0 else p.escape_turn
            self._escape_cmd = (0.0, 0.0, turn)
            self._escape_reason = "deadlock: rotate away"
        self._escape_until = inp.now + p.escape_time
        return self._escape_cmd, self._escape_reason

    def _peer_faces_us(self, own: Pose2D, peer: Pose2D) -> bool:
        bearing = relative_bearing(peer.x, peer.y, peer.theta, own.x, own.y)
        return abs(bearing) < math.radians(self.params.front_cone_deg)
