"""
Peer robot avoidance from overhead poses. Pure logic, no rclpy.

Time is passed in as a float in seconds. Used by the obstacle_guard node to
keep Ferb from driving into the other collector.
"""

import math
from dataclasses import dataclass

from mobile_base.mathutil import wrap_angle

# --- Defaults (the real values live in config/collector.yaml) -----------------
DEFAULT_POSE_MAX_AGE_S = 0.5
DEFAULT_FRONT_CONE_DEG = 60.0
DEFAULT_PEER_SLOW_DISTANCE_M = 0.60
DEFAULT_PEER_STOP_DISTANCE_M = 0.40
DEFAULT_AVOID_TURN_RAD_S = 0.6
DEFAULT_DEADLOCK_TIME_S = 3.0
DEFAULT_ESCAPE_TURN_RAD_S = 0.8

# --- Reasons returned next to the filtered command ----------------------------
REASON_OK = "ok"
REASON_DEADLOCK = "deadlock: rotate away"

# A velocity command: (vx m/s, vy m/s, wz rad/s).
Velocity = tuple[float, float, float]


@dataclass(frozen=True)
class Pose2D:
    """
    Planar pose in the table frame, with the time it was received.
    """

    x: float
    y: float
    theta: float
    stamp: float


@dataclass(frozen=True)
class PeerAvoidanceParams:
    """
    Tunable parameters of `PeerAvoidance`.

    Field names double as ROS parameter names (see recolector_common.params),
    so the two distances carry the `peer_` prefix to stay apart from the
    ultrasonic guard thresholds.
    """

    pose_max_age_s: float = DEFAULT_POSE_MAX_AGE_S
    front_cone_deg: float = DEFAULT_FRONT_CONE_DEG
    peer_slow_distance_m: float = DEFAULT_PEER_SLOW_DISTANCE_M
    peer_stop_distance_m: float = DEFAULT_PEER_STOP_DISTANCE_M
    avoid_turn_rad_s: float = DEFAULT_AVOID_TURN_RAD_S
    deadlock_time_s: float = DEFAULT_DEADLOCK_TIME_S
    escape_turn_rad_s: float = DEFAULT_ESCAPE_TURN_RAD_S


def relative_bearing(own: Pose2D, peer: Pose2D) -> float:
    """
    Bearing of `peer` seen from `own`, relative to own heading (left > 0).
    """
    world_bearing = math.atan2(peer.y - own.y, peer.x - own.x)
    return wrap_angle(world_bearing - own.theta)


class PeerAvoidance:
    """
    Filters velocity commands using overhead poses to avoid the peer
    collector.

    Ferb yields in a deadlock: after `deadlock_time_s` blocked by the peer,
    rotate in place at `escape_turn_rad_s` away from the peer until it leaves
    the front cone.
    """

    def __init__(self, params: PeerAvoidanceParams | None = None):
        """
        Start with no deadlock in progress.
        """
        self.params = params or PeerAvoidanceParams()
        self._blocked_since_s: float | None = None
        self._escaping = False

    def filter(
        self,
        cmd: Velocity,
        own_pose: Pose2D | None,
        peer_pose: Pose2D | None,
        now: float,
    ) -> tuple[Velocity, str]:
        """
        Return the (possibly modified) command and a human-readable reason.
        """
        p = self.params
        if not (
            self._is_fresh(own_pose, now) and self._is_fresh(peer_pose, now)
        ):
            return self._pass_through(cmd)

        bearing = relative_bearing(own_pose, peer_pose)
        if abs(bearing) >= math.radians(p.front_cone_deg):
            return self._pass_through(cmd)

        if self._escaping:
            return self._escape_command(bearing), REASON_DEADLOCK

        dist_m = math.hypot(peer_pose.x - own_pose.x, peer_pose.y - own_pose.y)
        return self._avoid(cmd, bearing, dist_m, now)

    def _is_fresh(self, pose: Pose2D | None, now: float) -> bool:
        """
        True when `pose` exists and is not older than `pose_max_age_s`.
        """
        if pose is None:
            return False
        return (now - pose.stamp) <= self.params.pose_max_age_s

    def _pass_through(self, cmd: Velocity) -> tuple[Velocity, str]:
        """
        Peer not relevant: reset the deadlock state and keep the command.
        """
        self._blocked_since_s = None
        self._escaping = False
        return cmd, REASON_OK

    def _escape_command(self, bearing: float) -> Velocity:
        """
        Rotate in place away from the peer.
        """
        turn = self.params.escape_turn_rad_s
        return (0.0, 0.0, -turn if bearing >= 0.0 else turn)

    def _avoid(
        self,
        cmd: Velocity,
        bearing: float,
        dist_m: float,
        now: float,
    ) -> tuple[Velocity, str]:
        """
        Slow down, stop and turn away while the peer is inside the front
        cone, and start the deadlock escape if blocked for too long.
        """
        p = self.params
        vx, vy, wz = cmd
        blocked = False
        reason = ""

        if dist_m < p.peer_stop_distance_m:
            blocked = vx > 0.0 or self._blocked_since_s is not None
            vx = min(vx, 0.0)
            reason = f"peer ahead at {dist_m:.2f} m: stop"
        elif dist_m < p.peer_slow_distance_m and vx > 0.0:
            span_m = p.peer_slow_distance_m - p.peer_stop_distance_m
            vx *= (dist_m - p.peer_stop_distance_m) / span_m
            reason = f"peer ahead at {dist_m:.2f} m: slow"

        if dist_m < p.peer_slow_distance_m:
            turn = p.avoid_turn_rad_s
            wz = -turn if bearing >= 0.0 else turn
            reason = (reason or f"peer ahead at {dist_m:.2f} m") + ", turn away"

        if self._is_deadlocked(blocked, now):
            self._escaping = True
            return self._escape_command(bearing), REASON_DEADLOCK
        return (vx, vy, wz), reason or REASON_OK

    def _is_deadlocked(self, blocked: bool, now: float) -> bool:
        """
        Track how long the robot has been blocked; True once it exceeds
        `deadlock_time_s`.
        """
        if not blocked:
            self._blocked_since_s = None
            return False
        if self._blocked_since_s is None:
            self._blocked_since_s = now
            return False
        return (now - self._blocked_since_s) >= self.params.deadlock_time_s
