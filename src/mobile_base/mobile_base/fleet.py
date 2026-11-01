"""
Pure logic of the zone-arbiter side of the fleet link (Phineas and Perry):
state vocabulary mapping, heartbeat check and delivery-zone lease client.

Peer robot avoidance lives in peer_avoidance.py. No rclpy: time is passed in
as a float in seconds.
"""

from dataclasses import dataclass

from mobile_base.states import FleetState, State

# --- Defaults (the real values live in config/collector.yaml) -----------------
DEFAULT_ROBOT_NAME = "ferb"
DEFAULT_HEARTBEAT_TIMEOUT_S = 2.0
DEFAULT_STATE_RATE_HZ = 2.0
DEFAULT_RENEW_PERIOD_S = 20.0

# Local behaviour state -> shared multi-robot state. Anything not listed
# (PAUSED, MANUAL, NO_CAMERA, unknown) maps to FleetState.STOP.
_STATE_MAP = {
    State.SEARCH: FleetState.EXPLORE,
    State.APPROACH: FleetState.APPROACH,
    State.ARRIVED: FleetState.CAPTURE,
    State.CATCH: FleetState.CAPTURE,
    State.CARRY: FleetState.CARRY,
    State.WAIT_ZONE: FleetState.CARRY,
    State.DELIVER: FleetState.DELIVER,
    State.RELEASE: FleetState.DELIVER,
    State.LINE_ESCAPE: FleetState.EXPLORE,
}


def to_fleet_state(local: str) -> str:
    """
    Map a local behaviour state name to the shared multi-robot vocabulary.
    """
    return _STATE_MAP.get(local, FleetState.STOP).value


@dataclass
class FleetLinkParams:
    """
    Tunable parameters of the `fleet_link` node.

    Field names double as ROS parameter names (see recolector_common.params).
    """

    robot_name: str = DEFAULT_ROBOT_NAME
    heartbeat_timeout_s: float = DEFAULT_HEARTBEAT_TIMEOUT_S
    state_rate_hz: float = DEFAULT_STATE_RATE_HZ
    renew_period_s: float = DEFAULT_RENEW_PERIOD_S


class Heartbeat:
    """
    Tracks the liveness of the zone arbiter heartbeat.
    """

    def __init__(self, timeout_s: float = DEFAULT_HEARTBEAT_TIMEOUT_S):
        """
        Start with no heartbeat received.
        """
        self.timeout_s = timeout_s
        self._last_beat_s: float | None = None

    def beat(self, now: float) -> None:
        """
        Record that a heartbeat arrived at time `now`.
        """
        self._last_beat_s = now

    def ok(self, now: float) -> bool:
        """
        True if a heartbeat arrived no longer than `timeout_s` ago.
        """
        if self._last_beat_s is None:
            return False
        return (now - self._last_beat_s) <= self.timeout_s


class ZoneClient:
    """
    Client-side state for requesting, renewing and releasing the delivery
    square.
    """

    def __init__(
        self,
        robot: str = DEFAULT_ROBOT_NAME,
        renew_period_s: float = DEFAULT_RENEW_PERIOD_S,
    ):
        """
        Start neither wanting nor holding the zone.
        """
        self.robot = robot
        self.renew_period_s = renew_period_s
        self.want = False
        self.occupant = ""
        self.delivered = False
        self.color = ""
        self._was_wanting = False
        self._pending_release = False
        self._last_request_s: float | None = None

    @property
    def holds(self) -> bool:
        """
        True when the arbiter reports this robot as the zone occupant.
        """
        return self.occupant == self.robot

    def update(
        self,
        want: bool | None = None,
        occupant: str | None = None,
    ) -> None:
        """
        Apply new information from the node; `None` leaves a field as is.
        """
        if occupant is not None:
            self.occupant = occupant
        if want is not None:
            if self._was_wanting and not want and self.holds:
                self._pending_release = True
            self.want = want
            self._was_wanting = want

    def mark_delivered(self, color: str) -> None:
        """
        Remember that a cube of `color` was dropped in the zone.
        """
        self.delivered = True
        self.color = color

    def should_request(self, now: float) -> bool:
        """
        True when the zone must be requested (or its lease renewed).
        """
        if not self.want:
            return False
        if not self.holds or self._last_request_s is None:
            return True
        return (now - self._last_request_s) >= self.renew_period_s

    def note_requested(self, now: float) -> None:
        """
        Record that a request was sent at time `now`.
        """
        self._last_request_s = now

    def should_release(self) -> bool:
        """
        True when the zone must be released.
        """
        return self._pending_release or (not self.want and self.holds)

    def pop_release(self) -> tuple[bool, str]:
        """
        Consume the release state and reset the delivered/color flags.

        Returns the (delivered, color) pair to send with the release.
        """
        self._pending_release = False
        self._last_request_s = None
        delivered, color = self.delivered, self.color
        self.delivered = False
        self.color = ""
        return delivered, color
