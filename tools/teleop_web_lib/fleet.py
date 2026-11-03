"""
Optional ROS 2 bridge between `tools/teleop_web.py` and the `fleet_link` node.

Pure state mapping and grant-freshness tracking live in `to_behavior_state` and
`BridgeTracker` (no `rclpy`, testable with `pytest`). `RosFleetBridge` imports
`rclpy` lazily only when `fleet.enabled` is true.
"""

import logging
import threading
import time

from teleop_web_lib.autopilot import (
    PREFIX_DONE,
    PREFIX_DROP,
    PREFIX_LINE,
    PREFIX_NO_CUBES,
    STATE_ALIGN,
    STATE_APPROACH,
    STATE_CARRY,
    STATE_CARRY_WAIT_ZONE,
    STATE_CATCH,
    STATE_CATCH_CHECK,
    STATE_DROP_BACKUP,
    STATE_DROP_TURN,
    STATE_EXPLORE,
    STATE_MANUAL,
    STATE_PAUSED,
    STATE_SEARCH,
)
from teleop_web_lib.config import FleetConfig

log = logging.getLogger("teleop_web")

# Relative topics inside the robot namespace (must match mobile_base/topics.py).
TOPIC_BEHAVIOR_STATE = "behavior/state"
TOPIC_SCOOP_HAS_CUBE = "scoop/has_cube"
TOPIC_ZONE_WANT = "zone/want"
TOPIC_ZONE_DELIVERED = "zone/delivered"
TOPIC_ZONE_GRANTED = "zone/granted"

NODE_NAME = "teleop_fleet_bridge"
QOS_DEPTH = 10
SPIN_TIMEOUT_S = 0.05
JOIN_TIMEOUT_S = 1.0

# mobile_base local State names consumed by fleet_link.
LOCAL_PAUSED = "PAUSED"
LOCAL_MANUAL = "MANUAL"
LOCAL_SEARCH = "SEARCH"
LOCAL_APPROACH = "APPROACH"
LOCAL_CATCH = "CATCH"
LOCAL_CARRY = "CARRY"
LOCAL_WAIT_ZONE = "WAIT_ZONE"
LOCAL_DELIVER = "DELIVER"
LOCAL_RELEASE = "RELEASE"
LOCAL_LINE_ESCAPE = "LINE_ESCAPE"

_EXACT_STATE_MAP = {
    STATE_PAUSED: LOCAL_PAUSED,
    STATE_MANUAL: LOCAL_MANUAL,
    STATE_SEARCH: LOCAL_SEARCH,
    STATE_APPROACH: LOCAL_APPROACH,
    STATE_ALIGN: LOCAL_APPROACH,
    STATE_CATCH: LOCAL_CATCH,
    STATE_CATCH_CHECK: LOCAL_CATCH,
    STATE_CARRY_WAIT_ZONE: LOCAL_WAIT_ZONE,
    STATE_DROP_BACKUP: LOCAL_RELEASE,
    STATE_DROP_TURN: LOCAL_RELEASE,
}


def to_behavior_state(pilot_state: str) -> str:
    """
    Map an `AutoPilot` state label to the `mobile_base` local `State` name
    expected by `fleet_link`.
    """
    exact = _EXACT_STATE_MAP.get(pilot_state)
    if exact is not None:
        return exact
    if pilot_state.startswith((PREFIX_DONE, PREFIX_NO_CUBES)):
        return LOCAL_PAUSED
    if pilot_state.startswith(PREFIX_LINE):
        return LOCAL_LINE_ESCAPE
    if pilot_state.startswith(STATE_EXPLORE):
        return LOCAL_SEARCH
    if pilot_state.startswith(STATE_CARRY):
        return LOCAL_CARRY
    if pilot_state.startswith(PREFIX_DROP):
        return LOCAL_DELIVER
    return LOCAL_PAUSED


class BridgeTracker:
    """
    Pure-logic state of the fleet bridge: grant freshness and change/periodic
    publishing decisions.
    """

    def __init__(
        self,
        grant_timeout_s: float = 1.0,
        republish_period_s: float = 0.5,
    ):
        """
        Start with no zone grant received and nothing published yet.
        """
        self.grant_timeout_s = grant_timeout_s
        self.republish_period_s = republish_period_s
        self._granted = False
        self._grant_stamp_s: float | None = None
        self._last_state: str | None = None
        self._last_carrying: bool | None = None
        self._last_want: bool | None = None
        self._last_pub_s: float | None = None

    def on_granted(self, granted: bool, now: float) -> None:
        """
        Record a `zone/granted` message received at `now`.
        """
        self._granted = bool(granted)
        self._grant_stamp_s = now

    def is_granted(self, now: float) -> bool:
        """
        True when `zone/granted` was true within `grant_timeout_s`.
        """
        if not self._granted or self._grant_stamp_s is None:
            return False
        return (now - self._grant_stamp_s) <= self.grant_timeout_s

    def outgoing(
        self,
        pilot_state: str,
        carrying: bool,
        want: bool,
        now: float,
    ) -> tuple[str | None, bool | None, bool | None]:
        """
        Values `(state, carrying, want)` that must be published at `now`;
        unchanged fields are `None` until `republish_period_s` elapses.
        """
        mapped = to_behavior_state(pilot_state)
        due = (
            self._last_pub_s is None
            or (now - self._last_pub_s) >= self.republish_period_s
        )
        out_state = mapped if due or mapped != self._last_state else None
        out_carry = carrying if due or carrying != self._last_carrying else None
        out_want = want if due or want != self._last_want else None
        if any(v is not None for v in (out_state, out_carry, out_want)):
            self._last_state = mapped
            self._last_carrying = carrying
            self._last_want = want
            self._last_pub_s = now
        return out_state, out_carry, out_want


class NullFleetBridge:
    """
    No-op fleet bridge used when standalone mode (`fleet.enabled: false`) is
    active: the zone is always treated as granted.
    """

    def granted(self) -> bool:
        """
        Always True when running without a fleet arbiter.
        """
        return True

    def sync(self, state: str, carrying: bool, want: bool) -> None:
        """
        Ignore state updates in standalone mode.
        """

    def delivered(self, color: str) -> None:
        """
        Ignore delivery reports in standalone mode.
        """

    def close(self) -> None:
        """
        Nothing to release in standalone mode.
        """


class RosFleetBridge:
    """
    ROS 2 bridge publishing `behavior/state`, `scoop/has_cube`, `zone/want`
    and `zone/delivered` to `fleet_link`, and subscribing to `zone/granted`.
    """

    def __init__(self, cfg: FleetConfig):
        """
        Initialise `rclpy` if needed, create the namespaced node and spin it
        on a background daemon thread.
        """
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from std_msgs.msg import Bool, String

        self._rclpy = rclpy
        self._bool_cls = Bool
        self._str_cls = String
        self._owns_rclpy = not rclpy.ok()
        if self._owns_rclpy:
            rclpy.init()

        ns = f"/{cfg.namespace.strip('/')}" if cfg.namespace.strip("/") else ""
        self._node = Node(NODE_NAME, namespace=ns)
        self._tracker = BridgeTracker(
            cfg.grant_timeout_s,
            cfg.republish_period_s,
        )
        self._lock = threading.Lock()
        self._pub_state = self._node.create_publisher(
            String,
            TOPIC_BEHAVIOR_STATE,
            QOS_DEPTH,
        )
        self._pub_carry = self._node.create_publisher(
            Bool,
            TOPIC_SCOOP_HAS_CUBE,
            QOS_DEPTH,
        )
        self._pub_want = self._node.create_publisher(
            Bool,
            TOPIC_ZONE_WANT,
            QOS_DEPTH,
        )
        self._pub_delivered = self._node.create_publisher(
            String,
            TOPIC_ZONE_DELIVERED,
            QOS_DEPTH,
        )
        self._node.create_subscription(
            Bool,
            TOPIC_ZONE_GRANTED,
            self._on_granted,
            QOS_DEPTH,
        )

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._running = True
        self._thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._thread.start()
        log.info("fleet bridge active under namespace %s", ns or "/")

    def _on_granted(self, msg) -> None:
        """
        Store the latest `zone/granted` boolean from `fleet_link`.
        """
        now = time.monotonic()
        with self._lock:
            self._tracker.on_granted(bool(msg.data), now)

    def _spin_loop(self) -> None:
        """
        Process incoming ROS callbacks until `close()` is called.
        """
        while self._running and self._rclpy.ok():
            self._executor.spin_once(timeout_sec=SPIN_TIMEOUT_S)

    def granted(self) -> bool:
        """
        True when `fleet_link` recently reported `zone/granted = True`.
        """
        now = time.monotonic()
        with self._lock:
            return self._tracker.is_granted(now)

    def sync(self, state: str, carrying: bool, want: bool) -> None:
        """
        Publish changed (or periodically refreshed) state, carry and want
        topics to `fleet_link`.
        """
        now = time.monotonic()
        with self._lock:
            out_state, out_carry, out_want = self._tracker.outgoing(
                state,
                carrying,
                want,
                now,
            )
        if out_state is not None:
            self._pub_state.publish(self._str_cls(data=out_state))
        if out_carry is not None:
            self._pub_carry.publish(self._bool_cls(data=out_carry))
        if out_want is not None:
            self._pub_want.publish(self._bool_cls(data=out_want))

    def delivered(self, color: str) -> None:
        """
        Publish the verified delivered cube colour on `zone/delivered`.
        """
        if color:
            self._pub_delivered.publish(self._str_cls(data=color))

    def close(self) -> None:
        """
        Stop spinning, destroy the node and shut down `rclpy` if owned.
        """
        self._running = False
        self._thread.join(timeout=JOIN_TIMEOUT_S)
        self._executor.remove_node(self._node)
        self._node.destroy_node()
        if self._owns_rclpy and self._rclpy.ok():
            self._rclpy.shutdown()


def build_fleet_bridge(cfg: FleetConfig, fake: bool = False):
    """
    Build `RosFleetBridge` when `cfg.enabled` is true (and not `fake`), else
    `NullFleetBridge`.
    """
    if not cfg.enabled or fake:
        return NullFleetBridge()
    return RosFleetBridge(cfg)
