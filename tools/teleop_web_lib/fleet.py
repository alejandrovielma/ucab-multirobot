"""
Link between the browser tool's Auto mode and the fleet (Perry's zone
arbiter). Auto only knows the `Fleet` protocol (autopilot.py); `FleetBridge`
is the ROS 2 side: it publishes what `fleet_link` listens to and reads
`zone/granted` back. `fleet_link` runs next to the tool and does the rest.

Only the bridge needs rclpy, imported when it is built, so the tool still
runs on the laptop without ROS.
"""

import logging
import sys
import threading
import time

from teleop_web_lib.config import MOBILE_BASE_PACKAGE_DIR, FleetConfig

log = logging.getLogger("teleop_web")

# Auto state prefix -> the `behavior/state` name `fleet_link` understands
# (mobile_base.states.State). The first prefix that matches wins; anything
# else is reported as paused (the fleet sees it as stopped).
_STATE_NAMES = (
    ("SEARCH", "SEARCH"),
    ("EXPLORE", "SEARCH"),
    ("APPROACH", "APPROACH"),
    ("ALIGN", "APPROACH"),
    ("CATCH", "CATCH"),
    ("CARRY (wait zone)", "WAIT_ZONE"),
    ("CARRY", "CARRY"),
    ("DROP", "DELIVER"),
)
STATE_PAUSED = "PAUSED"
NODE_NAME = "teleop_web_fleet"
# How long close() waits for the spin thread to finish.
JOIN_TIMEOUT_S = 1.0


def behavior_state_name(state: str) -> str:
    """
    The `behavior/state` name for an Auto state such as "CARRY (align)".
    """
    for prefix, name in _STATE_NAMES:
        if state.startswith(prefix):
            return name
    return STATE_PAUSED


def build_fleet(cfg: FleetConfig) -> "FleetBridge | None":
    """
    The bridge, or None when the fleet link is off in the YAML. Needs ROS 2
    sourced when on: failing here beats driving without turns.
    """
    return FleetBridge(cfg) if cfg.enabled else None


class FleetBridge:
    """
    Publishes Auto's zone wishes to `fleet_link` and reports whether the
    turn was granted. Thread-safe: Auto calls it from the control thread
    while rclpy spins in its own.
    """

    def __init__(self, cfg: FleetConfig):
        """
        Start the ROS node and its spin thread.
        """
        # Imported here: neither rclpy nor the ROS packages exist on the
        # laptop.
        import rclpy
        from std_msgs.msg import Bool, String

        sys.path.insert(0, str(MOBILE_BASE_PACKAGE_DIR))
        from mobile_base import topics

        self._cfg = cfg
        self._msg_bool, self._msg_string = Bool, String
        self._lock = threading.Lock()
        self._granted = False
        self._granted_s = float("-inf")
        self._sent: tuple | None = None
        self._sent_s = float("-inf")

        rclpy.init()
        self._rclpy = rclpy
        self._node = rclpy.create_node(NODE_NAME, namespace=cfg.namespace)
        make = self._node.create_publisher
        depth = topics.DEFAULT_QOS_DEPTH
        self._pub_want = make(Bool, topics.TOPIC_ZONE_WANT, depth)
        self._pub_delivered = make(String, topics.TOPIC_ZONE_DELIVERED, depth)
        self._pub_state = make(String, topics.TOPIC_BEHAVIOR_STATE, depth)
        self._pub_cube = make(Bool, topics.TOPIC_SCOOP_HAS_CUBE, depth)
        self._node.create_subscription(
            Bool,
            topics.TOPIC_ZONE_GRANTED,
            self._on_granted,
            depth,
        )
        self._thread = threading.Thread(
            target=rclpy.spin,
            args=(self._node,),
            daemon=True,
        )
        self._thread.start()
        log.info("fleet bridge up under /%s", cfg.namespace)

    def _on_granted(self, msg) -> None:
        """
        `fleet_link` says whether we hold the zone.
        """
        with self._lock:
            self._granted = bool(msg.data)
            self._granted_s = time.monotonic()

    def granted(self) -> bool:
        """
        True while `fleet_link` reported the zone as ours recently. Silence
        counts as not granted: a dead link must not leave us inside.
        """
        with self._lock:
            fresh = (
                time.monotonic() - self._granted_s
            ) <= self._cfg.granted_timeout_s
            return self._granted and fresh

    def update(self, state: str, want: bool, carrying: bool) -> None:
        """
        Publish the current Auto state, wish and cube flag when they change
        and, otherwise, once per `republish_period_s` (a restarted
        `fleet_link` must learn them again).
        """
        now = time.monotonic()
        current = (behavior_state_name(state), want, carrying)
        due = now - self._sent_s >= self._cfg.republish_period_s
        if current == self._sent and not due:
            return
        self._sent, self._sent_s = current, now
        name, want, carrying = current
        self._pub_state.publish(self._msg_string(data=name))
        self._pub_want.publish(self._msg_bool(data=want))
        self._pub_cube.publish(self._msg_bool(data=carrying))

    def delivered(self, color: str) -> None:
        """
        A cube of `color` was left in the area.
        """
        self._pub_delivered.publish(self._msg_string(data=color))

    def close(self) -> None:
        """
        Let go of the zone and stop the ROS node.
        """
        self.update(STATE_PAUSED, want=False, carrying=False)
        self._rclpy.shutdown()
        self._thread.join(timeout=JOIN_TIMEOUT_S)
