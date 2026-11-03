"""
behavior_fsm node: the collector's state machine (search, approach, catch,
carry to the delivery area, deliver, release) and count of delivered cubes.

The logic is in collector.py (on top of approach.py); this node only wires
it. Publishes cmd_vel_raw (goes through obstacle_guard), behavior/state,
scoop/lower, count, zone/want and zone/delivered.

Delivery turn: zone/want goes true when the robot is close to the area; it
waits outside until zone/granted is true, delivers, publishes the colour on
zone/delivered and then sets zone/want false (fleet_link releases the
zone). With `require_zone_grant: false` (simulation) it does not wait.

Manual control: if cmd_vel_teleop arrives (WASD from web_viewer), it is
passed as-is to cmd_vel_raw and the state is MANUAL, even when the behaviour
is paused. When it stops arriving for `teleop_timeout_s`, automatic
behaviour resumes where it was.

Parameters that can be changed at runtime (for example from web_viewer):
  enabled        true / false
  target_colors  list of colours, e.g. ["yellow"]
"""

import json
from dataclasses import dataclass, fields
from typing import Any

from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, String, UInt32

from mobile_base.approach import ApproachController, ApproachParams, Cube
from mobile_base.collector import (
    CollectorFsm,
    CollectorParams,
    Inputs,
    Output,
    Zone,
)
from mobile_base.states import State
from mobile_base.topics import (
    DEFAULT_QOS_DEPTH,
    TOPIC_BEHAVIOR_STATE,
    TOPIC_CMD_VEL_RAW,
    TOPIC_CMD_VEL_TELEOP,
    TOPIC_COUNT,
    TOPIC_CUBES,
    TOPIC_LINE,
    TOPIC_SCOOP_COLOR,
    TOPIC_SCOOP_HAS_CUBE,
    TOPIC_SCOOP_LOWER,
    TOPIC_ZONE_DELIVERED,
    TOPIC_ZONE_GRANTED,
    TOPIC_ZONE_SEEN,
    TOPIC_ZONE_WANT,
)
from recolector_common.node_utils import now_s, spin_node
from recolector_common.params import declare_params, read_params
from recolector_interfaces.msg import CubeDetectionArray, LineSensors

NODE_NAME = "behavior_fsm"

# --- Node parameter defaults (real values: config/collector.yaml) ---------
DEFAULT_CONTROL_RATE_HZ = 20.0
# Without detections newer than this, stay still.
DEFAULT_CUBES_TIMEOUT_S = 1.0
# Without WASD commands newer than this, automatic mode resumes.
DEFAULT_TELEOP_TIMEOUT_S = 0.3
# Line, scoop and zone-turn readings older than this are unknown.
DEFAULT_SENSOR_TIMEOUT_S = 0.5

# Decimals kept when the state report is rounded for the web viewer.
REPORT_DECIMALS = 3
# Minimum time between two "no line sensor" warnings.
LINE_WARN_THROTTLE_S = 10.0

# Names of the numeric parameters that must not be negative.
NUMERIC_PARAM_NAMES = tuple(
    f.name
    for params in (ApproachParams, CollectorParams)
    for f in fields(params)
    if f.type in (float, int, "float", "int")
)


@dataclass
class BehaviorNodeParams:
    """
    Node-level parameters (the controllers' own live in ApproachParams and
    CollectorParams).

    Field names double as ROS parameter names (see recolector_common.params).
    """

    enabled: bool = True
    control_rate_hz: float = DEFAULT_CONTROL_RATE_HZ
    cubes_timeout_s: float = DEFAULT_CUBES_TIMEOUT_S
    teleop_timeout_s: float = DEFAULT_TELEOP_TIMEOUT_S
    sensor_timeout_s: float = DEFAULT_SENSOR_TIMEOUT_S


class Latest:
    """
    The latest value received on a topic, with its arrival time.
    """

    def __init__(self):
        """
        Start with nothing received.
        """
        self.value: Any = None
        self.stamp_s: float | None = None

    def set(self, value: Any, now: float) -> None:
        """
        Store `value`, received at `now`.
        """
        self.value = value
        self.stamp_s = now

    def get(self, now: float, timeout_s: float) -> Any:
        """
        The value if it arrived less than `timeout_s` ago, else None.
        """
        if self.stamp_s is None or now - self.stamp_s > timeout_s:
            return None
        return self.value


class BehaviorFsmNode(Node):
    """
    Runs the collector state machine and arbitrates manual vs automatic
    control.
    """

    def __init__(self):
        """
        Declare the parameters and create the state machine and its topics.
        """
        super().__init__(NODE_NAME)
        declare_params(self, BehaviorNodeParams())
        declare_params(self, ApproachParams())
        declare_params(self, CollectorParams())
        self._node_params = read_params(self, BehaviorNodeParams())
        self._fsm = CollectorFsm(
            ApproachController(read_params(self, ApproachParams())),
            read_params(self, CollectorParams()),
        )
        self._fsm.approach.enabled = self._node_params.enabled
        self.add_on_set_parameters_callback(self._validate_params)
        self.add_post_set_parameters_callback(self._on_params_changed)

        self._cubes = Latest()
        self._zone = Latest()
        self._line = Latest()
        self._has_cube = Latest()
        self._scoop_color = Latest()
        self._granted = Latest()
        self._teleop = Latest()
        self._last_state: State | None = None
        self._last_out: Output | None = None
        self._last_count: int | None = None

        self._create_publishers()
        self._create_subscriptions()
        self.create_timer(1.0 / self._node_params.control_rate_hz, self._tick)
        self.get_logger().info(
            f"target colours: {self._fsm.approach.params.target_colors}"
        )

    def _create_publishers(self):
        """
        Create every publisher of the node.
        """

        def pub(msg_type, topic):
            """
            A publisher with the default QoS depth.
            """
            return self.create_publisher(msg_type, topic, DEFAULT_QOS_DEPTH)

        self._pub_cmd = pub(Twist, TOPIC_CMD_VEL_RAW)
        self._pub_state = pub(String, TOPIC_BEHAVIOR_STATE)
        self._pub_scoop_lower = pub(Bool, TOPIC_SCOOP_LOWER)
        self._pub_count = pub(UInt32, TOPIC_COUNT)
        self._pub_zone_want = pub(Bool, TOPIC_ZONE_WANT)
        self._pub_zone_delivered = pub(String, TOPIC_ZONE_DELIVERED)

    def _create_subscriptions(self):
        """
        Create every subscription; each one only stores the latest value.
        """

        def store(latest: Latest, convert):
            """
            A callback that stores `convert(msg)` in `latest`.
            """
            return lambda msg: latest.set(convert(msg), now_s(self))

        subscriptions = (
            (CubeDetectionArray, TOPIC_CUBES, self._cubes, _to_cubes),
            (CubeDetectionArray, TOPIC_ZONE_SEEN, self._zone, _to_zone),
            (Bool, TOPIC_SCOOP_HAS_CUBE, self._has_cube, _bool_data),
            (String, TOPIC_SCOOP_COLOR, self._scoop_color, _str_data),
            (Bool, TOPIC_ZONE_GRANTED, self._granted, _bool_data),
            (Twist, TOPIC_CMD_VEL_TELEOP, self._teleop, _same),
        )
        for msg_type, topic, latest, convert in subscriptions:
            self.create_subscription(
                msg_type,
                topic,
                store(latest, convert),
                DEFAULT_QOS_DEPTH,
            )
        self.create_subscription(
            LineSensors,
            TOPIC_LINE,
            store(self._line, lambda msg: bool(msg.black)),
            qos_profile_sensor_data,
        )

    # ---------- parameters ----------
    def _validate_params(self, params):
        """
        Reject negative values for the numeric controller parameters.
        """
        for p in params:
            if p.name in NUMERIC_PARAM_NAMES and p.value < 0:
                return SetParametersResult(
                    successful=False,
                    reason=f"{p.name} cannot be < 0",
                )
        return SetParametersResult(successful=True)

    def _on_params_changed(self, _params):
        """
        Reload the parameters into the state machine after a runtime change.
        """
        self._node_params = read_params(self, BehaviorNodeParams())
        self._fsm.approach.params = read_params(self, ApproachParams())
        self._fsm.params = read_params(self, CollectorParams())
        self._fsm.approach.enabled = self._node_params.enabled
        self.get_logger().info(
            f"enabled={self._fsm.approach.enabled} "
            f"target colours={self._fsm.approach.params.target_colors}"
        )

    # ---------- control loop ----------
    def _tick(self):
        """
        One control step: decide the source of the command, publish it and
        report the state.
        """
        now = now_s(self)
        manual = self._teleop.get(now, self._node_params.teleop_timeout_s)
        inputs = self._inputs(now)
        twist = Twist()
        if manual is not None:
            twist.linear.x = manual.linear.x
            twist.linear.y = manual.linear.y
            twist.angular.z = manual.angular.z
        else:
            out = self._fsm.step(now, inputs)
            twist.linear.x = out.cmd.vx
            twist.angular.z = out.cmd.wz
            self._publish_outputs(out)
        self._pub_cmd.publish(twist)
        state = self._current_state(manual is not None, inputs.vision_ok)
        self._publish_state(state, twist)

    def _inputs(self, now: float) -> Inputs:
        """
        Everything the state machine needs, with stale readings as None.
        """
        timeout_s = self._node_params.sensor_timeout_s
        cubes = self._cubes.get(now, self._node_params.cubes_timeout_s)
        line_black = self._line.get(now, timeout_s)
        if line_black is None:
            self.get_logger().warning(
                "no line sensor reading: the border is not seen",
                throttle_duration_sec=LINE_WARN_THROTTLE_S,
            )
        return Inputs(
            cubes=cubes or [],
            zone=self._zone.get(now, self._node_params.cubes_timeout_s),
            vision_ok=cubes is not None,
            line_black=bool(line_black),
            zone_granted=bool(self._granted.get(now, timeout_s)),
            has_cube=self._has_cube.get(now, timeout_s),
            scoop_color=self._scoop_color.get(now, timeout_s),
        )

    def _current_state(self, manual: bool, vision_ok: bool) -> State:
        """
        State to report: manual and missing-camera override the machine.
        """
        if manual:
            return State.MANUAL
        if not vision_ok and self._fsm.needs_vision:
            return State.NO_CAMERA
        return self._fsm.state

    # ---------- publishing ----------
    def _publish_outputs(self, out: Output):
        """
        Publish the scoop, zone and count outputs when they change.
        """
        last = self._last_out
        if last is None or out.scoop_lower != last.scoop_lower:
            self._pub_scoop_lower.publish(Bool(data=out.scoop_lower))
        if last is None or out.zone_want != last.zone_want:
            self._pub_zone_want.publish(Bool(data=out.zone_want))
        if out.delivered_color is not None:
            self._pub_zone_delivered.publish(String(data=out.delivered_color))
            self.get_logger().info(
                f"delivered {out.delivered_color}: count {self._fsm.count}"
            )
        if self._fsm.count != self._last_count:
            self._pub_count.publish(UInt32(data=self._fsm.count))
            self._last_count = self._fsm.count
        self._last_out = out

    def _publish_state(self, state: State, twist: Twist):
        """
        Publish the JSON state report and log state changes.
        """
        report = self._state_report(state, twist)
        self._pub_state.publish(String(data=json.dumps(report)))
        if state != self._last_state:
            self.get_logger().info(f"state: {state.value}")
            self._last_state = state

    def _state_report(self, state: State, twist: Twist) -> dict:
        """
        Dictionary published on behavior/state (read by web_viewer and
        fleet_link).
        """
        approach = self._fsm.approach
        target = approach.target
        target_info = None
        if target is not None:
            target_info = {
                "color": target.color,
                "distance_m": round(target.distance_m, REPORT_DECIMALS),
                "bearing_rad": round(target.bearing_rad, REPORT_DECIMALS),
            }
        return {
            "state": state.value,
            "enabled": approach.enabled,
            "target_colors": approach.params.target_colors,
            "stop_distance_m": approach.params.stop_distance_m,
            "target": target_info,
            "carrying": self._fsm.carrying_color,
            "count": self._fsm.count,
            "cmd": {
                "vx": round(twist.linear.x, REPORT_DECIMALS),
                "vy": round(twist.linear.y, REPORT_DECIMALS),
                "wz": round(twist.angular.z, REPORT_DECIMALS),
            },
        }


def _to_cubes(msg: CubeDetectionArray) -> list[Cube]:
    """
    The detections of a cubes message.
    """
    return [Cube(c.color, c.bearing_rad, c.distance_m) for c in msg.cubes]


def _to_zone(msg: CubeDetectionArray) -> Zone | None:
    """
    The delivery area of a zone/seen message (empty = not in view).
    """
    if not msg.cubes:
        return None
    z = msg.cubes[0]
    return Zone(z.color, z.bearing_rad, z.distance_m, z.v)


def _bool_data(msg: Bool) -> bool:
    """
    The value of a Bool message.
    """
    return bool(msg.data)


def _str_data(msg: String) -> str:
    """
    The value of a String message.
    """
    return msg.data


def _same(msg):
    """
    The message itself.
    """
    return msg


def main(args=None):
    """
    Entry point of the behavior_fsm node.
    """
    spin_node(BehaviorFsmNode, args)


if __name__ == "__main__":
    main()
