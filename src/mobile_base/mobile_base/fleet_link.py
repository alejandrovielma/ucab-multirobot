"""
fleet_link node: bridges Ferb's internal topics with Perry's zone arbiter and
the shared multi-robot state topic.
"""

import json

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from mobile_base.fleet import (
    FleetLinkParams,
    Heartbeat,
    ZoneClient,
    to_fleet_state,
)
from mobile_base.states import State
from mobile_base.topics import (
    DEFAULT_QOS_DEPTH,
    OCCUPANT_QOS_DEPTH,
    SERVICE_ZONE_RELEASE,
    SERVICE_ZONE_REQUEST,
    TOPIC_BEHAVIOR_STATE,
    TOPIC_FLEET_OK,
    TOPIC_ROBOT_STATE,
    TOPIC_SCOOP_HAS_CUBE,
    TOPIC_ZONE_DELIVERED,
    TOPIC_ZONE_GRANTED,
    TOPIC_ZONE_OCCUPANT,
    TOPIC_ZONE_WANT,
)
from multirobot_interfaces.msg import RobotState
from multirobot_interfaces.srv import ReleaseZone, RequestZone
from recolector_common.node_utils import now_s, spin_node
from recolector_common.params import declare_params, read_params

NODE_NAME = "fleet_link"

# Period of the heartbeat / zone-lease housekeeping loop.
FLEET_TICK_PERIOD_S = 0.2
# Minimum time between two "service not available" warnings.
SERVICE_WARN_THROTTLE_S = 5.0

OCCUPANT_QOS = QoSProfile(
    depth=OCCUPANT_QOS_DEPTH,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class FleetLinkNode(Node):
    """
    Publishes our state to the fleet and talks to the zone arbiter.
    """

    def __init__(self):
        """
        Declare the parameters and create the fleet interfaces.
        """
        super().__init__(NODE_NAME)
        declare_params(self, FleetLinkParams())
        params = read_params(self, FleetLinkParams())

        self._hb = Heartbeat(params.heartbeat_timeout_s)
        self._zone = ZoneClient(params.robot_name, params.renew_period_s)
        self._local_state = State.PAUSED.value
        self._carrying = False
        self._delivered_total = 0
        self._req_future = None
        self._rel_future = None

        self._create_interfaces()
        self.create_timer(1.0 / params.state_rate_hz, self._publish_state)
        self.create_timer(FLEET_TICK_PERIOD_S, self._tick_fleet)

    def _create_interfaces(self):
        """
        Create the publishers, subscriptions and service clients.
        """
        self._pub_state = self.create_publisher(
            RobotState,
            TOPIC_ROBOT_STATE,
            DEFAULT_QOS_DEPTH,
        )
        self._pub_ok = self.create_publisher(
            Bool,
            TOPIC_FLEET_OK,
            DEFAULT_QOS_DEPTH,
        )
        self._pub_granted = self.create_publisher(
            Bool,
            TOPIC_ZONE_GRANTED,
            DEFAULT_QOS_DEPTH,
        )

        self.create_subscription(
            String,
            TOPIC_BEHAVIOR_STATE,
            self._on_behavior_state,
            DEFAULT_QOS_DEPTH,
        )
        self.create_subscription(
            Bool,
            TOPIC_SCOOP_HAS_CUBE,
            self._on_has_cube,
            DEFAULT_QOS_DEPTH,
        )
        self.create_subscription(
            Bool,
            TOPIC_ZONE_WANT,
            self._on_want,
            DEFAULT_QOS_DEPTH,
        )
        self.create_subscription(
            String,
            TOPIC_ZONE_DELIVERED,
            self._on_delivered,
            DEFAULT_QOS_DEPTH,
        )
        self.create_subscription(
            String,
            TOPIC_ZONE_OCCUPANT,
            self._on_occupant,
            OCCUPANT_QOS,
        )

        self._req_client = self.create_client(
            RequestZone,
            SERVICE_ZONE_REQUEST,
        )
        self._rel_client = self.create_client(
            ReleaseZone,
            SERVICE_ZONE_RELEASE,
        )

    def _param(self, name: str):
        """
        Current value of one of this node's parameters.
        """
        return self.get_parameter(name).value

    # ---------- subscriptions ----------
    def _on_behavior_state(self, msg: String):
        """
        Remember the local state, sent either as JSON or as a bare name.
        """
        raw = msg.data.strip()
        if raw.startswith("{"):
            try:
                state = json.loads(raw).get("state", State.PAUSED.value)
                self._local_state = str(state)
                return
            except ValueError:
                pass
        self._local_state = raw

    def _on_has_cube(self, msg: Bool):
        """
        Remember whether the scoop holds a cube.
        """
        self._carrying = bool(msg.data)

    def _on_want(self, msg: Bool):
        """
        The behaviour wants (or no longer wants) the delivery zone.
        """
        self._zone.update(want=bool(msg.data))
        self._check_zone_services()

    def _on_delivered(self, msg: String):
        """
        A cube of the given colour was dropped in the zone.
        """
        if msg.data:
            self._zone.mark_delivered(msg.data)

    def _on_occupant(self, msg: String):
        """
        The arbiter announced the zone occupant; this is also its heartbeat.
        """
        self._hb.beat(now_s(self))
        self._zone.update(occupant=msg.data)
        self._publish_granted()

    # ---------- periodic publishing ----------
    def _publish_state(self):
        """
        Publish our shared multi-robot state.
        """
        msg = RobotState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.robot = str(self._param("robot_name"))
        msg.state = to_fleet_state(self._local_state)
        msg.carrying = self._carrying
        msg.delivered = self._delivered_total
        msg.gate_closed = False
        self._pub_state.publish(msg)

    def _publish_granted(self):
        """
        Publish whether we currently hold the delivery zone.
        """
        self._pub_granted.publish(Bool(data=self._zone.holds))

    def _tick_fleet(self):
        """
        Refresh live parameters, publish heartbeat status and drive the zone
        protocol.
        """
        self._hb.timeout_s = float(self._param("heartbeat_timeout_s"))
        self._zone.robot = str(self._param("robot_name"))
        self._zone.renew_period_s = float(self._param("renew_period_s"))
        self._pub_ok.publish(Bool(data=self._hb.ok(now_s(self))))
        self._publish_granted()
        self._check_zone_services()

    # ---------- zone protocol ----------
    def _check_zone_services(self):
        """
        Send a release or a request to the arbiter when one is due and no
        call of that kind is already in flight.
        """
        if self._zone.should_release() and self._rel_future is None:
            self._send_release()
        elif (
            self._zone.should_request(now_s(self)) and self._req_future is None
        ):
            self._send_request()

    def _send_release(self):
        """
        Call /zone/release with the delivered cube metadata.
        """
        if not self._rel_client.service_is_ready():
            self._warn_unavailable(SERVICE_ZONE_RELEASE)
            return
        delivered, color = self._zone.pop_release()
        req = ReleaseZone.Request()
        req.robot = self._zone.robot
        req.delivered = delivered
        req.color = color
        self._rel_future = self._rel_client.call_async(req)
        self._rel_future.add_done_callback(self._on_release_done)

    def _send_request(self):
        """
        Call /zone/request to get (or renew) the delivery zone.
        """
        if not self._req_client.service_is_ready():
            self._warn_unavailable(SERVICE_ZONE_REQUEST)
            return
        self._zone.note_requested(now_s(self))
        req = RequestZone.Request()
        req.robot = self._zone.robot
        self._req_future = self._req_client.call_async(req)
        self._req_future.add_done_callback(self._on_request_done)

    def _warn_unavailable(self, service: str):
        """
        Log (throttled) that a zone service is not up.
        """
        self.get_logger().warning(
            f"{service} service not available",
            throttle_duration_sec=SERVICE_WARN_THROTTLE_S,
        )

    def _on_request_done(self, future):
        """
        Handle the arbiter's answer to a zone request.
        """
        self._req_future = None
        try:
            res = future.result()
        except Exception as exc:
            self._warn_call_failed(SERVICE_ZONE_REQUEST, exc)
            return
        if res.granted:
            self._zone.update(occupant=self._zone.robot)
        elif res.holder:
            self._zone.update(occupant=res.holder)
        self._publish_granted()

    def _on_release_done(self, future):
        """
        Handle the arbiter's answer to a zone release.
        """
        self._rel_future = None
        try:
            res = future.result()
        except Exception as exc:
            self._warn_call_failed(SERVICE_ZONE_RELEASE, exc)
            return
        self._delivered_total = int(res.delivered_total)
        if res.ok and self._zone.holds:
            self._zone.update(occupant="")
        self._publish_granted()

    def _warn_call_failed(self, service: str, exc: Exception):
        """
        Log (throttled) that a zone service call raised.
        """
        self.get_logger().warning(
            f"{service} call failed: {exc}",
            throttle_duration_sec=SERVICE_WARN_THROTTLE_S,
        )


def main(args=None):
    """
    Entry point of the fleet_link node.
    """
    spin_node(FleetLinkNode, args)


if __name__ == "__main__":
    main()
