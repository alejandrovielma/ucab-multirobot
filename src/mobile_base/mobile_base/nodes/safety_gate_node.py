"""Safety gate node: filters cmd_vel_nav into cmd_vel at a fixed rate (see core/safety.py)."""

from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String

from mobile_base.core.safety import SafetyGate, SafetyInput, SafetyParams
from mobile_base.nodes.common import ZONE_OCCUPANT_QOS, declare_dataclass, declare_track, now_sec, pose_from_msg, spin_node, twist
from multirobot_interfaces.msg import RobotPose, RobotState


class SafetyGateNode(Node):
    def __init__(self):
        super().__init__("safety_gate")
        rate_hz = self.declare_parameter("rate_hz", 20.0).value
        self._cmd_timeout = self.declare_parameter("cmd_timeout", 0.5).value
        peer_topic = self.declare_parameter("peer_pose_topic", "/ferb/pose").value
        self._robot = self.declare_parameter("robot", self.get_namespace().strip("/")).value
        self.gate = SafetyGate(SafetyParams(**declare_dataclass(self, SafetyParams)), track=declare_track(self))

        self._cmd = (0.0, 0.0, 0.0)
        self._cmd_time: float | None = None
        self._line_front = False
        self._pose = None
        self._peer_pose = None
        self._heartbeat_time: float | None = None
        self._carrying = False
        self._gate_closed = False
        self._zone_occupant = ""
        self._last_reason = ""

        self.create_subscription(Twist, "cmd_vel_nav", self._on_cmd, 10)
        self.create_subscription(Bool, "line_front", self._on_line, 10)
        self.create_subscription(RobotPose, "pose", self._on_pose, 10)
        self.create_subscription(RobotPose, peer_topic, self._on_peer_pose, 10)
        self.create_subscription(RobotState, "state", self._on_state, 10)
        self.create_subscription(String, "/zone/occupant", self._on_zone, ZONE_OCCUPANT_QOS)
        self._pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_timer(1.0 / rate_hz, self._tick)

    def _on_cmd(self, msg: Twist):
        self._cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        self._cmd_time = now_sec(self)

    def _on_line(self, msg: Bool):
        self._line_front = msg.data

    def _on_pose(self, msg: RobotPose):
        self._pose = pose_from_msg(msg)

    def _on_peer_pose(self, msg: RobotPose):
        self._peer_pose = pose_from_msg(msg)

    def _on_state(self, msg: RobotState):
        self._carrying = msg.carrying
        self._gate_closed = msg.gate_closed

    def _on_zone(self, msg: String):
        self._heartbeat_time = now_sec(self)
        self._zone_occupant = msg.data

    def _tick(self):
        now = now_sec(self)
        fresh_cmd = self._cmd_time is not None and now - self._cmd_time <= self._cmd_timeout
        heartbeat_age = None if self._heartbeat_time is None else now - self._heartbeat_time
        cmd, reason = self.gate.filter(
            SafetyInput(
                cmd=self._cmd if fresh_cmd else (0.0, 0.0, 0.0),
                line_front=self._line_front,
                own_pose=self._pose,
                peer_pose=self._peer_pose,
                now=now,
                heartbeat_age=heartbeat_age,
                carrying=self._carrying,
                gate_closed=self._gate_closed,
                holds_zone=self._zone_occupant == self._robot,
            )
        )
        if reason != self._last_reason:
            self.get_logger().info(f"safety: {reason}")
            self._last_reason = reason
        self._pub.publish(twist(*cmd))


def main(args=None):
    spin_node(SafetyGateNode, args)


if __name__ == "__main__":
    main()
