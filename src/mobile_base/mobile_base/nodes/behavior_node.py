"""Behavior node: runs the BehaviorFSM and publishes the desired command on cmd_vel_nav."""

from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String

from mobile_base.core.behavior import BehaviorFSM, BehaviorInput, BehaviorParams, Cube
from mobile_base.core.colors import DEFAULT_ROBOT_COLORS
from mobile_base.nodes.common import ZONE_OCCUPANT_QOS, declare_dataclass, now_sec, pose_from_msg, spin_node, twist
from multirobot_interfaces.msg import CubeDetectionArray, RobotPose, RobotState


class BehaviorNode(Node):
    def __init__(self):
        super().__init__("behavior")
        self.robot = self.declare_parameter("robot", "phineas").value
        default_colors = DEFAULT_ROBOT_COLORS.get(self.robot, [])
        my_colors = self.declare_parameter("my_colors", default_colors).value
        rate_hz = self.declare_parameter("rate_hz", 10.0).value
        cube_timeout = self.declare_parameter("cube_timeout", 0.5).value
        peer_topic = self.declare_parameter("peer_pose_topic", "/ferb/pose").value
        values = declare_dataclass(self, BehaviorParams, skip=("my_colors",))
        self.fsm = BehaviorFSM(BehaviorParams(my_colors=tuple(my_colors), **values))
        self._cube_timeout = cube_timeout

        self._cubes: list[Cube] = []
        self._cubes_time = 0.0
        self._line_front = False
        self._pose = None
        self._peer_pose = None  # kept for the CARRY implementation (TODO team)
        self._zone_occupant = ""

        self.create_subscription(CubeDetectionArray, "cubes", self._on_cubes, 10)
        self.create_subscription(Bool, "line_front", self._on_line, 10)
        self.create_subscription(RobotPose, "pose", self._on_pose, 10)
        self.create_subscription(RobotPose, peer_topic, self._on_peer_pose, 10)
        self.create_subscription(String, "/zone/occupant", self._on_zone, ZONE_OCCUPANT_QOS)
        self._cmd_pub = self.create_publisher(Twist, "cmd_vel_nav", 10)
        self._state_pub = self.create_publisher(RobotState, "state", 10)
        self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(f"{self.robot}: my colors {list(my_colors)}")

    def _on_cubes(self, msg: CubeDetectionArray):
        self._cubes = [Cube(c.color, c.cx_norm, c.cy_norm, c.area_px) for c in msg.cubes]
        self._cubes_time = now_sec(self)

    def _on_line(self, msg: Bool):
        self._line_front = msg.data

    def _on_pose(self, msg: RobotPose):
        self._pose = pose_from_msg(msg)

    def _on_peer_pose(self, msg: RobotPose):
        self._peer_pose = pose_from_msg(msg)

    def _on_zone(self, msg: String):
        self._zone_occupant = msg.data

    def _tick(self):
        now = now_sec(self)
        cubes = self._cubes if now - self._cubes_time <= self._cube_timeout else []
        pose = (self._pose.x, self._pose.y, self._pose.theta) if self._pose else None
        previous = self.fsm.state
        vx, vy, wz = self.fsm.step(BehaviorInput(now=now, cubes=cubes, line_front=self._line_front, pose=pose))
        if self.fsm.state is not previous:
            self.get_logger().info(f"{previous.value} -> {self.fsm.state.value}")
        self._cmd_pub.publish(twist(vx, vy, wz))

        state = RobotState()
        state.stamp = self.get_clock().now().to_msg()
        state.robot = self.robot
        state.state = self.fsm.state.value
        state.carrying = self.fsm.carrying
        state.delivered = 0  # TODO(team): take delivered_total from /zone/release responses
        self._state_pub.publish(state)


def main(args=None):
    spin_node(BehaviorNode, args)


if __name__ == "__main__":
    main()
