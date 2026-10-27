"""Behavior node: runs the BehaviorFSM and publishes the desired command on cmd_vel_nav.

It also performs the FSM's zone actions as asynchronous calls to /zone/request and
/zone/release; the lease itself is read from /zone/occupant (holds_zone = occupant is me).

The mission comes from two string parameters, comma separated, that can change at runtime (the
command interface, or `ros2 param set /phineas/behavior my_colors "blue,green"`): my_colors (targets;
empty = the robot stays still) and avoid_colors (forbidden). Unknown color names are rejected.
"""

from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import Bool, String

from mobile_base.core.behavior import BehaviorFSM, BehaviorInput, BehaviorParams, Cube, ZoneAction
from mobile_base.core.colors import parse_colors
from mobile_base.nodes.common import ZONE_OCCUPANT_QOS, declare_dataclass, declare_track, now_sec, pose_from_msg, spin_node, twist
from multirobot_interfaces.msg import CubeDetectionArray, RobotPose, RobotState
from multirobot_interfaces.srv import ReleaseZone, RequestZone


class BehaviorNode(Node):
    def __init__(self):
        super().__init__("behavior")
        self.robot = self.declare_parameter("robot", "phineas").value
        targets = parse_colors(self.declare_parameter("my_colors", "").value)
        avoid = parse_colors(self.declare_parameter("avoid_colors", "").value)
        rate_hz = self.declare_parameter("rate_hz", 10.0).value
        cube_timeout = self.declare_parameter("cube_timeout", 0.5).value
        self.declare_parameter("peer_pose_topic", "/ferb/pose")  # still in the YAML; avoidance lives in the gate
        values = declare_dataclass(self, BehaviorParams, skip=("my_colors",))
        self.fsm = BehaviorFSM(BehaviorParams(my_colors=targets, **values), track=declare_track(self))
        self.fsm.set_mission(targets, avoid)
        self.add_on_set_parameters_callback(self._on_mission)
        self._cube_timeout = cube_timeout

        self._cubes: list[Cube] = []
        self._cubes_time = 0.0
        self._scoop_color = ""
        self._square = (False, 0.0, 0.0, 0.0)  # visible, cx, cy, area (blue delivery square)
        self._line_front = False
        self._pose = None
        self._zone_occupant = ""
        self._delivered = 0

        self.create_subscription(CubeDetectionArray, "cubes", self._on_cubes, 10)
        self.create_subscription(Bool, "line_front", self._on_line, 10)
        self.create_subscription(RobotPose, "pose", self._on_pose, 10)
        self.create_subscription(String, "/zone/occupant", self._on_zone, ZONE_OCCUPANT_QOS)
        self._cmd_pub = self.create_publisher(Twist, "cmd_vel_nav", 10)
        self._state_pub = self.create_publisher(RobotState, "state", 10)
        self._gate_pub = self.create_publisher(Bool, "gate_cmd", 10)  # true = closed (scoop gate servo)
        self._request_client = self.create_client(RequestZone, "/zone/request")
        self._release_client = self.create_client(ReleaseZone, "/zone/release")
        self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(f"{self.robot}: targets {list(targets) or 'none (idle)'}, avoid {list(avoid)}")

    def _on_mission(self, params):
        """Live mission change; a bad color name is refused and the current mission stays."""
        targets, avoid = self.fsm.targets, self.fsm.avoid
        try:
            for param in params:
                if param.name == "my_colors":
                    targets = parse_colors(param.value)
                elif param.name == "avoid_colors":
                    avoid = parse_colors(param.value)
        except (ValueError, AttributeError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        if (targets, avoid) != (self.fsm.targets, self.fsm.avoid):
            self.fsm.set_mission(targets, avoid)
            self.get_logger().info(f"mission: targets {list(targets) or 'none (idle)'}, avoid {list(avoid)}")
        return SetParametersResult(successful=True)

    def _on_cubes(self, msg: CubeDetectionArray):
        self._cubes = [Cube(c.color, c.cx_norm, c.cy_norm, c.area_px) for c in msg.cubes]
        self._scoop_color = msg.scoop_color
        self._square = (msg.square_visible, msg.square_cx, msg.square_cy, msg.square_area)
        self._cubes_time = now_sec(self)

    def _on_line(self, msg: Bool):
        self._line_front = msg.data

    def _on_pose(self, msg: RobotPose):
        self._pose = pose_from_msg(msg)

    def _on_zone(self, msg: String):
        self._zone_occupant = msg.data

    def _tick(self):
        now = now_sec(self)
        fresh = now - self._cubes_time <= self._cube_timeout
        cubes = self._cubes if fresh else []
        scoop_color = self._scoop_color if fresh else ""
        square_visible, square_cx, square_cy, square_area = self._square if fresh else (False, 0.0, 0.0, 0.0)
        pose = (self._pose.x, self._pose.y, self._pose.theta) if self._pose else None
        previous = self.fsm.state
        holds_zone = self._zone_occupant == self.robot
        vx, vy, wz = self.fsm.step(
            BehaviorInput(
                now=now,
                cubes=cubes,
                line_front=self._line_front,
                pose=pose,
                holds_zone=holds_zone,
                scoop_color=scoop_color,
                square_visible=square_visible,
                square_cx=square_cx,
                square_cy=square_cy,
                square_area=square_area,
            )
        )
        if self.fsm.state is not previous:
            note = f" ({self.fsm.note})" if self.fsm.note else ""
            self.get_logger().info(f"{previous.value} -> {self.fsm.state.value}{note}")
            self.fsm.note = ""
        for action in self.fsm.take_actions():
            self._perform(action)
        self._cmd_pub.publish(twist(vx, vy, wz))

        state = RobotState()
        state.stamp = self.get_clock().now().to_msg()
        state.robot = self.robot
        state.state = self.fsm.state.value
        state.carrying = self.fsm.carrying
        state.delivered = self._delivered
        state.gate_closed = self.fsm.gate_closed
        self._gate_pub.publish(Bool(data=self.fsm.gate_closed))
        self._state_pub.publish(state)

    def _perform(self, action: ZoneAction):
        # Fire and forget: the lease is observed on /zone/occupant and the FSM retries requests.
        if action.kind == "request":
            if self._request_client.service_is_ready():
                self._request_client.call_async(RequestZone.Request(robot=self.robot))
            else:
                self.get_logger().warning("/zone/request not available")
            return
        if not self._release_client.service_is_ready():
            self.get_logger().error("/zone/release not available: the lease will only expire by timeout")
            return
        request = ReleaseZone.Request(robot=self.robot, delivered=action.delivered, color=action.color)
        self._release_client.call_async(request).add_done_callback(self._on_released)

    def _on_released(self, future):
        response = future.result()
        if response is not None and response.ok:
            self._delivered = response.delivered_total
            self.get_logger().info(f"delivered: total {self._delivered}")


def main(args=None):
    spin_node(BehaviorNode, args)


if __name__ == "__main__":
    main()
