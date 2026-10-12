"""Small rclpy helpers shared by the mobile_base nodes."""

import dataclasses

import rclpy
from rclpy.parameter import Parameter
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from mobile_base.core.safety import Pose2D
from mobile_base.core.track import Track

# /zone/occupant is a latched heartbeat: late joiners get the last value immediately.
ZONE_OCCUPANT_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def declare_dataclass(node: Node, cls, skip: tuple[str, ...] = ()) -> dict:
    """Declare one ROS parameter per dataclass field (its default as fallback); return the values."""
    values = {}
    for field in dataclasses.fields(cls):
        if field.name in skip:
            continue
        values[field.name] = node.declare_parameter(field.name, field.default).value
    return values


def declare_track(node: Node) -> Track:
    """track_polygon comes from config/track.yaml; without it there is no geofence."""
    flat = node.declare_parameter("track_polygon", Parameter.Type.DOUBLE_ARRAY).value
    track = Track(flat)
    if not track.enabled:
        node.get_logger().warn("no track_polygon: geofence disabled, only the line sensor keeps the robot in")
    return track


def stamp_to_sec(stamp: TimeMsg) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def now_sec(node: Node) -> float:
    return node.get_clock().now().nanoseconds * 1e-9


def pose_from_msg(msg) -> Pose2D:
    return Pose2D(msg.x, msg.y, msg.theta, stamp=stamp_to_sec(msg.stamp))


def twist(vx: float, vy: float, wz: float) -> Twist:
    msg = Twist()
    msg.linear.x = float(vx)
    msg.linear.y = float(vy)
    msg.angular.z = float(wz)
    return msg


def spin_node(node_cls, args=None) -> None:
    """Standard entry point: init, spin until Ctrl-C or launch shutdown, clean up."""
    rclpy.init(args=args)
    node = node_cls()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
