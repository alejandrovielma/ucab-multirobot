"""Zone arbiter: exclusive lease on the delivery square plus the official cube count.

Services /zone/request and /zone/release; /zone/occupant (std_msgs/String, empty = free)
is published at heartbeat_hz with transient-local durability. The robots' safety gates
stop when this heartbeat is lost, so this node must run whenever the robots move.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from coordinator.core.zone_lease import ZoneLease
from multirobot_interfaces.srv import ReleaseZone, RequestZone

OCCUPANT_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class ZoneArbiterNode(Node):
    def __init__(self):
        super().__init__("zone_arbiter")
        lease_seconds = self.declare_parameter("lease_seconds", 60.0).value
        heartbeat_hz = self.declare_parameter("heartbeat_hz", 2.0).value
        self.lease = ZoneLease(lease_seconds)
        self._pub = self.create_publisher(String, "/zone/occupant", OCCUPANT_QOS)
        self.create_service(RequestZone, "/zone/request", self._on_request)
        self.create_service(ReleaseZone, "/zone/release", self._on_release)
        self.create_timer(1.0 / heartbeat_hz, self._publish_occupant)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_request(self, request, response):
        granted, holder = self.lease.request(request.robot, self._now())
        response.granted = granted
        response.holder = holder or ""
        response.lease_seconds = float(self.lease.lease_seconds)
        self.get_logger().info(f"request from {request.robot}: granted={granted} holder={holder}")
        return response

    def _on_release(self, request, response):
        response.ok = self.lease.release(request.robot, request.delivered, request.color, self._now())
        response.delivered_total = self.lease.delivered_total(request.robot)
        self.get_logger().info(
            f"release from {request.robot}: ok={response.ok} delivered={request.delivered} "
            f"color={request.color} counts={self.lease.counts}"
        )
        return response

    def _publish_occupant(self):
        self._pub.publish(String(data=self.lease.occupant(self._now()) or ""))


def main(args=None):
    rclpy.init(args=args)
    node = ZoneArbiterNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
