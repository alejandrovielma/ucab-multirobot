"""
rclpy helpers shared by the collector nodes: clock access and the
spin/shutdown boilerplate that every node's `main()` used to repeat.
"""

from collections.abc import Callable

import rclpy
from rclpy.node import Node

NANOSECONDS_PER_SECOND = 1e9


def now_s(node: Node) -> float:
    """
    Current ROS time of the node, in seconds.
    """
    return node.get_clock().now().nanoseconds / NANOSECONDS_PER_SECOND


def spin_node(node_factory: Callable[[], Node], args=None) -> None:
    """
    Init rclpy, spin the node until Ctrl+C and shut everything down.
    """
    rclpy.init(args=args)
    node = node_factory()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
