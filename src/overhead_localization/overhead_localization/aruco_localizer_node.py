"""Overhead ArUco localizer (skeleton): publishes /<robot>/pose for every robot marker seen.

Runs on Perry's Pi (or a dedicated Pi) with a camera looking down at the whole table.
Markers: DICT_4X4_50, id 1 = phineas, id 2 = ferb (id 10 marks the delivery square).
TODO(team): open the real overhead camera in _grab_frame() and calibrate `homography`
(see docs/team-todo.md). Until then the node runs but publishes nothing.
"""

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from multirobot_interfaces.msg import RobotPose
from overhead_localization.core.homography import marker_pose

IDENTITY = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]


def make_detector():
    """Return detect(gray) -> (corners, ids) for both the OpenCV >= 4.7 and the legacy aruco API."""
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    if hasattr(aruco, "ArucoDetector"):
        detector = aruco.ArucoDetector(dictionary, aruco.DetectorParameters())
        return lambda gray: detector.detectMarkers(gray)[:2]
    params = aruco.DetectorParameters_create()
    return lambda gray: aruco.detectMarkers(gray, dictionary, parameters=params)[:2]


class ArucoLocalizerNode(Node):
    def __init__(self):
        super().__init__("aruco_localizer")
        p = self.declare_parameter
        rate_hz = p("rate_hz", 10.0).value
        self.homography = np.array(p("homography", IDENTITY).value, dtype=float).reshape(3, 3)
        ids = p("marker_ids", [1, 2]).value
        robots = p("marker_robots", ["phineas", "ferb"]).value
        self.robot_by_id = dict(zip(ids, robots))
        self._detect = make_detector()
        self._pubs = {r: self.create_publisher(RobotPose, f"/{r}/pose", 10) for r in robots}
        self.create_timer(1.0 / rate_hz, self._tick)

    def _grab_frame(self):
        """TODO(team): return the latest BGR overhead frame, or None."""
        return None

    def _tick(self):
        frame = self._grab_frame()
        if frame is None:
            return
        corners, ids = self._detect(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        if ids is None:
            return
        stamp = self.get_clock().now().to_msg()
        for marker_corners, marker_id in zip(corners, ids.flatten()):
            robot = self.robot_by_id.get(int(marker_id))
            if robot is None:
                continue
            x, y, theta = marker_pose(self.homography, [tuple(c) for c in marker_corners.reshape(4, 2)])
            msg = RobotPose(robot=robot, x=x, y=y, theta=theta, confidence=1.0)
            msg.stamp = stamp
            self._pubs[robot].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoLocalizerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
