"""Onboard camera publisher for the real robot (placeholder).

TODO(team): the IMX219 on the Pi 5 CSI port is exposed through libcamera, and plain
OpenCV VideoCapture(0) may not work on Ubuntu 24.04. Verify one of: a libcamera-enabled
GStreamer pipeline (pass it as the `device` parameter), picamera2, or camera_ros.
Acceptance: 640x480 bgr8 at >= 10 Hz on camera/image_raw with a correct header stamp.
"""

import cv2
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from mobile_base.nodes.common import spin_node


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera")
        p = self.declare_parameter
        device = p("device", "0").value  # V4L2 index ("0") or a GStreamer pipeline string
        width = p("width", 640).value
        height = p("height", 480).value
        fps = p("fps", 15.0).value
        self.frame_id = p("frame_id", "camera").value
        self.cap = cv2.VideoCapture(int(device) if device.isdigit() else device)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if not self.cap.isOpened():
            self.get_logger().error(f"cannot open camera {device!r}")
        self._pub = self.create_publisher(Image, "camera/image_raw", qos_profile_sensor_data)
        self.create_timer(1.0 / fps, self._tick)

    def _tick(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warning("camera read failed", throttle_duration_sec=5.0)
            return
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height, msg.width = frame.shape[:2]
        msg.encoding = "bgr8"
        msg.step = msg.width * 3
        msg.data = frame.tobytes()
        self._pub.publish(msg)

    def destroy_node(self):
        self.cap.release()
        return super().destroy_node()


def main(args=None):
    spin_node(CameraNode, args)


if __name__ == "__main__":
    main()
