"""
camera_node node: opens the camera and publishes image_raw.

Uses OpenCV (V4L2): works with a USB webcam or with the Pi camera if the
system exposes it as /dev/videoN. Not used in simulation: webots_bridge
publishes image_raw.

TODO (role 2): test with the OV5647 on the Pi 5 (may need libcamera / a
GStreamer pipeline in `device`).
"""

from dataclasses import dataclass

import cv2
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from recolector_common.node_utils import spin_node
from recolector_common.params import declare_params, read_params
from vision.ros_image import bgr_to_image
from vision.topics import TOPIC_IMAGE_RAW

NODE_NAME = "camera_node"

# --- Parameter defaults (real values: config/hsv_ranges.yaml) -----------------
# /dev/videoN number or a GStreamer pipeline.
DEFAULT_DEVICE = "0"
DEFAULT_WIDTH_PX = 640
DEFAULT_HEIGHT_PX = 480
DEFAULT_FPS_HZ = 15.0
DEFAULT_FRAME_ID = "camera"

# Minimum time between two "no image" warnings.
NO_IMAGE_WARN_THROTTLE_S = 5.0


@dataclass
class CameraNodeParams:
    """
    Parameters of the camera node.

    Field names double as ROS parameter names (see recolector_common.params).
    """

    device: str = DEFAULT_DEVICE
    width_px: int = DEFAULT_WIDTH_PX
    height_px: int = DEFAULT_HEIGHT_PX
    fps_hz: float = DEFAULT_FPS_HZ
    frame_id: str = DEFAULT_FRAME_ID


class CameraNode(Node):
    """
    Grabs frames from the camera at a fixed rate and publishes them.
    """

    def __init__(self):
        """
        Declare the parameters, open the camera and start publishing.
        """
        super().__init__(NODE_NAME)
        declare_params(self, CameraNodeParams())
        self._params = read_params(self, CameraNodeParams())
        self._cap = self._open_capture(self._params.device)

        self._pub = self.create_publisher(
            Image,
            TOPIC_IMAGE_RAW,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0 / self._params.fps_hz, self._tick)

    def _open_capture(self, device: str) -> cv2.VideoCapture:
        """
        Open a V4L2 camera by index, or a GStreamer pipeline by string, and
        set the requested resolution.
        """
        if device.isdigit():
            cap = cv2.VideoCapture(int(device), cv2.CAP_V4L2)
        else:
            cap = cv2.VideoCapture(device, cv2.CAP_GSTREAMER)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._params.width_px)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._params.height_px)
        if not cap.isOpened():
            self.get_logger().error(f"could not open camera '{device}'")
        return cap

    def _tick(self):
        """
        Grab one frame and publish it.
        """
        ok, frame = self._cap.read()
        if not ok:
            self.get_logger().warning(
                "no image from the camera",
                throttle_duration_sec=NO_IMAGE_WARN_THROTTLE_S,
            )
            return
        msg = bgr_to_image(frame)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._params.frame_id
        self._pub.publish(msg)

    def destroy_node(self):
        """
        Release the camera before destroying the node.
        """
        self._cap.release()
        super().destroy_node()


def main(args=None):
    """
    Entry point of the camera_node node.
    """
    spin_node(CameraNode, args)


if __name__ == "__main__":
    main()
