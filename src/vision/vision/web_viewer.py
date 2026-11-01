"""
web_viewer node: web page to watch the robot camera and pick the target
colours.

Starts an HTTP server (port 8080 by default); the routes are listed in
web_server.py. Manual control (/api/teleop) publishes cmd_vel_teleop;
behavior_fsm gives it priority and routes it through obstacle_guard.

Works the same on the laptop (simulation) and on the Pi: from another machine
open http://<pi-ip>:8080 or go through an SSH tunnel (see
docs/ssh-deployment.md).
"""

import json
import threading
import time
from dataclasses import dataclass, field
from http.server import ThreadingHTTPServer
from pathlib import Path

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from recolector_common.params import declare_params, read_params
from recolector_interfaces.msg import CubeDetectionArray
from vision.colors import COLOR_NAMES
from vision.frame_buffer import FrameBuffer
from vision.ros_image import image_to_bgr
from vision.teleop import TeleopLimits, teleop_velocity
from vision.topics import (
    DEFAULT_QOS_DEPTH,
    TOPIC_BEHAVIOR_STATE,
    TOPIC_CMD_VEL_TELEOP,
    TOPIC_CUBES,
    TOPIC_CUBES_DEBUG_IMAGE,
    TOPIC_IMAGE_RAW,
)
from vision.web_server import make_handler

NODE_NAME = "web_viewer"

# --- Parameter defaults (real values: config/hsv_ranges.yaml) -----------------
DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 8080
DEFAULT_MAX_FPS_HZ = 15.0
DEFAULT_JPEG_QUALITY = 75
DEFAULT_BEHAVIOR_NODE = "behavior_fsm"
DEFAULT_TELEOP_MAX_LINEAR_M_S = 0.15
DEFAULT_TELEOP_MAX_ANGULAR_RAD_S = 1.0

# Where the page lives inside the installed package (see setup.py).
WEB_DIR_NAME = "web"
INDEX_FILE_NAME = "index.html"
PACKAGE_NAME = "vision"

# behavior_fsm parameters the web page is allowed to change.
ALLOWED_BEHAVIOR_PARAMS = frozenset(
    {"enabled", "target_colors", "stop_distance_m"}
)
SET_PARAMS_TIMEOUT_S = 2.0

# Without a debug image newer than this, fall back to image_raw.
DEBUG_IMAGE_FRESH_S = 1.0
# Exponential smoothing of the displayed fps: weight of the old value.
FPS_OLD_WEIGHT = 0.8
# A gap between frames longer than this resets the fps estimate.
FPS_RESET_GAP_S = 5.0
FPS_DISPLAY_DECIMALS = 1
REPORT_DECIMALS = 3
IMAGE_ERROR_THROTTLE_S = 5.0


def _param_msg(name: str, value) -> ParameterMsg:
    """
    Build a ROS parameter message from a JSON value (bool, number or list).
    """
    param_value = ParameterValue()
    if isinstance(value, bool):
        param_value.type = ParameterType.PARAMETER_BOOL
        param_value.bool_value = value
    elif isinstance(value, float | int):
        param_value.type = ParameterType.PARAMETER_DOUBLE
        param_value.double_value = float(value)
    elif isinstance(value, list):
        param_value.type = ParameterType.PARAMETER_STRING_ARRAY
        param_value.string_array_value = [str(x) for x in value]
    else:
        raise ValueError(f"unsupported type for {name}")
    return ParameterMsg(name=name, value=param_value)


@dataclass
class WebViewerParams:
    """
    Parameters of the web viewer.

    Field names double as ROS parameter names (see recolector_common.params).
    """

    host: str = DEFAULT_HOST
    port: int = DEFAULT_PORT
    max_fps_hz: float = DEFAULT_MAX_FPS_HZ
    jpeg_quality: int = DEFAULT_JPEG_QUALITY
    color_names: list[str] = field(default_factory=lambda: list(COLOR_NAMES))
    behavior_node: str = DEFAULT_BEHAVIOR_NODE
    teleop_max_linear_m_s: float = DEFAULT_TELEOP_MAX_LINEAR_M_S
    teleop_max_angular_rad_s: float = DEFAULT_TELEOP_MAX_ANGULAR_RAD_S


class WebViewerNode(Node):
    """
    Collects images, detections and behaviour state for the web page and
    forwards its commands to the robot.
    """

    def __init__(self):
        """
        Declare the parameters and create the ROS interfaces.
        """
        super().__init__(NODE_NAME)
        declare_params(self, WebViewerParams())
        self.params = read_params(self, WebViewerParams())
        self.add_post_set_parameters_callback(self._on_params_changed)

        self.frames = FrameBuffer()
        self._lock = threading.Lock()
        self._cubes: list[dict] = []
        self._behavior: dict | None = None
        self._last_debug_s = 0.0
        self._last_frame_s = 0.0
        self._fps = 0.0

        self.create_subscription(
            Image,
            TOPIC_CUBES_DEBUG_IMAGE,
            self._on_debug_image,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            TOPIC_IMAGE_RAW,
            self._on_raw_image,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CubeDetectionArray,
            TOPIC_CUBES,
            self._on_cubes,
            DEFAULT_QOS_DEPTH,
        )
        self.create_subscription(
            String,
            TOPIC_BEHAVIOR_STATE,
            self._on_behavior,
            DEFAULT_QOS_DEPTH,
        )
        behavior = self.params.behavior_node
        self._param_client = self.create_client(
            SetParameters,
            f"{behavior}/set_parameters",
        )
        self._pub_teleop = self.create_publisher(
            Twist,
            TOPIC_CMD_VEL_TELEOP,
            DEFAULT_QOS_DEPTH,
        )

    # ---------- parameters ----------
    def _on_params_changed(self, _params):
        """
        Apply runtime parameter changes (fps cap, JPEG quality, limits...).
        """
        self.params = read_params(self, WebViewerParams())

    # ---------- ROS ----------
    def _encode(self, msg: Image):
        """
        JPEG-encode an image (rate limited) and hand it to the HTTP clients.
        """
        now = time.monotonic()
        if now - self._last_frame_s < 1.0 / self.params.max_fps_hz:
            return
        self._update_fps(now)
        try:
            bgr = image_to_bgr(msg)
        except ValueError as e:
            self.get_logger().error(
                str(e),
                throttle_duration_sec=IMAGE_ERROR_THROTTLE_S,
            )
            return
        quality = int(self.params.jpeg_quality)
        ok, jpeg = cv2.imencode(
            ".jpg",
            bgr,
            [cv2.IMWRITE_JPEG_QUALITY, quality],
        )
        if ok:
            self.frames.put(jpeg.tobytes())

    def _update_fps(self, now: float):
        """
        Update the smoothed fps estimate and the last-frame time.
        """
        gap_s = now - self._last_frame_s
        if gap_s < FPS_RESET_GAP_S:
            instant_fps = 1.0 / gap_s
            self._fps = (
                FPS_OLD_WEIGHT * self._fps
                + (1.0 - FPS_OLD_WEIGHT) * instant_fps
            )
        else:
            self._fps = 0.0
        self._last_frame_s = now

    def _on_debug_image(self, msg: Image):
        """
        Annotated image from the detector: preferred video source.
        """
        self._last_debug_s = time.monotonic()
        self._encode(msg)

    def _on_raw_image(self, msg: Image):
        """
        Raw camera image, only used while no detector publishes the debug
        image.
        """
        if time.monotonic() - self._last_debug_s > DEBUG_IMAGE_FRESH_S:
            self._encode(msg)

    def _on_cubes(self, msg: CubeDetectionArray):
        """
        Store the latest detections in the JSON shape the page expects.
        """
        cubes = [
            {
                "color": c.color,
                "distance_m": round(c.distance_m, REPORT_DECIMALS),
                "bearing_rad": round(c.bearing_rad, REPORT_DECIMALS),
                "u": c.u,
                "v": c.v,
            }
            for c in msg.cubes
        ]
        with self._lock:
            self._cubes = cubes

    def _on_behavior(self, msg: String):
        """
        Store the latest behaviour state report.
        """
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self._lock:
            self._behavior = data

    # ---------- for the HTTP server (ViewerBackend) ----------
    def snapshot(self) -> dict:
        """
        State shown by the page: cubes, behaviour, fps and colour names.
        """
        with self._lock:
            return {
                "cubes": list(self._cubes),
                "behavior": self._behavior,
                "fps": round(self._fps, FPS_DISPLAY_DECIMALS),
                "color_names": list(self.params.color_names),
            }

    def teleop(self, data: dict) -> None:
        """
        Publish a manual-control command from the page's raw JSON.
        """
        limits = TeleopLimits(
            self.params.teleop_max_linear_m_s,
            self.params.teleop_max_angular_rad_s,
        )
        twist = Twist()
        twist.linear.x, twist.linear.y, twist.angular.z = teleop_velocity(
            data,
            limits,
        )
        self._pub_teleop.publish(twist)

    def set_behavior_params(
        self,
        values: dict,
        timeout_s: float = SET_PARAMS_TIMEOUT_S,
    ) -> tuple[bool, str]:
        """
        Change behavior_fsm parameters; returns (ok, message).
        """
        unknown = set(values) - ALLOWED_BEHAVIOR_PARAMS
        if unknown:
            return False, f"parameters not allowed: {sorted(unknown)}"
        if not self._param_client.service_is_ready():
            return False, "behavior_fsm is not running"
        request = SetParameters.Request()
        try:
            request.parameters = [_param_msg(k, v) for k, v in values.items()]
        except ValueError as e:
            return False, str(e)
        done = threading.Event()
        future = self._param_client.call_async(request)
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout_s):
            return False, "behavior_fsm did not respond"
        errors = [r.reason for r in future.result().results if not r.successful]
        return (not errors), "; ".join(errors) or "ok"


def _read_index_html() -> bytes:
    """
    The installed web page.
    """
    share = Path(get_package_share_directory(PACKAGE_NAME))
    return (share / WEB_DIR_NAME / INDEX_FILE_NAME).read_bytes()


def main(args=None):
    """
    Entry point of the web_viewer node: ROS spin plus an HTTP server thread.
    """
    rclpy.init(args=args)
    node = WebViewerNode()
    host = node.params.host
    port = node.params.port
    handler = make_handler(node, _read_index_html(), rclpy.ok)
    server = ThreadingHTTPServer((host, port), handler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    node.get_logger().info(f"web viewer at http://{host}:{port}")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
