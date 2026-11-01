"""
cube_detector node: finds cubes by colour (HSV) in image_raw and publishes
cubes, and locates the delivery area and publishes zone/seen.

zone/seen reuses CubeDetectionArray: empty when the area is not in view,
otherwise one entry whose `color` says how it was found ("floor" = the
blue/green paper up close, "gate" = between the green and red landmarks)
and whose bearing, distance, `v` and box describe the area. Cubes already
inside the area are left out of cubes. See vision/zone.py.

It also publishes cubes/debug_image: the same image with one box per cube
and the area, to view it in web_viewer or rqt_image_view.

HSV ranges, camera geometry and the `zone.*` parameters come from
config/hsv_ranges.yaml. They can be changed at runtime with
`ros2 param set /cube_detector ...`.
"""

from dataclasses import dataclass, field

from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from recolector_common.node_utils import spin_node
from recolector_common.params import declare_params, read_params
from recolector_interfaces.msg import CubeDetection, CubeDetectionArray
from vision.camera_model import CameraModel, CameraParams
from vision.color_detection import ColorDetector, DetectionFilter
from vision.colors import COLOR_NAMES
from vision.drawing import draw_detections, draw_region
from vision.hsv import parse_hsv_ranges
from vision.ros_image import bgr_to_image, image_to_bgr
from vision.topics import (
    DEFAULT_QOS_DEPTH,
    TOPIC_CUBES,
    TOPIC_CUBES_DEBUG_IMAGE,
    TOPIC_IMAGE_RAW,
    TOPIC_ZONE_SEEN,
)
from vision.zone import ZoneLocator, ZoneParams, without_cubes_in_zone

NODE_NAME = "cube_detector"

# Prefixes of the grouped parameters (hsv.red, camera.hfov_rad, ...).
HSV_PREFIX = "hsv."
CAMERA_PREFIX = "camera."
FILTER_PREFIX = "filter."
ZONE_PREFIX = "zone."

# Minimum time between two "bad image" errors.
IMAGE_ERROR_THROTTLE_S = 5.0


@dataclass
class CubeDetectorParams:
    """
    Node-level parameters; the HSV ranges (`hsv.<colour>`), `camera.*`,
    `filter.*` and `zone.*` groups are declared separately.

    Field names double as ROS parameter names (see recolector_common.params).
    """

    color_names: list[str] = field(default_factory=lambda: list(COLOR_NAMES))
    publish_debug_image: bool = True


class CubeDetectorNode(Node):
    """
    Runs the colour detector and the area locator on every incoming image.
    """

    def __init__(self):
        """
        Declare the parameters and create the detector's topics.
        """
        super().__init__(NODE_NAME)
        declare_params(self, CubeDetectorParams())
        declare_params(self, ZoneParams(), prefix=ZONE_PREFIX)
        self._params = read_params(self, CubeDetectorParams())
        zone_params = read_params(self, ZoneParams(), prefix=ZONE_PREFIX)
        for name in self._hsv_names(zone_params):
            self.declare_parameter(
                f"{HSV_PREFIX}{name}",
                Parameter.Type.INTEGER_ARRAY,
            )
        declare_params(self, CameraParams(), prefix=CAMERA_PREFIX)
        declare_params(self, DetectionFilter(), prefix=FILTER_PREFIX)

        self._detector = None
        self._zone_locator = None
        self._image_size = None
        self.add_on_set_parameters_callback(self._validate_params)
        self.add_post_set_parameters_callback(self._on_params_changed)

        self._pub_cubes = self.create_publisher(
            CubeDetectionArray,
            TOPIC_CUBES,
            DEFAULT_QOS_DEPTH,
        )
        self._pub_zone = self.create_publisher(
            CubeDetectionArray,
            TOPIC_ZONE_SEEN,
            DEFAULT_QOS_DEPTH,
        )
        self._pub_debug = self.create_publisher(
            Image,
            TOPIC_CUBES_DEBUG_IMAGE,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            TOPIC_IMAGE_RAW,
            self._on_image,
            qos_profile_sensor_data,
        )
        colors = ", ".join(self._params.color_names)
        self.get_logger().info(f"looking for cubes of colours: {colors}")

    def _hsv_names(self, zone_params: ZoneParams) -> list[str]:
        """
        Every colour that needs an HSV range: cubes, landmarks and paper.
        """
        names = [
            *self._params.color_names,
            *zone_params.landmark_colors,
            *zone_params.floor_colors,
        ]
        return list(dict.fromkeys(names))

    # ---------- parameters ----------
    def _validate_params(self, params):
        """
        Reject malformed HSV ranges before they are applied.
        """
        for p in params:
            if p.name.startswith(HSV_PREFIX):
                try:
                    parse_hsv_ranges(p.value)
                except (ValueError, TypeError) as e:
                    return SetParametersResult(successful=False, reason=str(e))
        return SetParametersResult(successful=True)

    def _on_params_changed(self, _params):
        """
        Drop the detectors; they are rebuilt on the next frame.
        """
        self._params = read_params(self, CubeDetectorParams())
        self._detector = None

    def _read_ranges(self, names: list[str]) -> dict:
        """
        Parsed HSV ranges of `names`; colours without a range are skipped.
        """
        ranges = {}
        for name in names:
            param = f"{HSV_PREFIX}{name}"
            if not self.has_parameter(param):
                continue
            values = self.get_parameter(param).value
            if not values:
                self.get_logger().warning(
                    f"colour '{name}' has no {param} range; ignored"
                )
                continue
            ranges[name] = parse_hsv_ranges(values)
        return ranges

    def _build_detectors(self, width: int, height: int):
        """
        Build the cube detector and the area locator for images of the
        given size from the parameters.
        """
        zone_params = read_params(self, ZoneParams(), prefix=ZONE_PREFIX)
        ranges = self._read_ranges(self._hsv_names(zone_params))
        camera_params = read_params(self, CameraParams(), prefix=CAMERA_PREFIX)
        camera = CameraModel(
            width_px=width,
            height_px=height,
            hfov_rad=camera_params.hfov_rad,
            height_m=camera_params.height_m,
            tilt_rad=camera_params.tilt_rad,
        )
        filters = read_params(self, DetectionFilter(), prefix=FILTER_PREFIX)
        cube_ranges = {
            c: ranges[c] for c in self._params.color_names if c in ranges
        }
        self._detector = ColorDetector(cube_ranges, camera, filters)
        self._zone_locator = ZoneLocator(ranges, camera, zone_params)

    # ---------- image ----------
    def _on_image(self, msg: Image):
        """
        Detect cubes and the area in one image and publish them (and the
        debug image).
        """
        try:
            bgr = image_to_bgr(msg)
        except ValueError as e:
            self.get_logger().error(
                str(e),
                throttle_duration_sec=IMAGE_ERROR_THROTTLE_S,
            )
            return
        size = (msg.width, msg.height)
        if self._detector is None or self._image_size != size:
            self._build_detectors(*size)
            self._image_size = size

        zone = self._zone_locator.locate(bgr)
        detections = without_cubes_in_zone(
            self._detector.detect(bgr),
            zone,
            self._zone_locator.params.ignore_margin_px,
        )
        self._pub_cubes.publish(_to_message(detections, msg.header))
        zone_msg = CubeDetectionArray(header=msg.header)
        if zone is not None:
            zone_msg.cubes.append(_region_to_detection(zone))
        self._pub_zone.publish(zone_msg)

        wants_debug = self._params.publish_debug_image
        if wants_debug and self._pub_debug.get_subscription_count() > 0:
            debug_image = draw_detections(bgr, detections)
            if zone is not None:
                draw_region(debug_image, zone)
            self._pub_debug.publish(bgr_to_image(debug_image, msg.header))


def _region_to_detection(zone) -> CubeDetection:
    """
    The delivery area as a CubeDetection (its box as width and height).
    """
    out = CubeDetection()
    out.color = zone.color
    out.bearing_rad = float(zone.bearing_rad)
    out.distance_m = float(zone.distance_m)
    out.u, out.v = int(zone.u), int(zone.v)
    out.width_px, out.height_px = int(zone.bbox[2]), int(zone.bbox[3])
    return out


def _to_message(detections, header) -> CubeDetectionArray:
    """
    Convert detector output into the CubeDetectionArray message.
    """
    out = CubeDetectionArray()
    out.header = header
    for d in detections:
        cube = CubeDetection()
        cube.color = d.color
        cube.bearing_rad = float(d.bearing_rad)
        cube.distance_m = float(d.distance_m)
        cube.u, cube.v = int(d.u), int(d.v)
        cube.width_px, cube.height_px = int(d.width_px), int(d.height_px)
        out.cubes.append(cube)
    return out


def main(args=None):
    """
    Entry point of the cube_detector node.
    """
    spin_node(CubeDetectorNode, args)


if __name__ == "__main__":
    main()
