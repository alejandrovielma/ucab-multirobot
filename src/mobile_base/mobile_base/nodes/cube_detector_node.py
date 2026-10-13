"""Cube detector node: camera/image_raw -> cubes (CubeDetectionArray)."""

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from mobile_base.core.colors import DEFAULT_HUE_RANGES, ranges_from_flat
from mobile_base.nodes.common import declare_dataclass, spin_node
from mobile_base.vision import VisionParams, detect_cubes, image_to_bgr, scoop_content
from multirobot_interfaces.msg import CubeDetection, CubeDetectionArray


class CubeDetectorNode(Node):
    def __init__(self):
        super().__init__("cube_detector")
        values = declare_dataclass(self, VisionParams, skip=("hue_ranges",))
        flat = {}
        for color, spans in DEFAULT_HUE_RANGES.items():
            default = [bound for span in spans for bound in span]
            flat[color] = self.declare_parameter(f"hue_ranges.{color}", default).value
        self.params = VisionParams(hue_ranges=ranges_from_flat(flat), **values)
        self.create_subscription(Image, "camera/image_raw", self._on_image, qos_profile_sensor_data)
        self._pub = self.create_publisher(CubeDetectionArray, "cubes", 10)

    def _on_image(self, msg: Image):
        try:
            bgr = image_to_bgr(msg.height, msg.width, msg.encoding, msg.step, bytes(msg.data))
        except ValueError as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=5.0)
            return
        out = CubeDetectionArray()
        out.stamp = msg.header.stamp
        for det in detect_cubes(bgr, self.params):
            cube = CubeDetection()
            cube.color = det.color
            cube.cx_norm = float(det.cx_norm)
            cube.cy_norm = float(det.cy_norm)
            cube.area_px = float(det.area_px)
            out.cubes.append(cube)
        out.scoop_color, fill = scoop_content(bgr, self.params)
        out.scoop_fill = float(fill)
        self._pub.publish(out)


def main(args=None):
    spin_node(CubeDetectorNode, args)


if __name__ == "__main__":
    main()
