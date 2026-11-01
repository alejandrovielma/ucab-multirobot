"""Cube detector node: camera/image_raw -> cubes (recolector_interfaces/CubeDetectionArray) and cubes/scene."""

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from mobile_base.core.adapters import cube_to_ferb_fields
from mobile_base.core.colors import DEFAULT_HUE_RANGES, ranges_from_flat
from mobile_base.nodes.common import declare_dataclass, spin_node
from mobile_base.vision import VisionParams, detect_cubes, detect_square, image_to_bgr, scoop_content
from recolector_interfaces.msg import CubeDetection, CubeDetectionArray, Scene
from vision.camera_model import CameraModel, CameraParams


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
        # Same camera.* names and defaults as Ferb's vision node (vision.camera_model), for bearing/distance.
        default = CameraParams()
        self._camera = {f: self.declare_parameter(f"camera.{f}", getattr(default, f)).value for f in vars(default)}
        self._pub = self.create_publisher(CubeDetectionArray, "cubes", 10)
        self._scene_pub = self.create_publisher(Scene, "cubes/scene", 10)

    def _on_image(self, msg: Image):
        try:
            bgr = image_to_bgr(msg.height, msg.width, msg.encoding, msg.step, bytes(msg.data))
        except ValueError as exc:
            self.get_logger().warning(str(exc), throttle_duration_sec=5.0)
            return
        camera = CameraModel(msg.width, msg.height, **self._camera)
        out = CubeDetectionArray()
        out.header = msg.header
        for det in detect_cubes(bgr, self.params):
            fields = cube_to_ferb_fields(det.color, det.cx_norm, det.cy_norm, det.area_px, msg.width, msg.height, camera)
            out.cubes.append(CubeDetection(**fields))
        self._pub.publish(out)
        scene = Scene()
        scene.header = msg.header
        scene.scoop_color, fill = scoop_content(bgr, self.params)
        scene.scoop_fill = float(fill)
        square = detect_square(bgr, self.params)
        scene.square_visible = square.visible
        scene.square_cx = float(square.cx_norm)
        scene.square_cy = float(square.cy_norm)
        scene.square_area = float(square.area_px)
        self._scene_pub.publish(scene)


def main(args=None):
    spin_node(CubeDetectorNode, args)


if __name__ == "__main__":
    main()
