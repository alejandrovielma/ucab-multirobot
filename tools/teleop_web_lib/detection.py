"""
Cube and delivery-area detection for the browser tool, on top of the vision
package's pure logic (the same code the ROS cube_detector node uses,
including vision/zone.py for the area).

Needs OpenCV, NumPy and PyYAML. `camera.py` imports this module lazily, so
the robot still drives when they are missing.
"""

import logging
import sys
import threading
import time

import numpy as np

from teleop_web_lib.border import BorderParams, without_cubes_near_border
from teleop_web_lib.config import (
    DEFAULT_DETECTION_FILTER,
    HSV_CONFIG_PATH,
    HSV_NODE_NAME,
    VISION_PACKAGE_DIR,
    read_node_params,
)
from teleop_web_lib.support import SupportParams, without_cubes_off_table

log = logging.getLogger("teleop_web")

# Section of hsv_ranges.yaml with the delivery area parameters.
ZONE_SECTION = "zone"
# Key of the camera section that overrides the camera geometry.
MODEL_SECTION = "model"
# Key of the camera section with the border filter settings.
BORDER_SECTION = "border"
# Key of the camera section with the cube-on-the-table filter settings.
SUPPORT_SECTION = "support"
# Key of the camera section with extra HSV ranges for the area paper.
AREA_HSV_KEY = "area_hsv"


class DetectionPipeline:
    """
    Finds the best cube and the delivery area in each frame, remembers the
    latest of each, and returns the annotated frame for the video stream.
    """

    def __init__(
        self,
        detector,
        zone_locator,
        border: BorderParams | None = None,
        support: SupportParams | None = None,
    ):
        """
        Keep the cube detector, the area locator (vision/zone.py) and the
        border filter settings (off when not given).
        """
        self._detector = detector
        self._zone_locator = zone_locator
        self._border = border or BorderParams()
        self._support = support or SupportParams()
        self._lock = threading.Lock()
        self._det = None
        self._det_stamp_s = 0.0
        self._zone = None
        self._zone_stamp_s = 0.0

    def latest_detection(self, now: float, timeout_s: float):
        """
        The best cube seen less than `timeout_s` ago, or None.
        """
        with self._lock:
            if self._det is not None and (now - self._det_stamp_s) <= timeout_s:
                return self._det
            return None

    def latest_zone(self, now: float, timeout_s: float):
        """
        The delivery area seen less than `timeout_s` ago, or None.
        """
        with self._lock:
            if (
                self._zone is not None
                and (now - self._zone_stamp_s) <= timeout_s
            ):
                return self._zone
            return None

    def process(self, frame: np.ndarray) -> np.ndarray:
        """
        Detect in `frame`, remember the result and return the annotated
        frame.
        """
        from vision.drawing import draw_detections, draw_region
        from vision.zone import without_cubes_in_zone

        zone = self._zone_locator.locate(frame)
        zone_params = self._zone_locator.params
        detections = without_cubes_in_zone(
            self._detector.detect(frame),
            zone,
            zone_params.ignore_margin_px,
            zone_params.ignore_floor_margin_m,
            zone_params.ignore_gate_depth_m,
        )
        detections = without_cubes_near_border(frame, detections, self._border)
        detections = without_cubes_off_table(frame, detections, self._support)
        best = detections[0] if detections else None
        self._remember(best, zone)

        annotated = draw_detections(frame, detections, best)
        if zone is not None:
            draw_region(annotated, zone)
        return annotated

    def _remember(self, best, zone) -> None:
        """
        Store the latest detections with the time they were seen.
        """
        now = time.monotonic()
        with self._lock:
            self._det = best
            if best is not None:
                self._det_stamp_s = now
            self._zone = zone
            if zone is not None:
                self._zone_stamp_s = now


def build_pipeline(
    camera_cfg: dict,
    detection_overrides: dict | None = None,
) -> DetectionPipeline | None:
    """
    Build the pipeline from the camera section and the shared HSV file.

    Returns None (with a warning) when no target colours are configured or
    when anything needed to detect is missing: control keeps working
    without detection.
    """
    colors = camera_cfg.get("target_colors") or []
    if not colors:
        return None
    try:
        return _build_pipeline(camera_cfg, colors, detection_overrides or {})
    except Exception as exc:
        log.warning("no colour detector: %s", exc)
        return None


def _build_pipeline(
    camera_cfg: dict,
    colors: list[str],
    detection_overrides: dict,
) -> DetectionPipeline:
    """
    The actual construction; may raise when files or colours are missing.
    """
    sys.path.insert(0, str(VISION_PACKAGE_DIR))
    from vision.camera_model import CameraModel
    from vision.color_detection import ColorDetector, DetectionFilter
    from vision.hsv import parse_hsv_ranges
    from vision.zone import ZoneLocator, ZoneParams

    hsv_cfg = read_node_params(HSV_CONFIG_PATH, HSV_NODE_NAME)
    # Colours of this tool only (the delivery area paper), added to the
    # shared ones without touching the file the ROS nodes read.
    hsv_values = {**hsv_cfg["hsv"], **camera_cfg.get(AREA_HSV_KEY, {})}
    ranges = {c: parse_hsv_ranges(v) for c, v in hsv_values.items()}

    # The tool's camera section overrides the shared geometry (its sensor
    # mode is not the one the shared values were measured in).
    geometry = {**hsv_cfg["camera"], **camera_cfg.get(MODEL_SECTION, {})}
    camera_model = CameraModel(
        width_px=camera_cfg["width_px"],
        height_px=camera_cfg["height_px"],
        hfov_rad=geometry["hfov_rad"],
        height_m=geometry["height_m"],
        tilt_rad=geometry["tilt_rad"],
    )
    filters = DetectionFilter(
        cube_size_m=hsv_cfg["filter"]["cube_size_m"],
        **{**DEFAULT_DETECTION_FILTER, **detection_overrides},
    )
    detector = ColorDetector(
        {c: ranges[c] for c in colors if c in ranges},
        camera_model,
        filters,
    )
    # The camera section may override the shared zone settings (e.g. turn the
    # area paper off: `zone: {floor_colors: []}`).
    zone_params = ZoneParams(
        **{**hsv_cfg.get(ZONE_SECTION, {}), **camera_cfg.get(ZONE_SECTION, {})}
    )
    return DetectionPipeline(
        detector,
        ZoneLocator(ranges, camera_model, zone_params),
        BorderParams(**camera_cfg.get(BORDER_SECTION, {})),
        SupportParams(**camera_cfg.get(SUPPORT_SECTION, {})),
    )
