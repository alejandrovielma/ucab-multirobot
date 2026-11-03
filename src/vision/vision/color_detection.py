"""
Cube detection by colour (HSV) and camera geometry. Pure logic, no rclpy.

Flow: BGR image -> HSV mask per colour -> contours -> filters -> list of
`Detection` with the bearing and approximate distance on the floor (camera
tilted down).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import cv2
import numpy as np

from vision.camera_model import CameraModel
from vision.hsv import HsvRange

# --- Detection filter defaults (real values: config/hsv_ranges.yaml) ----------
DEFAULT_MIN_AREA_PX = 40
DEFAULT_CUBE_SIZE_M = 0.025
# Seen width / expected width at that distance: outside the range it is not
# a cube.
DEFAULT_MIN_SIZE_RATIO = 0.45
DEFAULT_MAX_SIZE_RATIO = 2.2
DEFAULT_MAX_DISTANCE_M = 2.5

# --- Detector constants -------------------------------------------------------
DEFAULT_MORPH_KERNEL_PX = 3
# A cube bounding box is roughly square (width / height within this band).
MIN_CUBE_ASPECT_RATIO = 0.55
MAX_CUBE_ASPECT_RATIO = 1.75
# A cube contour is convex and fills most of its bounding box.
MIN_CUBE_SOLIDITY = 0.75
MIN_CUBE_EXTENT = 0.45
# A line blob must be at least this many times longer than wide.
DEFAULT_MIN_ELONGATION = 3.0
# Narrowest blob width considered when measuring elongation (avoids dividing
# by a near-zero width).
MIN_BLOB_WIDTH_PX = 1.0
# A blob within this many px of an image edge touches it.
EDGE_MARGIN_PX = 1
# Rows (counted up from the lowest one) averaged to find the aim column of
# a line blob.
REGION_NEAR_ROWS_PX = 3
# Real height (m) of the coloured section of a landmark, without its base
# (measured: 6.4 cm; the whole lego is 8.4 cm). The distance by size is
# focal_px * this / height_px, so it needs the true focal length: in the
# cropped 640x480 sensor mode the configured hfov was twice the real one and
# this had to be inflated to 13.25 cm. The tool now requests the full field
# of view and sets hfov in its own YAML (camera.model), so the real value
# works.
LANDMARK_HEIGHT_M = 0.064
# Where the graph paper sits between the left (green) and right (red) landmarks.
GATE_AIM_FRACTION = 0.38
# Horizontal offset (in landmark heights) from a single visible landmark to
# the paper.
LEFT_LANDMARK_AIM_HEIGHTS = 2.4
RIGHT_LANDMARK_AIM_HEIGHTS = 3.2
# Names of the two kinds of delivery-area estimate (`Region.color`).
REGION_GATE = "gate"
REGION_FLOOR = "floor"

# Binary mask values.
MASK_ON = 255
MASK_OFF = 0


@dataclass(frozen=True)
class DetectionFilter:
    """
    Filters that discard blobs that are not cubes.

    Field names double as the `filter.*` ROS parameter names.
    """

    min_area_px: int = DEFAULT_MIN_AREA_PX
    cube_size_m: float = DEFAULT_CUBE_SIZE_M
    min_size_ratio: float = DEFAULT_MIN_SIZE_RATIO
    max_size_ratio: float = DEFAULT_MAX_SIZE_RATIO
    # Discard blobs touching the bottom edge (the scoop shows there).
    ignore_touching_bottom: bool = True
    # Discard blobs cut by the left, right or top edge: their real size is
    # unknown, so a corner of the delivery area could pass for a cube.
    ignore_touching_sides: bool = True
    max_distance_m: float = DEFAULT_MAX_DISTANCE_M


@dataclass
class Detection:
    """
    A cube seen in the image.
    """

    color: str
    u: int  # centre column (px)
    v: int  # bottom edge row (px), where the cube touches the floor
    width_px: int
    height_px: int
    bbox: tuple[int, int, int, int]  # x, y, w, h
    bearing_rad: float = 0.0
    distance_m: float = math.inf


@dataclass(frozen=True)
class Region:
    """
    A large colour blob (the delivery line), not a cube.
    """

    color: str
    u: int  # column of the nearest point (px)
    v: int  # lowest row of the blob (px)
    area_px: int
    bbox: tuple[int, int, int, int]
    bearing_rad: float
    distance_m: float
    # Turn (rad, positive = left) that leaves the robot facing the area's
    # entry line squarely; None when the line is not in view.
    entry_angle_rad: float | None = None
    # Distance (m) to the area's far edge, straight across its middle; None
    # when unknown (above the horizon, or not the area paper).
    far_distance_m: float | None = None


@dataclass(frozen=True)
class Marker:
    """
    An upright coloured landmark (e.g. one of the two traffic-light pieces
    flanking the delivery area).
    """

    color: str
    u: int  # centre column (px)
    v: int  # lowest row (px), where it touches the floor
    area_px: int
    bbox: tuple[int, int, int, int]


@dataclass
class ColorDetector:
    """
    Multi-colour cube detector with its camera model.
    """

    ranges: dict[str, list[HsvRange]]
    camera: CameraModel
    filters: DetectionFilter = field(default_factory=DetectionFilter)
    morph_kernel_px: int = DEFAULT_MORPH_KERNEL_PX

    def mask(self, hsv: np.ndarray, color: str) -> np.ndarray:
        """
        Binary mask (0/255) of one colour in an HSV image.
        """
        total = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for r in self.ranges[color]:
            total |= cv2.inRange(hsv, np.array(r.lower), np.array(r.upper))
        if self.morph_kernel_px > 1:
            kernel = np.ones((self.morph_kernel_px,) * 2, np.uint8)
            total = cv2.morphologyEx(total, cv2.MORPH_OPEN, kernel)
        return total

    def detect(self, bgr: np.ndarray) -> list[Detection]:
        """
        Find cubes of every colour, sorted from nearest to farthest.
        """
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        image_size_px = bgr.shape[:2]
        found: list[Detection] = []
        for color in self.ranges:
            for contour in self._contours_for(hsv, color):
                detection = self._detection_from_contour(
                    color,
                    contour,
                    image_size_px,
                )
                if detection is not None:
                    found.append(detection)
        found.sort(key=lambda d: d.distance_m)
        return found

    def find_region(
        self,
        bgr: np.ndarray,
        min_area_px: int,
        min_elongation: float = DEFAULT_MIN_ELONGATION,
    ) -> tuple[Region | None, np.ndarray]:
        """
        The largest elongated blob of any of the colours (e.g. the
        delivery line).

        Only blobs at least `min_elongation` times longer than wide count,
        so a cube of the same colour is not mistaken for the line. Returns
        the region (with bearing and distance of its nearest point) and the
        mask of those line blobs, to tell which cubes are on it.
        """
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        line_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        best = None  # (color, area, contour) of the largest line blob
        for color in self.ranges:
            for contour in self._contours_for(hsv, color):
                area = cv2.contourArea(contour)
                if area < min_area_px:
                    continue
                if not _is_elongated(contour, min_elongation):
                    continue  # squarish: a cube, not the line
                cv2.drawContours(line_mask, [contour], -1, MASK_ON, cv2.FILLED)
                if best is None or area > best[1]:
                    best = (color, area, contour)
        if best is None:
            return None, line_mask
        return self._region_from_contour(*best), line_mask

    def find_marker(
        self,
        bgr: np.ndarray,
        color: str,
        min_area_px: int,
        min_height_ratio: float,
    ) -> Marker | None:
        """
        The largest upright blob of `color` (taller than wide by at least
        `min_height_ratio`), so a cube of the same colour is not mistaken for
        a landmark.
        """
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        best: Marker | None = None
        for contour in self._contours_for(hsv, color):
            area = cv2.contourArea(contour)
            if area < min_area_px:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            if h < min_height_ratio * w:
                continue
            if best is None or area > best.area_px:
                best = Marker(
                    color,
                    x + w // 2,
                    y + h - 1,
                    int(area),
                    (x, y, w, h),
                )
        return best

    def gate_between(
        self,
        left: Marker | None,
        right: Marker | None,
        aim_fraction: float = GATE_AIM_FRACTION,
        left_aim_heights: float = LEFT_LANDMARK_AIM_HEIGHTS,
        right_aim_heights: float = RIGHT_LANDMARK_AIM_HEIGHTS,
    ) -> Region | None:
        """
        The delivery area (graph paper) located from the two traffic-light
        landmarks (`left` green, `right` red), or from whichever one is in
        view when the robot is close enough that both no longer fit in the
        frame.
        """
        cam = self.camera
        if left is not None and right is not None:
            if left.u >= right.u:
                return None
            u = round(left.u + aim_fraction * (right.u - left.u))
            v = (left.v + right.v) // 2
            h = (left.bbox[3] + right.bbox[3]) / 2.0
            x0 = min(left.bbox[0], right.bbox[0])
            y0 = min(left.bbox[1], right.bbox[1])
            x1 = max(left.bbox[0] + left.bbox[2], right.bbox[0] + right.bbox[2])
            y1 = max(left.bbox[1] + left.bbox[3], right.bbox[1] + right.bbox[3])
            bbox = (x0, y0, x1 - x0, y1 - y0)
            area = left.area_px + right.area_px
        elif left is not None:
            h = float(left.bbox[3])
            u = min(
                cam.width_px - 1,
                round(left.u + left_aim_heights * h),
            )
            v = left.v
            bbox = left.bbox
            area = left.area_px
        elif right is not None:
            h = float(right.bbox[3])
            u = max(0, round(right.u - right_aim_heights * h))
            v = right.v
            bbox = right.bbox
            area = right.area_px
        else:
            return None
        bearing_rad = math.atan2((cam.width_px / 2.0 - u), cam.focal_px)
        distance_m = cam.focal_px * LANDMARK_HEIGHT_M / max(1.0, h)
        return Region(REGION_GATE, u, v, area, bbox, bearing_rad, distance_m)

    def find_floor_area(
        self,
        bgr: np.ndarray,
        min_area_px: int,
        min_flatness: float,
    ) -> Region | None:
        """
        The delivery area on the floor: the largest blob of the union of
        every colour (e.g. blue + green paper side by side), or None.

        It must be at least `min_area_px` and `min_flatness` times wider
        than tall: paper lying on the floor looks squashed from the low
        camera, unlike a cube or an upright landmark.
        """
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        union = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for color in self.ranges:
            union |= self.mask(hsv, color)
        contours, _ = cv2.findContours(
            union,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        if not contours:
            return None
        contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(contour)
        _, _, w, h = cv2.boundingRect(contour)
        if area < min_area_px or w < min_flatness * h:
            return None
        return self._region_from_contour(REGION_FLOOR, area, contour)

    def _contours_for(self, hsv: np.ndarray, color: str) -> list:
        """
        External contours of the blobs of one colour.
        """
        contours, _ = cv2.findContours(
            self.mask(hsv, color),
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        return contours

    def _detection_from_contour(
        self,
        color: str,
        contour: np.ndarray,
        image_size_px: tuple[int, int],
    ) -> Detection | None:
        """
        A located `Detection` if the blob passes every filter, else None.

        `image_size_px` is (height, width).
        """
        area = cv2.contourArea(contour)
        if area < self.filters.min_area_px:
            return None
        x, y, w, h = cv2.boundingRect(contour)
        if h <= 0 or w <= 0:
            return None
        aspect = w / h
        if not (MIN_CUBE_ASPECT_RATIO <= aspect <= MAX_CUBE_ASPECT_RATIO):
            return None
        if area / (w * h) < MIN_CUBE_EXTENT:
            return None
        hull_area = cv2.contourArea(cv2.convexHull(contour))
        if hull_area <= 0 or area / hull_area < MIN_CUBE_SOLIDITY:
            return None
        if self._is_cut_by_edge((x, y, w, h), image_size_px):
            return None
        detection = Detection(color, x + w // 2, y + h - 1, w, h, (x, y, w, h))
        return detection if self._locate(detection) else None

    def _is_cut_by_edge(
        self,
        bbox: tuple[int, int, int, int],
        image_size_px: tuple[int, int],
    ) -> bool:
        """
        True when the blob touches an image edge the filters reject.
        """
        x, y, w, h = bbox
        height_px, width_px = image_size_px
        touches_bottom = y + h >= height_px - EDGE_MARGIN_PX
        touches_sides = (
            x <= EDGE_MARGIN_PX
            or y <= EDGE_MARGIN_PX
            or x + w >= width_px - EDGE_MARGIN_PX
        )
        return (self.filters.ignore_touching_bottom and touches_bottom) or (
            self.filters.ignore_touching_sides and touches_sides
        )

    def _region_from_contour(
        self,
        color: str,
        area: float,
        contour: np.ndarray,
    ) -> Region | None:
        """
        Region aimed at the lowest part of a line blob, or None if that point
        is above the horizon.
        """
        points = contour.reshape(-1, 2)
        v = int(points[:, 1].max())
        # Mean column of the lowest part of the blob: that is where the robot
        # aims.
        near_bottom = points[:, 1] >= v - REGION_NEAR_ROWS_PX
        u = int(points[near_bottom, 0].mean())
        bearing_distance = self.camera.bearing_distance(u, v)
        if bearing_distance is None:
            return None
        return Region(
            color,
            u,
            v,
            int(area),
            cv2.boundingRect(contour),
            *bearing_distance,
        )

    def _locate(self, det: Detection) -> bool:
        """
        Compute bearing and distance; False if the blob does not look like a
        cube.
        """
        cam = self.camera
        point = cam.ground_point(det.u, det.v)
        if point is None:
            return False
        x, y, depth = point
        det.bearing_rad = math.atan2(y, x)
        det.distance_m = math.hypot(x, y)
        if det.distance_m > self.filters.max_distance_m:
            return False
        expected_px = cam.focal_px * self.filters.cube_size_m / depth
        width_ratio = det.width_px / expected_px
        height_ratio = det.height_px / expected_px
        lo, hi = self.filters.min_size_ratio, self.filters.max_size_ratio
        return lo <= width_ratio <= hi and lo <= height_ratio <= hi


def _is_elongated(contour: np.ndarray, min_elongation: float) -> bool:
    """
    True when the contour's minimum-area rectangle is at least
    `min_elongation` times longer than wide.
    """
    width, height = cv2.minAreaRect(contour)[1]
    short_side = max(MIN_BLOB_WIDTH_PX, min(width, height))
    return max(width, height) >= min_elongation * short_side
