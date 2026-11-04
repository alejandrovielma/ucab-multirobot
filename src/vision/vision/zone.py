"""
Delivery area location. Pure logic, no rclpy.

The area is blue and green paper in a gap of the black line, flanked by two
upright traffic-light landmarks (green on the left, red on the right, seen
from the arena). Two estimates, best one first:

- floor: the area paper itself, once it is in view at the bottom of the
  image (precise up close);
- gate: the point between the landmarks, which are tall and seen from
  afar.

Cubes already inside the area are dropped so they are not picked up again.
Used by the cube_detector node and by tools/teleop_web_lib.
"""

import math
from dataclasses import dataclass, field, replace

import cv2
import numpy as np

from vision.camera_model import CameraModel
from vision.color_detection import (
    LANDMARK_HEIGHT_M,
    LEFT_LANDMARK_AIM_HEIGHTS,
    REGION_FLOOR,
    RIGHT_LANDMARK_AIM_HEIGHTS,
    ColorDetector,
    Detection,
    Marker,
    Region,
)
from vision.hsv import HsvRange

# --- Defaults (real values: config/hsv_ranges.yaml, `zone.*`) -----------------
# [left, right] landmark colours, seen from the arena.
DEFAULT_LANDMARK_COLORS = ("green", "red")
DEFAULT_FLOOR_COLORS = ("blue", "green")
DEFAULT_LANDMARK_MIN_AREA_PX = 200
# Landmarks are upright: height at least this times the width.
DEFAULT_LANDMARK_MIN_HEIGHT_RATIO = 1.2
# Smallest area paper blob (TODO measure on the track).
DEFAULT_FLOOR_MIN_AREA_PX = 1500
# Paper on the floor looks at least this many times wider than tall.
DEFAULT_FLOOR_MIN_FLATNESS = 1.5
# Cubes up to this many px nearer than the landmarks' bases count as in the
# area when only the gate is known.
DEFAULT_IGNORE_MARGIN_PX = 80
# The same, measured on the floor (m; 0 = use the pixel margins): with the
# paper in view, cubes up to this much nearer than its near edge (on the entry
# strip); with only the gate, up to this much nearer than the landmarks. Both
# also cover anything behind the area, outside the arena.
DEFAULT_IGNORE_FLOOR_MARGIN_M = 0.0
DEFAULT_IGNORE_GATE_DEPTH_M = 0.0
# Where the area sits between the left and right landmarks, as a fraction of
# the distance between them (0 = left one, 1 = right one). 0.38 is where the
# original graph paper was; a centred area is 0.5.
DEFAULT_GATE_AIM_FRACTION = 0.38
# Largest area paper blob accepted (0 = no limit): under some white balances
# the whole pale table reads as paper and fills the lower image.
DEFAULT_FLOOR_MAX_AREA_PX = 0
# Drop an area blob that reaches the bottom of the image: the real paper has
# the floor in front of it, a blob of the table itself does not.
DEFAULT_FLOOR_IGNORE_TOUCHING_BOTTOM = False
# A blob whose lowest row is within this many px of the image bottom
# touches it.
FLOOR_BOTTOM_MARGIN_PX = 3

# Smallest entry line blob (px) whose angle is trusted (TODO measure).
DEFAULT_ENTRY_MIN_AREA_PX = 150
# Fewest outline points needed to fit the entry line.
MIN_ENTRY_POINTS = 2
# The entry line is searched from the paper's top down to one paper height
# below it, and this many paper widths past each side: on the real area the
# pink strip is a separate blob just below the blue sheet, outside its
# bounding box.
ENTRY_SIDE_MARGIN_WIDTHS = 0.5
ENTRY_BELOW_HEIGHTS = 1.0
# The entry strip looks at least this many times wider than tall; the table
# tinted pink near the image corners makes big blobs that are not.
ENTRY_MIN_FLATNESS = 3.0

# Smaller polygons (twice their area, m^2) have no usable centroid.
MIN_DOUBLE_AREA_M2 = 1e-6

# Half width of the area around the gate aim column, in landmark heights.
PAPER_HALF_SPAN_HEIGHTS = 1.8
LANDMARK_COUNT = 2


@dataclass
class ZoneParams:
    """
    Parameters of the delivery area locator.

    Field names double as the `zone.*` ROS parameter names.
    """

    landmark_colors: list[str] = field(
        default_factory=lambda: list(DEFAULT_LANDMARK_COLORS),
    )
    floor_colors: list[str] = field(
        default_factory=lambda: list(DEFAULT_FLOOR_COLORS),
    )
    landmark_min_area_px: int = DEFAULT_LANDMARK_MIN_AREA_PX
    landmark_min_height_ratio: float = DEFAULT_LANDMARK_MIN_HEIGHT_RATIO
    floor_min_area_px: int = DEFAULT_FLOOR_MIN_AREA_PX
    floor_min_flatness: float = DEFAULT_FLOOR_MIN_FLATNESS
    ignore_margin_px: int = DEFAULT_IGNORE_MARGIN_PX
    ignore_floor_margin_m: float = DEFAULT_IGNORE_FLOOR_MARGIN_M
    ignore_gate_depth_m: float = DEFAULT_IGNORE_GATE_DEPTH_M
    floor_max_area_px: int = DEFAULT_FLOOR_MAX_AREA_PX
    gate_aim_fraction: float = DEFAULT_GATE_AIM_FRACTION
    # Horizontal offset, in landmark heights, from a single visible landmark
    # to the area (left one: to its right; right one: to its left).
    left_aim_heights: float = LEFT_LANDMARK_AIM_HEIGHTS
    right_aim_heights: float = RIGHT_LANDMARK_AIM_HEIGHTS
    # Shortest a landmark may look, as a fraction of its height at the distance
    # its base is seen: a cube of its colour is about half as tall. 0 = off.
    landmark_min_height_fraction: float = 0.0
    floor_ignore_touching_bottom: bool = DEFAULT_FLOOR_IGNORE_TOUCHING_BOTTOM
    # Aim (bearing) at the paper's centre on the floor (its outline projected
    # on the floor) instead of the middle of its near edge; the distance stays
    # the near edge's.
    floor_aim_centre: bool = False
    # Colour of the line the robot enters the area across (one of
    # `floor_colors`); its angle on the floor is measured so the robot can
    # face it squarely. Empty = not measured.
    entry_color: str = ""
    entry_min_area_px: int = DEFAULT_ENTRY_MIN_AREA_PX


def is_plausible_floor(
    floor: Region | None,
    frame_height_px: int,
    params: ZoneParams,
) -> bool:
    """
    True when `floor` can be the area paper: it exists, is not larger than
    `floor_max_area_px` and, if asked, does not reach the image bottom.
    """
    if floor is None:
        return False
    if params.floor_max_area_px and floor.area_px > params.floor_max_area_px:
        return False
    touches_bottom = floor.v >= frame_height_px - FLOOR_BOTTOM_MARGIN_PX
    return not (params.floor_ignore_touching_bottom and touches_bottom)


def choose_zone(gate: Region | None, floor: Region | None) -> Region | None:
    """
    The floor estimate when the area paper is in view, else the gate.
    """
    return floor if floor is not None else gate


def in_zone(
    det: Detection,
    zone: Region,
    margin_px: int,
    floor_margin_m: float = 0.0,
    gate_depth_m: float = 0.0,
) -> bool:
    """
    True when cube `det` already lies inside the delivery area, or on its
    edge or behind it (outside the arena) when the floor margins are set.
    """
    x, y, w, h = zone.bbox
    if zone.color == REGION_FLOOR:
        if not x <= det.u <= x + w:
            return False
        if floor_margin_m > 0:
            return det.distance_m >= zone.distance_m - floor_margin_m
        return y <= det.v <= y + h
    half_span_px = PAPER_HALF_SPAN_HEIGHTS * h
    if abs(det.u - zone.u) > half_span_px:
        return False
    if gate_depth_m > 0:
        return det.distance_m >= zone.distance_m - gate_depth_m
    return det.v <= y + h + margin_px


def polygon_centroid(
    xs: np.ndarray,
    ys: np.ndarray,
) -> tuple[float, float] | None:
    """
    Centroid of the simple polygon with vertices (`xs`, `ys`) (shoelace
    formula), or None when it has no area.
    """
    next_xs, next_ys = np.roll(xs, -1), np.roll(ys, -1)
    cross = xs * next_ys - next_xs * ys
    double_area = cross.sum()
    if abs(double_area) < MIN_DOUBLE_AREA_M2:
        return None
    scale = 3.0 * double_area
    return (
        float(((xs + next_xs) * cross).sum() / scale),
        float(((ys + next_ys) * cross).sum() / scale),
    )


def is_flat_strip(contour: np.ndarray) -> bool:
    """
    True when `contour` looks like a strip lying on the floor (much wider
    than tall), as the area's entry line does.
    """
    _, _, width_px, height_px = cv2.boundingRect(contour)
    return width_px >= ENTRY_MIN_FLATNESS * max(height_px, 1)


def entry_angle(points: np.ndarray) -> float:
    """
    Turn (rad, positive = left) that leaves the robot facing squarely the
    straight line through the floor `points` ((forward_m, left_m) rows).
    Facing it squarely, the line runs across the robot (along `left`).
    """
    forward, left = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)[:2, 0]
    if left < 0:
        forward, left = -forward, -left
    return math.atan2(-forward, left)


def without_cubes_in_zone(
    detections: list[Detection],
    zone: Region | None,
    margin_px: int,
    floor_margin_m: float = 0.0,
    gate_depth_m: float = 0.0,
) -> list[Detection]:
    """
    The detections that are not already inside the delivery area (see
    `in_zone`).
    """
    if zone is None:
        return list(detections)
    return [
        d
        for d in detections
        if not in_zone(d, zone, margin_px, floor_margin_m, gate_depth_m)
    ]


class ZoneLocator:
    """
    Finds the delivery area in a frame from the landmarks and the paper.
    """

    def __init__(
        self,
        ranges: dict[str, list[HsvRange]],
        camera: CameraModel,
        params: ZoneParams,
    ):
        """
        Build one detector for the landmarks and one for the area paper
        from the HSV `ranges` (colours without a range are skipped).
        """
        self.params = params
        self._camera = camera
        self._landmarks = None
        if len(params.landmark_colors) == LANDMARK_COUNT and all(
            c in ranges for c in params.landmark_colors
        ):
            self._landmarks = ColorDetector(
                {c: ranges[c] for c in params.landmark_colors},
                camera,
            )
        floor_ranges = {
            c: ranges[c] for c in params.floor_colors if c in ranges
        }
        self._floor = (
            ColorDetector(floor_ranges, camera) if floor_ranges else None
        )

    def locate(self, bgr: np.ndarray) -> Region | None:
        """
        The best estimate of the delivery area in `bgr`, or None.
        """
        return choose_zone(self._find_gate(bgr), self._find_floor(bgr))

    def _find_gate(self, bgr: np.ndarray) -> Region | None:
        """
        The area located from the left and/or right landmark.
        """
        if self._landmarks is None:
            return None
        p = self.params
        left_color, right_color = p.landmark_colors
        left, right = (
            self._on_the_floor(
                self._landmarks.find_marker(
                    bgr,
                    color,
                    p.landmark_min_area_px,
                    p.landmark_min_height_ratio,
                )
            )
            for color in (left_color, right_color)
        )
        return self._landmarks.gate_between(
            left,
            right,
            p.gate_aim_fraction,
            p.left_aim_heights,
            p.right_aim_heights,
        )

    def _on_the_floor(self, marker: Marker | None) -> Marker | None:
        """
        `marker` if its base is below the horizon (a landmark stands on the
        table), else None: something red at the top of the image, in the
        room behind the table, was taken for the red landmark.
        """
        if marker is None:
            return None
        point = self._camera.ground_point(marker.u, marker.v)
        if point is None:
            return None
        expected_px = self._camera.focal_px * LANDMARK_HEIGHT_M / point[2]
        minimum_px = self.params.landmark_min_height_fraction * expected_px
        return marker if marker.bbox[3] >= minimum_px else None

    def _find_floor(self, bgr: np.ndarray) -> Region | None:
        """
        The area paper on the floor.
        """
        if self._floor is None:
            return None
        floor = self._floor.find_floor_area(
            bgr,
            self.params.floor_min_area_px,
            self.params.floor_min_flatness,
        )
        if not is_plausible_floor(floor, bgr.shape[0], self.params):
            return None
        if self.params.floor_aim_centre:
            floor = self._aim_at_floor_centre(bgr, floor)
        else:
            floor = self._aim_at_center(floor)
        return self._with_entry_angle(bgr, self._with_far_edge(floor))

    def _with_far_edge(self, floor: Region) -> Region:
        """
        The area paper with the distance to its far edge (the top of its
        blob, in the middle of its width): it stays in view while the robot
        pushes a cube in, after the near edge has gone under the scoop.
        """
        x, y, width_px, _ = floor.bbox
        far = self._camera.bearing_distance(x + width_px // 2, y)
        if far is None:
            return floor
        return replace(floor, far_distance_m=far[1])

    def _with_entry_angle(self, bgr: np.ndarray, floor: Region) -> Region:
        """
        The area paper with the turn that faces its entry line squarely: the
        line's outline projected on the floor and fitted with a straight
        line. Unchanged when the line is not measured or not in view.
        """
        color = self.params.entry_color
        if not color or color not in self._floor.ranges:
            return floor
        frame_h, frame_w = bgr.shape[:2]
        left, top, width_px, paper_h = floor.bbox
        margin_px = int(width_px * ENTRY_SIDE_MARGIN_WIDTHS)
        x = max(0, left - margin_px)
        y = top
        width_px = min(frame_w, left + width_px + margin_px) - x
        bottom = top + paper_h + int(paper_h * ENTRY_BELOW_HEIGHTS)
        height_px = min(frame_h, bottom) - y
        hsv = cv2.cvtColor(
            bgr[y : y + height_px, x : x + width_px],
            cv2.COLOR_BGR2HSV,
        )
        contours, _ = cv2.findContours(
            self._floor.mask(hsv, color),
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_NONE,
        )
        strips = [c for c in contours if is_flat_strip(c)]
        if not strips:
            return floor
        outline = max(strips, key=cv2.contourArea)
        if cv2.contourArea(outline) < self.params.entry_min_area_px:
            return floor
        ground = [
            self._camera.ground_point(u + x, v + y)
            for u, v in outline.reshape(-1, 2)
        ]
        points = [point[:2] for point in ground if point is not None]
        if len(points) < MIN_ENTRY_POINTS:
            return floor
        return replace(
            floor,
            entry_angle_rad=entry_angle(np.array(points, dtype=np.float32)),
        )

    def _aim_at_floor_centre(self, bgr: np.ndarray, floor: Region) -> Region:
        """
        The area paper aimed at its centre on the floor: the outline of the
        paper projected on the floor and averaged as a polygon. Its image
        centre is not its floor centre (the near part looks bigger), and the
        robot must push the cube there from any side. Only the bearing: the
        far outline's rows are a few pixels from the horizon, so the
        centroid's distance was off by up to 20 cm; the near edge's is not.
        """
        x, y, width_px, height_px = floor.bbox
        hsv = cv2.cvtColor(
            bgr[y : y + height_px, x : x + width_px],
            cv2.COLOR_BGR2HSV,
        )
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for color in self._floor.ranges:
            mask |= self._floor.mask(hsv, color)
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        if not contours:
            return self._aim_at_center(floor)
        outline = max(contours, key=cv2.contourArea).reshape(-1, 2)
        ground = [self._camera.ground_point(u + x, v + y) for u, v in outline]
        if None in ground:
            return self._aim_at_center(floor)
        centre = polygon_centroid(
            np.array([point[0] for point in ground]),
            np.array([point[1] for point in ground]),
        )
        if centre is None:
            return self._aim_at_center(floor)
        forward_m, left_m = centre
        return replace(
            floor,
            u=x + width_px // 2,
            bearing_rad=math.atan2(left_m, forward_m),
        )

    def _aim_at_center(self, floor: Region) -> Region:
        """
        The area paper aimed at the middle of its width. Its nearest point
        jumps from one end of the thin strip to the other as the robot
        sways, so the robot aligned left and right without ever driving in.
        """
        x, _, width_px, _ = floor.bbox
        u = x + width_px // 2
        aim = self._camera.bearing_distance(u, floor.v)
        if aim is None:
            return floor
        bearing_rad, distance_m = aim
        return replace(
            floor,
            u=u,
            bearing_rad=bearing_rad,
            distance_m=distance_m,
        )
