"""
Tests: HSV colour detection on synthetic images (needs OpenCV).

HSV parsing and camera geometry have their own OpenCV-free tests
(test_hsv.py and test_camera_model.py).
"""

import cv2
import numpy as np
import pytest

from vision.camera_model import CameraModel
from vision.color_detection import ColorDetector, DetectionFilter
from vision.hsv import parse_hsv_ranges

CAM = CameraModel(
    width_px=640,
    height_px=480,
    hfov_rad=0.94,
    height_m=0.11,
    tilt_rad=0.42,
)

RANGES = {
    "red": parse_hsv_ranges(
        [0, 110, 100, 6, 255, 255, 173, 110, 100, 179, 255, 255]
    ),
    "orange": parse_hsv_ranges([8, 120, 100, 18, 255, 255]),
    "blue": parse_hsv_ranges([100, 110, 90, 130, 255, 255]),
}

IMAGE_SHAPE = (480, 640, 3)
FIELD_WHITE = 235
CUBE_SIZE_M = 0.025


def blank():
    # White background like the field.
    return np.full(IMAGE_SHAPE, FIELD_WHITE, dtype=np.uint8)


def paint_hsv(img, x, y, w, h, hsv):
    bgr = cv2.cvtColor(np.uint8([[hsv]]), cv2.COLOR_HSV2BGR)[0, 0]
    img[y : y + h, x : x + w] = bgr


def cube_box(u, v_bottom):
    """
    Box the expected size of a 2.5 cm cube with its bottom edge at (u, v).
    """
    _, _, depth = CAM.ground_point(u, v_bottom)
    side = round(CAM.focal_px * CUBE_SIZE_M / depth)
    return u - side // 2, v_bottom - side + 1, side, side


def test_detects_each_color_with_its_position():
    img = blank()
    x, y, w, h = cube_box(200, 300)
    paint_hsv(img, x, y, w, h, (2, 200, 180))  # red near 0
    x2, y2, w2, h2 = cube_box(450, 260)
    paint_hsv(img, x2, y2, w2, h2, (114, 170, 160))  # blue
    det = ColorDetector(RANGES, CAM).detect(img)
    assert [d.color for d in det] == ["red", "blue"]  # sorted by distance
    assert det[0].bearing_rad > 0 > det[1].bearing_rad
    expected_distance = CAM.bearing_distance(det[0].u, det[0].v)[1]
    assert det[0].distance_m == pytest.approx(expected_distance)


def test_red_wraps_around_hue_179():
    img = blank()
    x, y, w, h = cube_box(320, 300)
    paint_hsv(img, x, y, w, h, (177, 190, 170))
    det = ColorDetector(RANGES, CAM).detect(img)
    assert [d.color for d in det] == ["red"]


def test_orange_is_not_red():
    img = blank()
    x, y, w, h = cube_box(320, 300)
    paint_hsv(img, x, y, w, h, (12, 190, 197))
    det = ColorDetector(RANGES, CAM).detect(img)
    assert [d.color for d in det] == ["orange"]


def test_ignores_white_black_and_gray():
    img = blank()
    img[100:140, 100:400] = 20  # black line
    img[300:340, 300:340] = 128  # gray
    assert ColorDetector(RANGES, CAM).detect(img) == []


def test_ignores_blob_touching_bottom_edge():
    img = blank()
    # The orange scoop shows at the bottom of the image.
    paint_hsv(img, 100, 380, 60, 100, (12, 190, 197))
    assert ColorDetector(RANGES, CAM).detect(img) == []
    keep = ColorDetector(
        RANGES,
        CAM,
        DetectionFilter(ignore_touching_bottom=False),
    )
    assert len(keep.detect(img)) <= 1


def test_rejects_cube_sized_blob_cut_by_a_side_edge():
    img = blank()
    _, y, w, h = cube_box(320, 300)
    # A corner of the blue delivery area peeking in at the left edge.
    paint_hsv(img, 0, y, w, h, (114, 170, 160))
    assert ColorDetector(RANGES, CAM).detect(img) == []
    keep = ColorDetector(
        RANGES,
        CAM,
        DetectionFilter(ignore_touching_sides=False),
    )
    assert len(keep.detect(img)) == 1


def test_same_blob_away_from_the_edges_is_a_cube():
    img = blank()
    x, y, w, h = cube_box(320, 300)
    paint_hsv(img, x, y, w, h, (114, 170, 160))
    assert len(ColorDetector(RANGES, CAM).detect(img)) == 1


def test_find_floor_area_joins_colours_side_by_side():
    img = blank()
    # Blue and green paper lying on the floor, wide and squashed.
    paint_hsv(img, 200, 380, 120, 60, (114, 170, 160))
    paint_hsv(img, 320, 380, 120, 60, (60, 170, 160))
    ranges = {**RANGES, "green": parse_hsv_ranges([45, 80, 50, 92, 255, 255])}
    det = ColorDetector({c: ranges[c] for c in ("blue", "green")}, CAM)
    floor = det.find_floor_area(img, min_area_px=1500, min_flatness=1.5)
    assert floor is not None
    assert floor.color == "floor"
    assert floor.bbox[2] >= 230  # both papers in one region
    assert 300 <= floor.u <= 340


def test_find_floor_area_ignores_a_cube():
    img = blank()
    x, y, w, h = cube_box(320, 300)
    paint_hsv(img, x, y, w, h, (114, 170, 160))
    det = ColorDetector({"blue": RANGES["blue"]}, CAM)
    assert det.find_floor_area(img, min_area_px=1500, min_flatness=1.5) is None


def test_rejects_blob_too_big_for_a_cube():
    img = blank()
    x, y, w, h = cube_box(320, 260)
    # A blob much larger than a cube, e.g. another robot's chassis.
    paint_hsv(img, x - 3 * w, y - h, 7 * w, 2 * h, (114, 170, 160))
    assert ColorDetector(RANGES, CAM).detect(img) == []


def test_find_region_returns_nearest_point_of_largest_line():
    img = blank()
    # A blue line across the image, and a small blob that is not the line.
    paint_hsv(img, 100, 300, 400, 20, (110, 200, 200))
    paint_hsv(img, 300, 100, 10, 10, (110, 200, 200))
    region, mask = ColorDetector(RANGES, CAM).find_region(
        img,
        min_area_px=300,
    )
    assert region.color == "blue"
    assert 315 <= region.v <= 320 and 280 <= region.u <= 320
    assert abs(region.bearing_rad) < 0.05
    # Only the line is in the mask.
    assert mask[310, 300] and not mask[105, 305]


def test_find_region_none_without_color():
    region, mask = ColorDetector(RANGES, CAM).find_region(
        blank(),
        min_area_px=300,
    )
    assert region is None and not mask.any()


def test_find_region_ignores_a_cube_of_the_line_colour():
    img = blank()
    # A big blue cube up close.
    paint_hsv(img, 300, 250, 60, 60, (110, 200, 200))
    region, mask = ColorDetector(RANGES, CAM).find_region(
        img,
        min_area_px=300,
    )
    assert region is None and not mask.any()


def test_find_marker_and_gate_between_two_landmarks():
    img = blank()
    # Left upright blue landmark and right upright red landmark.
    paint_hsv(img, 140, 200, 30, 80, (110, 200, 200))
    paint_hsv(img, 460, 200, 30, 80, (2, 200, 180))
    det = ColorDetector(RANGES, CAM)
    left = det.find_marker(img, "blue", min_area_px=250, min_height_ratio=1.4)
    right = det.find_marker(img, "red", min_area_px=250, min_height_ratio=1.4)
    assert left is not None and right is not None
    gate = det.gate_between(left, right)
    assert gate is not None
    assert gate.color == "gate"
    assert 260 <= gate.u <= 300
    assert 275 <= gate.v <= 280
    # Swapped order (left on the right) is rejected.
    assert det.gate_between(right, left) is None
    # Single visible landmark still aims toward the paper.
    assert det.gate_between(left, None) is not None
    assert det.gate_between(None, right) is not None
