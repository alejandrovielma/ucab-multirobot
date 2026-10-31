"""
Tests: delivery area location (landmarks far away, area paper up close)
and dropping cubes already inside it (needs OpenCV).
"""

import cv2
import numpy as np

from vision.camera_model import CameraModel
from vision.color_detection import Detection, Region
from vision.hsv import parse_hsv_ranges
from vision.zone import (
    ZoneLocator,
    ZoneParams,
    choose_zone,
    entry_edge,
    is_plausible_floor,
    without_cubes_in_zone,
)

CAM = CameraModel(
    width_px=640,
    height_px=480,
    hfov_rad=0.94,
    height_m=0.10,
    tilt_rad=0.20,
)
RANGES = {
    "red": parse_hsv_ranges(
        [0, 110, 80, 8, 255, 255, 165, 110, 80, 179, 255, 255]
    ),
    "green": parse_hsv_ranges([45, 80, 50, 92, 255, 255]),
    "blue": parse_hsv_ranges([103, 120, 70, 128, 255, 255]),
}
GREEN = (60, 200, 180)
RED = (2, 200, 180)
BLUE = (115, 200, 180)


def blank():
    return np.full((480, 640, 3), 235, dtype=np.uint8)


def paint_hsv(img, x, y, w, h, hsv):
    bgr = cv2.cvtColor(np.uint8([[hsv]]), cv2.COLOR_HSV2BGR)[0, 0]
    img[y : y + h, x : x + w] = bgr


def region(color, u, v, bbox):
    return Region(color, u, v, 1000, bbox, 0.0, 0.5)


def test_choose_zone_prefers_the_floor():
    gate = region("gate", 300, 200, (250, 150, 100, 50))
    floor = region("floor", 320, 400, (200, 380, 240, 60))
    assert choose_zone(gate, floor) is floor
    assert choose_zone(gate, None) is gate
    assert choose_zone(None, None) is None


def test_far_away_only_the_landmarks_locate_the_zone():
    img = blank()
    paint_hsv(img, 140, 150, 20, 50, GREEN)
    paint_hsv(img, 460, 150, 20, 50, RED)
    zone = ZoneLocator(RANGES, CAM, ZoneParams()).locate(img)
    assert zone is not None and zone.color == "gate"
    assert 150 < zone.u < 470


def test_up_close_the_area_paper_wins():
    img = blank()
    paint_hsv(img, 140, 150, 20, 50, GREEN)
    paint_hsv(img, 460, 150, 20, 50, RED)
    paint_hsv(img, 180, 400, 140, 70, BLUE)
    paint_hsv(img, 320, 400, 140, 70, GREEN)
    zone = ZoneLocator(RANGES, CAM, ZoneParams()).locate(img)
    assert zone is not None and zone.color == "floor"


def test_nothing_in_view_gives_no_zone():
    assert ZoneLocator(RANGES, CAM, ZoneParams()).locate(blank()) is None


def test_cubes_inside_the_floor_area_are_dropped():
    floor = region("floor", 320, 470, (180, 400, 280, 70))
    inside = Detection("yellow", 300, 440, 20, 20, (290, 420, 20, 20))
    outside = Detection("yellow", 100, 300, 20, 20, (90, 280, 20, 20))
    kept = without_cubes_in_zone([inside, outside], floor, margin_px=80)
    assert kept == [outside]
    assert without_cubes_in_zone([inside], None, margin_px=80) == [inside]


def floor_blob(area_px, v):
    return Region("floor", 320, v, area_px, (0, 0, 640, 40), 0.0, 0.5)


def test_an_area_blob_larger_than_the_limit_is_not_the_paper():
    params = ZoneParams(floor_max_area_px=50000)
    assert is_plausible_floor(floor_blob(14000, 250), 480, params)
    assert not is_plausible_floor(floor_blob(90000, 250), 480, params)


def test_an_area_blob_reaching_the_bottom_is_not_the_paper_when_asked():
    asked = ZoneParams(floor_ignore_touching_bottom=True)
    assert is_plausible_floor(floor_blob(14000, 445), 480, asked)
    assert not is_plausible_floor(floor_blob(14000, 479), 480, asked)
    assert is_plausible_floor(floor_blob(14000, 479), 480, ZoneParams())


def test_there_is_no_paper_without_a_blob():
    assert not is_plausible_floor(None, 480, ZoneParams())


def test_the_area_paper_is_aimed_at_the_middle_of_its_width():
    # A strip whose lowest rows are at its left end, like a robot seen at an
    # angle: the nearest point is on the left, the aim must stay centred.
    img = blank()
    paint_hsv(img, 100, 300, 400, 40, BLUE)
    paint_hsv(img, 100, 340, 60, 20, BLUE)
    params = ZoneParams(floor_colors=["blue"])
    zone = ZoneLocator(RANGES, CAM, params).locate(img)
    assert zone is not None and zone.color == "floor"
    assert abs(zone.u - 300) <= 2  # the middle of x 100..500


def test_the_gate_can_be_aimed_at_the_midpoint_between_the_legos():
    img = blank()
    paint_hsv(img, 140, 150, 20, 50, GREEN)
    paint_hsv(img, 460, 150, 20, 50, RED)
    default = ZoneLocator(RANGES, CAM, ZoneParams()).locate(img)
    centred = ZoneLocator(
        RANGES, CAM, ZoneParams(gate_aim_fraction=0.5)
    ).locate(img)
    assert default is not None and centred is not None
    assert centred.u > default.u  # 0.5 is further right than 0.38
    assert abs(centred.u - 310) <= 2  # between x 150 and 470


def test_a_delivered_cube_near_the_right_edge_of_the_area_is_ignored():
    img = blank()
    paint_hsv(img, 140, 150, 20, 50, GREEN)
    paint_hsv(img, 460, 150, 20, 50, RED)
    # Inside the area (it spans about x 246..374 around the midpoint, 310),
    # near its right edge.
    delivered = Detection("yellow", 372, 195, 12, 12, (366, 183, 12, 12))
    old = ZoneLocator(RANGES, CAM, ZoneParams()).locate(img)
    centred = ZoneLocator(
        RANGES, CAM, ZoneParams(gate_aim_fraction=0.5)
    ).locate(img)
    # Aimed at 0.38 the right part of the area is not covered: the cube was
    # picked up again. Aimed at the midpoint it is.
    assert without_cubes_in_zone([delivered], old, margin_px=10) == [delivered]
    assert without_cubes_in_zone([delivered], centred, margin_px=10) == []


def test_entry_edge_square_on_has_no_yaw_and_aims_at_its_middle():
    xs = np.arange(200, 441)
    u, v, yaw_rad = entry_edge(xs, np.full(xs.shape, 400), CAM)
    assert (u, round(v)) == (320, 400)
    assert abs(yaw_rad) < 1e-6


def test_entry_edge_nearer_on_the_right_means_turn_right():
    # The right end lower in the image is nearer: the robot is turned to
    # the left of the edge's normal and must turn right (negative).
    xs = np.arange(200, 441)
    _, _, yaw_rad = entry_edge(xs, 400 + (xs - 200) * 0.2, CAM)
    assert yaw_rad < -0.05
    _, _, mirrored = entry_edge(xs, 448 - (xs - 200) * 0.2, CAM)
    assert mirrored > 0.05


def test_the_paper_is_aimed_at_its_entry_edge_when_it_is_in_view():
    ranges = {**RANGES, "pink": parse_hsv_ranges([160, 80, 80, 175, 255, 255])}
    params = ZoneParams(floor_colors=["blue", "pink"], entry_color="pink")
    img = blank()
    paint_hsv(img, 150, 330, 300, 50, BLUE)
    paint_hsv(img, 150, 380, 300, 14, (168, 200, 200))
    zone = ZoneLocator(ranges, CAM, params).locate(img)
    assert zone.color == "floor" and abs(zone.entry_angle_rad) < 0.01
    assert abs(zone.u - 300) <= 2 and 380 <= zone.v <= 394
    no_entry = ZoneLocator(ranges, CAM, ZoneParams(floor_colors=["blue"]))
    assert no_entry.locate(img).entry_angle_rad is None
