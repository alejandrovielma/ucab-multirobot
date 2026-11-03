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
    entry_angle,
    is_plausible_floor,
    polygon_centroid,
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


def test_with_a_floor_margin_cubes_on_the_strip_and_behind_are_dropped():
    floor = Region("floor", 320, 300, 9000, (200, 250, 240, 50), 0.0, 0.20)
    on_strip = cube_at(300, 310, 20, 0.18)
    behind = cube_at(260, 240, 12, 0.40)
    in_front = cube_at(320, 380, 30, 0.12)
    to_a_side = cube_at(100, 300, 20, 0.20)
    cubes = [on_strip, behind, in_front, to_a_side]
    # By pixels only the paper's box counts: the strip cube is kept.
    assert on_strip in without_cubes_in_zone(cubes, floor, margin_px=10)
    kept = without_cubes_in_zone(cubes, floor, 10, floor_margin_m=0.035)
    assert kept == [in_front, to_a_side]


def test_with_a_gate_depth_cubes_beside_the_legos_middle_are_dropped():
    # Only the red lego (64 px tall): the area is aimed 2 heights to its left.
    gate = Region("gate", 400, 200, 2000, (500, 136, 20, 64), 0.0, 0.32)
    at_edge = cube_at(410, 260, 20, 0.26)
    nearer = cube_at(410, 400, 40, 0.15)
    # By pixels (10 px below the lego's base) the cube at the edge is kept.
    assert at_edge in without_cubes_in_zone([at_edge], gate, margin_px=10)
    cubes = [at_edge, nearer]
    kept = without_cubes_in_zone(cubes, gate, 10, gate_depth_m=0.13)
    assert kept == [nearer]


def cube_at(u, v, size_px, distance_m):
    """
    A yellow cube whose bottom centre is at (`u`, `v`), `distance_m` away.
    """
    bbox = (u - size_px // 2, v - size_px, size_px, size_px)
    return Detection("yellow", u, v, size_px, size_px, bbox, 0.0, distance_m)


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


def test_polygon_centroid_of_a_square_and_of_nothing():
    xs, ys = np.array([0.0, 2.0, 2.0, 0.0]), np.array([0.0, 0.0, 2.0, 2.0])
    assert polygon_centroid(xs, ys) == (1.0, 1.0)
    assert polygon_centroid(np.zeros(3), np.zeros(3)) is None


def test_the_paper_can_be_aimed_at_its_centre_on_the_floor():
    img = blank()
    paint_hsv(img, 220, 330, 200, 50, BLUE)  # centred left-right
    params = ZoneParams(floor_colors=["blue"])
    near_edge = ZoneLocator(RANGES, CAM, params).locate(img)
    centre = ZoneLocator(
        RANGES,
        CAM,
        ZoneParams(floor_colors=["blue"], floor_aim_centre=True),
    ).locate(img)
    assert (
        abs(centre.distance_m - near_edge.distance_m) < 0.005
    )  # its near edge
    assert abs(centre.bearing_rad) < 0.02
    # Off to the left: the bearing points at the paper's centre.
    img = blank()
    paint_hsv(img, 40, 330, 200, 50, BLUE)
    centre = ZoneLocator(
        RANGES,
        CAM,
        ZoneParams(floor_colors=["blue"], floor_aim_centre=True),
    ).locate(img)
    assert abs(centre.bearing_rad - CAM.bearing_distance(140, 355)[0]) < 0.05


def line_points(yaw_rad):
    """
    Floor points (forward_m, left_m) of a line 0.2 m ahead, seen by a robot
    turned `yaw_rad` to the left of facing it squarely.
    """
    lefts = np.linspace(-0.1, 0.1, 11)
    forwards = np.full_like(lefts, 0.2)
    cos_yaw, sin_yaw = np.cos(-yaw_rad), np.sin(-yaw_rad)
    return np.stack(
        [
            cos_yaw * forwards - sin_yaw * lefts,
            sin_yaw * forwards + cos_yaw * lefts,
        ],
        axis=1,
    ).astype(np.float32)


def test_entry_angle_is_the_turn_that_faces_the_line_squarely():
    assert abs(entry_angle(line_points(0.0))) < 1e-3
    # Turned 0.3 rad to the left of square: turn 0.3 rad right.
    assert abs(entry_angle(line_points(0.3)) + 0.3) < 1e-3
    assert abs(entry_angle(line_points(-0.2)) - 0.2) < 1e-3


def entry_params(**kwargs):
    return ZoneParams(floor_colors=["blue", "red"], **kwargs)


def test_the_entry_line_angle_is_measured_only_when_asked_and_in_view():
    img = blank()
    paint_hsv(img, 220, 330, 200, 30, BLUE)
    paint_hsv(img, 220, 360, 200, 12, RED)  # the entry strip, square
    off = ZoneLocator(RANGES, CAM, entry_params()).locate(img)
    assert off.entry_angle_rad is None
    on = ZoneLocator(RANGES, CAM, entry_params(entry_color="red")).locate(img)
    assert abs(on.entry_angle_rad) < 0.02
    img = blank()
    paint_hsv(img, 220, 330, 200, 42, BLUE)  # no entry strip
    zone = ZoneLocator(RANGES, CAM, entry_params(entry_color="red")).locate(
        img
    )
    assert zone is not None and zone.entry_angle_rad is None


def test_an_entry_line_nearer_on_the_right_means_turning_right():
    img = blank()
    paint_hsv(img, 200, 300, 240, 60, BLUE)
    strip = np.array([[200, 350], [440, 380], [440, 392], [200, 362]])
    bgr = cv2.cvtColor(np.uint8([[RED]]), cv2.COLOR_HSV2BGR)[0, 0]
    cv2.fillPoly(img, [strip], [int(c) for c in bgr])
    zone = ZoneLocator(RANGES, CAM, entry_params(entry_color="red")).locate(
        img
    )
    assert zone.entry_angle_rad < -0.05


def test_an_entry_strip_apart_from_the_paper_is_found_below_it():
    # Real area: black tape between the blue sheet and the pink strip, so the
    # strip is its own blob outside the paper's bounding box.
    img = blank()
    paint_hsv(img, 220, 300, 200, 40, BLUE)
    paint_hsv(img, 200, 350, 240, 12, RED)
    zone = ZoneLocator(RANGES, CAM, entry_params(entry_color="red")).locate(
        img
    )
    assert zone.entry_angle_rad is not None
    assert abs(zone.entry_angle_rad) < 0.02


def test_a_big_pink_blob_that_is_not_a_strip_is_not_the_entry_line():
    # The table tinted pink near an image corner: big, but not flat.
    img = blank()
    paint_hsv(img, 220, 300, 200, 40, BLUE)
    paint_hsv(img, 400, 346, 30, 30, RED)
    zone = ZoneLocator(RANGES, CAM, entry_params(entry_color="red")).locate(
        img
    )
    assert zone.entry_angle_rad is None


def test_a_landmark_above_the_horizon_is_something_in_the_room():
    # Something red at the top of the image, its base above the horizon
    # (row ~113 for this camera): not a landmark standing on the table.
    img = blank()
    paint_hsv(img, 300, 20, 20, 50, RED)
    assert ZoneLocator(RANGES, CAM, ZoneParams()).locate(img) is None
    img = blank()
    paint_hsv(img, 300, 150, 20, 50, RED)
    assert ZoneLocator(RANGES, CAM, ZoneParams()).locate(img) is not None


def test_a_single_landmark_aims_by_the_configured_offset():
    img = blank()
    paint_hsv(img, 450, 150, 20, 50, RED)  # only the right one, 50 px tall
    near = ZoneLocator(RANGES, CAM, ZoneParams(right_aim_heights=1.0))
    far = ZoneLocator(RANGES, CAM, ZoneParams(right_aim_heights=4.0))
    assert near.locate(img).u - far.locate(img).u == 150  # 3 heights more


def test_the_area_paper_reports_its_far_edge_beyond_its_near_edge():
    img = blank()
    paint_hsv(img, 220, 300, 200, 60, BLUE)
    zone = ZoneLocator(RANGES, CAM, ZoneParams(floor_colors=["blue"])).locate(
        img
    )
    assert zone.far_distance_m is not None
    far_expected = CAM.bearing_distance(320, 300)[1]
    assert abs(zone.far_distance_m - far_expected) < 0.005
    assert zone.far_distance_m > zone.distance_m
