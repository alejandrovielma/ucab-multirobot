"""
Tests: camera geometry (pixel -> bearing and distance on the floor), no
OpenCV needed.
"""

import math

import pytest

from vision.camera_model import CameraModel

CAM = CameraModel(
    width_px=640,
    height_px=480,
    hfov_rad=0.94,
    height_m=0.11,
    tilt_rad=0.42,
)


def test_center_bottom_pixel_is_straight_ahead():
    bearing, dist = CAM.bearing_distance(320, 240)
    assert bearing == pytest.approx(0.0)
    assert dist == pytest.approx(0.11 / math.tan(0.42))


def test_lower_rows_are_closer_and_left_is_positive():
    _, near = CAM.bearing_distance(320, 400)
    _, far = CAM.bearing_distance(320, 260)
    assert near < far
    left, _ = CAM.bearing_distance(100, 300)
    right, _ = CAM.bearing_distance(540, 300)
    assert left > 0 > right


def test_above_horizon_has_no_ground_point():
    # The top row is just below the horizon: very far away.
    assert CAM.bearing_distance(320, 0)[1] > 1.5
    # Camera looking up: the image centre does not hit the floor.
    cam_up = CameraModel(640, 480, 0.94, 0.11, tilt_rad=-0.5)
    assert cam_up.ground_point(320, 100) is None
