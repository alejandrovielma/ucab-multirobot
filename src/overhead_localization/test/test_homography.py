import math

import numpy as np
import pytest

from overhead_localization.core.homography import fit_homography, marker_pose, pixel_to_table

# Synthetic camera: table point (x, y) in meters -> pixel (u, v) with a mild perspective.
TRUE_H_TABLE_TO_PIXEL = np.array([[400.0, -20.0, 320.0], [10.0, -380.0, 240.0], [0.02, 0.05, 1.0]])


def _project(h, x, y):
    u, v, w = h @ np.array([x, y, 1.0])
    return u / w, v / w


TABLE_CORNERS = [(-0.9, -0.6), (0.9, -0.6), (0.9, 0.6), (-0.9, 0.6)]
PIXEL_CORNERS = [_project(TRUE_H_TABLE_TO_PIXEL, x, y) for x, y in TABLE_CORNERS]


def test_fit_recovers_table_points():
    h = fit_homography(PIXEL_CORNERS, TABLE_CORNERS)
    for (u, v), (x, y) in zip(PIXEL_CORNERS, TABLE_CORNERS):
        assert pixel_to_table(h, u, v) == pytest.approx((x, y), abs=1e-6)


def test_fit_generalizes_to_interior_points():
    h = fit_homography(PIXEL_CORNERS, TABLE_CORNERS)
    u, v = _project(TRUE_H_TABLE_TO_PIXEL, 0.3, -0.2)
    assert pixel_to_table(h, u, v) == pytest.approx((0.3, -0.2), abs=1e-6)


def test_fit_needs_four_points():
    with pytest.raises(ValueError):
        fit_homography(PIXEL_CORNERS[:3], TABLE_CORNERS[:3])


def test_marker_pose_center_and_heading():
    h = fit_homography(PIXEL_CORNERS, TABLE_CORNERS)
    # Marker 6 cm wide centered at (0.2, 0.1), rotated 30 deg; ArUco corner order:
    # top-left, top-right, bottom-right, bottom-left, with "top" pointing to the robot front.
    theta = math.radians(30.0)
    half = 0.03
    local = [(half, half), (half, -half), (-half, -half), (-half, half)]
    corners = []
    for fx, fy in local:
        x = 0.2 + fx * math.cos(theta) - fy * math.sin(theta)
        y = 0.1 + fx * math.sin(theta) + fy * math.cos(theta)
        corners.append(_project(TRUE_H_TABLE_TO_PIXEL, x, y))
    x, y, heading = marker_pose(h, corners)
    assert (x, y) == pytest.approx((0.2, 0.1), abs=1e-3)
    assert heading == pytest.approx(theta, abs=1e-3)
