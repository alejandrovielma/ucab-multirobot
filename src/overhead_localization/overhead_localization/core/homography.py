"""Pixel -> table-plane homography and marker pose (numpy only, no OpenCV).

The table frame is planar: x, y in meters on the board surface, theta in radians (CCW).
Calibrate once by clicking (or detecting) >= 4 known table points in the overhead image.
"""

import math
from collections.abc import Sequence

import numpy as np

Point = tuple[float, float]


def fit_homography(pixel_pts: Sequence[Point], table_pts: Sequence[Point]) -> np.ndarray:
    """Direct linear transform: 3x3 H such that table ~ H @ [u, v, 1]."""
    if len(pixel_pts) < 4 or len(pixel_pts) != len(table_pts):
        raise ValueError("need at least 4 matching pixel/table points")
    rows = []
    for (u, v), (x, y) in zip(pixel_pts, table_pts):
        rows.append([-u, -v, -1.0, 0.0, 0.0, 0.0, u * x, v * x, x])
        rows.append([0.0, 0.0, 0.0, -u, -v, -1.0, u * y, v * y, y])
    _, _, vt = np.linalg.svd(np.asarray(rows))
    h = vt[-1].reshape(3, 3)
    return h / h[2, 2]


def pixel_to_table(h: np.ndarray, u: float, v: float) -> Point:
    x, y, w = h @ np.array([u, v, 1.0])
    return float(x / w), float(y / w)


def marker_pose(h: np.ndarray, corners: Sequence[Point]) -> tuple[float, float, float]:
    """Pose of a marker from its 4 pixel corners in ArUco order (TL, TR, BR, BL).

    The marker is mounted with its top edge toward the robot front, so the heading is
    the direction from the bottom-edge midpoint to the top-edge midpoint.
    """
    tl, tr, br, bl = (np.array(pixel_to_table(h, u, v)) for u, v in corners)
    center = (tl + tr + br + bl) / 4.0
    forward = (tl + tr) / 2.0 - (bl + br) / 2.0
    return float(center[0]), float(center[1]), math.atan2(forward[1], forward[0])
