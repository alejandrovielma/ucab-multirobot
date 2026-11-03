"""
Finds the robot and the delivery area in the overhead (DroidCam) frame.

The robot is the largest dark blob inside the arena (black chassis on the white
track); the area is the largest teal rectangle. Positions are in pixels of the
frame given. The arena polygon depends on where the phone stands: measure it
again if the phone moves (`overhead.arena_px` in teleop_web.yaml).
"""

from dataclasses import dataclass

import cv2
import numpy as np

# Pixels (frame 640 px wide) of the arena's inner corners: top-left, top-right,
# bottom-right, bottom-left. Measured on 2026-11-03.
DEFAULT_ARENA_PX = ((177, 127), (481, 123), (523, 297), (137, 303))
DARK_VALUE_MAX = 90  # HSV value under this is the chassis
MIN_ROBOT_AREA_PX = 400
TEAL_HSV_LOW = (82, 140, 110)
TEAL_HSV_HIGH = (102, 255, 255)
MIN_ZONE_AREA_PX = 300
# The area sits left of this column in the 640 px frame (cubes are darker blue, far from it).
ZONE_MAX_COLUMN_FRACTION = 0.47
ERODE_PX = 9  # keeps the border tape out of the arena mask
CLOSE_PX = 7


@dataclass(frozen=True)
class OverheadPose:
    """
    Pixel positions seen in one overhead frame; None when not found.
    """

    robot_px: tuple[float, float] | None
    zone_px: tuple[float, float] | None


def _largest_blob(mask: np.ndarray, min_area_px: int) -> tuple[float, float] | None:
    """
    Centre of the largest connected region of `mask`, or None if it is small.
    """
    count, _, stats, centres = cv2.connectedComponentsWithStats(mask)
    if count <= 1:
        return None
    best = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    if stats[best, cv2.CC_STAT_AREA] < min_area_px:
        return None
    return float(centres[best][0]), float(centres[best][1])


def locate(bgr: np.ndarray, arena_px=DEFAULT_ARENA_PX) -> OverheadPose:
    """
    Robot and delivery-area positions in `bgr`.
    """
    height, width = bgr.shape[:2]
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    arena = np.zeros((height, width), np.uint8)
    cv2.fillPoly(arena, [np.array(arena_px, np.int32)], 255)
    arena = cv2.erode(arena, np.ones((ERODE_PX, ERODE_PX), np.uint8))
    dark = ((hsv[..., 2] < DARK_VALUE_MAX) & (arena > 0)).astype(np.uint8) * 255
    dark = cv2.morphologyEx(
        dark, cv2.MORPH_CLOSE, np.ones((CLOSE_PX, CLOSE_PX), np.uint8)
    )
    teal = cv2.inRange(hsv, TEAL_HSV_LOW, TEAL_HSV_HIGH)
    teal[:, int(width * ZONE_MAX_COLUMN_FRACTION) :] = 0
    return OverheadPose(
        _largest_blob(dark, MIN_ROBOT_AREA_PX),
        _largest_blob(teal, MIN_ZONE_AREA_PX),
    )


def annotate(bgr: np.ndarray, pose: OverheadPose) -> np.ndarray:
    """
    The frame with a red ring on the robot and a green ring on the area.
    """
    out = bgr.copy()
    if pose.robot_px is not None:
        cv2.circle(out, (int(pose.robot_px[0]), int(pose.robot_px[1])), 8, (0, 0, 255), 2)
    if pose.zone_px is not None:
        cv2.circle(out, (int(pose.zone_px[0]), int(pose.zone_px[1])), 8, (0, 255, 0), 2)
    return out
