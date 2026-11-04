"""
Finds the robot, the delivery area and the cubes (with their colours) in the overhead (DroidCam)
frame.

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
# The area sits between these columns (fractions of the width) of the frame: where it falls depends
# on where the phone stands. Cubes are darker blue, far from it.
DEFAULT_ZONE_COLUMNS = (0.0, 0.47)
CUBE_DRAW_BGR = {"blue": (255, 120, 0), "yellow": (0, 255, 255), "red": (0, 0, 255), "green": (0, 255, 0)}
ERODE_PX = 9  # keeps the border tape out of the arena mask
CLOSE_PX = 7
# HSV ranges of the cubes seen from above (a 3 cm cube is ~6-8 px wide in a 640 px frame).
CUBE_HSV = {
    "blue": [((100, 110, 40), (130, 255, 255))],
    "yellow": [((18, 110, 110), (36, 255, 255))],
    "red": [((0, 130, 90), (8, 255, 255)), ((170, 130, 90), (180, 255, 255))],
    "green": [((45, 90, 50), (85, 255, 255))],
}
MIN_CUBE_AREA_PX = 12
MAX_CUBE_AREA_PX = 220
MAX_CUBE_SIDE_PX = 22
MAX_CUBE_ASPECT = 2.2
# Blobs this close to the robot's centre are its own parts (board, wheels), not cubes.
ROBOT_EXCLUDE_RADIUS_PX = 45


@dataclass(frozen=True)
class OverheadPose:
    """
    Pixel positions seen in one overhead frame; None when not found.
    """

    robot_px: tuple[float, float] | None
    zone_px: tuple[float, float] | None
    # (colour, x, y) of every cube inside the arena.
    cubes: tuple = ()


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


def find_cubes(hsv: np.ndarray, arena: np.ndarray, robot_px) -> tuple:
    """
    (colour, x, y) of the small compact blobs of each cube colour inside the
    arena mask, leaving out the robot's own parts.
    """
    cubes = []
    for colour, ranges in CUBE_HSV.items():
        mask = np.zeros(arena.shape, np.uint8)
        for low, high in ranges:
            mask |= cv2.inRange(hsv, low, high)
        mask &= arena
        count, _, stats, centres = cv2.connectedComponentsWithStats(mask)
        for i in range(1, count):
            area = stats[i, cv2.CC_STAT_AREA]
            width, height = stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT]
            side, short = max(width, height), max(1, min(width, height))
            if not MIN_CUBE_AREA_PX <= area <= MAX_CUBE_AREA_PX:
                continue
            if side > MAX_CUBE_SIDE_PX or side > MAX_CUBE_ASPECT * short:
                continue
            x, y = float(centres[i][0]), float(centres[i][1])
            if robot_px is not None and (
                (x - robot_px[0]) ** 2 + (y - robot_px[1]) ** 2
                < ROBOT_EXCLUDE_RADIUS_PX**2
            ):
                continue
            cubes.append((colour, x, y))
    return tuple(cubes)


def locate(
    bgr: np.ndarray, arena_px=DEFAULT_ARENA_PX, zone_columns=DEFAULT_ZONE_COLUMNS
) -> OverheadPose:
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
    teal[:, : int(width * zone_columns[0])] = 0
    teal[:, int(width * zone_columns[1]) :] = 0
    robot = _largest_blob(dark, MIN_ROBOT_AREA_PX)
    return OverheadPose(
        robot,
        _largest_blob(teal, MIN_ZONE_AREA_PX),
        find_cubes(hsv, cv2.dilate(arena, np.ones((ERODE_PX, ERODE_PX), np.uint8)), robot),
    )


def annotate(bgr: np.ndarray, pose: OverheadPose) -> np.ndarray:
    """
    The frame with a red ring on the robot, a green ring on the area and a small
    ring on each cube in its own colour.
    """
    out = bgr.copy()
    for colour, x, y in pose.cubes:
        cv2.circle(out, (int(x), int(y)), 6, CUBE_DRAW_BGR[colour], 1)
    if pose.robot_px is not None:
        cv2.circle(out, (int(pose.robot_px[0]), int(pose.robot_px[1])), 8, (0, 0, 255), 2)
    if pose.zone_px is not None:
        cv2.circle(out, (int(pose.zone_px[0]), int(pose.zone_px[1])), 8, (0, 255, 0), 2)
    return out
