"""
Cube-at-the-border filter for the browser tool. Pure NumPy, no OpenCV.

The scoop only pushes forward, so a cube with the black border line just
behind it (or under it) cannot be brought back: catching it shoves it over
the line for good. Such cubes are dropped from the detections.

The test looks for black tape in a strip behind the cube and beside it. It
is OFF by default: its thresholds must be tuned on real frames first, or it
could hide valid cubes (a dark background behind a far cube looks the same).

A second test drops cubes outside the arena: the border tape crosses the
floor between the robot and the cube (below the cube in the image). The
arena is a rectangle, so from inside it no tape lies between the robot and a
cube that is inside too. The tape is compared with the table's own
brightness: this camera renders it grey (~100 against ~210), never black.
"""

from dataclasses import dataclass

import numpy as np

# --- Defaults (real values: tools/teleop_web.yaml, camera.border) -------------
DEFAULT_MAX_VALUE = 70
DEFAULT_ROWS_BEHIND_RATIO = 1.5
DEFAULT_SIDE_RATIO = 1.0
DEFAULT_MIN_DARK_FRACTION = 0.12
# Beyond the line: a pixel is tape when its brightest channel is at most this
# fraction of the table's (the median below the cube), a row is tape when at
# least this share of it is, and the tape needs this many rows in a row.
DEFAULT_BEYOND_DARK_RATIO = 0.55
DEFAULT_BEYOND_ROW_FRACTION = 0.6
DEFAULT_BEYOND_MIN_ROWS = 3
DEFAULT_IGNORE_BOTTOM_PX = 0


@dataclass(frozen=True)
class BorderParams:
    """
    Settings of the border filter; field names are the YAML keys.
    """

    enabled: bool = False
    max_value: int = DEFAULT_MAX_VALUE
    rows_behind_ratio: float = DEFAULT_ROWS_BEHIND_RATIO
    side_ratio: float = DEFAULT_SIDE_RATIO
    min_dark_fraction: float = DEFAULT_MIN_DARK_FRACTION
    beyond_line: bool = False
    beyond_dark_ratio: float = DEFAULT_BEYOND_DARK_RATIO
    beyond_row_fraction: float = DEFAULT_BEYOND_ROW_FRACTION
    beyond_min_rows: int = DEFAULT_BEYOND_MIN_ROWS
    ignore_bottom_px: int = DEFAULT_IGNORE_BOTTOM_PX


def is_beyond_line(
    frame_bgr: np.ndarray,
    bbox: tuple[int, int, int, int],
    params: BorderParams,
) -> bool:
    """
    True when a band of tape crosses the floor between the robot and the
    cube `bbox` (x, y, w, h): the columns of the cube, from its bottom down
    to the image bottom.
    """
    x, y, w, h = bbox
    # The scoop shows dark at the bottom of the image: it is not tape on the floor.
    bottom = frame_bgr.shape[0] - params.ignore_bottom_px
    strip = frame_bgr[y + h : bottom, max(0, x) : x + w]
    if strip.size == 0:
        return False
    value = strip.max(axis=2)
    table = float(np.median(value))
    dark = value <= params.beyond_dark_ratio * table
    tape_rows = dark.mean(axis=1) >= params.beyond_row_fraction
    run = longest = 0
    for is_tape in tape_rows:
        run = run + 1 if is_tape else 0
        longest = max(longest, run)
    return longest >= params.beyond_min_rows


def is_near_border(
    frame_bgr: np.ndarray,
    bbox: tuple[int, int, int, int],
    params: BorderParams,
) -> bool:
    """
    True when enough black pixels lie behind or beside the cube `bbox`
    (x, y, w, h), in a strip `rows_behind_ratio` cube heights tall above it
    and `side_ratio` cube widths wider on each side.
    """
    x, y, w, h = bbox
    height_px, width_px = frame_bgr.shape[:2]
    top = max(0, y - int(params.rows_behind_ratio * h))
    bottom = min(height_px, y + h)
    left = max(0, x - int(params.side_ratio * w))
    right = min(width_px, x + w + int(params.side_ratio * w))
    area = frame_bgr[top:bottom, left:right]
    if area.size == 0:
        return False
    dark = area.max(axis=2) <= params.max_value
    return bool(dark.mean() >= params.min_dark_fraction)


def without_cubes_near_border(
    frame_bgr: np.ndarray,
    detections: list,
    params: BorderParams,
) -> list:
    """
    The detections that are neither at the border (when `enabled`) nor
    beyond the line, outside the arena (when `beyond_line`).
    """

    def dropped(det) -> bool:
        if params.enabled and is_near_border(frame_bgr, det.bbox, params):
            return True
        return params.beyond_line and is_beyond_line(
            frame_bgr, det.bbox, params
        )

    return [d for d in detections if not dropped(d)]
