"""
Cube-at-the-border filter for the browser tool. Pure NumPy, no OpenCV.

The scoop only pushes forward, so a cube with the black border line just
behind it (or under it) cannot be brought back: catching it shoves it over
the line for good. Such cubes are dropped from the detections.

The test looks for black tape in a strip behind the cube and beside it. It
is OFF by default: its thresholds must be tuned on real frames first, or it
could hide valid cubes (a dark background behind a far cube looks the same).
"""

from dataclasses import dataclass

import numpy as np

# --- Defaults (real values: tools/teleop_web.yaml, camera.border) -------------
DEFAULT_MAX_VALUE = 70
DEFAULT_ROWS_BEHIND_RATIO = 1.5
DEFAULT_SIDE_RATIO = 1.0
DEFAULT_MIN_DARK_FRACTION = 0.12


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
    The detections that are not at the border (all of them when the filter
    is off).
    """
    if not params.enabled:
        return list(detections)
    return [
        d for d in detections if not is_near_border(frame_bgr, d.bbox, params)
    ]
