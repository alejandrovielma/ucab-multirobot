"""
Cube-on-the-table filter for the browser tool. Pure NumPy, no OpenCV.

A cube lies on the white table, so the strip of image just below it is bright
and unsaturated. A yellow object in the background of the room (on a chair, a
shelf or the wall) has something dark or coloured below it. The camera model
computes a distance for any blob as if it stood on the floor, so such an
object looked like a cube 80 to 120 cm away: the robot drove toward the table
edge after it and never gave up searching.
"""

from dataclasses import dataclass

import numpy as np

# --- Defaults (real values: tools/teleop_web.yaml, camera.support) ------------
DEFAULT_ROWS_BELOW_RATIO = 0.6
DEFAULT_SIDE_RATIO = 0.5
DEFAULT_MIN_TABLE_FRACTION = 0.5
DEFAULT_MIN_VALUE = 150
DEFAULT_MAX_SATURATION = 90
# Fewer rows than this below the cube (it touches the image bottom): the
# table cannot be judged, so the cube is accepted.
MIN_STRIP_ROWS_PX = 2
MAX_CHANNEL = 255


@dataclass(frozen=True)
class SupportParams:
    """
    Settings of the cube-on-the-table filter; field names are the YAML keys.
    """

    enabled: bool = False
    rows_below_ratio: float = DEFAULT_ROWS_BELOW_RATIO
    side_ratio: float = DEFAULT_SIDE_RATIO
    min_table_fraction: float = DEFAULT_MIN_TABLE_FRACTION
    min_value: int = DEFAULT_MIN_VALUE
    max_saturation: int = DEFAULT_MAX_SATURATION


def is_on_table(
    frame_bgr: np.ndarray,
    bbox: tuple[int, int, int, int],
    params: SupportParams,
) -> bool:
    """
    True when the strip below the cube `bbox` (x, y, w, h) looks like the
    white table: at least `min_table_fraction` of its pixels have a value of
    `min_value` or more and a saturation of `max_saturation` or less.
    """
    x, y, w, h = bbox
    height_px, width_px = frame_bgr.shape[:2]
    top = y + h
    bottom = min(height_px, top + max(1, int(params.rows_below_ratio * h)))
    if bottom - top < MIN_STRIP_ROWS_PX:
        return True
    left = max(0, x - int(params.side_ratio * w))
    right = min(width_px, x + w + int(params.side_ratio * w))
    # 32 bits: (brightest - darkest) * 255 overflows 16 bits.
    strip = frame_bgr[top:bottom, left:right].astype(np.int32)
    brightest = strip.max(axis=2)
    darkest = strip.min(axis=2)
    saturation = (brightest - darkest) * MAX_CHANNEL // np.maximum(brightest, 1)
    table = (brightest >= params.min_value) & (
        saturation <= params.max_saturation
    )
    return bool(table.mean() >= params.min_table_fraction)


def without_cubes_off_table(
    frame_bgr: np.ndarray,
    detections: list,
    params: SupportParams,
) -> list:
    """
    The detections that stand on the table (all of them when the filter is
    off).
    """
    if not params.enabled:
        return list(detections)
    return [d for d in detections if is_on_table(frame_bgr, d.bbox, params)]
