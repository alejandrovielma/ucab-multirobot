"""
Debug-image drawing for detections. Needs OpenCV, no rclpy.

Used by the cube_detector node (debug image) and by the browser tool.
"""

import math

import cv2
import numpy as np

from vision.color_detection import Detection, Region
from vision.colors import DRAW_COLORS_BGR, FALLBACK_DRAW_COLOR_BGR

FONT = cv2.FONT_HERSHEY_SIMPLEX
CM_PER_M = 100

# --- Detection boxes ----------------------------------------------------------
LABEL_FONT_SCALE = 0.4
LABEL_THICKNESS = 1
LABEL_OUTLINE_THICKNESS = 3
LABEL_OUTLINE_BGR = (0, 0, 0)
# Gap (px) between a box and its label, and the lowest row a label may use.
LABEL_OFFSET_PX = 4
LABEL_MIN_Y_PX = 12
BOX_THICKNESS = 1
HIGHLIGHT_BOX_THICKNESS = 3
HIGHLIGHT_MARKER_BGR = (0, 0, 255)
HIGHLIGHT_MARKER_SIZE_PX = 14
HIGHLIGHT_MARKER_THICKNESS = 2

# --- Delivery line region -----------------------------------------------------
REGION_BGR = (255, 255, 255)
REGION_BOX_THICKNESS = 2
REGION_MARKER_SIZE_PX = 18
REGION_MARKER_THICKNESS = 2
REGION_FONT_SCALE = 0.5


def draw_detections(
    bgr: np.ndarray,
    detections: list[Detection],
    highlight: Detection | None = None,
) -> np.ndarray:
    """
    Copy of the image with a box and label per cube.

    `highlight` is drawn thicker and marked with a cross.
    """
    out = bgr.copy()
    for det in detections:
        x, y, w, h = det.bbox
        color = DRAW_COLORS_BGR.get(det.color, FALLBACK_DRAW_COLOR_BGR)
        thickness = (
            HIGHLIGHT_BOX_THICKNESS if det is highlight else BOX_THICKNESS
        )
        cv2.rectangle(out, (x, y), (x + w, y + h), color, thickness)
        label = (
            f"{det.color} {det.distance_m * CM_PER_M:.0f}cm "
            f"{math.degrees(det.bearing_rad):+.0f}"
        )
        _put_label(out, label, (x, y), color)
    if highlight is not None:
        cv2.drawMarker(
            out,
            (highlight.u, highlight.v),
            HIGHLIGHT_MARKER_BGR,
            cv2.MARKER_CROSS,
            HIGHLIGHT_MARKER_SIZE_PX,
            HIGHLIGHT_MARKER_THICKNESS,
        )
    return out


def draw_region(bgr: np.ndarray, region: Region) -> np.ndarray:
    """
    Draw the delivery-line region on `bgr` (in place) and return it.
    """
    x, y, w, h = region.bbox
    cv2.rectangle(
        bgr,
        (x, y),
        (x + w, y + h),
        REGION_BGR,
        REGION_BOX_THICKNESS,
    )
    cv2.drawMarker(
        bgr,
        (region.u, region.v),
        REGION_BGR,
        cv2.MARKER_CROSS,
        REGION_MARKER_SIZE_PX,
        REGION_MARKER_THICKNESS,
    )
    label = f"zone {region.color} {region.distance_m * CM_PER_M:.0f}cm"
    cv2.putText(
        bgr,
        label,
        (x, max(LABEL_MIN_Y_PX, y - LABEL_OFFSET_PX)),
        FONT,
        REGION_FONT_SCALE,
        REGION_BGR,
        LABEL_THICKNESS,
    )
    return bgr


def _put_label(
    image: np.ndarray,
    text: str,
    box_origin: tuple[int, int],
    color: tuple[int, int, int],
) -> None:
    """
    Write `text` just above a box with a dark outline for readability.
    """
    x, y = box_origin
    position = (x, max(LABEL_MIN_Y_PX, y - LABEL_OFFSET_PX))
    for text_color, thickness in (
        (LABEL_OUTLINE_BGR, LABEL_OUTLINE_THICKNESS),
        (color, LABEL_THICKNESS),
    ):
        cv2.putText(
            image,
            text,
            position,
            FONT,
            LABEL_FONT_SCALE,
            text_color,
            thickness,
        )
