"""Cube detection on a BGR image with OpenCV (kept apart from the node so it is unit-testable).

The white board and the black border line have low saturation, so a saturation/value
threshold keeps only painted cubes; shape filters reject elongated or tiny blobs.
"""

import math
from dataclasses import dataclass, field

import cv2
import numpy as np

from mobile_base.core.colors import DEFAULT_HUE_RANGES, DEFAULT_S_MIN, DEFAULT_V_MIN, HueRanges, classify


@dataclass(frozen=True)
class VisionParams:
    s_min: int = DEFAULT_S_MIN
    v_min: int = DEFAULT_V_MIN
    min_area: float = 150.0  # px
    max_area: float = 60000.0  # px
    aspect_min: float = 0.6
    aspect_max: float = 1.6
    min_solidity: float = 0.8
    open_kernel: int = 5  # px, morphological opening kernel size
    hue_ranges: HueRanges = field(default_factory=lambda: DEFAULT_HUE_RANGES)
    # Window between the scoop walls where a cube INSIDE the U appears (normalized like Detection).
    # Measured in Webots with calibracion_pala.wbt and the IMX219-120 camera: cube in the U -> 0.60
    # of the window, cube at the mouth -> 0.04. Re-measure on the real robot.
    scoop_x_half: float = 0.2
    scoop_y_min: float = 0.74
    scoop_y_max: float = 0.84
    scoop_min_fill: float = 0.30


@dataclass(frozen=True)
class Detection:
    color: str
    cx_norm: float  # [-1, 1], left negative
    cy_norm: float  # [0, 1], 0 = top
    area_px: float


def _dominant_hue(hues: np.ndarray) -> float:
    """Median hue that is robust to the red wrap-around at 0/179."""
    angles = hues.astype(np.float64) * (2.0 * math.pi / 180.0)
    ref = math.atan2(np.sin(angles).mean(), np.cos(angles).mean()) * 180.0 / (2.0 * math.pi)
    diffs = (hues.astype(np.float64) - ref + 90.0) % 180.0 - 90.0
    return float((ref + np.median(diffs)) % 180.0)


def detect_cubes(bgr: np.ndarray, params: VisionParams = VisionParams()) -> list[Detection]:
    height, width = bgr.shape[:2]
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = ((hsv[..., 1] > params.s_min) & (hsv[..., 2] > params.v_min)).astype(np.uint8) * 255
    kernel = np.ones((params.open_kernel, params.open_kernel), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    detections = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if not params.min_area <= area <= params.max_area:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        if not params.aspect_min <= w / h <= params.aspect_max:
            continue
        hull_area = cv2.contourArea(cv2.convexHull(contour))
        if hull_area <= 0.0 or area / hull_area < params.min_solidity:
            continue
        blob = np.zeros(mask.shape, np.uint8)
        cv2.drawContours(blob, [contour], -1, 255, thickness=cv2.FILLED)
        pixels = hsv[blob > 0]
        hue = round(_dominant_hue(pixels[:, 0])) % 180
        color = classify(
            hue,
            float(np.median(pixels[:, 1])),
            float(np.median(pixels[:, 2])),
            params.hue_ranges,
            params.s_min,
            params.v_min,
        )
        if color is None:
            continue
        cx = x + w / 2.0
        cy = y + h / 2.0
        detections.append(
            Detection(color, (cx / width) * 2.0 - 1.0, cy / height, float(area))
        )
    return detections


def scoop_content(bgr: np.ndarray, params: VisionParams = VisionParams()) -> tuple[str, float]:
    """Color that fills the scoop window, and its fill fraction ("" when below scoop_min_fill).

    A cube inside the U is seen from above as a wide, flat band cut by the scoop walls, so the
    blob shape filters of detect_cubes reject it; a fixed window does not depend on its shape and
    no cube farther away can appear there.
    """
    height, width = bgr.shape[:2]
    x0 = int((1.0 - params.scoop_x_half) / 2.0 * width)
    x1 = int((1.0 + params.scoop_x_half) / 2.0 * width)
    y0, y1 = int(params.scoop_y_min * height), int(params.scoop_y_max * height)
    hsv = cv2.cvtColor(bgr[y0:y1, x0:x1], cv2.COLOR_BGR2HSV)
    if hsv.size == 0:
        return "", 0.0
    hue, sat, val = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    painted = (sat > params.s_min) & (val > params.v_min)
    best, best_fill = "", 0.0
    for name, spans in params.hue_ranges.items():
        in_range = np.zeros(hue.shape, bool)
        for low, high in spans:
            in_range |= (hue >= low) & (hue <= high)
        fill = float((in_range & painted).mean())
        if fill > best_fill:
            best, best_fill = name, fill
    return (best if best_fill >= params.scoop_min_fill else ""), best_fill


_CHANNELS = {"bgr8": 3, "rgb8": 3, "bgra8": 4, "rgba8": 4}
_TO_BGR = {"rgb8": cv2.COLOR_RGB2BGR, "bgra8": cv2.COLOR_BGRA2BGR, "rgba8": cv2.COLOR_RGBA2BGR}


def image_to_bgr(height: int, width: int, encoding: str, step: int, data: bytes) -> np.ndarray:
    """Convert raw sensor_msgs/Image fields to a BGR array (avoids the cv_bridge dependency)."""
    if encoding not in _CHANNELS:
        raise ValueError(f"unsupported image encoding {encoding!r}")
    channels = _CHANNELS[encoding]
    rows = np.frombuffer(data, dtype=np.uint8).reshape(height, step)
    img = rows[:, : width * channels].reshape(height, width, channels)
    if encoding == "bgr8":
        return img.copy()
    return cv2.cvtColor(img, _TO_BGR[encoding])
