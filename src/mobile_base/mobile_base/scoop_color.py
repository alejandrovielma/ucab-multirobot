"""
Colour of the cube in the scoop from TCS34725 counts. Pure logic, no rclpy.

The counts are white-balanced (divided by what the sensor reads on white
paper), turned into hue and saturation, and matched to the nearest
reference hue. Too little light (no cube) or too grey gives NO_COLOR.
"""

import colorsys
from dataclasses import dataclass, field

# Published when there is no cube or the colour is unclear.
NO_COLOR = ""
DEGREES_PER_TURN = 360.0
HALF_TURN_DEG = 180.0

# --- Defaults (real values: config/collector.yaml) ----------------------------
DEFAULT_COLOR_NAMES = ("red", "orange", "yellow", "green", "blue", "purple")
# Hue of each colour as the sensor sees it (TODO measure with the cubes).
DEFAULT_REFERENCE_HUES_DEG = (0.0, 25.0, 55.0, 120.0, 220.0, 290.0)
# Counts of each channel on white paper (TODO measure).
DEFAULT_WHITE_BALANCE = (1.0, 1.0, 1.0)
# Below this clear count nothing is in front of the sensor (TODO measure).
DEFAULT_MIN_CLEAR = 200
DEFAULT_MIN_SATURATION = 0.15
DEFAULT_MAX_HUE_ERROR_DEG = 30.0


@dataclass
class ScoopColorParams:
    """
    Calibration of the scoop colour classifier.

    Field names double as ROS parameter names (see recolector_common.params).
    """

    color_names: list[str] = field(
        default_factory=lambda: list(DEFAULT_COLOR_NAMES),
    )
    reference_hues_deg: list[float] = field(
        default_factory=lambda: list(DEFAULT_REFERENCE_HUES_DEG),
    )
    white_balance: list[float] = field(
        default_factory=lambda: list(DEFAULT_WHITE_BALANCE),
    )
    min_clear: int = DEFAULT_MIN_CLEAR
    min_saturation: float = DEFAULT_MIN_SATURATION
    max_hue_error_deg: float = DEFAULT_MAX_HUE_ERROR_DEG


def hue_distance_deg(a_deg: float, b_deg: float) -> float:
    """
    Shortest distance between two hues on the colour wheel (0..180 deg).
    """
    diff = abs(a_deg - b_deg) % DEGREES_PER_TURN
    return min(diff, DEGREES_PER_TURN - diff)


def classify_rgbc(
    red: int,
    green: int,
    blue: int,
    clear: int,
    p: ScoopColorParams,
) -> str:
    """
    Colour name of the cube in front of the sensor, or NO_COLOR.
    """
    if clear < p.min_clear:
        return NO_COLOR
    balanced = [
        c / w for c, w in zip((red, green, blue), p.white_balance, strict=True)
    ]
    peak = max(balanced)
    if peak <= 0:
        return NO_COLOR
    hue, saturation, _ = colorsys.rgb_to_hsv(*(c / peak for c in balanced))
    if saturation < p.min_saturation:
        return NO_COLOR
    hue_deg = hue * DEGREES_PER_TURN
    error_deg, name = min(
        (hue_distance_deg(hue_deg, ref), name)
        for name, ref in zip(
            p.color_names,
            p.reference_hues_deg,
            strict=True,
        )
    )
    return name if error_deg <= p.max_hue_error_deg else NO_COLOR
