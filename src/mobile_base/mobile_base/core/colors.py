"""Hue-based cube color classification (OpenCV HSV convention: H 0-179, S and V 0-255)."""

from collections.abc import Iterable, Mapping, Sequence

HueRanges = Mapping[str, Sequence[tuple[int, int]]]

# Defaults tuned against the Webots cube colors; calibrate with the real cubes and room light.
DEFAULT_HUE_RANGES: dict[str, list[tuple[int, int]]] = {
    "red": [(0, 8), (169, 179)],
    "orange": [(9, 18)],
    "yellow": [(19, 34)],
    "green": [(35, 89)],
    "blue": [(90, 124)],
    "purple": [(125, 168)],
}
DEFAULT_S_MIN = 80
DEFAULT_V_MIN = 50


def classify(
    hue: float,
    saturation: float,
    value: float,
    ranges: HueRanges | None = None,
    s_min: float = DEFAULT_S_MIN,
    v_min: float = DEFAULT_V_MIN,
) -> str | None:
    """Return the color name, or None for white/gray (low S), black (low V) or unknown hue."""
    if saturation < s_min or value < v_min:
        return None
    for name, spans in (ranges or DEFAULT_HUE_RANGES).items():
        if any(low <= hue <= high for low, high in spans):
            return name
    return None


def ranges_from_flat(flat: Mapping[str, Sequence[int]]) -> dict[str, list[tuple[int, int]]]:
    """Convert ROS-parameter friendly lists [lo1, hi1, lo2, hi2, ...] into hue range pairs."""
    ranges = {}
    for name, values in flat.items():
        if len(values) % 2:
            raise ValueError(f"hue range for {name!r} needs an even number of values")
        ranges[name] = [(int(values[i]), int(values[i + 1])) for i in range(0, len(values), 2)]
    return ranges


def is_mine(color: str | None, my_colors: Iterable[str]) -> bool:
    return color is not None and color in set(my_colors)


# Color ownership from the team plan; the YAML config is the source of truth at runtime.
DEFAULT_ROBOT_COLORS: dict[str, list[str]] = {
    "phineas": ["red", "orange", "yellow"],
    "ferb": ["green", "blue", "purple"],
}
