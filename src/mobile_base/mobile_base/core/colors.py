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


def parse_colors(text: str, known: Iterable[str] = DEFAULT_HUE_RANGES) -> tuple[str, ...]:
    """Mission colors from "blue, green" (the ROS parameter and the lab tool): ("blue", "green").

    An unknown name raises ValueError instead of silently making a mission nobody can complete."""
    colors = tuple(c.strip().lower() for c in text.split(",") if c.strip())
    unknown = [c for c in colors if c not in set(known)]
    if unknown:
        raise ValueError(f"unknown color(s) {unknown}: use {', '.join(known)}")
    return colors


# Competition rule: only yellow and blue cubes are collected, and each robot is assigned ONE of them.
# The detector still sees every color (other cubes are obstacles to sidestep); this limits the mission only.
MISSION_COLORS = ("yellow", "blue")


def check_targets(targets: Iterable[str]) -> tuple[str, ...]:
    """The mission targets as a tuple, or ValueError unless empty (IDLE) or exactly one of MISSION_COLORS."""
    targets = tuple(targets)
    bad = [c for c in targets if c not in MISSION_COLORS]
    if bad:
        raise ValueError(f"target color(s) {bad} not allowed: each robot collects one color, yellow or blue")
    if len(targets) > 1:
        raise ValueError(f"a robot collects ONE color, got {list(targets)}")
    return targets
