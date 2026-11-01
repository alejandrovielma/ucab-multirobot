"""
HSV colour ranges and their flat-list parameter format. Pure logic, no
rclpy and no OpenCV.

OpenCV uses H 0-179 and S, V 0-255. A colour is described by one or more
ranges; red needs two because its hue wraps around (near 0 and near 179).
"""

from dataclasses import dataclass

# OpenCV channel limits.
HUE_MAX = 179
SATURATION_MAX = 255
VALUE_MAX = 255

# A range is stored as 6 flat numbers: (h, s, v) minimum then (h, s, v) maximum.
VALUES_PER_BOUND = 3
VALUES_PER_RANGE = 2 * VALUES_PER_BOUND


@dataclass(frozen=True)
class HsvRange:
    """
    One OpenCV HSV range (H 0-179, S and V 0-255).
    """

    lower: tuple[int, int, int]
    upper: tuple[int, int, int]


def parse_hsv_ranges(values: list[int] | tuple[int, ...]) -> list[HsvRange]:
    """
    Convert a flat list [h,s,v,h,s,v, ...] (multiple of 6) into ranges.

    Each group of 6 numbers is one range: minimum (h, s, v) and maximum
    (h, s, v). Red uses two ranges because the hue wraps around (near 0 and
    near 179).
    """
    values = [int(x) for x in values]
    if not values or len(values) % VALUES_PER_RANGE != 0:
        raise ValueError(
            f"expected groups of {VALUES_PER_RANGE} numbers "
            f"(h,s,v min and max), got {values}"
        )
    ranges = []
    for start in range(0, len(values), VALUES_PER_RANGE):
        middle = start + VALUES_PER_BOUND
        lower = tuple(values[start:middle])
        upper = tuple(values[middle : middle + VALUES_PER_BOUND])
        _check_range(lower, upper)
        ranges.append(HsvRange(lower, upper))
    return ranges


def _check_range(lower: tuple, upper: tuple) -> None:
    """
    Raise ValueError if the bounds are inverted or outside OpenCV limits.
    """
    if not all(a <= b for a, b in zip(lower, upper, strict=True)):
        raise ValueError(f"invalid HSV range: {lower} > {upper}")
    if min(lower) < 0:
        raise ValueError(f"invalid HSV range: {lower} is negative")
    hue, saturation, value = upper
    if hue > HUE_MAX or saturation > SATURATION_MAX or value > VALUE_MAX:
        raise ValueError(f"HSV range out of bounds: {upper}")
