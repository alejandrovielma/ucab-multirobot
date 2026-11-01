"""
Tests: HSV range parsing (no OpenCV needed).
"""

import pytest

from vision.hsv import HsvRange, parse_hsv_ranges


def test_parse_ranges_groups_of_six():
    r = parse_hsv_ranges([0, 1, 2, 10, 11, 12, 170, 1, 2, 179, 255, 255])
    assert r == [
        HsvRange((0, 1, 2), (10, 11, 12)),
        HsvRange((170, 1, 2), (179, 255, 255)),
    ]


@pytest.mark.parametrize(
    "bad",
    [
        [],
        [1, 2, 3],
        [10, 0, 0, 5, 255, 255],
        [0, 0, 0, 200, 255, 255],
        [-1, 0, 0, 5, 255, 255],
    ],
)
def test_parse_ranges_rejects_bad_input(bad):
    with pytest.raises(ValueError):
        parse_hsv_ranges(bad)
