import pytest

from mobile_base.core.colors import (
    DEFAULT_HUE_RANGES,
    classify,
    is_mine,
    ranges_from_flat,
)


@pytest.mark.parametrize(
    "hue, expected",
    [
        (2, "red"),
        (176, "red"),
        (13, "orange"),
        (25, "yellow"),
        (74, "green"),
        (112, "blue"),
        (162, "purple"),
    ],
)
def test_classify_each_color(hue, expected):
    assert classify(hue, 200, 200) == expected


def test_low_saturation_is_white_or_gray():
    assert classify(0, 10, 250) is None


def test_low_value_is_black():
    assert classify(112, 200, 10) is None


def test_custom_ranges_are_used():
    assert classify(50, 200, 200, ranges={"green": [(40, 60)]}) == "green"
    assert classify(10, 200, 200, ranges={"green": [(40, 60)]}) is None


def test_ranges_from_flat_pairs():
    ranges = ranges_from_flat({"red": [0, 8, 169, 179], "blue": [90, 124]})
    assert ranges == {"red": [(0, 8), (169, 179)], "blue": [(90, 124)]}
    with pytest.raises(ValueError):
        ranges_from_flat({"red": [0, 8, 169]})


def test_default_ranges_cover_all_six_colors():
    assert set(DEFAULT_HUE_RANGES) == {"red", "orange", "yellow", "green", "blue", "purple"}


def test_is_mine():
    assert is_mine("red", ["red", "orange", "yellow"])
    assert not is_mine("blue", ["red", "orange", "yellow"])
    assert not is_mine(None, ["red"])
