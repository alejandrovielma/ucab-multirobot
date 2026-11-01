"""
Tests: colour of the cube in the scoop from TCS34725 counts.
"""

from mobile_base.scoop_color import (
    NO_COLOR,
    ScoopColorParams,
    classify_rgbc,
    hue_distance_deg,
)

P = ScoopColorParams()


def test_hue_distance_wraps_around():
    assert hue_distance_deg(350.0, 10.0) == 20.0
    assert hue_distance_deg(0.0, 180.0) == 180.0


def test_classifies_saturated_colors():
    assert classify_rgbc(900, 100, 100, 1100, P) == "red"
    assert classify_rgbc(900, 800, 100, 1800, P) == "yellow"
    assert classify_rgbc(100, 900, 100, 1100, P) == "green"
    assert classify_rgbc(100, 300, 900, 1300, P) == "blue"


def test_red_near_360_is_still_red():
    assert classify_rgbc(900, 100, 250, 1250, P) == "red"


def test_no_cube_when_too_dark():
    assert classify_rgbc(50, 10, 10, 70, P) == NO_COLOR


def test_no_color_when_grey():
    assert classify_rgbc(500, 480, 470, 1500, P) == NO_COLOR


def test_white_balance_corrects_a_bluish_sensor():
    bluish = ScoopColorParams(white_balance=[1.0, 1.0, 2.0])
    # Raw counts look purple-ish; balanced they are red.
    assert classify_rgbc(900, 100, 200, 1200, bluish) == "red"
