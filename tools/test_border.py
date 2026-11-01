"""
Tests for the cube-at-the-border filter (run with: pytest tools). Frames are
synthetic: a white floor, a yellow cube and, sometimes, a black band.
"""

import numpy as np

from teleop_web_lib.border import (
    BorderParams,
    is_near_border,
    without_cubes_near_border,
)

FRAME_H, FRAME_W = 240, 320
CUBE = (150, 150, 20, 20)  # x, y, w, h
WHITE = 255
YELLOW_BGR = (0, 255, 255)
BLACK = 0


class FakeDetection:
    """
    Only the field the filter reads.
    """

    def __init__(self, bbox):
        self.bbox = bbox


def make_frame(band_rows=None):
    frame = np.full((FRAME_H, FRAME_W, 3), WHITE, np.uint8)
    x, y, w, h = CUBE
    frame[y : y + h, x : x + w] = YELLOW_BGR
    if band_rows is not None:
        frame[band_rows[0] : band_rows[1], :] = BLACK
    return frame


def test_cube_on_open_floor_is_not_at_the_border():
    assert not is_near_border(make_frame(), CUBE, BorderParams(enabled=True))


def test_black_line_just_behind_the_cube_is_the_border():
    frame = make_frame(band_rows=(128, 138))  # right behind the cube
    assert is_near_border(frame, CUBE, BorderParams(enabled=True))


def test_black_line_under_the_cube_is_the_border():
    frame = make_frame(band_rows=(158, 166))  # crosses the cube's lower part
    assert is_near_border(frame, CUBE, BorderParams(enabled=True))


def test_black_line_far_behind_the_cube_does_not_count():
    frame = make_frame(band_rows=(20, 30))
    assert not is_near_border(frame, CUBE, BorderParams(enabled=True))


def test_cube_cut_by_the_frame_edge_does_not_crash():
    frame = make_frame()
    assert not is_near_border(frame, (0, 0, 20, 20), BorderParams())
    assert not is_near_border(frame, (300, 230, 40, 40), BorderParams())


def test_filter_keeps_every_cube_when_disabled():
    frame = make_frame(band_rows=(128, 138))
    cubes = [FakeDetection(CUBE)]
    assert without_cubes_near_border(frame, cubes, BorderParams()) == cubes


def test_filter_drops_only_the_cube_at_the_border_when_enabled():
    frame = make_frame(band_rows=(128, 138))
    safe = FakeDetection((20, 200, 20, 20))  # far from the band
    at_border = FakeDetection(CUBE)
    kept = without_cubes_near_border(
        frame,
        [at_border, safe],
        BorderParams(enabled=True),
    )
    assert kept == [safe]
