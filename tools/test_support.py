"""
Tests for the cube-on-the-table filter (run with: pytest tools). Frames are
synthetic: a white table, a yellow cube and, sometimes, something dark.
"""

import numpy as np

from teleop_web_lib.support import (
    SupportParams,
    is_on_table,
    without_cubes_off_table,
)

FRAME_H, FRAME_W = 240, 320
CUBE = (150, 100, 20, 20)  # x, y, w, h
WHITE_BGR = (205, 215, 215)  # the pale, slightly warm table
DARK_BGR = (40, 30, 30)
YELLOW_BGR = (0, 255, 255)


class FakeDetection:
    """
    Only the field the filter reads.
    """

    def __init__(self, bbox):
        self.bbox = bbox


def make_frame(below_bgr=WHITE_BGR):
    frame = np.full((FRAME_H, FRAME_W, 3), WHITE_BGR, np.uint8)
    x, y, w, h = CUBE
    frame[y : y + h, x : x + w] = YELLOW_BGR
    frame[y + h :, :] = below_bgr
    return frame


def test_a_cube_on_the_white_table_is_accepted():
    assert is_on_table(make_frame(), CUBE, SupportParams(enabled=True))


def test_a_yellow_object_with_something_dark_below_is_rejected():
    frame = make_frame(below_bgr=DARK_BGR)  # a chair or the room's dark floor
    assert not is_on_table(frame, CUBE, SupportParams(enabled=True))


def test_a_yellow_object_with_a_colourful_surface_below_is_rejected():
    frame = make_frame(below_bgr=(60, 60, 220))  # saturated red
    assert not is_on_table(frame, CUBE, SupportParams(enabled=True))


def test_a_cube_touching_the_image_bottom_is_accepted():
    frame = make_frame(below_bgr=DARK_BGR)
    at_bottom = (150, FRAME_H - 20, 20, 20)
    assert is_on_table(frame, at_bottom, SupportParams(enabled=True))


def test_the_filter_keeps_every_detection_when_disabled():
    frame = make_frame(below_bgr=DARK_BGR)
    cubes = [FakeDetection(CUBE)]
    assert without_cubes_off_table(frame, cubes, SupportParams()) == cubes


def test_the_filter_drops_only_what_is_off_the_table_when_enabled():
    frame = make_frame()
    frame[130:, 20:80] = DARK_BGR  # something dark below the left cube
    on_table = FakeDetection(CUBE)
    off_table = FakeDetection((40, 110, 20, 20))
    kept = without_cubes_off_table(
        frame,
        [off_table, on_table],
        SupportParams(enabled=True),
    )
    assert kept == [on_table]
