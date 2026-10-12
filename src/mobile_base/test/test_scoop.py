"""Capture verified with the robot camera: a cube inside the U shows up as a band of its color in a
fixed window between the scoop walls (Webots run 7 counted 5 deliveries with 1 cube in the square,
because CAPTURE assumed success after pushing for 1 s)."""

import math
import pathlib
import random

import cv2
import numpy as np
import pytest

from mobile_base.core.behavior import BehaviorFSM, BehaviorInput, BehaviorParams, Cube, State, ZoneAction
from mobile_base.vision import VisionParams, scoop_content

FIXTURES = pathlib.Path(__file__).resolve().parent / "fixtures"
PHINEAS = ("red", "orange", "yellow")
SOUTH = -math.pi / 2


# ---- vision: Webots frames taken with calibracion_pala.wbt and the IMX219-120 camera


def test_webots_frame_with_cube_inside_the_u():
    color, fill = scoop_content(cv2.imread(str(FIXTURES / "webots_cube_in_scoop.jpg")), VisionParams())
    assert color == "red"
    assert fill > 0.5


def test_webots_frame_with_cube_only_at_the_mouth_is_not_captured():
    color, fill = scoop_content(cv2.imread(str(FIXTURES / "webots_cube_at_mouth.jpg")), VisionParams())
    assert color == ""
    assert fill < 0.1


def test_white_scoop_window_is_empty():
    color, fill = scoop_content(np.full((480, 640, 3), 255, np.uint8), VisionParams())
    assert (color, fill) == ("", 0.0)


# ---- behavior


def tick(fsm, now, cubes=(), scoop="", pose=None, holds_zone=False, line_front=False):
    return fsm.step(
        BehaviorInput(now=now, cubes=list(cubes), line_front=line_front, pose=pose, holds_zone=holds_zone,
                      scoop_color=scoop)
    )


def capture(fsm, scoop_at_end):
    tick(fsm, 0.0, cubes=[Cube("red", 0.0, 0.3, 400.0)])
    tick(fsm, 0.1, cubes=[Cube("red", 0.0, 0.7, 3000.0)])
    assert fsm.state is State.CAPTURE
    return tick(fsm, 1.2, scoop=scoop_at_end)


def make_fsm(**kw):
    return BehaviorFSM(BehaviorParams(my_colors=PHINEAS, capture_time=1.0, **kw), rng=random.Random(3))


def test_capture_confirmed_by_the_camera_goes_to_carry_with_the_seen_color():
    fsm = make_fsm()
    capture(fsm, scoop_at_end="orange")
    assert fsm.state is State.CARRY
    tick(fsm, 1.3, scoop="orange", pose=(-0.13, 0.08, SOUTH), holds_zone=True)
    tick(fsm, 1.4, scoop="orange", pose=(-0.13, -0.13, SOUTH), holds_zone=True)
    tick(fsm, 1.5, scoop="", pose=(-0.13, 0.03, SOUTH), holds_zone=True)
    assert ZoneAction("release", delivered=True, color="orange") in fsm.take_actions()


def test_capture_without_cube_in_the_scoop_retries_instead_of_carrying():
    fsm = make_fsm()
    capture(fsm, scoop_at_end="")
    assert fsm.state is State.APPROACH
    assert not fsm.carrying


def test_foreign_color_in_the_scoop_is_not_a_capture():
    fsm = make_fsm()
    capture(fsm, scoop_at_end="blue")
    assert fsm.state is not State.CARRY


def test_cube_lost_while_carrying_gives_up_and_frees_the_lease():
    fsm = make_fsm(scoop_lost_timeout=1.0)
    capture(fsm, scoop_at_end="red")
    fsm.take_actions()
    pose = (-0.6, 0.0, 0.0)
    tick(fsm, 2.0, scoop="red", pose=pose, holds_zone=True)
    tick(fsm, 2.5, scoop="", pose=pose, holds_zone=True)
    assert fsm.state is State.CARRY  # short dropouts are tolerated
    tick(fsm, 3.1, scoop="", pose=pose, holds_zone=True)
    assert fsm.state is State.EXPLORE
    assert fsm.take_actions() == [ZoneAction("release", delivered=False, color="red")]


def test_cube_still_in_the_scoop_after_reversing_is_not_counted():
    fsm = make_fsm()
    capture(fsm, scoop_at_end="red")
    fsm.take_actions()
    tick(fsm, 1.3, scoop="red", pose=(-0.13, 0.08, SOUTH), holds_zone=True)
    tick(fsm, 1.4, scoop="red", pose=(-0.13, -0.13, SOUTH), holds_zone=True)
    tick(fsm, 1.5, scoop="red", pose=(-0.13, 0.03, SOUTH), holds_zone=True)  # cube dragged back out
    assert fsm.take_actions() == [ZoneAction("release", delivered=False, color="red")]


@pytest.mark.parametrize("cy", [0.55, 0.61])
def test_capture_starts_only_when_the_cube_reaches_the_mouth(cy):
    fsm = make_fsm()
    tick(fsm, 0.0, cubes=[Cube("red", 0.0, 0.3, 400.0)])
    tick(fsm, 0.1, cubes=[Cube("red", 0.0, cy, 2000.0)])
    assert fsm.state is State.APPROACH
    tick(fsm, 0.2, cubes=[Cube("red", 0.0, 0.66, 3000.0)])  # measured mouth position (IMX219-120)
    assert fsm.state is State.CAPTURE
