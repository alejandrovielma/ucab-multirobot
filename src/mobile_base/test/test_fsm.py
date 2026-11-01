"""
Tests: state machine transitions (for now SEARCH / APPROACH / ARRIVED).
"""

import pytest

from mobile_base.approach import (
    ApproachController,
    ApproachParams,
    Command,
    Cube,
    State,
)


def ctrl(**kw):
    return ApproachController(
        ApproachParams(target_colors=["red", "orange"], **kw)
    )


def test_searches_turning_when_nothing_seen():
    c = ctrl()
    cmd = c.update([], 0.0)
    assert c.state == State.SEARCH
    assert cmd.vx == 0.0 and cmd.wz != 0.0


def test_ignores_colors_not_in_target():
    c = ctrl()
    c.update([Cube("blue", 0.0, 0.5)], 0.0)
    assert c.state == State.SEARCH


def test_picks_nearest_target_cube():
    c = ctrl()
    c.update(
        [
            Cube("red", 0.0, 0.9),
            Cube("green", 0.0, 0.2),
            Cube("orange", 0.1, 0.4),
        ],
        0.0,
    )
    assert c.state == State.APPROACH
    assert c.target == Cube("orange", 0.1, 0.4)


def test_approach_turns_toward_cube():
    c = ctrl()
    left = c.update([Cube("red", 0.2, 0.5)], 0.0)
    right = c.update([Cube("red", -0.2, 0.5)], 0.1)
    assert left.wz > 0 > right.wz
    assert left.vx > 0 and right.vx > 0


def test_turns_in_place_when_badly_aligned():
    c = ctrl()
    cmd = c.update([Cube("red", 0.8, 0.5)], 0.0)
    assert cmd.vx == 0.0 and cmd.wz > 0


def test_slows_down_near_the_cube():
    c = ctrl()
    far = c.update([Cube("red", 0.0, 0.8)], 0.0)
    near = c.update([Cube("red", 0.0, 0.2)], 0.1)
    assert far.vx > near.vx > 0


def test_stops_at_stop_distance_and_stays():
    c = ctrl(stop_distance_m=0.17, resume_margin_m=0.04)
    assert c.update([Cube("red", 0.0, 0.16)], 0.0) == Command()
    assert c.state == State.ARRIVED
    # small noise in the distance: stays still
    assert c.update([Cube("red", 0.0, 0.19)], 0.1) == Command()
    assert c.state == State.ARRIVED
    # the cube moved away: approach again
    assert c.update([Cube("red", 0.0, 0.3)], 0.2).vx > 0
    assert c.state == State.APPROACH


def test_brief_loss_waits_then_searches():
    c = ctrl(lost_timeout_s=1.0)
    c.update([Cube("red", -0.1, 0.5)], 0.0)
    assert c.update([], 0.5) == Command()
    assert c.state == State.APPROACH
    cmd = c.update([], 1.5)
    assert c.state == State.SEARCH
    assert cmd.wz < 0  # searches towards where it last saw it (right)


def test_disabled_does_not_move():
    c = ctrl()
    c.enabled = False
    assert c.update([Cube("red", 0.0, 0.5)], 0.0) == Command()
    assert c.state == State.PAUSED
    c.enabled = True
    c.update([Cube("red", 0.0, 0.5)], 0.1)
    assert c.state == State.APPROACH


def test_target_colors_can_change():
    c = ctrl()
    c.params.target_colors = ["blue"]
    c.update([Cube("red", 0.0, 0.5), Cube("blue", 0.0, 0.7)], 0.0)
    assert c.target.color == "blue"


@pytest.mark.parametrize("dist", [0.18, 0.3, 1.0])
def test_never_drives_backwards(dist):
    c = ctrl()
    assert c.update([Cube("red", 0.0, dist)], 0.0).vx >= 0
