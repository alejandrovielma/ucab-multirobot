"""Regressions for stalls found by the closed-loop simulation (tools/sim/test_no_stall.py)."""
import random

import pytest

from mobile_base.core.behavior import BehaviorFSM, BehaviorInput, BehaviorParams, Cube, State

PHINEAS = ("red", "orange", "yellow")


def carrying_fsm(**overrides):
    """Camera carry (no pose) right after capturing a red cube."""
    fsm = BehaviorFSM(BehaviorParams(my_colors=PHINEAS, capture_time=0.1, use_lease=False, **overrides),
                      rng=random.Random(1))
    fsm.step(BehaviorInput(now=0.0, cubes=[Cube("red", 0.0, 0.3, 400.0)], line_front=False))
    fsm.step(BehaviorInput(now=0.1, cubes=[Cube("red", 0.0, 0.95, 3000.0)], line_front=False))
    fsm.step(BehaviorInput(now=0.5, cubes=[], line_front=False, scoop_color="red"))
    assert fsm.state is State.CARRY
    return fsm


def carry(fsm, now, cubes):
    return fsm.step(BehaviorInput(now=now, cubes=list(cubes), line_front=False, pose=None, scoop_color="red",
                                  square_visible=True, square_cx=0.0, square_cy=0.3, square_area=20000.0))


def test_the_sidestep_keeps_one_direction_when_the_nearest_obstacle_flips_sides():
    # Two cubes either side (seed blue 29): the nearest one, and the strafe sign, alternated every tick.
    fsm = carrying_fsm()
    left, right = Cube("green", -0.2, 0.52, 800.0), Cube("blue", 0.29, 0.65, 800.0)
    first = carry(fsm, 1.0, [left])
    assert first[1] == pytest.approx(-fsm.params.axis_strafe_speed)
    for i in range(1, 40):
        cmd = carry(fsm, 1.0 + i * 0.1, [right] if i % 2 else [left])
        assert cmd[1] == first[1]


def test_a_persistent_obstacle_ends_the_transport_within_dodge_time():
    fsm = carrying_fsm(dodge_time=6.0)
    t = 1.0
    while fsm.state is State.CARRY and t < 20.0:
        carry(fsm, t, [Cube("green", 0.1, 0.7, 800.0)])
        t += 0.1
    assert fsm.state is State.EXPLORE and "blocking" in fsm.note
    assert t - 1.0 <= fsm.params.dodge_time + 0.5


def test_a_cube_that_blinks_does_not_hold_the_robot_forever():
    # Episodes restart each time it blinks out: the episode count still ends the transport.
    fsm = carrying_fsm()
    cube = Cube("green", 0.1, 0.7, 800.0)
    for i in range(100):
        carry(fsm, 1.0 + i * 0.1, [] if i % 2 else [cube])
        if fsm.state is not State.CARRY:
            break
    assert fsm.state is State.EXPLORE
