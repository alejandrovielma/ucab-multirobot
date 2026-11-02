"""Think before acting: stop to verify a target cube, and scan slowly in place while exploring."""
import random

from mobile_base.core.behavior import ZERO, BehaviorFSM, BehaviorInput, BehaviorParams, Cube, State

YELLOW = Cube("yellow", 0.0, 0.3, 800.0)


def make_fsm(**overrides):
    params = BehaviorParams(my_colors=("yellow",), use_lease=False, **overrides)
    return BehaviorFSM(params, rng=random.Random(1))


def step(fsm, now, cubes=()):
    return fsm.step(BehaviorInput(now=now, cubes=list(cubes), line_front=False))


def test_a_seen_target_stops_the_robot_until_verify_time_then_it_approaches():
    fsm = make_fsm(verify_time=1.0)
    assert step(fsm, 0.0, [YELLOW]) == ZERO
    assert step(fsm, 0.5, [YELLOW]) == ZERO
    assert fsm.state is State.EXPLORE
    step(fsm, 1.1, [YELLOW])
    assert fsm.state is State.APPROACH


def test_a_target_that_vanishes_before_verify_time_is_not_approached():
    fsm = make_fsm(verify_time=1.0)
    step(fsm, 0.0, [YELLOW])
    cmd = step(fsm, 0.5)  # one-frame color: drive on
    assert fsm.state is State.EXPLORE and cmd[0] > 0.0
    step(fsm, 0.6, [YELLOW])  # seen again: the verification starts over
    step(fsm, 1.2, [YELLOW])
    assert fsm.state is State.EXPLORE


def test_without_verify_time_the_target_is_approached_at_once():
    fsm = make_fsm()
    step(fsm, 0.0, [YELLOW])
    assert fsm.state is State.APPROACH


def test_exploring_scans_a_full_slow_turn_then_drives_then_scans_again():
    fsm = make_fsm(scan_period=5.0, scan_turn_speed=0.5, scan_angle=1.0)  # 2 s per scan
    assert step(fsm, 0.0) == (0.0, 0.0, 0.5)
    assert step(fsm, 1.9) == (0.0, 0.0, 0.5)
    assert step(fsm, 2.1)[0] > 0.0  # scan done: drive
    assert step(fsm, 6.9)[0] > 0.0
    assert step(fsm, 7.2) == (0.0, 0.0, 0.5)  # 5 s later: scan again


def test_a_target_seen_while_scanning_stops_the_scan_to_verify_it():
    fsm = make_fsm(scan_period=5.0, scan_turn_speed=0.5, verify_time=1.0)
    step(fsm, 0.0)
    assert step(fsm, 0.5, [YELLOW]) == ZERO
    step(fsm, 1.6, [YELLOW])
    assert fsm.state is State.APPROACH
