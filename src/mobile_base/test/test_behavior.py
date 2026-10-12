import random

import pytest

from mobile_base.core.behavior import BehaviorFSM, BehaviorInput, BehaviorParams, Cube, State

PHINEAS = ("red", "orange", "yellow")


def make_fsm(**overrides):
    params = BehaviorParams(my_colors=PHINEAS, **overrides)
    return BehaviorFSM(params, rng=random.Random(42))


def tick(fsm, now, cubes=(), line_front=False):
    return fsm.step(BehaviorInput(now=now, cubes=list(cubes), line_front=line_front))


def test_starts_exploring_forward():
    fsm = make_fsm()
    cmd = tick(fsm, 0.0)
    assert fsm.state is State.EXPLORE
    assert cmd[0] == pytest.approx(fsm.params.explore_speed)
    assert cmd[2] == pytest.approx(0.0)


def test_border_triggers_turn_in_place_then_resumes():
    fsm = make_fsm(turn_min=1.0, turn_max=1.0)
    tick(fsm, 0.0)
    cmd = tick(fsm, 0.1, line_front=True)
    assert cmd[0] == pytest.approx(0.0)
    assert abs(cmd[2]) == pytest.approx(fsm.params.turn_speed)
    cmd = tick(fsm, 0.6)  # still turning even if the line is gone
    assert cmd[0] == pytest.approx(0.0) and cmd[2] != 0.0
    cmd = tick(fsm, 1.2)
    assert cmd[0] == pytest.approx(fsm.params.explore_speed)
    assert cmd[2] == pytest.approx(0.0)


def test_turn_duration_is_within_bounds_and_seeded():
    durations = []
    for _ in range(2):
        fsm = make_fsm(turn_min=0.5, turn_max=1.5)
        tick(fsm, 0.0, line_front=True)
        t = 0.0
        while tick(fsm, t)[2] != 0.0:
            t += 0.01
        durations.append(t)
    assert 0.5 <= durations[0] <= 1.51
    assert durations[0] == pytest.approx(durations[1])


def test_own_color_cube_triggers_approach():
    fsm = make_fsm()
    tick(fsm, 0.0, cubes=[Cube("red", 0.0, 0.3, 400.0)])
    assert fsm.state is State.APPROACH


def test_foreign_color_cube_is_ignored():
    fsm = make_fsm()
    tick(fsm, 0.0, cubes=[Cube("blue", 0.0, 0.3, 400.0)])
    assert fsm.state is State.EXPLORE


@pytest.mark.parametrize("cx, sign", [(0.5, -1.0), (-0.5, 1.0)])
def test_approach_steers_toward_cube(cx, sign):
    fsm = make_fsm()
    tick(fsm, 0.0, cubes=[Cube("red", cx, 0.3, 400.0)])
    cmd = tick(fsm, 0.1, cubes=[Cube("red", cx, 0.3, 400.0)])
    assert cmd[2] * sign > 0  # cube on the right (cx > 0) -> turn clockwise (wz < 0)
    assert 0.0 < cmd[0] < fsm.params.approach_speed


def test_approach_targets_largest_own_cube():
    fsm = make_fsm()
    cubes = [Cube("red", -0.5, 0.3, 100.0), Cube("orange", 0.5, 0.4, 900.0), Cube("blue", 0.0, 0.5, 5000.0)]
    tick(fsm, 0.0, cubes=cubes)
    cmd = tick(fsm, 0.1, cubes=cubes)
    assert cmd[2] < 0  # steering toward the orange cube on the right


def test_lost_target_returns_to_explore():
    fsm = make_fsm(lost_timeout=0.5)
    tick(fsm, 0.0, cubes=[Cube("red", 0.0, 0.3, 400.0)])
    tick(fsm, 0.3)
    assert fsm.state is State.APPROACH
    tick(fsm, 0.7)
    assert fsm.state is State.EXPLORE


def test_close_cube_triggers_capture_then_carry():
    fsm = make_fsm(capture_time=1.0)
    tick(fsm, 0.0, cubes=[Cube("red", 0.0, 0.3, 400.0)])
    cmd = tick(fsm, 0.1, cubes=[Cube("red", 0.0, 0.95, 3000.0)])
    assert fsm.state is State.CAPTURE
    assert cmd[0] == pytest.approx(fsm.params.capture_speed)
    tick(fsm, 0.6)
    assert fsm.state is State.CAPTURE
    tick(fsm, 1.2)
    assert fsm.state is State.CARRY
    assert fsm.carrying


def test_carry_never_reverses():
    fsm = make_fsm(capture_time=0.1)
    tick(fsm, 0.0, cubes=[Cube("red", 0.0, 0.3, 400.0)])
    tick(fsm, 0.1, cubes=[Cube("red", 0.0, 0.95, 3000.0)])
    tick(fsm, 0.5)
    assert fsm.state is State.CARRY
    for i in range(50):
        cmd = tick(fsm, 1.0 + i * 0.1, cubes=[Cube("blue", 0.0, 0.9, 900.0)], line_front=i % 2 == 0)
        assert cmd[0] >= 0.0
        assert fsm.state is State.CARRY


def test_stop_outputs_zero():
    fsm = make_fsm()
    fsm.stop()
    assert tick(fsm, 0.0, cubes=[Cube("red", 0.0, 0.3, 400.0)]) == (0.0, 0.0, 0.0)
    assert fsm.state is State.STOP
