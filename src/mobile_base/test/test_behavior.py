import math
import random

import pytest

from mobile_base.core.behavior import BehaviorFSM, BehaviorInput, BehaviorParams, Cube, State

PHINEAS = ("red", "orange", "yellow")


def make_fsm(**overrides):
    params = BehaviorParams(my_colors=PHINEAS, **overrides)
    return BehaviorFSM(params, rng=random.Random(42))


def tick(fsm, now, cubes=(), line_front=False, scoop=""):
    return fsm.step(BehaviorInput(now=now, cubes=list(cubes), line_front=line_front, scoop_color=scoop))


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


def test_line_backoff_reverses_before_spinning_without_pose():
    # Line sensor at the robot center: the body front is already past the tape when it fires.
    fsm = make_fsm(line_backoff=0.14, turn_min=1.0, turn_max=1.0)
    tick(fsm, 0.0)
    back = 0.14 / fsm.params.reverse_speed
    assert tick(fsm, 0.1, line_front=True) == pytest.approx((-fsm.params.reverse_speed, 0.0, 0.0))
    assert tick(fsm, 0.1 + back - 0.05)[0] == pytest.approx(-fsm.params.reverse_speed)
    vx, _, wz = tick(fsm, 0.1 + back + 0.05)
    assert vx == pytest.approx(0.0) and abs(wz) == pytest.approx(fsm.params.turn_speed)
    assert tick(fsm, 0.1 + back + 0.9)[2] != 0.0  # the full turn still runs after the reverse
    assert tick(fsm, 0.1 + back + 1.1)[0] == pytest.approx(fsm.params.explore_speed)


def test_border_before_the_cube_also_backs_off():
    fsm = make_fsm(line_backoff=0.14)
    tick(fsm, 0.0, cubes=[Cube("red", 0.0, 0.3, 400.0)])
    assert fsm.state is State.APPROACH
    cmd = tick(fsm, 0.1, cubes=[Cube("red", 0.0, 0.3, 400.0)], line_front=True)
    assert fsm.state is State.EXPLORE
    assert cmd == pytest.approx((-fsm.params.reverse_speed, 0.0, 0.0))


def test_zero_line_backoff_spins_at_once():
    fsm = make_fsm(line_backoff=0.0)
    tick(fsm, 0.0)
    vx, _, wz = tick(fsm, 0.1, line_front=True)
    assert vx == pytest.approx(0.0) and abs(wz) == pytest.approx(fsm.params.turn_speed)


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
    # The big foreign cube is off the scoop's path (a closer one in the path is tested below).
    cubes = [Cube("red", -0.5, 0.3, 100.0), Cube("orange", 0.5, 0.4, 900.0), Cube("blue", -0.8, 0.5, 5000.0)]
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
    tick(fsm, 1.2, scoop="red")
    assert fsm.state is State.CARRY
    assert fsm.carrying


def test_carry_never_reverses():
    fsm = make_fsm(capture_time=0.1)
    tick(fsm, 0.0, cubes=[Cube("red", 0.0, 0.3, 400.0)])
    tick(fsm, 0.1, cubes=[Cube("red", 0.0, 0.95, 3000.0)])
    tick(fsm, 0.5, scoop="red")
    assert fsm.state is State.CARRY
    for i in range(50):
        cmd = tick(fsm, 1.0 + i * 0.1, cubes=[Cube("blue", 0.0, 0.9, 900.0)], scoop="red")
        assert cmd[0] >= 0.0
        assert fsm.state is State.CARRY


NOT_RED = ("orange", "yellow", "green", "blue", "purple")


def not_red_fsm(**overrides):
    return BehaviorFSM(BehaviorParams(my_colors=NOT_RED, **overrides), rng=random.Random(42))


@pytest.mark.parametrize("cx, sign", [(0.1, 1.0), (-0.1, -1.0)])
def test_explore_turns_away_from_a_red_cube_in_the_path(cx, sign):
    fsm = not_red_fsm()
    vx, _, wz = tick(fsm, 0.0, cubes=[Cube("red", cx, 0.6, 900.0)])
    assert fsm.state is State.EXPLORE
    assert vx == pytest.approx(0.0)
    assert wz * sign > 0  # red on the right (cx > 0) -> counterclockwise, away from it


@pytest.mark.parametrize("cube", [Cube("red", 0.0, 0.3, 400.0), Cube("red", 0.6, 0.7, 900.0)])
def test_explore_ignores_a_red_cube_far_or_off_the_path(cube):
    fsm = not_red_fsm()
    assert tick(fsm, 0.0, cubes=[cube]) == pytest.approx((fsm.params.explore_speed, 0.0, 0.0))


def test_approach_skips_a_target_behind_a_red_cube():
    fsm = not_red_fsm(skip_time=3.0)
    target = Cube("blue", 0.0, 0.4, 400.0)
    tick(fsm, 0.0, cubes=[target])
    assert fsm.state is State.APPROACH
    vx, _, wz = tick(fsm, 0.1, cubes=[target, Cube("red", 0.1, 0.55, 600.0)])
    assert fsm.state is State.EXPLORE
    assert vx == pytest.approx(0.0) and wz > 0.0  # steering around the red cube
    tick(fsm, 0.2, cubes=[target])
    assert fsm.state is State.EXPLORE  # the target is skipped for skip_time
    tick(fsm, 3.2, cubes=[target])
    assert fsm.state is State.APPROACH


def test_approach_keeps_a_target_in_front_of_a_red_cube():
    fsm = not_red_fsm()
    target = Cube("green", 0.0, 0.55, 800.0)
    tick(fsm, 0.0, cubes=[target])
    tick(fsm, 0.1, cubes=[target, Cube("red", 0.1, 0.5, 600.0)])
    assert fsm.state is State.APPROACH


def test_explore_stops_dodging_a_red_cube_that_turns_with_the_robot():
    # Stuck in the scoop but not reported as scoop_color, or a misread part of the robot: it never
    # leaves the path, so after avoid_timeout it is ignored for skip_time instead of spinning forever.
    fsm = not_red_fsm(avoid_timeout=3.0, skip_time=3.0)
    stuck = [Cube("red", 0.1, 0.6, 900.0)]
    assert tick(fsm, 0.0, cubes=stuck)[2] > 0.0
    assert tick(fsm, 2.9, cubes=stuck)[2] > 0.0
    assert tick(fsm, 3.1, cubes=stuck) == pytest.approx((fsm.params.explore_speed, 0.0, 0.0))
    assert tick(fsm, 6.0, cubes=stuck) == pytest.approx((fsm.params.explore_speed, 0.0, 0.0))
    assert tick(fsm, 6.2, cubes=stuck)[2] > 0.0  # dodged again after skip_time


def test_red_cube_in_the_scoop_is_backed_off_then_turned_away():
    fsm = not_red_fsm(shed_distance=0.08, shed_confirm=0.3, turn_min=1.0, turn_max=1.0)
    tick(fsm, 0.0)
    back = 0.08 / fsm.params.reverse_speed
    assert tick(fsm, 0.1, scoop="red")[0] == pytest.approx(fsm.params.explore_speed)  # not confirmed yet
    assert tick(fsm, 0.4, scoop="red") == pytest.approx((-fsm.params.reverse_speed, 0.0, 0.0))
    assert fsm.state is State.EXPLORE
    # Still seen in the U while reversing: the reverse is not restarted (it would never end).
    assert tick(fsm, 0.4 + back - 0.05, scoop="red")[0] == pytest.approx(-fsm.params.reverse_speed)
    vx, _, wz = tick(fsm, 0.4 + back + 0.05)
    assert vx == pytest.approx(0.0) and abs(wz) == pytest.approx(fsm.params.turn_speed)
    assert tick(fsm, 0.4 + back + 1.1)[0] == pytest.approx(fsm.params.explore_speed)


def test_one_misread_frame_in_the_scoop_is_not_backed_off():
    # Red and orange are neighbor hues: a single red frame must not abort anything.
    fsm = not_red_fsm(shed_confirm=0.3)
    tick(fsm, 0.0, scoop="red")
    assert tick(fsm, 0.1)[0] == pytest.approx(fsm.params.explore_speed)
    assert tick(fsm, 0.35, scoop="red")[0] == pytest.approx(fsm.params.explore_speed)


def test_red_cube_in_the_scoop_on_the_line_reverses_at_least_line_backoff():
    # Line sensor at the robot center: the body is already past the tape, a short shed would spin on it.
    fsm = not_red_fsm(shed_distance=0.08, line_backoff=0.14, shed_confirm=0.3)
    tick(fsm, 0.0, scoop="red")
    tick(fsm, 0.1, scoop="red")
    # The line arrives together with the confirmation: the shed starts here, not the line turn.
    assert tick(fsm, 0.3, scoop="red", line_front=True)[0] == pytest.approx(-fsm.params.reverse_speed)
    back = 0.14 / fsm.params.reverse_speed
    assert tick(fsm, 0.3 + back - 0.05)[0] == pytest.approx(-fsm.params.reverse_speed)
    assert tick(fsm, 0.3 + back + 0.05)[0] == pytest.approx(0.0)


def test_red_cube_in_the_scoop_after_a_capture_is_not_carried():
    fsm = not_red_fsm(capture_time=0.5, shed_confirm=0.3)
    tick(fsm, 0.0, cubes=[Cube("yellow", 0.0, 0.3, 400.0)])
    tick(fsm, 0.1, cubes=[Cube("yellow", 0.0, 0.95, 3000.0)])
    assert fsm.state is State.CAPTURE
    tick(fsm, 0.3, scoop="red")
    cmd = tick(fsm, 0.7, scoop="red")
    assert fsm.state is State.EXPLORE
    assert not fsm.carrying
    assert cmd == pytest.approx((-fsm.params.reverse_speed, 0.0, 0.0))


def test_own_cube_in_the_scoop_is_not_backed_off():
    fsm = not_red_fsm()
    assert tick(fsm, 0.0, scoop="blue")[0] == pytest.approx(fsm.params.explore_speed)


def test_stop_outputs_zero():
    fsm = make_fsm()
    fsm.stop()
    assert tick(fsm, 0.0, cubes=[Cube("red", 0.0, 0.3, 400.0)]) == (0.0, 0.0, 0.0)
    assert fsm.state is State.STOP


def turn_until_straight(fsm, pose_at, start=0.2, dt=0.05, limit=10.0):
    """Feed poses that integrate the commanded rotation; return the total turned angle."""
    t, theta, turned = start, 0.0, 0.0
    while t < limit:
        vx, _, wz = fsm.step(BehaviorInput(now=t, cubes=[], line_front=False, pose=pose_at(theta)))
        if wz == 0.0:
            return turned
        theta += wz * dt
        turned += abs(wz * dt)
        t += dt
    raise AssertionError("never stopped turning")


def test_border_turn_with_pose_turns_far_enough_to_face_inward():
    # Webots run 4: a timed 55 deg turn left the front sensor beyond the tape and the robot drove off.
    for seed in range(20):
        fsm = BehaviorFSM(BehaviorParams(my_colors=PHINEAS), rng=random.Random(seed))
        fsm.step(BehaviorInput(now=0.0, cubes=[], line_front=False, pose=(0.0, 0.0, 0.0)))
        fsm.step(BehaviorInput(now=0.1, cubes=[], line_front=True, pose=(0.0, 0.0, 0.0)))
        turned = turn_until_straight(fsm, lambda th: (0.0, 0.0, th))
        assert math.radians(135) - 0.1 <= turned <= math.radians(225) + 0.1
