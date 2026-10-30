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


def tick_square(fsm, now, square_cy, cubes=(), scoop=""):
    # The line sensor reads the blue square as white: only the camera sees it.
    return fsm.step(
        BehaviorInput(
            now=now,
            cubes=list(cubes),
            line_front=False,
            scoop_color=scoop,
            square_visible=True,
            square_cx=0.0,
            square_cy=square_cy,
            square_area=5000.0,
        )
    )


def test_close_blue_square_is_a_border_while_exploring():
    fsm = make_fsm(square_border_cy=0.55, line_backoff=0.14)
    tick(fsm, 0.0)
    assert tick_square(fsm, 0.1, 0.6) == pytest.approx((-fsm.params.reverse_speed, 0.0, 0.0))


def test_far_blue_square_keeps_exploring():
    fsm = make_fsm(square_border_cy=0.55, line_backoff=0.14)
    tick(fsm, 0.0)
    assert tick_square(fsm, 0.1, 0.4) == pytest.approx((fsm.params.explore_speed, 0.0, 0.0))


def test_square_border_disabled_by_default():
    fsm = make_fsm(square_border_cy=0.0, line_backoff=0.14)
    tick(fsm, 0.0)
    assert tick_square(fsm, 0.1, 0.9) == pytest.approx((fsm.params.explore_speed, 0.0, 0.0))


def test_close_blue_square_is_a_border_while_approaching():
    fsm = make_fsm(square_border_cy=0.55, line_backoff=0.14)
    tick(fsm, 0.0, cubes=[Cube("red", 0.0, 0.3, 400.0)])
    assert fsm.state is State.APPROACH
    cmd = tick_square(fsm, 0.1, 0.6, cubes=[Cube("red", 0.0, 0.3, 400.0)])
    assert fsm.state is State.EXPLORE
    assert fsm.note == "blue square ahead"
    assert cmd == pytest.approx((-fsm.params.reverse_speed, 0.0, 0.0))


def test_blue_square_is_not_a_border_while_carrying():
    fsm = make_fsm(square_border_cy=0.55, line_backoff=0.14, capture_time=0.1)
    tick(fsm, 0.0, cubes=[Cube("red", 0.0, 0.3, 400.0)])
    tick(fsm, 0.1, cubes=[Cube("red", 0.0, 0.95, 3000.0)])
    tick(fsm, 0.5, scoop="red")
    assert fsm.state is State.CARRY
    plain = make_fsm(square_border_cy=0.55, line_backoff=0.14, capture_time=0.1)
    tick(plain, 0.0, cubes=[Cube("red", 0.0, 0.3, 400.0)])
    tick(plain, 0.1, cubes=[Cube("red", 0.0, 0.95, 3000.0)])
    tick(plain, 0.5, scoop="red")
    cmd = tick_square(fsm, 0.6, 0.9, scoop="red")
    # Carrying, the square is the destination (camera delivery), never a border to back off from.
    assert fsm.state in (State.CARRY, State.DELIVER)
    assert cmd[0] >= 0.0


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
def test_approach_spins_in_place_toward_cube(cx, sign):
    fsm = make_fsm()
    tick(fsm, 0.0, cubes=[Cube("red", cx, 0.3, 400.0)])
    cmd = tick(fsm, 0.1, cubes=[Cube("red", cx, 0.3, 400.0)])
    assert cmd[2] * sign > 0  # cube on the right (cx > 0) -> turn clockwise (wz < 0)
    assert cmd[0] == 0.0 and cmd[1] == 0.0  # star motion: no translation while turning


def test_approach_drives_straight_once_centered_with_hysteresis():
    fsm = make_fsm(approach_align=0.15)
    tick(fsm, 0.0, cubes=[Cube("red", 0.4, 0.3, 400.0)])
    assert tick(fsm, 0.1, cubes=[Cube("red", 0.1, 0.3, 400.0)])[2] != 0.0  # still above align / 2
    assert tick(fsm, 0.2, cubes=[Cube("red", 0.05, 0.3, 400.0)]) == pytest.approx((fsm.params.approach_speed, 0.0, 0.0))
    assert tick(fsm, 0.3, cubes=[Cube("red", 0.12, 0.3, 400.0)]) == pytest.approx((fsm.params.approach_speed, 0.0, 0.0))


def test_approach_spin_never_goes_below_the_motor_start_speed():
    fsm = make_fsm(approach_align=0.15, approach_turn_min=0.4)
    tick(fsm, 0.0, cubes=[Cube("red", 0.16, 0.3, 400.0)])
    assert abs(tick(fsm, 0.1, cubes=[Cube("red", 0.16, 0.3, 400.0)])[2]) == pytest.approx(0.4)


def test_a_cube_off_center_at_the_scoop_mouth_is_centered_before_the_capture():
    fsm = make_fsm()
    tick(fsm, 0.0, cubes=[Cube("red", 0.0, 0.3, 400.0)])
    tick(fsm, 0.1, cubes=[Cube("red", 0.5, 0.9, 3000.0)])
    assert fsm.state is State.APPROACH


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


# ---- Mission from the command interface (target colors; forbidden and neutral cubes are avoided)


def test_without_a_mission_the_robot_stays_still():
    fsm = BehaviorFSM(BehaviorParams(), rng=random.Random(1))
    assert fsm.state is State.IDLE
    assert tick(fsm, 0.0, cubes=[Cube("blue", 0.0, 0.5, 900.0)]) == (0.0, 0.0, 0.0)


def test_a_mission_starts_the_robot_and_clearing_it_stops_it():
    fsm = BehaviorFSM(BehaviorParams(), rng=random.Random(1))
    fsm.set_mission(["blue"], avoid=["red"])
    tick(fsm, 0.0, cubes=[Cube("blue", 0.0, 0.3, 400.0)])
    assert fsm.state is State.APPROACH
    fsm.set_mission([])
    assert tick(fsm, 0.1, cubes=[Cube("blue", 0.0, 0.3, 400.0)]) == (0.0, 0.0, 0.0)
    assert fsm.state is State.IDLE


def test_clearing_the_mission_while_carrying_releases_the_lease():
    fsm = BehaviorFSM(BehaviorParams(capture_time=0.1), rng=random.Random(1))
    fsm.set_mission(["blue"])
    tick(fsm, 0.0, cubes=[Cube("blue", 0.0, 0.3, 400.0)])
    tick(fsm, 0.1, cubes=[Cube("blue", 0.0, 0.95, 3000.0)])
    tick(fsm, 0.3, scoop="blue")
    assert fsm.state is State.CARRY
    fsm.set_mission([])
    fsm.step(BehaviorInput(now=0.4, cubes=[], line_front=False, holds_zone=True, scoop_color="blue"))
    assert fsm.state is State.IDLE
    assert [a.kind for a in fsm.take_actions()] == ["release"]


def test_only_target_colors_are_chased():
    fsm = BehaviorFSM(BehaviorParams(), rng=random.Random(1))
    fsm.set_mission(["green"], avoid=["red"])
    tick(fsm, 0.0, cubes=[Cube("yellow", 0.0, 0.3, 900.0)])  # neutral
    assert fsm.state is State.EXPLORE
    tick(fsm, 0.1, cubes=[Cube("green", 0.5, 0.3, 300.0)])
    assert fsm.state is State.APPROACH


def test_a_forbidden_cube_caught_in_the_scoop_is_counted():
    fsm = BehaviorFSM(BehaviorParams(shed_confirm=0.2), rng=random.Random(1))
    fsm.set_mission(["green"], avoid=["red"])
    tick(fsm, 0.0, scoop="red")
    tick(fsm, 0.3, scoop="red")
    assert fsm.forbidden_touches == 1
    neutral = BehaviorFSM(BehaviorParams(shed_confirm=0.2), rng=random.Random(1))
    neutral.set_mission(["green"], avoid=["red"])
    tick(neutral, 0.0, scoop="yellow")
    tick(neutral, 0.3, scoop="yellow")
    assert neutral.forbidden_touches == 0  # backed off too, but touching a neutral costs nothing


def test_a_target_that_keeps_hiding_behind_another_cube_is_reached_by_pushing_it_aside():
    # Rule: other cubes may be moved on the track when that is the only way to a target.
    fsm = BehaviorFSM(BehaviorParams(skip_time=1.0, clear_after=2, shed_confirm=0.2), rng=random.Random(1))
    fsm.set_mission(["blue"], avoid=["red"])
    target, obstacle = Cube("blue", 0.0, 0.4, 400.0), Cube("red", 0.05, 0.55, 600.0)
    t = 0.0
    for _ in range(2):  # skipped twice
        tick(fsm, t, cubes=[target])
        tick(fsm, t + 0.1, cubes=[target, obstacle])
        assert fsm.state is State.EXPLORE
        t += 2.0  # skip_time over
    tick(fsm, t, cubes=[target])
    cmd = tick(fsm, t + 0.1, cubes=[target, obstacle])
    assert fsm.state is State.APPROACH and cmd == pytest.approx((fsm.params.approach_speed, 0.0, 0.0))
    # The red cube ends up in the scoop: it is spun out (thrown aside), not backed off.
    tick(fsm, t + 0.2, cubes=[target], scoop="red")
    cmd = tick(fsm, t + 0.5, cubes=[target], scoop="red")
    assert fsm.state is State.EXPLORE
    assert cmd[0] == 0.0 and abs(cmd[2]) == pytest.approx(fsm.params.turn_speed)
    assert fsm.forbidden_touches == 1


def test_repeated_border_hits_in_a_corner_back_off_further_and_turn_around():
    fsm = make_fsm(line_backoff=0.1, corner_hits=3, corner_window=10.0, turn_min=0.5, turn_max=0.5)
    reverse = 0.1 / fsm.params.reverse_speed
    t = 0.0
    for _ in range(2):  # two ordinary border reactions
        tick(fsm, t, line_front=True)
        t += reverse + 0.6  # backoff and turn done
        tick(fsm, t)
    tick(fsm, t, line_front=True)  # third within the window: a corner
    assert "corner" in fsm.note
    assert tick(fsm, t + 2 * reverse - 0.05)[0] < 0.0  # still backing off: twice line_backoff
    spin = math.pi / fsm.params.turn_speed
    assert tick(fsm, t + 2 * reverse + spin - 0.05)[2] != 0.0  # a half turn
    assert tick(fsm, t + 2 * reverse + spin + 0.05)[0] == pytest.approx(fsm.params.explore_speed)


@pytest.mark.parametrize("seed", range(5))
def test_every_command_is_a_translation_or_a_spin_never_both(seed):
    # Team rule 2026-10-27: random inputs through every state must never yield an arc.
    rng = random.Random(seed)
    fsm = BehaviorFSM(BehaviorParams(capture_time=0.2, shed_confirm=0.1), rng=random.Random(seed))
    fsm.set_mission(["blue", "green"], avoid=["red"])
    colors = ["blue", "green", "red", "yellow"]
    for k in range(400):
        cubes = [Cube(rng.choice(colors), rng.uniform(-1, 1), rng.uniform(0, 1), rng.uniform(100, 5000))
                 for _ in range(rng.randint(0, 3))]
        vx, vy, wz = fsm.step(BehaviorInput(
            now=k * 0.1, cubes=cubes, line_front=rng.random() < 0.05, pose=None,
            holds_zone=rng.random() < 0.5, scoop_color=rng.choice(colors + ["", ""]),
        ))
        assert wz == 0.0 or (vx == 0.0 and vy == 0.0), (fsm.state, vx, vy, wz)


def test_a_lease_granted_after_the_mission_was_cleared_is_given_back():
    fsm = BehaviorFSM(BehaviorParams(request_period=1.0), rng=random.Random(1))
    fsm.step(BehaviorInput(now=0.0, cubes=[], line_front=False))
    fsm.step(BehaviorInput(now=2.0, cubes=[], line_front=False, holds_zone=True))
    assert [a.kind for a in fsm.take_actions()] == ["release"]
    fsm.step(BehaviorInput(now=2.5, cubes=[], line_front=False, holds_zone=True))
    assert fsm.take_actions() == []  # not every tick


def test_a_color_cannot_be_both_target_and_forbidden():
    fsm = BehaviorFSM(BehaviorParams(), rng=random.Random(1))
    with pytest.raises(ValueError):
        fsm.set_mission(["blue", "red"], avoid=["red"])
    assert fsm.targets == ()


def test_a_close_cube_off_the_scoop_axis_is_centered_by_sliding_not_spinning():
    fsm = BehaviorFSM(BehaviorParams(my_colors=("red",), approach_strafe_cy=0.45, approach_align=0.15))
    fsm.set_mission(("red",), ())
    fsm.step(BehaviorInput(now=0.0, cubes=[Cube("red", 0.0, 0.3, 800.0)], line_front=False))
    cmd = fsm.step(BehaviorInput(now=0.1, cubes=[Cube("red", 0.08, 0.55, 2500.0)], line_front=False))
    assert cmd == pytest.approx((0.0, -fsm.params.approach_strafe_speed, 0.0))  # cube to the right: slide right
    centered = fsm.step(BehaviorInput(now=0.2, cubes=[Cube("red", 0.01, 0.56, 2500.0)], line_front=False))
    assert centered[0] > 0.0 and centered[1] == 0.0


def test_a_target_that_turns_with_the_robot_is_left_behind():
    fsm = BehaviorFSM(BehaviorParams(my_colors=("red",), approach_spin_timeout=4.0, approach_align=0.15))
    fsm.set_mission(("red",), ())
    stuck = [Cube("red", -0.3, 0.7, 2000.0)]  # always at the same place in the image: it rides on the robot
    t = 0.0
    while fsm.state is not State.EXPLORE or t == 0.0:
        cmd = fsm.step(BehaviorInput(now=t, cubes=stuck, line_front=False))
        t += 0.1
        assert t < 6.0
    assert cmd[0] < 0.0 and "turns with the robot" in fsm.note
    assert fsm.step(BehaviorInput(now=t, cubes=stuck, line_front=False))[0] < 0.0  # still backing off


def test_a_cube_that_vanishes_at_the_mouth_while_driving_on_is_assumed_captured():
    fsm = BehaviorFSM(BehaviorParams(my_colors=("red",), assume_capture=True, capture_time=1.0))
    fsm.set_mission(("red",), ())
    fsm.step(BehaviorInput(now=0.0, cubes=[Cube("red", 0.0, 0.4, 900.0)], line_front=False))
    fsm.step(BehaviorInput(now=0.1, cubes=[Cube("red", 0.0, 0.7, 3000.0)], line_front=False))  # at the mouth
    assert fsm.state is State.CAPTURE
    for t in (0.5, 1.0):  # the plate hides it: nothing in the image, nothing in the scoop window
        fsm.step(BehaviorInput(now=t, cubes=[], line_front=False))
    fsm.step(BehaviorInput(now=1.3, cubes=[], line_front=False))
    assert fsm.state is State.CARRY and fsm.carrying
    assert fsm._carried_color == "red"


def test_without_a_sighting_at_the_mouth_nothing_is_assumed():
    fsm = BehaviorFSM(BehaviorParams(my_colors=("red",), assume_capture=True, capture_time=1.0))
    fsm.set_mission(("red",), ())
    fsm.step(BehaviorInput(now=0.0, cubes=[Cube("red", 0.0, 0.4, 900.0)], line_front=False))
    for t in (0.5, 1.0, 1.5, 2.0):
        fsm.step(BehaviorInput(now=t, cubes=[], line_front=False))
    assert fsm.state is not State.CARRY
