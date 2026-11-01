"""Regressions for stalls found by the closed-loop simulation (tools/sim/test_no_stall.py)."""
import math
import random

import pytest

from mobile_base.core.behavior import BehaviorFSM, BehaviorInput, BehaviorParams, Cube, State

PHINEAS = ("red", "orange", "yellow")


def carrying_fsm(**overrides):
    """Camera carry (no pose) right after capturing a red cube."""
    fsm = BehaviorFSM(BehaviorParams(my_colors=PHINEAS, capture_distance=0.008, use_lease=False, **overrides),
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


def exploring_fsm(**overrides):
    fsm = BehaviorFSM(BehaviorParams(my_colors=PHINEAS, use_lease=False, **overrides), rng=random.Random(1))
    fsm.set_mission(PHINEAS, ("blue",))
    return fsm


def explore(fsm, now, cubes, goal=(0.0, 0.0), est=(0.0, 0.3, 0.0), tape=None):
    """Guided exploration (estimated pose): the goal is behind the robot's left, past a blue cube."""
    return fsm.step(BehaviorInput(now=now, cubes=list(cubes), line_front=False, pose=None, est_pose=est,
                                  explore_goal=goal, tape_ahead=tape))


def test_avoiding_a_cube_and_heading_for_the_goal_do_not_alternate_forever():
    # Seed yellow 6: the avoid spin (cx<0: clockwise) clears the cube, the guided spin (goal behind it:
    # counterclockwise) brings it back into the path, and avoid_timeout never accumulates because the
    # episode restarts every time the cube leaves the path. The spin direction flipped every ~0.5 s.
    fsm = exploring_fsm()
    path, aside = Cube("blue", -0.15, 0.51, 800.0), Cube("blue", -0.44, 0.51, 800.0)
    turns = []
    for i in range(300):
        cubes = [path] if (i // 5) % 2 == 0 else [aside]
        turns.append(explore(fsm, 1.0 + i * 0.1, cubes, goal=(-1.0, 1.0), est=(0.0, 0.0, 0.0))[2])
    # After the bound, the robot must stop spinning (drive straight on the clear heading) at least in the
    # ticks where the cube is out of the path.
    assert any(w == 0.0 for w in turns[100:]), "never stopped alternating the two spins"
    flips = sum(1 for a, b in zip(turns, turns[1:]) if a * b < 0)
    assert flips <= 8


def approaching_fsm(**overrides):
    fsm = BehaviorFSM(BehaviorParams(my_colors=PHINEAS, use_lease=False, **overrides), rng=random.Random(1))
    fsm.set_mission(PHINEAS, ("blue",))
    return fsm


def approach(fsm, now, cube, line_front=False, scoop=""):
    return fsm.step(BehaviorInput(now=now, cubes=[cube] if cube else [], line_front=line_front, pose=None,
                                  scoop_color=scoop))


def test_a_cube_that_keeps_slipping_out_of_the_scoop_is_given_up_after_capture_fails():
    # Scoop never holds it, the cube stays at the mouth: APPROACH <-> CAPTURE rebounded forever.
    fsm = approaching_fsm()
    cube = Cube("red", 0.0, 0.7, 3000.0)
    states, t = [], 0.0
    while t < 20.0:
        approach(fsm, t, cube)
        states.append((t, fsm.state))
        t += 0.1
    left = next(t for t, s in states if s is State.EXPLORE)
    assert left < 3 * (fsm.params.capture_distance / fsm.params.capture_speed + 0.3)
    assert all(s is State.EXPLORE for t, s in states if left <= t < left + fsm.params.skip_time - 0.2)


def test_the_approach_strafe_stops_after_approach_strafe_time():
    # The cube sits just beyond approach_strafe_tol (it never centers): the slide must not last forever.
    fsm = approaching_fsm(approach_strafe_cy=0.45, approach_strafe_time=3.0)
    cube = Cube("red", 0.06, 0.5, 3000.0)
    slide, t = 0.0, 0.0
    while t < 10.0:
        cmd = approach(fsm, t, cube)
        slide += 0.1 if cmd[1] != 0.0 else 0.0
        t += 0.1
    assert 0.0 < slide <= 3.0 + 0.2


def test_a_cube_abandoned_at_the_border_is_not_approached_again_at_once():
    fsm = approaching_fsm()
    cube = Cube("red", 0.0, 0.5, 3000.0)
    approach(fsm, 0.0, cube)
    assert fsm.state is State.APPROACH
    approach(fsm, 0.1, cube, line_front=True)
    assert fsm.state is State.EXPLORE
    for i in range(2, 25):  # up to ~2.4 s: inside skip_time
        approach(fsm, 0.1 * i, cube)
        assert fsm.state is not State.APPROACH


@pytest.mark.parametrize("turn_speed", [1.2, 0.5])
def test_the_blind_border_turn_covers_the_same_angle_at_any_turn_speed(turn_speed):
    # The lab caps the spin at 0.5 rad/s; a turn timed in seconds then covered 23-57 deg instead of 55-137.
    fsm = BehaviorFSM(BehaviorParams(my_colors=PHINEAS, use_lease=False, turn_speed=turn_speed,
                                     blind_turn_min=1.0, blind_turn_max=1.0), rng=random.Random(1))
    fsm.step(BehaviorInput(now=0.0, cubes=[], line_front=False))
    fsm.step(BehaviorInput(now=0.1, cubes=[], line_front=True))
    t = 0.1
    while fsm.step(BehaviorInput(now=t, cubes=[], line_front=False))[2] != 0.0:
        t += 0.01
    assert (t - 0.1) * turn_speed == pytest.approx(1.0, abs=0.05)


def test_the_turn_away_from_the_estimated_tape_keeps_one_direction():
    # Seed yellow 33 at the lab caps: the goal is past the tape and just outside explore_heading_tol, so
    # toward-the-goal (err -0.32) and the turn-away override (+pi/2) alternated every tick and the slew
    # limit left the robot rocking in place for minutes.
    fsm = exploring_fsm(turn_speed=0.5)
    goal, th, signs = (0.0, 1.0), math.pi / 2 - 0.32, []  # bearing to the goal: pi/2
    for i in range(120):
        wz = explore(fsm, 1.0 + i * 0.1, [], goal=goal, est=(0.0, 0.0, th), tape=0.1)[2]
        th += wz * 0.1
        signs.append(wz)
    assert sum(1 for a, b in zip(signs, signs[1:]) if a * b < 0) <= 1


@pytest.mark.parametrize("capture_speed", [0.08, 0.05])
def test_the_capture_push_covers_its_distance_at_any_capture_speed(capture_speed):
    # Timed in seconds the lab-capped push (0.05 m/s) was 5 cm against a U 4.8 cm deep, 8 cm at the YAML speed.
    fsm = approaching_fsm(capture_speed=capture_speed, capture_distance=0.08)
    approach(fsm, 0.0, Cube("red", 0.0, 0.4, 900.0))
    approach(fsm, 0.1, Cube("red", 0.0, 0.7, 3000.0))
    assert fsm.state is State.CAPTURE
    t = 0.1
    while fsm.state is State.CAPTURE and t < 5.0:
        t += 0.01
        approach(fsm, t, None, scoop="red")
    assert (t - 0.1) * capture_speed == pytest.approx(0.08, abs=0.001)


def test_the_square_probe_covers_its_distance_at_the_carry_speed():
    fsm = carrying_fsm(square_search_time=1.0, square_probe_distance=0.12, carry_speed=0.05)

    def look(now):
        return fsm.step(BehaviorInput(now=now, cubes=[], line_front=False, pose=None, scoop_color="red"))

    look(1.0)
    assert look(2.1)[0] == pytest.approx(0.05)  # probe starts: 0.12 / 0.05 = 2.4 s
    assert look(4.4)[0] == pytest.approx(0.05)
    assert look(4.6) == pytest.approx((0.0, 0.0, fsm.params.carry_max_turn))


# ---- Behavior audit (2026-10-31): episode state, assumed captures, border reactions

AT_MOUTH = Cube("red", 0.0, 0.7, 3000.0)


def assuming_fsm(**overrides):
    fsm = BehaviorFSM(BehaviorParams(my_colors=PHINEAS, assume_capture=True, use_lease=False, **overrides),
                      rng=random.Random(1))
    fsm.set_mission(PHINEAS, ("blue",))
    return fsm


def tick(fsm, now, cubes=(), **kw):
    return fsm.step(BehaviorInput(now=now, cubes=list(cubes), line_front=kw.pop("line_front", False), pose=None, **kw))


def test_one_dropped_frame_in_capture_is_not_taken_for_a_capture():
    fsm = assuming_fsm()
    tick(fsm, 0.0, [Cube("red", 0.0, 0.4, 900.0)])
    tick(fsm, 0.1, [AT_MOUTH])
    assert fsm.state is State.CAPTURE
    tick(fsm, 0.2)  # a single frame without the cube
    for i in range(3, 12):
        tick(fsm, 0.1 * i, [AT_MOUTH])
    tick(fsm, 1.2, [AT_MOUTH])  # the push is over and the cube is still in view: it was never captured
    assert not fsm.carrying and "capture failed" in fsm.note


def deliver_after_assumed_capture(fsm):
    tick(fsm, 0.0, [Cube("red", 0.0, 0.4, 900.0)])
    tick(fsm, 0.1, [AT_MOUTH])
    t = 0.2
    while fsm.state is State.CAPTURE and t < 5.0:  # the plate hides the cube from the camera
        tick(fsm, t)
        t += 0.1
    assert fsm.state is State.CARRY
    while fsm.state is not State.EXPLORE and t < 40.0:
        tick(fsm, t, square_visible=True, square_cx=0.0, square_cy=0.8)
        t += 0.1


def test_a_delivery_after_an_assumed_capture_counts():
    # scoop_color is the substituted color all along, so "the cube left the scoop" was never true.
    fsm = assuming_fsm()
    deliver_after_assumed_capture(fsm)
    assert fsm.delivered == 1 and fsm.note == "delivered"


def test_the_delivery_axis_is_forgotten_after_a_sidestep():
    # A 6 s sidestep moves the robot off the axis: it must line up again instead of driving straight on.
    fsm = carrying_fsm(axis_delivery=True)
    on_axis = dict(refs_visible=True, refs_cx=0.0, refs_skew=0.0, square_visible=True, square_cx=0.0, square_cy=0.3)
    fsm.step(BehaviorInput(now=1.0, cubes=[], line_front=False, pose=None, scoop_color="red", **on_axis))
    fsm.step(BehaviorInput(now=1.1, cubes=[Cube("green", 0.1, 0.7, 800.0)], line_front=False, pose=None,
                           scoop_color="red", **on_axis))
    cmd = fsm.step(BehaviorInput(now=1.2, cubes=[], line_front=False, pose=None, scoop_color="red",
                                 square_visible=True, square_cx=0.5, square_cy=0.3))
    assert cmd[0] == 0.0 and cmd[2] != 0.0


def test_a_new_approach_does_not_inherit_the_centering_timer_of_an_old_one():
    fsm = approaching_fsm(approach_spin_timeout=4.0)
    off = Cube("red", 0.6, 0.3, 3000.0)
    approach(fsm, 0.0, off)  # centering by spinning starts
    for i in range(1, 25):  # lost for 2.4 s: back to EXPLORE
        approach(fsm, 0.1 * i, None)
    assert fsm.state is State.EXPLORE
    approach(fsm, 10.0, off)
    assert fsm.state is State.APPROACH and "turns with the robot" not in fsm.note


def test_a_new_transport_does_not_inherit_the_square_search_timer():
    fsm = carrying_fsm()
    fsm.step(BehaviorInput(now=1.0, cubes=[], line_front=False, pose=None, scoop_color="red"))  # searching
    fsm.step(BehaviorInput(now=1.1, cubes=[], line_front=True, pose=None, scoop_color="red"))
    assert fsm.state is State.EXPLORE
    fsm.state = State.CARRY  # next transport, long after
    cmd = fsm.step(BehaviorInput(now=100.0, cubes=[], line_front=False, pose=None, scoop_color="red"))
    assert cmd[0] == 0.0 and cmd[2] != 0.0  # spins to look, no forward probe


def test_an_obstacle_seen_right_after_a_border_turn_is_still_avoided():
    fsm = exploring_fsm(turn_speed=0.5, line_backoff=0.14)
    obstacle = Cube("blue", 0.1, 0.6, 800.0)
    tick(fsm, 0.0, [obstacle])  # the avoid spin starts
    tick(fsm, 0.2, [obstacle], line_front=True)  # reverse, then a long turn (cubes are not looked at meanwhile)
    t = 0.3
    cmd = tick(fsm, t, [obstacle])
    while fsm._turn_until is not None and t < 30.0:
        t += 0.1
        cmd = tick(fsm, t, [obstacle])
    assert t > 3.0 and cmd[0] == 0.0 and cmd[2] != 0.0  # the first look after the turn: avoid it, do not ignore it


def test_clearing_the_mission_resets_the_dodge_state():
    fsm = carrying_fsm()
    carry(fsm, 1.0, [Cube("green", 0.1, 0.7, 800.0)])
    assert fsm._dodging
    fsm.set_mission((), ())
    carry(fsm, 1.1, [])
    assert fsm.state is State.IDLE and fsm._dodges == 0 and not fsm._dodging


def test_a_border_during_capture_backs_off_before_turning():
    fsm = approaching_fsm(line_backoff=0.14)
    approach(fsm, 0.0, Cube("red", 0.0, 0.4, 900.0))
    approach(fsm, 0.1, AT_MOUTH)
    assert fsm.state is State.CAPTURE
    cmd = approach(fsm, 0.2, AT_MOUTH, line_front=True)
    assert fsm.state is State.EXPLORE and cmd[0] < 0.0


def test_the_line_while_carrying_backs_off_before_turning():
    fsm = carrying_fsm(line_backoff=0.14)
    cmd = fsm.step(BehaviorInput(now=1.0, cubes=[], line_front=True, pose=None, scoop_color="red"))
    assert "line while carrying" in fsm.note and cmd[0] < 0.0


def test_giving_up_a_cube_after_repeated_failed_captures_backs_off_and_turns_away():
    fsm = approaching_fsm()
    left, t = None, 0.0
    while t < 20.0:
        cmd = approach(fsm, t, AT_MOUTH)
        if fsm.state is State.EXPLORE and left is None:
            left = t
        if left is not None and t < left + 2.5:
            assert cmd[0] <= 0.0  # it used to drive straight on into the cube it was leaving
        t += 0.1
    assert left is not None


@pytest.mark.parametrize("bad", [float("inf"), float("nan")])
def test_a_non_finite_estimated_pose_is_ignored(bad):
    fsm = exploring_fsm()
    cmd = explore(fsm, 1.0, [], est=(0.0, 0.3, bad))
    assert cmd == (fsm.params.explore_speed, 0.0, 0.0)
