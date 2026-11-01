"""Carry planner: with a cube the robot is a forward-only arc car (Dubins car) whose body must stay
inside the track, so steering toward the goal greedily dead-ends in narrow parts (Webots run 13).
A* over three motion primitives finds a sequence that keeps the body inside."""

import math
import random

import pytest

from mobile_base.core.behavior import BehaviorFSM, BehaviorInput, BehaviorParams, Cube, State
from mobile_base.core.planner import Plan, plan_arcs
from mobile_base.core.track import Footprint, Track, path_inside, predict

ARENA = Track((0.0, 0.0, 2.0, 0.0, 2.0, 2.0, 0.0, 2.0))
FP = Footprint()
SOUTH = -math.pi / 2


def facing_south_near(x, y):
    def goal(p):
        return math.hypot(p[0] - x, p[1] - y) < 0.06 and abs(math.remainder(p[2] - SOUTH, 2 * math.pi)) < 0.5

    return goal


def run(plan: Plan, start):
    """Replay the plan and check the body after every primitive."""
    pose = start
    for vx, wz, duration in plan.steps:
        assert path_inside(ARENA, FP, pose, vx, wz, duration, steps=4)
        pose = predict(pose, vx, wz, duration)
    return pose


def test_turns_around_with_forward_arcs_only():
    start = (1.0, 1.0, math.pi / 2)  # facing north, goal: same spot facing south
    plan = plan_arcs(ARENA, FP, start, facing_south_near(1.0, 0.9), (1.0, 0.9), straight_speed=0.08,
                     arc_speed=0.05, turn=0.6)
    assert plan is not None
    assert all(vx > 0.0 for vx, _, _ in plan.steps)
    end = run(plan, start)
    assert facing_south_near(1.0, 0.9)(end)


def test_no_plan_when_every_forward_motion_crosses_the_tape():
    start = (2.0 - 0.005 - FP.front, 1.0, 0.0)
    assert plan_arcs(ARENA, FP, start, facing_south_near(1.0, 0.9), (1.0, 0.9), 0.08, 0.05, 0.6) is None


def test_expected_pose_follows_the_steps():
    start = (1.0, 1.0, 0.0)
    plan = plan_arcs(ARENA, FP, start, facing_south_near(1.0, 0.9), (1.0, 0.9), 0.08, 0.05, 0.6)
    t = plan.steps[0][2]
    assert plan.expected(t) == pytest.approx(predict(start, plan.steps[0][0], plan.steps[0][1], t))
    assert plan.command(t / 2) == pytest.approx(plan.steps[0][:2])
    assert plan.command(plan.duration + 1.0) is None


# ---- behavior with the planner (zone moved inside the 2 m arena)

# Square at y 0.0-0.8 with the boundary at y 0.8, approached heading south: entry y 1.2, drop 0.9, exit 1.1.
ZONE = dict(zone_x=1.0, zone_y=0.4, zone_heading=-math.pi / 2, zone_square_depth=0.8, zone_entry_offset=0.4,
            zone_drop_offset=0.1, zone_exit_offset=0.3)


def fsm():
    params = BehaviorParams(my_colors=("red",), capture_distance=0.008, **ZONE)
    return BehaviorFSM(params, rng=random.Random(2), track=ARENA)


def step(f, now, pose, scoop="", cubes=(), holds_zone=True):
    return f.step(BehaviorInput(now=now, cubes=list(cubes), line_front=False, pose=pose, scoop_color=scoop,
                                holds_zone=holds_zone))


def carrying(f, pose):
    step(f, 0.0, pose, cubes=[Cube("red", 0.0, 0.3, 400.0)])
    step(f, 0.1, pose, cubes=[Cube("red", 0.0, 0.7, 3000.0)])
    step(f, 0.3, pose, scoop="red")
    assert f.state is State.CARRY
    f.take_actions()


def test_carry_follows_the_planned_primitives():
    f = fsm()
    start = (1.5, 1.5, math.pi / 2)
    carrying(f, start)
    cmd = step(f, 1.0, start, scoop="red")
    assert f.plan is not None
    assert cmd[:3:2] == pytest.approx(f.plan.command(0.0))
    assert cmd[1] == 0.0 and (cmd[0] == 0.0 or cmd[2] == 0.0)  # a straight or an in-place spin, no arc
    assert all(vx == 0.0 or wz == 0.0 for vx, wz, _ in f.plan.steps)


def test_carry_replans_when_the_robot_drifts_from_the_plan():
    f = fsm()
    start = (1.5, 1.5, math.pi / 2)
    carrying(f, start)
    step(f, 1.0, start, scoop="red")
    first = f.plan
    step(f, 1.2, (1.3, 1.6, 0.0), scoop="red")  # pushed away (peer, slip)
    assert f.plan is not first


def test_capture_skipped_when_no_plan_reaches_the_square():
    f = fsm()
    # Body front 2 cm from the wall, facing it: after the capture push there is no room to spin
    # (sweep radius 12.5 cm) and no way forward; with a loose cube reversing is not planned.
    pose = (2.0 - 0.02 - FP.front, 1.5, 0.0)
    step(f, 0.0, pose, cubes=[Cube("red", 0.0, 0.3, 400.0)])
    step(f, 0.1, pose, cubes=[Cube("red", 0.0, 0.7, 3000.0)])
    assert f.state is State.EXPLORE
