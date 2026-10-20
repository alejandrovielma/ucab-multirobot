"""Scoop gate: a servo door at the mouth of the U retains the cube, so with the gate closed the robot
may spin in place and reverse while carrying (Webots, calibracion_pala.wbt: the cube stayed in the U
through a 5 s spin and a 12 cm reverse, and was left behind once the gate opened)."""

import math
import random

import pytest

from mobile_base.core.behavior import BehaviorFSM, BehaviorInput, BehaviorParams, Cube, State, ZoneAction
from mobile_base.core.planner import plan_arcs
from mobile_base.core.safety import Pose2D, SafetyGate, SafetyInput, SafetyParams
from mobile_base.core.track import Footprint, Track, path_inside, predict

FP = Footprint()
SOUTH = -math.pi / 2
ARENA = Track((0.0, 0.0, 2.0, 0.0, 2.0, 2.0, 0.0, 2.0))
# Square at y 0.0-0.8 with the boundary at y 0.8, approached heading south: entry y 1.2, drop 0.9, exit 1.1.
ZONE = dict(zone_x=1.0, zone_y=0.4, zone_heading=-math.pi / 2, zone_square_depth=0.8, zone_entry_offset=0.4,
            zone_drop_offset=0.1, zone_exit_offset=0.3)


# ---- planner: spins and reverse are only offered with the gate closed


def test_planner_turns_around_in_a_corridor_too_narrow_for_arcs_when_it_may_spin():
    corridor = Track((0.0, 0.0, 0.30, 0.0, 0.30, 2.0, 0.0, 2.0))  # 30 cm: no U-turn with r = 8 cm arcs + body
    start = (0.15, 1.0, math.pi / 2)

    def goal(p):
        return abs(math.remainder(p[2] - SOUTH, 2 * math.pi)) < 0.3

    args = (corridor, FP, start, goal, (0.15, 0.6), 0.08, 0.05, 0.6)
    assert plan_arcs(*args, max_expansions=4000) is None
    plan = plan_arcs(*args, spin_speed=1.2, reverse_speed=0.04)
    assert plan is not None
    pose = start
    for vx, wz, duration in plan.steps:
        assert path_inside(corridor, FP, pose, vx, wz, duration, steps=4)
        pose = predict(pose, vx, wz, duration)
    assert goal(pose)


# ---- safety gate


def gate_input(cmd, **kw):
    values = dict(cmd=cmd, line_front=False, own_pose=Pose2D(1.0, 1.0, 0.0, stamp=10.0), peer_pose=None, now=10.0,
                  heartbeat_age=0.1, carrying=True)
    values.update(kw)
    return SafetyInput(**values)


def test_closed_gate_lets_a_carrying_robot_spin_in_place():
    cmd, _ = SafetyGate(SafetyParams(), track=ARENA).filter(gate_input((0.0, 0.0, 1.2), gate_closed=True))
    assert cmd == pytest.approx((0.0, 0.0, 1.2))


def test_closed_gate_lets_a_carrying_robot_reverse_with_the_pose():
    params = SafetyParams()
    cmd, _ = SafetyGate(params, track=ARENA).filter(gate_input((-0.04, 0.0, 0.0), gate_closed=True))
    assert cmd == pytest.approx((-0.04, 0.0, 0.0))


def test_open_gate_keeps_the_fixed_scoop_rules():
    gate = SafetyGate(SafetyParams(), track=ARENA)
    spin, _ = gate.filter(gate_input((0.0, 0.0, 1.2), gate_closed=False))
    assert spin[0] > 0.0  # arc, not an in-place spin
    back, _ = gate.filter(gate_input((-0.04, 0.0, 0.0), gate_closed=False))
    assert back[0] == pytest.approx(0.0)


# ---- behavior


def fsm(has_gate=True, track=ARENA):
    params = BehaviorParams(my_colors=("red",), capture_time=0.1, has_gate=has_gate, **ZONE)
    return BehaviorFSM(params, rng=random.Random(4), track=track)


def step(f, now, pose=(1.5, 1.5, math.pi / 2), scoop="", cubes=(), holds_zone=True, line_front=False):
    return f.step(BehaviorInput(now=now, cubes=list(cubes), line_front=line_front, pose=pose, scoop_color=scoop,
                                holds_zone=holds_zone))


def capture(f, pose=(1.5, 1.5, math.pi / 2)):
    step(f, 0.0, pose, cubes=[Cube("red", 0.0, 0.3, 400.0)])
    step(f, 0.1, pose, cubes=[Cube("red", 0.0, 0.7, 3000.0)])
    assert not f.gate_closed  # open while capturing
    return step(f, 0.3, pose, scoop="red")


def test_gate_closes_after_a_confirmed_capture_and_the_robot_waits_for_it():
    f = fsm()
    cmd = capture(f)
    assert f.state is State.CARRY
    assert f.gate_closed
    assert cmd == (0.0, 0.0, 0.0)
    assert step(f, 0.3 + f.params.gate_close_time + 0.05, scoop="red") != (0.0, 0.0, 0.0)


def test_without_a_gate_nothing_changes():
    f = fsm(has_gate=False)
    capture(f)
    assert f.state is State.CARRY
    assert not f.gate_closed


def test_line_while_carrying_with_the_gate_closed_does_not_drop_the_cube():
    f = fsm()
    capture(f)
    step(f, 1.0, scoop="red", line_front=True)
    assert f.state is State.CARRY
    assert f.gate_closed


def test_drop_opens_the_gate_before_reversing():
    f = fsm()
    capture(f)
    f.take_actions()
    z = f.params.zone
    t = 1.0
    step(f, t, z.pose(z.entry_offset), scoop="red")
    assert f.state is State.DELIVER
    cmd = step(f, t + 0.1, z.pose(z.drop_offset - 0.001), scoop="red")
    assert not f.gate_closed
    assert cmd == (0.0, 0.0, 0.0)  # waiting for the door to open
    cmd = step(f, t + 0.1 + f.params.gate_open_time + 0.05, z.pose(z.drop_offset - 0.001), scoop="red")
    assert cmd[0] < 0.0
    step(f, t + 3.0, z.pose(z.exit_offset + 0.001), scoop="")
    assert ZoneAction("release", delivered=True, color="red") in f.take_actions()
    assert not f.gate_closed


def test_abandoning_a_transport_opens_the_gate():
    f = fsm()
    capture(f)
    step(f, 1.0, scoop="")  # cube lost from view for longer than scoop_lost_timeout
    step(f, 2.5, scoop="")
    assert f.state is State.EXPLORE
    assert not f.gate_closed


# ---- Webots run 19: Phineas entered 5 cm off the axis and 27 deg off south and froze for 7 minutes


def carrying_at(f, t0=0.0):
    capture(f)
    f.take_actions()
    return 0.3 + f.params.gate_close_time + 0.05


def test_with_the_gate_the_entry_must_be_precise_before_delivering():
    f = fsm()
    t = carrying_at(f)
    z = f.params.zone
    step(f, t, (*z.point(z.entry_offset, across=0.05), SOUTH + math.radians(27)), scoop="red")
    assert f.state is State.CARRY
    step(f, t + 0.1, (*z.point(z.entry_offset, across=0.01), SOUTH + math.radians(5)), scoop="red")
    assert f.state is State.DELIVER


def test_blocked_entry_backs_off_with_the_cube_and_replans():
    # An obstacle line 3.5 cm before the drop point (beyond drop_tolerance): the entry is really blocked.
    wall = Track((0.0, 0.83, 2.0, 0.83, 2.0, 2.0, 0.0, 2.0))
    f = fsm(track=wall)
    t = carrying_at(f)
    z = f.params.zone
    step(f, t, z.pose(z.entry_offset), scoop="red")
    assert f.state is State.DELIVER
    cmd = step(f, t + 0.1, (z.x, 0.935, SOUTH), scoop="red")  # forward would put the front past 0.83
    assert f.state is State.CARRY
    assert f.gate_closed and f.carrying
    assert cmd[0] < 0.0
