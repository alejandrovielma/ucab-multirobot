"""Geofence with the overhead pose: the single front line sensor cannot tell the white inside the
track from the white outside it (Webots run 6: after a turn on the tape the sensor ended beyond it
and Phineas drove off the board reading white)."""

import math
import pathlib
import random

import pytest
import yaml

from mobile_base.core.behavior import BehaviorFSM, BehaviorInput, BehaviorParams, State
from mobile_base.core.delivery import DeliveryZone
from mobile_base.core.safety import Pose2D, SafetyGate, SafetyInput, SafetyParams
from mobile_base.core.track import Track

CONFIG_DIR = pathlib.Path(__file__).resolve().parents[1] / "config"
SQUARE = Track((0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0))


def real_track() -> Track:
    data = yaml.safe_load((CONFIG_DIR / "track.yaml").read_text())
    return Track(data["/**"]["ros__parameters"]["track_polygon"])


def test_contains_and_centroid():
    assert SQUARE.contains(0.5, 0.5)
    assert not SQUARE.contains(1.5, 0.5)
    assert not SQUARE.contains(0.5, -0.01)
    assert SQUARE.centroid == pytest.approx((0.5, 0.5))


def test_empty_track_is_disabled_and_contains_everything():
    track = Track(())
    assert not track.enabled
    assert track.contains(100.0, 100.0)


def test_odd_number_of_values_is_rejected():
    with pytest.raises(ValueError):
        Track((0.0, 0.0, 1.0))


def test_webots_track_contains_the_key_points_but_not_the_square():
    track = real_track()
    z = DeliveryZone()
    for name, (x, y) in {
        "phineas start": (-0.45, 0.0),
        "ferb start": (0.5, -0.25),
        "entry": (z.x, z.entry_y),
        "exit": (z.x, z.exit_y),
        "phineas wait": (-0.45, 0.05),
        "ferb wait": (0.25, 0.05),
    }.items():
        assert track.contains(x, y), name
    assert not track.contains(z.x, z.square_y)
    assert track.contains(*track.centroid)  # the way back in from outside points here
    assert not track.contains(-0.65, 0.81)  # where Phineas ended in run 6


def gate_input(pose, cmd=(0.1, 0.0, 0.0), **kw):
    values = dict(cmd=cmd, line_front=False, own_pose=pose, peer_pose=None, now=10.0, heartbeat_age=0.1)
    values.update(kw)
    return SafetyInput(**values)


def test_gate_blocks_forward_when_the_front_leaves_the_track():
    gate = SafetyGate(SafetyParams(), track=SQUARE)
    cmd, reason = gate.filter(gate_input(Pose2D(0.95, 0.5, 0.0, stamp=10.0), cmd=(0.1, 0.0, 0.3)))
    assert cmd == pytest.approx((0.0, 0.0, 0.3))
    assert "track edge" in reason


def test_gate_lets_the_robot_drive_back_in_from_outside():
    gate = SafetyGate(SafetyParams(), track=SQUARE)
    toward = Pose2D(1.2, 0.5, math.pi, stamp=10.0)  # outside, facing the centroid
    away = Pose2D(1.2, 0.5, 0.0, stamp=10.0)
    assert gate.filter(gate_input(toward))[0][0] == pytest.approx(0.1)
    assert gate.filter(gate_input(away))[0][0] == pytest.approx(0.0)
    # Off the board the floor reads as black tape: going back in must cross the tape anyway.
    assert gate.filter(gate_input(toward, line_front=True))[0][0] == pytest.approx(0.1)


def test_gate_track_edge_while_carrying_turns_in_place():
    gate = SafetyGate(SafetyParams(), track=SQUARE)
    cmd, _ = gate.filter(gate_input(Pose2D(0.95, 0.5, 0.0, stamp=10.0), cmd=(0.1, 0.0, 0.3), carrying=True))
    assert cmd == pytest.approx((0.0, 0.0, 0.3))


def test_gate_track_ignored_in_delivery_corridor_with_lease():
    track = real_track()
    gate = SafetyGate(SafetyParams(), track=track)
    pose = Pose2D(-0.13, -0.12, -math.pi / 2, stamp=10.0)  # front already over the square
    cmd, _ = gate.filter(gate_input(pose, cmd=(0.05, 0.0, 0.0), holds_zone=True, carrying=True))
    assert cmd[0] == pytest.approx(0.05)


def make_fsm(seed=1):
    return BehaviorFSM(BehaviorParams(my_colors=("red",)), rng=random.Random(seed), track=SQUARE)


def test_behavior_treats_track_edge_as_line():
    fsm = make_fsm()
    cmd = fsm.step(BehaviorInput(now=0.0, cubes=[], line_front=False, pose=(0.95, 0.5, 0.0)))
    assert cmd[0] == 0.0 and cmd[2] != 0.0
    assert fsm.state is State.EXPLORE


def test_behavior_outside_turns_toward_the_track_then_drives_in():
    fsm = make_fsm()
    cmd = fsm.step(BehaviorInput(now=0.0, cubes=[], line_front=True, pose=(1.2, 0.5, 0.0)))
    assert cmd[0] == 0.0 and cmd[2] != 0.0
    cmd = fsm.step(BehaviorInput(now=0.1, cubes=[], line_front=True, pose=(1.2, 0.5, math.pi)))
    assert cmd == pytest.approx((fsm.params.explore_speed, 0.0, 0.0))
