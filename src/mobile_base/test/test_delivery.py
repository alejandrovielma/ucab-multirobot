"""CARRY and DELIVER: drive to the square, take the zone lease, push the cube in, reverse out."""

import math
import random

import pytest

from mobile_base.core.behavior import BehaviorFSM, BehaviorInput, BehaviorParams, Cube, State, ZoneAction
from mobile_base.core.delivery import DeliveryZone

PHINEAS = ("red", "orange", "yellow")
SOUTH = -math.pi / 2


def carrying_fsm(**overrides):
    """FSM that just captured a red cube (state CARRY at t=0.5)."""
    fsm = BehaviorFSM(BehaviorParams(my_colors=PHINEAS, capture_time=0.1, **overrides), rng=random.Random(1))
    fsm.step(BehaviorInput(now=0.0, cubes=[Cube("red", 0.0, 0.3, 400.0)], line_front=False))
    fsm.step(BehaviorInput(now=0.1, cubes=[Cube("red", 0.0, 0.95, 3000.0)], line_front=False))
    fsm.step(BehaviorInput(now=0.5, cubes=[], line_front=False, scoop_color="red"))
    assert fsm.state is State.CARRY
    fsm.take_actions()
    return fsm


def tick(fsm, now, pose, holds_zone=False, line_front=False, scoop="red"):
    """By default the camera keeps seeing the red cube in the scoop."""
    return fsm.step(
        BehaviorInput(now=now, cubes=[], line_front=line_front, pose=pose, holds_zone=holds_zone, scoop_color=scoop)
    )


def zone(fsm) -> DeliveryZone:
    return fsm.params.zone


def test_carry_without_pose_holds_still():
    fsm = carrying_fsm()
    assert tick(fsm, 1.0, None) == (0.0, 0.0, 0.0)
    assert fsm.state is State.CARRY


def test_carry_steers_toward_entry_in_an_arc():
    fsm = carrying_fsm()
    # Robot in the left lobe facing east; the entry point is ahead and slightly to the left.
    vx, vy, wz = tick(fsm, 1.0, (-0.6, -0.1, 0.0))
    assert vx >= fsm.params.carry_min_speed
    assert vy == 0.0
    assert wz > 0.0


def test_carry_turns_toward_entry_when_it_is_behind():
    fsm = carrying_fsm()
    vx, _, wz = tick(fsm, 1.0, (-0.6, 0.0, math.pi))  # facing west, entry behind
    assert vx == pytest.approx(fsm.params.carry_min_speed)
    assert abs(wz) == pytest.approx(fsm.params.carry_max_turn)


def test_line_while_carrying_gives_up_the_cube_and_the_lease():
    # The gate turns in place at the line (the cube drops): the behavior must stop pretending to carry.
    fsm = carrying_fsm()
    tick(fsm, 1.0, (-0.6, 0.0, 0.0), holds_zone=True)
    cmd = tick(fsm, 1.1, (-0.6, 0.0, 0.0), holds_zone=True, line_front=True)
    assert fsm.state is State.EXPLORE
    assert not fsm.carrying
    assert fsm.take_actions() == [ZoneAction("release", delivered=False, color="red")]
    assert cmd[0] == 0.0 and cmd[2] != 0.0


def test_line_while_carrying_without_lease_releases_nothing():
    fsm = carrying_fsm()
    tick(fsm, 1.0, (-0.6, 0.0, 0.0))
    fsm.take_actions()
    tick(fsm, 1.1, (-0.6, 0.0, 0.0), line_front=True)
    assert fsm.state is State.EXPLORE
    assert fsm.take_actions() == []


def test_requests_lease_while_carrying_and_retries_periodically():
    fsm = carrying_fsm(request_period=1.0)
    pose = (-0.7, 0.0, 0.0)
    tick(fsm, 1.0, pose)
    assert fsm.take_actions() == [ZoneAction("request")]
    tick(fsm, 1.5, pose)
    assert fsm.take_actions() == []
    tick(fsm, 2.1, pose)
    assert fsm.take_actions() == [ZoneAction("request")]


def test_no_request_once_granted():
    fsm = carrying_fsm()
    tick(fsm, 2.0, (-0.7, 0.0, 0.0), holds_zone=True)
    assert fsm.take_actions() == []


def test_without_lease_goes_to_its_waiting_point_not_the_entry():
    # Webots run 4: waiting AT the entry blocked the robot holding the lease until it expired (60 s).
    fsm = carrying_fsm(wait_x=-0.45, wait_y=0.05)
    _, _, wz_to_wait = tick(fsm, 1.0, (-0.13, 0.30, SOUTH))  # facing south, straight at the entry
    _, _, wz_to_entry = tick(fsm, 1.1, (-0.13, 0.30, SOUTH), holds_zone=True)
    assert wz_to_wait < 0.0  # the waiting point is south-west: turn right
    assert wz_to_entry == pytest.approx(0.0)


def test_holds_still_at_waiting_point_without_lease():
    fsm = carrying_fsm(wait_x=-0.45, wait_y=0.05)
    cmd = tick(fsm, 2.0, (-0.45, 0.06, 0.0))
    assert cmd == (0.0, 0.0, 0.0)
    assert fsm.state is State.CARRY
    assert fsm.carrying


def test_entry_without_lease_is_not_a_place_to_stop():
    fsm = carrying_fsm(wait_x=-0.45, wait_y=0.05)
    z = zone(fsm)
    vx, _, _ = tick(fsm, 2.0, (z.x, z.entry_y, SOUTH))
    assert vx > 0.0
    assert fsm.state is State.CARRY


def test_enters_square_with_lease_and_keeps_cube():
    fsm = carrying_fsm()
    z = zone(fsm)
    vx, _, _ = tick(fsm, 2.0, (z.x, z.entry_y, SOUTH), holds_zone=True)
    assert fsm.state is State.DELIVER
    assert fsm.carrying
    assert vx == pytest.approx(fsm.params.enter_speed)


def test_enter_ignores_the_square_tape():
    fsm = carrying_fsm()
    z = zone(fsm)
    tick(fsm, 2.0, (z.x, z.entry_y, SOUTH), holds_zone=True)
    vx, _, _ = tick(fsm, 2.1, (z.x, z.drop_y + 0.03, SOUTH), holds_zone=True, line_front=True)
    assert vx > 0.0
    assert fsm.state is State.DELIVER


def test_enter_steers_back_to_the_square_axis():
    fsm = carrying_fsm()
    z = zone(fsm)
    tick(fsm, 2.0, (z.x, z.entry_y, SOUTH), holds_zone=True)
    # Facing south, east of the axis: the axis is on the robot's right -> negative wz.
    _, _, wz = tick(fsm, 2.1, (z.x + 0.05, 0.0, SOUTH), holds_zone=True)
    assert wz < 0.0


def test_reaching_drop_point_reverses_without_cube():
    fsm = carrying_fsm()
    z = zone(fsm)
    tick(fsm, 2.0, (z.x, z.entry_y, SOUTH), holds_zone=True)
    vx, vy, wz = tick(fsm, 4.0, (z.x, z.drop_y - 0.001, SOUTH), holds_zone=True)
    assert vx == pytest.approx(-fsm.params.reverse_speed)
    assert (vy, wz) == (0.0, 0.0)
    assert not fsm.carrying
    assert fsm.state is State.DELIVER


def test_exit_releases_lease_with_color_and_explores():
    fsm = carrying_fsm()
    z = zone(fsm)
    tick(fsm, 2.0, (z.x, z.entry_y, SOUTH), holds_zone=True)
    tick(fsm, 4.0, (z.x, z.drop_y - 0.001, SOUTH), holds_zone=True)
    tick(fsm, 5.0, (z.x, z.exit_y + 0.001, SOUTH), holds_zone=True, scoop="")
    assert fsm.take_actions() == [ZoneAction("release", delivered=True, color="red")]
    assert fsm.state is State.EXPLORE
    assert not fsm.carrying


def test_reverse_times_out_and_still_releases():
    fsm = carrying_fsm(drop_timeout=3.0)
    z = zone(fsm)
    tick(fsm, 2.0, (z.x, z.entry_y, SOUTH), holds_zone=True)
    tick(fsm, 4.0, (z.x, z.drop_y - 0.001, SOUTH), holds_zone=True)
    tick(fsm, 6.0, (z.x, z.drop_y, SOUTH), holds_zone=True)  # stuck: the gate refused the reverse
    assert fsm.state is State.DELIVER
    tick(fsm, 7.1, (z.x, z.drop_y, SOUTH), holds_zone=True, scoop="")
    assert fsm.take_actions() == [ZoneAction("release", delivered=True, color="red")]
    assert fsm.state is State.EXPLORE


def test_never_reverses_while_carrying():
    fsm = carrying_fsm()
    z = zone(fsm)
    poses = [(-0.6, 0.0, 0.0), (z.x, z.entry_y, SOUTH), (z.x + 0.04, -0.05, SOUTH), (z.x, -0.1, SOUTH)]
    for i, pose in enumerate(poses * 5):
        cmd = tick(fsm, 1.0 + i * 0.1, pose, holds_zone=i % 3 != 0)
        if fsm.carrying:
            assert cmd[0] >= 0.0


def test_corridor_contains_entry_and_drop_but_not_the_square_south_edge():
    z = DeliveryZone()
    assert z.in_corridor(z.x, z.entry_y)
    assert z.in_corridor(z.x, z.drop_y)
    assert not z.in_corridor(z.x, z.square_y - 0.08)
    assert not z.in_corridor(z.x + 0.3, 0.0)
    assert z.aligned(SOUTH + 0.3)
    assert not z.aligned(math.pi / 2)


def test_drop_point_puts_the_cube_inside_the_square():
    z = DeliveryZone()
    cube_y = z.drop_y - z.cube_ahead
    assert z.square_y - z.square_half + 0.015 <= cube_y <= z.square_y + z.square_half - 0.015
