"""CARRY and DELIVER: drive to the square, take the zone lease, push the cube in, reverse out.

Real track: the robot approaches the blue square heading WEST (core/delivery.py)."""

import math
import random

import pytest

from mobile_base.core.behavior import BehaviorFSM, BehaviorInput, BehaviorParams, Cube, State, ZoneAction
from mobile_base.core.delivery import DeliveryZone

PHINEAS = ("red", "orange", "yellow")
WEST = math.pi


def carrying_fsm(**overrides):
    """FSM that just captured a red cube (state CARRY at t=0.5)."""
    fsm = BehaviorFSM(BehaviorParams(my_colors=PHINEAS, capture_time=0.1, **overrides), rng=random.Random(1))
    fsm.step(BehaviorInput(now=0.0, cubes=[Cube("red", 0.0, 0.3, 400.0)], line_front=False))
    fsm.step(BehaviorInput(now=0.1, cubes=[Cube("red", 0.0, 0.95, 3000.0)], line_front=False))
    fsm.step(BehaviorInput(now=0.5, cubes=[], line_front=False, scoop_color="red"))
    assert fsm.state is State.CARRY
    fsm.take_actions()
    return fsm


def tick(fsm, now, pose, holds_zone=False, line_front=False, scoop="red", **kw):
    """By default the camera keeps seeing the red cube in the scoop."""
    return fsm.step(
        BehaviorInput(now=now, cubes=[], line_front=line_front, pose=pose, holds_zone=holds_zone, scoop_color=scoop,
                      **kw)
    )


def zone(fsm) -> DeliveryZone:
    return fsm.params.zone


def test_carry_without_pose_holds_still():
    fsm = carrying_fsm()
    assert tick(fsm, 1.0, None) == (0.0, 0.0, 0.0)
    assert fsm.state is State.CARRY


def test_carry_steers_toward_entry_in_an_arc():
    fsm = carrying_fsm()
    # Robot south-west of the entry facing east; the entry point is ahead and to the left.
    vx, vy, wz = tick(fsm, 1.0, (-0.5, -0.2, 0.0))
    assert vx >= fsm.params.carry_min_speed
    assert vy == 0.0
    assert wz > 0.0


def test_carry_turns_toward_entry_when_it_is_behind():
    fsm = carrying_fsm()
    vx, _, wz = tick(fsm, 1.0, (-0.1, 0.0, 0.0))  # facing east, entry behind
    assert vx == pytest.approx(fsm.params.carry_min_speed)
    assert abs(wz) == pytest.approx(fsm.params.carry_max_turn)


def test_line_while_carrying_gives_up_the_cube_and_the_lease():
    # The gate turns in place at the line (the cube drops): the behavior must stop pretending to carry.
    fsm = carrying_fsm()
    tick(fsm, 1.0, (0.0, 0.0, 0.0), holds_zone=True)
    cmd = tick(fsm, 1.1, (0.0, 0.0, 0.0), holds_zone=True, line_front=True)
    assert fsm.state is State.EXPLORE
    assert not fsm.carrying
    assert fsm.take_actions() == [ZoneAction("release", delivered=False, color="red")]
    assert cmd[0] == 0.0 and cmd[2] != 0.0


def test_line_while_carrying_without_lease_releases_nothing():
    fsm = carrying_fsm()
    tick(fsm, 1.0, (0.0, 0.0, 0.0))
    fsm.take_actions()
    tick(fsm, 1.1, (0.0, 0.0, 0.0), line_front=True)
    assert fsm.state is State.EXPLORE
    assert fsm.take_actions() == []


def test_requests_lease_while_carrying_and_retries_periodically():
    fsm = carrying_fsm(request_period=1.0)
    pose = (0.2, 0.0, 0.0)
    tick(fsm, 1.0, pose)
    assert fsm.take_actions() == [ZoneAction("request")]
    tick(fsm, 1.5, pose)
    assert fsm.take_actions() == []
    tick(fsm, 2.1, pose)
    assert fsm.take_actions() == [ZoneAction("request")]


def test_no_request_once_granted():
    fsm = carrying_fsm()
    tick(fsm, 2.0, (0.2, 0.0, 0.0), holds_zone=True)
    assert fsm.take_actions() == []


def test_without_lease_goes_to_its_waiting_point_not_the_entry():
    # Webots run 4: waiting AT the entry blocked the robot holding the lease until it expired (60 s).
    fsm = carrying_fsm(wait_x=0.0, wait_y=0.22)
    z = zone(fsm)
    on_axis = z.pose(0.585)  # on the axis at x 0.0, facing west, straight at the entry
    _, _, wz_to_wait = tick(fsm, 1.0, on_axis)
    _, _, wz_to_entry = tick(fsm, 1.1, on_axis, holds_zone=True)
    assert wz_to_wait < 0.0  # the waiting point is north, on the right when facing west: turn right
    assert wz_to_entry == pytest.approx(0.0)


def test_holds_still_at_waiting_point_without_lease():
    fsm = carrying_fsm(wait_x=0.0, wait_y=0.22)
    cmd = tick(fsm, 2.0, (0.0, 0.21, 0.0))
    assert cmd == (0.0, 0.0, 0.0)
    assert fsm.state is State.CARRY
    assert fsm.carrying


def test_entry_without_lease_is_not_a_place_to_stop():
    fsm = carrying_fsm(wait_x=0.0, wait_y=0.22)
    z = zone(fsm)
    vx, _, _ = tick(fsm, 2.0, z.pose(z.entry_offset))
    assert vx > 0.0
    assert fsm.state is State.CARRY


def test_enters_square_with_lease_and_keeps_cube():
    fsm = carrying_fsm()
    z = zone(fsm)
    vx, _, _ = tick(fsm, 2.0, z.pose(z.entry_offset), holds_zone=True)
    assert fsm.state is State.DELIVER
    assert fsm.carrying
    assert vx == pytest.approx(fsm.params.enter_speed)


def test_enter_ignores_the_square_tape():
    fsm = carrying_fsm()
    z = zone(fsm)
    tick(fsm, 2.0, z.pose(z.entry_offset), holds_zone=True)
    vx, _, _ = tick(fsm, 2.1, z.pose(z.drop_offset + 0.03), holds_zone=True, line_front=True)
    assert vx > 0.0
    assert fsm.state is State.DELIVER


def test_enter_steers_back_to_the_square_axis():
    fsm = carrying_fsm()
    z = zone(fsm)
    tick(fsm, 2.0, z.pose(z.entry_offset), holds_zone=True)
    # Facing the square 5 cm to the left of the axis: the axis is on the robot's right -> negative wz.
    _, _, wz = tick(fsm, 2.1, z.pose(0.2, across=0.05), holds_zone=True)
    assert wz < 0.0


def test_reaching_drop_point_reverses_without_cube():
    fsm = carrying_fsm()
    z = zone(fsm)
    tick(fsm, 2.0, z.pose(z.entry_offset), holds_zone=True)
    vx, vy, wz = tick(fsm, 4.0, z.pose(z.drop_offset - 0.001), holds_zone=True)
    assert vx == pytest.approx(-fsm.params.reverse_speed)
    assert (vy, wz) == (0.0, 0.0)
    assert not fsm.carrying
    assert fsm.state is State.DELIVER


def test_exit_releases_lease_with_color_and_explores():
    fsm = carrying_fsm()
    z = zone(fsm)
    tick(fsm, 2.0, z.pose(z.entry_offset), holds_zone=True)
    tick(fsm, 4.0, z.pose(z.drop_offset - 0.001), holds_zone=True)
    tick(fsm, 5.0, z.pose(z.exit_offset + 0.001), holds_zone=True, scoop="")
    assert fsm.take_actions() == [ZoneAction("release", delivered=True, color="red")]
    assert fsm.state is State.EXPLORE
    assert not fsm.carrying


def test_reverse_times_out_and_still_releases():
    fsm = carrying_fsm(drop_timeout=3.0)
    z = zone(fsm)
    tick(fsm, 2.0, z.pose(z.entry_offset), holds_zone=True)
    tick(fsm, 4.0, z.pose(z.drop_offset - 0.001), holds_zone=True)
    tick(fsm, 6.0, z.pose(z.drop_offset), holds_zone=True)  # stuck: the gate refused the reverse
    assert fsm.state is State.DELIVER
    tick(fsm, 7.1, z.pose(z.drop_offset), holds_zone=True, scoop="")
    assert fsm.take_actions() == [ZoneAction("release", delivered=True, color="red")]
    assert fsm.state is State.EXPLORE


def test_never_reverses_while_carrying():
    fsm = carrying_fsm()
    z = zone(fsm)
    poses = [(0.0, 0.0, 0.0), z.pose(z.entry_offset), z.pose(0.15, across=0.04), z.pose(0.12)]
    for i, pose in enumerate(poses * 5):
        cmd = tick(fsm, 1.0 + i * 0.1, pose, holds_zone=i % 3 != 0)
        if fsm.carrying:
            assert cmd[0] >= 0.0


def test_corridor_contains_entry_and_drop_but_not_the_square_interior():
    z = DeliveryZone()
    assert z.in_corridor(*z.entry_point)
    assert z.in_corridor(*z.drop_point)
    assert not z.in_corridor(*z.point(-0.15))  # deep inside the square
    assert not z.in_corridor(*z.point(0.2, across=0.3))
    assert z.aligned(WEST + 0.3)
    assert not z.aligned(0.0)


def test_drop_point_puts_the_cube_inside_the_square():
    z = DeliveryZone()
    assert z.inside_square(*z.cube_drop_point, margin=0.015)  # whole 2 cm cube, 5 mm to spare


# ---- zone math (any approach heading)


def test_real_zone_points_on_the_west_approach():
    z = DeliveryZone()
    assert z.heading == pytest.approx(WEST)
    assert z.entry_point == pytest.approx((-0.287, 0.0085))
    assert z.drop_point == pytest.approx((-0.487, 0.0085))
    assert z.exit_point == pytest.approx((-0.38, 0.0085))
    assert z.cube_drop_point == pytest.approx((-0.6265, 0.0085))
    # Body front (0.095 m) 3 mm before the boundary (rectangle inner tape edge at x -0.585).
    assert z.drop_point[0] - 0.095 == pytest.approx(-0.582)


def test_along_and_across_on_the_west_approach():
    z = DeliveryZone()
    assert z.along_across(-0.585, 0.0085) == pytest.approx((0.0, 0.0))  # on the boundary, on the axis
    # Facing west, north (+y) is to the right: across is negative.
    assert z.along_across(-0.6865, 0.0585) == pytest.approx((0.1015, -0.05))
    assert z.before(-0.487, 0.0) == pytest.approx(0.098)
    assert z.inside_square(-0.70, 0.05)
    assert not z.inside_square(-0.55, 0.0)  # track side of the boundary
    assert not z.inside_square(-0.70, 0.10)  # beyond the north side of the blue


def test_corridor_and_alignment_on_the_west_approach():
    z = DeliveryZone()
    assert z.in_corridor(-0.40, 0.0085 + 0.07)
    assert not z.in_corridor(-0.40, 0.0085 + 0.09)  # 9 cm off the axis
    assert not z.in_corridor(-0.20, 0.0085)  # east of entry + margin (x -0.237)
    assert not z.in_corridor(-0.60, 0.0085)  # body past the boundary
    assert z.aligned(math.radians(150.0)) and z.aligned(math.radians(-150.0))
    assert not z.aligned(math.radians(90.0))


@pytest.mark.parametrize("heading", [0.0, math.pi / 2, math.pi, -math.pi / 2, 0.7])
def test_points_and_coordinates_round_trip_for_any_heading(heading):
    z = DeliveryZone(x=0.3, y=-0.2, heading=heading, square_depth=0.2)
    for before, across in ((0.0, 0.0), (0.25, 0.03), (-0.05, -0.04)):
        along, side = z.along_across(*z.point(before, across))
        assert (along, side) == pytest.approx((-before, across))
    # The boundary is half the square depth before the center, against the heading.
    bx, by = z.point(0.0)
    assert (bx, by) == pytest.approx((0.3 - 0.1 * math.cos(heading), -0.2 - 0.1 * math.sin(heading)))
    assert z.pose(0.1)[2] == heading
