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


# ---- Without the overhead pose: the robot's own camera finds the blue square (team decision 2026-10-27)


def cam(fsm, now, square=None, scoop="red"):
    """No pose; square = (cx, cy) when the camera sees the blue square."""
    visible = square is not None
    cx, cy = square if visible else (0.0, 0.0)
    return fsm.step(BehaviorInput(now=now, cubes=[], line_front=False, pose=None, scoop_color=scoop,
                                  square_visible=visible, square_cx=cx, square_cy=cy, square_area=20000.0))


def test_without_pose_the_carrying_robot_spins_slowly_looking_for_the_square():
    fsm = carrying_fsm(use_lease=False)
    assert cam(fsm, 1.0) == pytest.approx((0.0, 0.0, fsm.params.carry_max_turn))
    assert fsm.state is State.CARRY


def test_without_pose_a_square_never_seen_makes_the_robot_look_from_elsewhere():
    fsm = carrying_fsm(use_lease=False, square_search_time=5.0, square_probe_time=1.0)
    cam(fsm, 1.0)
    assert cam(fsm, 6.1) == pytest.approx((fsm.params.carry_speed, 0.0, 0.0))  # drive forward a bit
    assert cam(fsm, 7.2)[2] != 0.0  # and spin again


def test_without_pose_the_square_is_centered_then_approached_then_entered():
    fsm = carrying_fsm(use_lease=False, square_align=0.15, square_reach_cy=0.6)
    p = fsm.params
    assert cam(fsm, 1.0, square=(0.5, 0.3)) == pytest.approx((0.0, 0.0, -p.carry_max_turn))  # right: spin right
    assert cam(fsm, 1.1, square=(0.05, 0.3)) == pytest.approx((p.carry_speed, 0.0, 0.0))
    cmd = cam(fsm, 1.2, square=(0.0, 0.7))
    assert fsm.state is State.DELIVER
    assert cmd == pytest.approx((p.enter_speed, 0.0, 0.0))


def test_without_pose_the_cube_is_pushed_in_then_left_behind_and_counted():
    fsm = carrying_fsm(use_lease=False, deliver_push_time=2.0, deliver_back_time=2.5)
    p = fsm.params
    cam(fsm, 1.0, square=(0.0, 0.7))
    assert fsm.state is State.DELIVER
    vx, vy, wz = cam(fsm, 2.0, square=(0.2, 0.8))
    assert vx == pytest.approx(p.enter_speed) and vy < 0.0 and wz == 0.0  # square to the right: slide right
    assert cam(fsm, 3.1, square=(0.0, 0.9)) == pytest.approx((-p.reverse_speed, 0.0, 0.0))  # push over: back out
    assert cam(fsm, 5.5, scoop="") == pytest.approx((-p.reverse_speed, 0.0, 0.0))
    cam(fsm, 5.7, scoop="")
    assert fsm.state is State.EXPLORE
    assert fsm.delivered == 1
    assert fsm.take_actions() == []  # no arbiter: nothing to request or release


def test_without_pose_and_with_the_arbiter_the_robot_waits_for_the_lease():
    fsm = carrying_fsm()  # use_lease True (Webots, Perry)
    assert cam(fsm, 1.0, square=(0.0, 0.7)) == (0.0, 0.0, 0.0)
    assert fsm.take_actions() == [ZoneAction("request")]
    cam(fsm, 1.1, square=(0.0, 0.7))
    assert fsm.state is State.CARRY


def test_carry_spins_slowly_toward_the_entry_then_drives_straight():
    fsm = carrying_fsm()
    # Robot south-west of the entry facing east; the entry point is ahead and to the left (~44 deg).
    assert tick(fsm, 1.0, (-0.5, -0.2, 0.0)) == pytest.approx((0.0, 0.0, fsm.params.carry_max_turn))
    ex, ey = zone(fsm).entry_point
    facing = math.atan2(ey + 0.2, ex + 0.5)
    assert tick(fsm, 1.1, (-0.5, -0.2, facing)) == pytest.approx((fsm.params.carry_speed, 0.0, 0.0))


def test_carry_turns_toward_entry_when_it_is_behind():
    fsm = carrying_fsm()
    vx, _, wz = tick(fsm, 1.0, (-0.1, 0.0, 0.0))  # facing east, entry behind
    assert vx == 0.0
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
    to_entry = tick(fsm, 1.1, on_axis, holds_zone=True)
    assert wz_to_wait < 0.0  # the waiting point is north, on the right when facing west: turn right
    assert to_entry == pytest.approx((fsm.params.carry_speed, 0.0, 0.0))


def test_holds_still_at_waiting_point_without_lease():
    fsm = carrying_fsm(wait_x=0.0, wait_y=0.22)
    cmd = tick(fsm, 2.0, (0.0, 0.21, 0.0))
    assert cmd == (0.0, 0.0, 0.0)
    assert fsm.state is State.CARRY
    assert fsm.carrying


def test_entry_without_lease_is_not_a_place_to_stop():
    fsm = carrying_fsm(wait_x=0.0, wait_y=0.22)
    z = zone(fsm)
    cmd = tick(fsm, 2.0, z.pose(z.entry_offset))
    assert cmd != (0.0, 0.0, 0.0)  # on its way to the waiting point (spinning toward it first)
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


def test_enter_slides_back_to_the_square_axis_without_turning():
    fsm = carrying_fsm()
    z = zone(fsm)
    tick(fsm, 2.0, z.pose(z.entry_offset), holds_zone=True)
    # Facing the square 5 cm to the left of the axis: the axis is on the robot's right -> negative vy.
    vx, vy, wz = tick(fsm, 2.1, z.pose(0.2, across=0.05), holds_zone=True)
    assert vx > 0.0 and vy < 0.0 and wz == 0.0


def test_enter_spins_back_when_the_heading_drifts_off_the_axis():
    fsm = carrying_fsm()
    z = zone(fsm)
    tick(fsm, 2.0, z.pose(z.entry_offset), holds_zone=True)
    x, y, theta = z.pose(0.2)
    cmd = tick(fsm, 2.1, (x, y, theta + 0.3), holds_zone=True)
    assert cmd == pytest.approx((0.0, 0.0, -fsm.params.carry_max_turn))


def test_enter_aligns_with_the_blue_square_seen_by_the_camera():
    # On the axis the pose alone says "straight"; the square seen to the right adds a slide right.
    fsm = carrying_fsm()
    z = zone(fsm)
    p = fsm.params
    tick(fsm, 2.0, z.pose(z.entry_offset), holds_zone=True)
    blind = tick(fsm, 2.1, z.pose(0.2), holds_zone=True)
    seen = tick(fsm, 2.2, z.pose(0.2), holds_zone=True, square_visible=True, square_cx=0.25)
    assert blind == pytest.approx((p.enter_speed, 0.0, 0.0))
    assert seen[1] == pytest.approx(-p.square_align_gain * p.enter_speed * 0.25) and seen[2] == 0.0
    # Bounded by the entry speed.
    far = tick(fsm, 2.3, z.pose(0.2, across=0.05), holds_zone=True, square_visible=True, square_cx=1.0)
    assert far[1] == pytest.approx(-p.enter_speed)


def test_square_alignment_is_not_used_while_reversing_out():
    fsm = carrying_fsm()
    z = zone(fsm)
    tick(fsm, 2.0, z.pose(z.entry_offset), holds_zone=True)
    cmd = tick(fsm, 4.0, z.pose(z.drop_offset - 0.001), holds_zone=True, square_visible=True, square_cx=0.5)
    assert cmd == pytest.approx((-fsm.params.reverse_speed, 0.0, 0.0))


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


def test_drop_starts_where_the_gate_stops_the_robot():
    # Kinematic check (behavior + safety gate on the track polygon): the gate's 0.3 s geofence
    # prediction stops the entering robot ~1.3 cm before a drop point that leaves 3 mm to the
    # boundary, so the drop never started. Within drop_tolerance the drop starts anyway.
    fsm = carrying_fsm()
    z = zone(fsm)
    tick(fsm, 2.0, z.pose(z.entry_offset), holds_zone=True)
    vx, _, _ = tick(fsm, 4.0, z.pose(z.drop_offset + 0.013), holds_zone=True)
    assert vx == pytest.approx(-fsm.params.reverse_speed)
    assert not fsm.carrying
    # Farther than the tolerance it still enters.
    fsm2 = carrying_fsm()
    tick(fsm2, 2.0, z.pose(z.entry_offset), holds_zone=True)
    vx, _, _ = tick(fsm2, 4.0, z.pose(z.drop_offset + 0.05), holds_zone=True)
    assert vx > 0.0


def test_behavior_and_gate_complete_a_delivery_on_the_real_track():
    """Integration: entry -> drop -> reverse -> release, with the safety gate's body geofence active."""
    import pathlib

    import yaml

    from mobile_base.core.safety import Pose2D, SafetyGate, SafetyInput, SafetyParams
    from mobile_base.core.track import Track, predict

    cfg = pathlib.Path(__file__).resolve().parents[1] / "config"
    track = Track(yaml.safe_load((cfg / "track.yaml").read_text())["/**"]["ros__parameters"]["track_polygon"])
    fsm = carrying_fsm()
    fsm._track = track  # same polygon for the behavior's own geofence checks
    gate = SafetyGate(SafetyParams(), track=track)
    z = zone(fsm)
    pose = z.pose(z.entry_offset)
    released = None
    t = 1.0
    while t < 30.0 and released is None:
        scoop = "red" if fsm.carrying else ""
        want = fsm.step(BehaviorInput(now=t, cubes=[], line_front=False, pose=pose, holds_zone=True,
                                      scoop_color=scoop))
        cmd, _ = gate.filter(SafetyInput(cmd=want, line_front=False, own_pose=Pose2D(*pose, stamp=t),
                                         peer_pose=None, now=t, heartbeat_age=0.1, carrying=fsm.carrying,
                                         holds_zone=True))
        pose = predict(pose, cmd[0], cmd[2], 0.1)
        released = next((a for a in fsm.take_actions() if a.kind == "release"), None)
        t += 0.1
    assert released == ZoneAction("release", delivered=True, color="red"), f"stuck at t={t:.1f} pose={pose}"


# ---- Delivery along an axis: line up on the perpendicular between the green and red rectangles, push straight


def axis_cam(fsm, now, refs=None, square=None, scoop="red"):
    """refs = (center_cx, skew) when both rectangles are seen; square = (cx, cy) when the blue square is."""
    rv = refs is not None
    sv = square is not None
    return fsm.step(BehaviorInput(
        now=now, cubes=[], line_front=False, pose=None, scoop_color=scoop,
        refs_visible=rv, refs_cx=refs[0] if rv else 0.0, refs_skew=refs[1] if rv else 0.0,
        square_visible=sv, square_cx=square[0] if sv else 0.0, square_cy=square[1] if sv else 0.0,
        square_area=20000.0))


def axis_fsm(**kw):
    return carrying_fsm(use_lease=False, axis_delivery=True, **kw)


def test_axis_the_robot_spins_to_center_the_rectangles_before_anything_else():
    fsm = axis_fsm()
    p = fsm.params
    assert axis_cam(fsm, 1.0, refs=(0.5, 0.3)) == pytest.approx((0.0, 0.0, -p.carry_max_turn))
    assert axis_cam(fsm, 1.1, refs=(-0.5, 0.3)) == pytest.approx((0.0, 0.0, p.carry_max_turn))


def test_axis_left_of_the_axis_strafes_right_and_right_of_it_strafes_left_never_turning_and_moving():
    fsm = axis_fsm()
    p = fsm.params
    assert axis_cam(fsm, 1.0, refs=(0.0, 0.2)) == pytest.approx((0.0, -p.axis_strafe_speed, 0.0))  # left nearer
    assert axis_cam(fsm, 1.1, refs=(0.0, -0.2)) == pytest.approx((0.0, p.axis_strafe_speed, 0.0))
    assert p.axis_strafe_speed <= 0.05  # not above the gate's max_side_speed


def test_axis_on_the_axis_it_drives_straight_and_keeps_going_when_the_rectangles_leave_the_view():
    fsm = axis_fsm()
    p = fsm.params
    assert axis_cam(fsm, 1.0, refs=(0.02, 0.01), square=(0.0, 0.3)) == pytest.approx((p.carry_speed, 0.0, 0.0))
    # near the square the rectangles are out of the frame: no spinning around to look for them
    assert axis_cam(fsm, 1.2, square=(0.0, 0.4)) == pytest.approx((p.carry_speed, 0.0, 0.0))


def test_axis_from_afar_it_heads_for_the_square_until_the_rectangles_show_and_searches_if_it_sees_nothing():
    fsm = axis_fsm()
    p = fsm.params
    assert axis_cam(fsm, 1.0) == pytest.approx((0.0, 0.0, p.carry_max_turn))  # nothing in view: search
    assert axis_cam(fsm, 1.1, square=(0.5, 0.3)) == pytest.approx((0.0, 0.0, -p.carry_max_turn))  # right: spin right
    assert axis_cam(fsm, 1.2, square=(0.0, 0.3)) == pytest.approx((p.carry_speed, 0.0, 0.0))  # centered: drive
    assert axis_cam(fsm, 1.3, refs=(0.5, 0.2), square=(0.0, 0.3))[2] != 0.0  # the rectangles take over


def test_axis_the_push_is_straight_even_if_the_square_looks_off_center():
    fsm = axis_fsm()
    p = fsm.params
    axis_cam(fsm, 1.0, refs=(0.0, 0.0), square=(0.0, 0.7))
    assert fsm.state is State.DELIVER
    assert axis_cam(fsm, 2.0, square=(0.4, 0.8)) == pytest.approx((p.enter_speed, 0.0, 0.0))


# ---- Reactive gaps: the other robot at the square, non-target cubes on the way

def cam2(fsm, now, cubes=(), peer=None, refs=(0.0, 0.0), square=(0.0, 0.3), scoop="red"):
    return fsm.step(BehaviorInput(
        now=now, cubes=list(cubes), line_front=False, pose=None, scoop_color=scoop, peer_seen=peer,
        refs_visible=refs is not None, refs_cx=refs[0] if refs else 0.0, refs_skew=refs[1] if refs else 0.0,
        square_visible=square is not None, square_cx=square[0] if square else 0.0,
        square_cy=square[1] if square else 0.0, square_area=20000.0))


def test_the_yielding_robot_waits_for_the_other_and_gives_up_after_the_timeout():
    fsm = axis_fsm(yield_to_peer=True, peer_yield_distance=0.5, yield_timeout=3.0)
    assert cam2(fsm, 1.0, peer=(0.0, 0.3)) == (0.0, 0.0, 0.0)
    assert cam2(fsm, 2.0, peer=(0.0, 0.3)) == (0.0, 0.0, 0.0)
    assert cam2(fsm, 3.0, peer=(0.0, 0.9))[0] == pytest.approx(fsm.params.carry_speed)  # it left: go on
    cam2(fsm, 4.0, peer=(0.0, 0.3))
    cam2(fsm, 7.5, peer=(0.0, 0.3))
    assert fsm.state is State.EXPLORE and "blocks" in fsm.note


def test_the_robot_with_priority_does_not_wait():
    fsm = axis_fsm(yield_to_peer=False)
    assert cam2(fsm, 1.0, peer=(0.0, 0.3))[0] == pytest.approx(fsm.params.carry_speed)


def test_a_non_target_cube_ahead_is_sidestepped_to_the_free_side_then_given_up_after_too_many_tries():
    fsm = axis_fsm(dodge_limit=2)
    p = fsm.params
    green_right = Cube("green", 0.1, 0.7, 800.0)
    assert cam2(fsm, 1.0, cubes=[green_right]) == pytest.approx((0.0, p.axis_strafe_speed, 0.0))  # cube right: go left
    # the direction is latched for the episode: the cube crossing the center does not flip it
    assert cam2(fsm, 1.1, cubes=[Cube("green", -0.1, 0.7, 800.0)]) == pytest.approx((0.0, p.axis_strafe_speed, 0.0))
    cam2(fsm, 1.2)  # clear: it goes straight again
    cam2(fsm, 1.3, cubes=[green_right])  # second episode
    cam2(fsm, 1.4)
    cam2(fsm, 1.5, cubes=[green_right])  # third episode: over the limit
    assert fsm.state is State.EXPLORE and "blocking" in fsm.note


def test_the_push_is_called_off_if_a_non_target_cube_is_in_its_path():
    fsm = axis_fsm()
    cam2(fsm, 1.0, square=(0.0, 0.7))
    assert fsm.state is State.DELIVER
    cam2(fsm, 1.5, cubes=[Cube("blue", 0.0, 0.8, 800.0)], square=(0.0, 0.8))
    assert fsm.state is State.EXPLORE and "push path" in fsm.note


def test_a_carry_spin_does_not_flip_direction_tick_by_tick():
    fsm = axis_fsm(spin_hold=1.0)
    p = fsm.params
    # (the search spin that started the carry at 0.5 s has its own hold: the test starts after it)
    first = axis_cam(fsm, 2.0, square=(0.5, 0.3))  # right: spin right
    flip = axis_cam(fsm, 2.1, refs=(-0.5, 0.2), square=(0.5, 0.3))  # the rectangles say left
    assert first[2] < 0.0 and flip[2] == first[2]  # held
    later = axis_cam(fsm, 3.2, refs=(-0.5, 0.2), square=(0.5, 0.3))  # the hold is over
    assert later[2] == pytest.approx(p.carry_max_turn)
