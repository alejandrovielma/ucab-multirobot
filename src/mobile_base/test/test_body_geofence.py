"""Rule (user, 2026-10-12): the scoop may step on the tape, the robot body may never cross it.

The geofence checks the body rectangle (wheels included, scoop excluded) along the predicted motion,
not a single point ahead: an arc sweeps the outer front corner well beyond the front edge, and an
in-place spin sweeps every corner on a 12.5 cm circle.
"""

import math
import pathlib
import random

import pytest
import yaml

from mobile_base.core.behavior import BehaviorFSM, BehaviorInput, BehaviorParams, Cube, State, ZoneAction
from mobile_base.core.delivery import DeliveryZone
from mobile_base.core.safety import Pose2D, SafetyGate, SafetyInput, SafetyParams
from mobile_base.core.track import Footprint, Track, path_inside, penetration, predict

CONFIG_DIR = pathlib.Path(__file__).resolve().parents[1] / "config"
# 2 m x 2 m arena: walls at x = 0, x = 2, y = 0, y = 2.
ARENA = Track((0.0, 0.0, 2.0, 0.0, 2.0, 2.0, 0.0, 2.0))
FP = Footprint()
SOUTH = -math.pi / 2


def real_track() -> Track:
    data = yaml.safe_load((CONFIG_DIR / "track.yaml").read_text())
    return Track(data["/**"]["ros__parameters"]["track_polygon"])


# ---- geometry


def test_footprint_matches_the_robot_without_scoop():
    assert FP.front == pytest.approx(0.095) and FP.half_width == pytest.approx(0.082)
    assert FP.sweep_radius == pytest.approx(math.hypot(0.095, 0.082))


def test_corners_rotate_with_the_robot():
    corners = sorted(FP.corners(0.0, 0.0, math.pi / 2))
    assert corners[0] == pytest.approx((-FP.half_width, -FP.rear))
    assert corners[-1] == pytest.approx((FP.half_width, FP.front))


def test_predict_straight_and_arc():
    assert predict((0.0, 0.0, 0.0), 0.1, 0.0, 2.0) == pytest.approx((0.2, 0.0, 0.0))
    x, y, th = predict((0.0, 0.0, 0.0), 0.05, 0.6, math.pi / 2 / 0.6)  # quarter circle, r = 0.0833
    assert (x, y, th) == pytest.approx((0.05 / 0.6, 0.05 / 0.6, math.pi / 2))


def test_distance_to_edge():
    assert ARENA.distance_to_edge(0.3, 1.0) == pytest.approx(0.3)
    assert ARENA.distance_to_edge(1.0, 1.0) == pytest.approx(1.0)


def test_path_inside_catches_the_outer_corner_of_an_arc():
    # Facing the east wall with the front 6 cm from it: going straight 3 cm is fine, but a left
    # arc swings the right front corner into the wall first.
    pose = (2.0 - 0.06 - FP.front, 1.0, 0.0)
    assert path_inside(ARENA, FP, pose, 0.06, 0.0, 0.5)
    assert not path_inside(ARENA, FP, pose, 0.05, 0.6, 2.6)


# ---- gate


def gate_input(pose, cmd, **kw):
    values = dict(cmd=cmd, line_front=False, own_pose=Pose2D(*pose, stamp=10.0), peer_pose=None, now=10.0,
                  heartbeat_age=0.1)
    values.update(kw)
    return SafetyInput(**values)


def test_gate_forward_stops_before_the_body_crosses():
    gate = SafetyGate(SafetyParams(), track=ARENA)
    near = (2.0 - 0.02 - FP.front, 1.0, 0.0)  # front 2 cm from the edge; 0.3 s at 0.12 m/s is 3.6 cm
    cmd, reason = gate.filter(gate_input(near, (0.12, 0.0, 0.0)))
    assert cmd[0] == pytest.approx(0.0)
    assert "track edge" in reason
    close_but_ok = (2.0 - 0.05 - FP.front, 1.0, 0.0)  # 5 cm: no spin room needed to go on
    assert gate.filter(gate_input(close_but_ok, (0.12, 0.0, 0.0)))[0][0] == pytest.approx(0.12)


def test_gate_in_place_spin_that_would_cross_the_tape_is_refused():
    gate = SafetyGate(SafetyParams(), track=ARENA)
    # Front 1 cm from the wall: a spin swings the front corners over it.
    pose = (2.0 - 0.01 - FP.front, 1.0, 0.0)
    cmd, reason = gate.filter(gate_input(pose, (0.0, 0.0, 1.2)))
    assert cmd == pytest.approx((0.0, 0.0, 0.0))
    assert "body" in reason


def test_gate_reverse_allowed_when_the_pose_shows_room_behind():
    params = SafetyParams()
    gate = SafetyGate(params, track=ARENA)
    pose = (2.0 - 0.01 - FP.front, 1.0, 0.0)
    cmd, reason = gate.filter(gate_input(pose, (-0.2, 0.0, 0.0)))
    assert cmd == pytest.approx((-params.max_reverse, 0.0, 0.0))
    assert "reverse" in reason


def test_gate_reverse_refused_without_track_or_while_carrying():
    pose = (1.0, 1.0, 0.0)
    no_track = SafetyGate(SafetyParams())
    assert no_track.filter(gate_input(pose, (-0.04, 0.0, 0.0)))[0][0] == pytest.approx(0.0)
    with_track = SafetyGate(SafetyParams(), track=ARENA)
    assert with_track.filter(gate_input(pose, (-0.04, 0.0, 0.0), carrying=True))[0][0] == pytest.approx(0.0)


def test_gate_carrying_arc_may_take_the_scoop_over_the_tape_but_not_the_body():
    params = SafetyParams()
    gate = SafetyGate(params, track=ARENA)
    # Body front 4 cm from the wall: the scoop (to 0.20 m) is already over it; a short arc keeps the body in.
    pose = (2.0 - 0.04 - FP.front, 1.0, 0.0)
    cmd, _ = gate.filter(gate_input(pose, (0.05, 0.0, 0.6), carrying=True))
    assert cmd == pytest.approx((0.05, 0.0, 0.6))


def test_gate_carrying_takes_the_other_arc_when_one_side_is_blocked():
    gate = SafetyGate(SafetyParams(), track=ARENA)
    # Facing 10 deg east of north with the highest corner 6 mm from the north wall: the right arc
    # swings it into the wall, the left arc does not (computed with path_inside).
    th = math.radians(80.0)
    top = max(cy for _, cy in FP.corners(1.0, 0.0, th))
    pose = (1.0, 2.0 - 0.006 - top, th)
    cmd, reason = gate.filter(gate_input(pose, (0.05, 0.0, -0.6), carrying=True))
    assert cmd == pytest.approx((0.05, 0.0, 0.6))
    assert "other way" in reason


def test_gate_carrying_stops_when_no_arc_keeps_the_body_inside():
    gate = SafetyGate(SafetyParams(), track=ARENA)
    pose = (2.0 - 0.005 - FP.front, 1.0, 0.0)
    cmd, reason = gate.filter(gate_input(pose, (0.05, 0.0, 0.6), carrying=True))
    assert cmd == pytest.approx((0.0, 0.0, 0.0))


# ---- behavior


# Square at y 0.0-0.8 with the boundary at y 0.8, approached heading south: entry y 1.2, drop 0.9, exit 1.1.
ZONE = dict(zone_x=1.0, zone_y=0.4, zone_heading=-math.pi / 2, zone_square_depth=0.8, zone_entry_offset=0.4,
            zone_drop_offset=0.1, zone_exit_offset=0.3)  # inside ARENA


def fsm(**kw):
    return BehaviorFSM(BehaviorParams(my_colors=("red",), **ZONE, **kw), rng=random.Random(5), track=ARENA)


def step(f, now, pose, cubes=(), scoop="", line_front=False, holds_zone=False):
    return f.step(BehaviorInput(now=now, cubes=list(cubes), line_front=line_front, pose=pose, scoop_color=scoop,
                                holds_zone=holds_zone))


def test_explore_turns_before_the_body_reaches_the_tape():
    f = fsm()
    cmd = step(f, 0.0, (2.0 - 0.128, 1.0, 0.0))  # 3.6 cm ahead would cross; spin radius 12.55 cm fits
    assert cmd[0] == 0.0 and cmd[2] != 0.0


def test_explore_backs_off_when_there_is_no_room_to_turn():
    f = fsm()
    cmd = step(f, 0.0, (2.0 - 0.01 - FP.front, 1.0, 0.0), line_front=True)
    assert cmd == pytest.approx((-f.params.reverse_speed, 0.0, 0.0))
    # Still too close after a while: keep backing off; with room, the spin starts.
    assert step(f, 0.5, (2.0 - 0.115, 1.0, 0.0))[0] < 0.0
    cmd = step(f, 1.0, (2.0 - FP.sweep_radius - 0.001, 1.0, 0.0))
    assert cmd[0] == 0.0 and cmd[2] != 0.0


def test_spin_room_is_the_full_sweep_circle_not_the_next_instant():
    # Webots run 12: 0.3 s of rotation fitted, the whole turn did not, and the robot rocked in a corner.
    f = fsm()
    pose = (2.0 - 0.115, 1.0, math.pi / 2)  # parallel to the wall: 20 deg fit, a full spin does not
    assert path_inside(ARENA, FP, pose, 0.0, 1.2, 0.3)
    cmd = step(f, 0.0, pose, line_front=True)
    assert cmd[0] < 0.0


def test_capture_skipped_when_no_plan_could_leave_the_tape_afterwards():
    f = fsm()
    pose = (2.0 - 0.10 - FP.front, 1.0, 0.0)  # cube right at the wall, robot facing it
    step(f, 0.0, pose, cubes=[Cube("red", 0.0, 0.3, 400.0)])
    step(f, 0.1, pose, cubes=[Cube("red", 0.0, 0.7, 3000.0)])
    assert f.state is State.EXPLORE
    # The skipped cube is ignored for a while instead of being approached again at once.
    step(f, 0.2, (1.0, 1.0, 0.0), cubes=[Cube("red", 0.0, 0.3, 400.0)])
    assert f.state is State.EXPLORE


def test_capture_allowed_when_a_plan_to_the_square_exists():
    f = fsm()
    pose = (1.0, 1.0, 0.0)
    step(f, 0.0, pose, cubes=[Cube("red", 0.0, 0.3, 400.0)])
    step(f, 0.1, pose, cubes=[Cube("red", 0.0, 0.7, 3000.0)])
    assert f.state is State.CAPTURE


def test_carry_with_no_plan_drops_the_cube_by_backing_off():
    f = fsm(capture_time=0.1)
    step(f, 0.0, (1.0, 1.0, 0.0), cubes=[Cube("red", 0.0, 0.3, 400.0)])
    step(f, 0.1, (1.0, 1.0, 0.0), cubes=[Cube("red", 0.0, 0.7, 3000.0)])
    step(f, 0.3, (1.0, 1.0, 0.0), scoop="red")
    assert f.state is State.CARRY
    f.take_actions()
    stuck = (2.0 - 0.005 - FP.front, 1.0, 0.0)
    # Webots run 16: one failed replan dropped the cube 27 times; holding still and retrying is safe.
    assert step(f, 0.4, stuck, scoop="red", holds_zone=True) == (0.0, 0.0, 0.0)
    assert f.state is State.CARRY
    cmd = step(f, 0.5 + f.params.plan_give_up, stuck, scoop="red", holds_zone=True)
    assert f.state is State.EXPLORE
    assert not f.carrying
    assert cmd[0] < 0.0  # reverse leaves the cube; a spin here would put a corner over the tape
    assert f.take_actions() == [ZoneAction("release", delivered=False, color="red")]


# ---- delivery geometry on the real track (config/track.yaml)


def test_drop_point_keeps_the_body_out_of_the_square_and_the_cube_in():
    z = DeliveryZone()
    track = real_track()
    assert track.enabled
    drop = z.pose(z.drop_offset)
    assert all(track.contains(cx, cy) for cx, cy in FP.corners(*drop))  # body before the tape
    assert penetration(track, FP, drop) == 0.0
    cx, cy = z.cube_drop_point
    # Whole 2 cm cube (5 mm to spare) inside the blue interior x -0.788..-0.585, y -0.075..0.092.
    assert -0.788 + 0.015 <= cx <= -0.585 - 0.015
    assert -0.075 + 0.015 <= cy <= 0.092 - 0.015


def test_border_turn_points_to_open_space_not_a_random_heading():
    # Kinematic harness (Webots track): a random 135-225 deg turn in the narrow waist often pointed
    # at the tape again, and the robot spun in place for 16 s. In a 0.4 m wide corridor facing a
    # side wall, the open directions are along the corridor (+-90 deg), not back at the other wall.
    corridor = Track((0.0, 0.0, 0.4, 0.0, 0.4, 2.0, 0.0, 2.0))
    for seed in range(8):
        f = BehaviorFSM(BehaviorParams(my_colors=("red",), **ZONE), rng=random.Random(seed), track=corridor)
        pose = (0.2, 1.0, 0.0)
        f.step(BehaviorInput(now=0.0, cubes=[], line_front=True, pose=pose))
        t, theta = 0.1, 0.0
        while f._turn_until is not None and t < 10.0:
            cmd = f.step(BehaviorInput(now=t, cubes=[], line_front=False, pose=(0.2, 1.0, theta)))
            theta += cmd[2] * 0.1
            t += 0.1
        assert abs(abs(math.remainder(theta, 2 * math.pi)) - math.pi / 2) < math.radians(35), seed


def test_approach_near_the_tape_turns_toward_a_side_cube_instead_of_giving_up():
    # Webots run 15: 69 approaches abandoned as "border before the cube" because the body was a few
    # cm from the tape, although the cube was inside the track and off to one side.
    f = fsm()
    pose = (2.0 - 0.128, 1.0, 0.0)  # straight ahead does not fit, a spin does
    step(f, 0.0, pose, cubes=[Cube("red", 0.0, 0.3, 400.0)])
    cmd = step(f, 0.1, pose, cubes=[Cube("red", 0.6, 0.4, 600.0)])  # cube to the right
    assert f.state is State.APPROACH
    assert cmd[2] < 0.0  # turning toward it (a slow arc or in place, whatever keeps the body in)
    assert path_inside(ARENA, FP, pose, cmd[0], cmd[2], 0.3)


def test_approach_gives_up_when_the_cube_is_ahead_and_the_body_cannot_advance():
    f = fsm()
    pose = (2.0 - 0.12, 1.0, 0.0)  # front 2.5 cm from the wall
    step(f, 0.0, pose, cubes=[Cube("red", 0.0, 0.3, 400.0)])
    step(f, 0.1, pose, cubes=[Cube("red", 0.02, 0.4, 600.0)])
    assert f.state is State.EXPLORE


# ---- Webots run 19: a protruding track corner inside the body, and pose noise at the edge

# Arena with a notch: the outside pokes into the track as a triangle with its tip at (1.0, 1.0).
NOTCH = Track((0.0, 0.0, 2.0, 0.0, 2.0, 2.0, 1.1, 2.0, 1.0, 1.0, 0.9, 2.0, 0.0, 2.0))


def test_penetration_counts_a_track_vertex_inside_the_body():
    from mobile_base.core.track import penetration

    pose = (1.0, 0.95, 0.0)  # the notch tip (1.0, 1.0) is 5 cm left of the center: inside the body
    assert all(NOTCH.contains(cx, cy) for cx, cy in FP.corners(*pose))  # the old corner check passed
    assert penetration(NOTCH, FP, pose) == pytest.approx(FP.half_width - 0.05, abs=1e-6)
    assert penetration(NOTCH, FP, (1.0, 0.5, 0.0)) == 0.0


def test_motion_into_a_protruding_corner_is_refused():
    pose = (1.0, 0.89, math.pi / 2)  # heading north: 3.6 cm more puts the notch tip 2 cm into the body
    assert not path_inside(NOTCH, FP, pose, 0.12, 0.0, 0.3)


def test_a_robot_already_touching_the_edge_can_still_move_away():
    # With 5 mm pose noise a robot against the tape reads slightly outside; refusing every command
    # froze both robots for minutes. Motions that do not deepen the penetration are allowed.
    pose = (2.0 - FP.front + 0.004, 1.0, 0.0)  # front 4 mm over the edge (noise)
    assert path_inside(ARENA, FP, pose, -0.04, 0.0, 0.3)  # backing away
    assert not path_inside(ARENA, FP, pose, 0.05, 0.0, 0.3)  # pushing deeper
