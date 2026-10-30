import math

import pytest

from mobile_base.core.delivery import DeliveryZone
from mobile_base.core.safety import Pose2D, SafetyGate, SafetyInput, SafetyParams

FORWARD = (0.2, 0.0, 0.0)


def make_input(**overrides):
    values = dict(
        cmd=FORWARD,
        line_front=False,
        own_pose=Pose2D(0.0, 0.0, 0.0, stamp=10.0),
        peer_pose=None,
        now=10.0,
        heartbeat_age=0.1,
        carrying=False,
    )
    values.update(overrides)
    return SafetyInput(**values)


def peer_at(x, y, theta=math.pi, stamp=10.0):
    return Pose2D(x, y, theta, stamp=stamp)


def test_passes_command_when_everything_is_fine():
    cmd, reason = SafetyGate(SafetyParams()).filter(make_input())
    assert cmd == pytest.approx(FORWARD)
    assert reason == "ok"


def test_emergency_stop_wins_over_everything_and_clears():
    gate = SafetyGate(SafetyParams(require_heartbeat=False, require_pose=False))
    for cmd in (FORWARD, (0.0, 0.0, 1.0), (-0.05, 0.0, 0.0)):
        assert gate.filter(make_input(cmd=cmd, estop=True)) == ((0.0, 0.0, 0.0), "stop: emergency stop")
    assert gate.filter(make_input())[0] == pytest.approx(FORWARD)


def test_emergency_stop_cancels_a_deadlock_escape():
    gate = SafetyGate(SafetyParams(has_priority=False, deadlock_time=1.0, escape_time=1.5))

    def at(t, **kw):  # peer dead ahead at 0.3 m, facing us
        return make_input(now=t, own_pose=Pose2D(0.0, 0.0, 0.0, stamp=t), peer_pose=peer_at(0.3, 0.0, stamp=t), **kw)

    for t in (10.0, 10.5):
        gate.filter(at(t))
    assert "deadlock" in gate.filter(at(11.1))[1]  # escape running until 12.6
    gate.filter(at(11.2, estop=True))
    # Still pushing forward against the peer: without the reset the escape would restart at once.
    assert "deadlock" not in gate.filter(at(11.3))[1]
    assert "deadlock" not in gate.filter(at(12.2))[1]
    assert "deadlock" in gate.filter(at(12.4))[1]  # a fresh deadlock_time after the stop


def test_stale_heartbeat_stops():
    cmd, reason = SafetyGate(SafetyParams()).filter(make_input(heartbeat_age=2.5))
    assert cmd == (0.0, 0.0, 0.0)
    assert "heartbeat" in reason


def test_missing_heartbeat_stops():
    cmd, _ = SafetyGate(SafetyParams()).filter(make_input(heartbeat_age=None))
    assert cmd == (0.0, 0.0, 0.0)


def test_heartbeat_ignored_when_not_required():
    gate = SafetyGate(SafetyParams(require_heartbeat=False))
    cmd, _ = gate.filter(make_input(heartbeat_age=None))
    assert cmd == pytest.approx(FORWARD)


def test_stale_own_pose_stops():
    cmd, reason = SafetyGate(SafetyParams()).filter(
        make_input(own_pose=Pose2D(0.0, 0.0, 0.0, stamp=9.0))
    )
    assert cmd == (0.0, 0.0, 0.0)
    assert "pose" in reason


def test_missing_own_pose_stops_only_when_required():
    assert SafetyGate(SafetyParams()).filter(make_input(own_pose=None))[0] == (0.0, 0.0, 0.0)
    gate = SafetyGate(SafetyParams(require_pose=False))
    assert gate.filter(make_input(own_pose=None))[0] == pytest.approx(FORWARD)


def test_line_blocks_translation_toward_the_tape_but_allows_a_spin():
    gate = SafetyGate(SafetyParams())
    cmd, reason = gate.filter(make_input(cmd=(0.2, 0.0, 0.0), line_front=True))
    assert cmd == pytest.approx((0.0, 0.0, 0.0))
    assert "line" in reason
    assert gate.filter(make_input(cmd=(0.0, 0.0, 0.5), line_front=True))[0] == pytest.approx((0.0, 0.0, 0.5))


# Star motion (team rule 2026-10-27): translate in any direction or spin in place, never both.
def test_translation_with_a_spin_keeps_only_the_spin():
    cmd, reason = SafetyGate(SafetyParams()).filter(make_input(cmd=(0.1, 0.05, 0.5)))
    assert cmd == pytest.approx((0.0, 0.0, 0.5))
    assert "spin in place only" in reason


@pytest.mark.parametrize("cmd", [(0.1, 0.1, 0.0), (0.0, -0.2, 0.0), (-0.1, 0.0, 0.0), (-0.1, -0.1, 0.0)])
def test_translation_that_is_not_straight_ahead_passes_slowly_keeping_its_direction(cmd):
    # The only line sensor is at the center: sideways, diagonal or backward it sees the tape late.
    params = SafetyParams()
    out, reason = SafetyGate(params).filter(make_input(cmd=cmd))
    assert math.hypot(out[0], out[1]) == pytest.approx(params.max_side_speed)
    assert math.atan2(out[1], out[0]) == pytest.approx(math.atan2(cmd[1], cmd[0]))
    assert "slow" in reason


def test_slow_sideways_translation_passes_unchanged():
    assert SafetyGate(SafetyParams()).filter(make_input(cmd=(0.0, 0.03, 0.0)))[0] == pytest.approx((0.0, 0.03, 0.0))


def test_on_the_line_the_robot_may_back_away_the_way_it_came():
    gate = SafetyGate(SafetyParams(require_pose=False))
    gate.filter(make_input(cmd=(0.0, 0.05, 0.0), own_pose=None))  # strafing left onto the tape
    cmd, _ = gate.filter(make_input(cmd=(0.0, 0.05, 0.0), line_front=True, own_pose=None))
    assert cmd == pytest.approx((0.0, 0.0, 0.0))  # no further left
    cmd, _ = gate.filter(make_input(cmd=(0.0, -0.05, 0.0), line_front=True, own_pose=None))
    assert cmd == pytest.approx((0.0, -0.05, 0.0))  # back to the right


def test_backing_off_one_tape_does_not_cross_another_behind():
    # A corner: the robot backs off the tape it drove onto and meets the other one behind it.
    gate = SafetyGate(SafetyParams(require_pose=False))
    back = (-0.04, 0.0, 0.0)

    def at(t, cmd, line):
        return gate.filter(make_input(now=t, cmd=cmd, line_front=line, own_pose=None))

    at(10.0, FORWARD, False)
    assert at(10.1, back, True)[0] == pytest.approx(back)
    assert at(10.5, back, False)[0] == pytest.approx(back)  # off the first tape
    cmd, reason = at(11.5, back, True)  # the second one, a second later
    assert cmd == pytest.approx((0.0, 0.0, 0.0))
    assert "line" in reason


def test_the_tape_direction_turns_with_the_robot_without_pose():
    # Reviewer's case: after a half turn on the tape the tape is BEHIND; the old rule (fixed in the
    # robot frame) let the robot back into it and blocked driving away (and pinned it in corners).
    gate = SafetyGate(SafetyParams(require_pose=False))
    gate.filter(make_input(now=10.0, cmd=FORWARD, own_pose=None))
    gate.filter(make_input(now=10.1, cmd=FORWARD, line_front=True, own_pose=None))  # tape ahead
    t = 10.1
    while t < 10.1 + math.pi / 1.2:  # spin half a turn at 1.2 rad/s, still on the tape
        gate.filter(make_input(now=t, cmd=(0.0, 0.0, 1.2), line_front=True, own_pose=None))
        t += 0.05
    t = 10.1 + math.pi / 1.2
    assert gate.filter(make_input(now=t, cmd=(-0.04, 0.0, 0.0), line_front=True, own_pose=None))[0] == pytest.approx((0.0, 0.0, 0.0))
    assert gate.filter(make_input(now=t, cmd=FORWARD, line_front=True, own_pose=None))[0] == pytest.approx(FORWARD)


def test_the_tape_direction_turns_with_the_pose_heading():
    gate = SafetyGate(SafetyParams())
    gate.filter(make_input(now=10.0, cmd=FORWARD, own_pose=Pose2D(0.0, 0.0, 0.0, stamp=10.0)))
    gate.filter(make_input(now=10.1, cmd=FORWARD, line_front=True, own_pose=Pose2D(0.0, 0.0, 0.0, stamp=10.1)))
    # The pose says the robot now faces north: the tape (east) is on its right.
    turned = Pose2D(0.0, 0.0, math.pi / 2, stamp=10.5)
    right, left = (0.0, -0.04, 0.0), (0.0, 0.04, 0.0)
    assert gate.filter(make_input(now=10.5, cmd=right, line_front=True, own_pose=turned))[0] == pytest.approx((0.0, 0.0, 0.0))
    assert gate.filter(make_input(now=10.5, cmd=left, line_front=True, own_pose=turned))[0] == pytest.approx(left)


def test_a_flicker_on_the_tape_edge_is_the_same_tape():
    # TCRT5000 without debounce: one white reading while backing off must not re-aim the rule.
    gate = SafetyGate(SafetyParams(require_pose=False))
    back = (-0.04, 0.0, 0.0)
    gate.filter(make_input(now=10.0, cmd=FORWARD, own_pose=None))
    gate.filter(make_input(now=10.1, cmd=back, line_front=True, own_pose=None))
    gate.filter(make_input(now=10.2, cmd=back, line_front=False, own_pose=None))  # flicker
    assert gate.filter(make_input(now=10.3, cmd=back, line_front=True, own_pose=None))[0] == pytest.approx(back)
    assert gate.filter(make_input(now=10.4, cmd=FORWARD, line_front=True, own_pose=None))[0] == pytest.approx((0.0, 0.0, 0.0))


def test_backing_toward_a_peer_behind_stops_without_swinging_the_rear_into_it():
    params = SafetyParams()
    behind_left = peer_at(-0.25, 0.15, theta=0.0)
    cmd, _ = SafetyGate(params).filter(make_input(cmd=(-0.04, 0.0, 0.0), peer_pose=behind_left))
    assert cmd == pytest.approx((0.0, 0.0, 0.0))


def test_peer_very_close_in_front_blocks_forward_and_turns_away():
    cmd, reason = SafetyGate(SafetyParams()).filter(make_input(peer_pose=peer_at(0.3, 0.0)))
    assert cmd[0] == pytest.approx(0.0)
    assert cmd[1] == pytest.approx(0.0)
    assert cmd[2] != 0.0
    assert "peer" in reason


def test_peer_in_avoid_band_makes_the_robot_spin_away_from_it():
    params = SafetyParams()
    mid = (params.stop_distance + params.slow_distance) / 2
    cmd, _ = SafetyGate(params).filter(make_input(peer_pose=peer_at(mid, 0.05)))
    assert cmd == pytest.approx((0.0, 0.0, -params.avoid_turn))  # peer slightly left -> spin right


def test_peer_in_slow_band_scales_the_translation_toward_it():
    params = SafetyParams(avoid_distance=0.0)  # isolate the slow-down from the turn-away
    mid = (params.stop_distance + params.slow_distance) / 2
    cmd, _ = SafetyGate(params).filter(make_input(peer_pose=peer_at(mid, 0.0)))
    assert cmd == pytest.approx((FORWARD[0] * 0.5, 0.0, 0.0), abs=0.02)


def test_moving_away_from_a_close_peer_is_not_held_back():
    cmd, _ = SafetyGate(SafetyParams()).filter(make_input(cmd=(-0.04, 0.0, 0.0), peer_pose=peer_at(0.3, 0.0)))
    assert cmd == pytest.approx((-0.04, 0.0, 0.0))


def test_peer_slightly_right_turns_left():
    params = SafetyParams()
    cmd, _ = SafetyGate(params).filter(make_input(peer_pose=peer_at(0.45, -0.05)))
    assert cmd[2] == pytest.approx(params.avoid_turn)


def test_contact_distance_accounts_for_scoop_length():
    # Center-to-center distance when scoop tips touch is ~0.38 m; stopping must happen before that.
    assert SafetyParams().stop_distance >= 0.35


def test_peer_behind_is_ignored():
    cmd, _ = SafetyGate(SafetyParams()).filter(make_input(peer_pose=peer_at(-0.2, 0.0)))
    assert cmd == pytest.approx(FORWARD)


def test_peer_far_ahead_is_ignored():
    cmd, _ = SafetyGate(SafetyParams()).filter(make_input(peer_pose=peer_at(1.0, 0.0)))
    assert cmd == pytest.approx(FORWARD)


def test_stale_peer_pose_is_ignored():
    cmd, _ = SafetyGate(SafetyParams()).filter(
        make_input(peer_pose=peer_at(0.2, 0.0, stamp=5.0))
    )
    assert cmd == pytest.approx(FORWARD)


def _run_deadlock(gate, start, duration, carrying=False, step=0.1):
    t = start
    result = None
    while t <= start + duration + 1e-9:
        result = gate.filter(
            make_input(
                now=t,
                own_pose=Pose2D(0.0, 0.0, 0.0, stamp=t),
                peer_pose=peer_at(0.2, 0.0, stamp=t),
                carrying=carrying,
            )
        )
        t += step
    return result


def test_deadlock_robot_without_priority_rotates_away_without_reversing():
    params = SafetyParams(has_priority=False)
    cmd, reason = _run_deadlock(SafetyGate(params), 10.0, params.deadlock_time + 0.2)
    assert cmd[0] == pytest.approx(0.0)
    assert cmd[2] != 0.0
    assert "rotate away" in reason


def test_deadlock_robot_with_priority_waits():
    params = SafetyParams(has_priority=True)
    cmd, _ = _run_deadlock(SafetyGate(params), 10.0, params.deadlock_time + 0.2)
    assert cmd[0] == pytest.approx(0.0)


def test_deadlock_while_carrying_holds_still():
    # Reversing leaves the cube behind and spinning in place ejects it from the fixed scoop.
    params = SafetyParams(has_priority=False)
    cmd, reason = _run_deadlock(SafetyGate(params), 10.0, params.deadlock_time + 0.2, carrying=True)
    assert cmd == pytest.approx((0.0, 0.0, 0.0))
    assert "hold" in reason


def test_carrying_spins_in_place_but_slowly():
    # A fast spin throws the cube out of the fixed U; the behavior recaptures it if it slips.
    params = SafetyParams()
    cmd, reason = SafetyGate(params).filter(make_input(cmd=(0.0, 0.0, 1.2), carrying=True))
    assert cmd == pytest.approx((0.0, 0.0, params.carry_max_turn))
    assert "slow spin" in reason


def test_carrying_limits_turn_rate():
    params = SafetyParams()
    cmd, _ = SafetyGate(params).filter(make_input(cmd=(0.1, 0.0, -3.0), carrying=True))
    assert cmd[2] == pytest.approx(-params.carry_max_turn)


def test_carrying_never_strafes_even_to_avoid_peer():
    params = SafetyParams()
    cmd, _ = SafetyGate(params).filter(
        make_input(cmd=(0.1, 0.05, 0.0), peer_pose=peer_at(0.4, 0.0), carrying=True)
    )
    assert cmd[1] == pytest.approx(0.0)


def test_carrying_spins_slowly_away_from_a_peer_dead_ahead():
    params = SafetyParams()
    cmd, _ = SafetyGate(params).filter(make_input(cmd=(0.1, 0.0, 0.0), peer_pose=peer_at(0.3, 0.0), carrying=True))
    assert cmd == pytest.approx((0.0, 0.0, -params.carry_max_turn))


def test_carrying_at_the_line_spins_in_place_even_if_the_cube_drops():
    # Webots run 5: moving on with a cube crossed the tape and the robot left the board.
    # Losing the cube at the border is recoverable; leaving the track is not.
    params = SafetyParams()
    cmd, reason = SafetyGate(params).filter(make_input(cmd=(0.1, 0.0, 0.0), line_front=True, carrying=True))
    assert cmd == pytest.approx((0.0, 0.0, 0.0))
    assert "line ahead" in reason
    cmd, _ = SafetyGate(params).filter(make_input(cmd=(0.0, 0.0, 0.5), line_front=True, carrying=True))
    assert cmd == pytest.approx((0.0, 0.0, params.carry_max_turn))


def test_carrying_never_moves_forward_over_the_line_to_avoid_a_peer():
    peer = peer_at(0.36 * math.cos(math.radians(38)), 0.36 * math.sin(math.radians(38)), theta=math.pi / 2)
    cmd, _ = SafetyGate(SafetyParams()).filter(
        make_input(cmd=(0.08, 0.0, 0.4), peer_pose=peer, line_front=True, carrying=True)
    )
    assert cmd[0] == pytest.approx(0.0)


def test_carrying_spins_away_from_a_peer_already_touching():
    params = SafetyParams()
    peer = peer_at(0.2 * math.cos(math.radians(40)), 0.2 * math.sin(math.radians(40)), theta=math.pi / 2)
    cmd, _ = SafetyGate(params).filter(make_input(cmd=(0.08, 0.0, 0.0), peer_pose=peer, carrying=True))
    assert cmd == pytest.approx((0.0, 0.0, -params.carry_max_turn))  # swings the scoop away from it


def test_not_carrying_keeps_in_place_rotation():
    cmd, _ = SafetyGate(SafetyParams()).filter(make_input(cmd=(0.0, 0.0, 0.5)))
    assert cmd == pytest.approx((0.0, 0.0, 0.5))


def test_escape_ends_after_escape_time():
    params = SafetyParams(has_priority=False)
    gate = SafetyGate(params)
    end = 10.0 + params.deadlock_time + 0.2
    _run_deadlock(gate, 10.0, params.deadlock_time + 0.2)
    later = end + params.escape_time + 0.1
    cmd, _ = gate.filter(
        make_input(now=later, own_pose=Pose2D(0.0, 0.0, 0.0, stamp=later), peer_pose=None)
    )
    assert cmd == pytest.approx(FORWARD)


# ---- Delivery square: the gate decides from its own pose and /zone/occupant, not from the behavior.
ZONE = DeliveryZone()
WEST = math.pi
IN_CORRIDOR = Pose2D(*ZONE.pose(0.15), stamp=10.0)  # facing the square, between entry and drop


def test_reverse_out_of_the_square_is_slow():
    params = SafetyParams()
    cmd, _ = SafetyGate(params).filter(make_input(cmd=(-0.5, 0.0, 0.0), own_pose=IN_CORRIDOR, holds_zone=True))
    assert cmd == pytest.approx((-params.max_side_speed, 0.0, 0.0))


def test_square_tape_crossed_with_lease_while_carrying():
    cmd, reason = SafetyGate(SafetyParams()).filter(
        make_input(cmd=(0.04, 0.01, 0.0), line_front=True, own_pose=IN_CORRIDOR, holds_zone=True, carrying=True)
    )
    assert cmd == pytest.approx((0.04, 0.01, 0.0))
    assert "line ahead" not in reason


def test_square_tape_blocks_without_lease():
    cmd, _ = SafetyGate(SafetyParams()).filter(
        make_input(cmd=(0.05, 0.0, 0.0), line_front=True, own_pose=IN_CORRIDOR, holds_zone=False)
    )
    assert cmd[0] == pytest.approx(0.0)


def test_carrying_spins_away_from_a_peer_beside_it():
    # Webots run 4: Ferb (carrying) froze for minutes next to a robot 38 deg to its left at 0.36 m.
    params = SafetyParams()
    peer = peer_at(0.36 * math.cos(math.radians(38)), 0.36 * math.sin(math.radians(38)), theta=math.pi / 2)
    cmd, reason = SafetyGate(params).filter(make_input(cmd=(0.08, 0.0, 0.0), peer_pose=peer, carrying=True))
    assert cmd == pytest.approx((0.0, 0.0, -params.carry_max_turn))
    assert "turn away" in reason


def test_carrying_never_translates_into_a_peer_dead_ahead():
    cmd, _ = SafetyGate(SafetyParams()).filter(
        make_input(cmd=(0.08, 0.0, 0.0), peer_pose=peer_at(0.35, 0.02), carrying=True)
    )
    assert cmd[0] == pytest.approx(0.0) and cmd[1] == pytest.approx(0.0)


def test_delivering_robot_is_not_turned_away_by_the_peer_outside_the_corridor():
    # Webots run 17: Ferb, entering the square with the lease, was told to "turn away" from Phineas
    # waiting 0.48 m off; the body geofence forbade the turn and Ferb froze for 8 minutes.
    params = SafetyParams()
    own = Pose2D(*ZONE.pose(0.13), stamp=10.0)
    # 0.47 m away, inside the front cone (0.4 m ahead along the approach, 0.25 m to the left).
    peer = Pose2D(own.x - 0.4, own.y - 0.25, 0.0, stamp=10.0)
    cmd, reason = SafetyGate(params).filter(
        make_input(cmd=(0.05, 0.0, 0.0), own_pose=own, peer_pose=peer, holds_zone=True, carrying=True)
    )
    assert cmd[2] == pytest.approx(0.0)
    assert "turn away" not in reason


def test_lease_holder_is_not_turned_away_by_the_waiting_robot():
    # Webots run 20: Ferb held the lease and lined up at the entry; Phineas, waiting 0.5 m away, kept
    # triggering "turn away", the plan drifted and was replanned for 10 minutes (livelock).
    own = Pose2D(0.0, 0.1, math.pi, stamp=10.0)
    peer = Pose2D(-0.48, 0.05, 0.0, stamp=10.0)
    cmd, reason = SafetyGate(SafetyParams()).filter(
        make_input(cmd=(0.0, 0.0, 1.2), own_pose=own, peer_pose=peer, holds_zone=True, carrying=True)
    )
    assert cmd[2] == pytest.approx(SafetyParams().carry_max_turn)  # its own (carry-limited) turn, not the avoidance turn
    assert "turn away" not in reason
    # Still stops before contact.
    close = Pose2D(-0.35, 0.1, 0.0, stamp=10.0)
    cmd, _ = SafetyGate(SafetyParams()).filter(
        make_input(cmd=(0.1, 0.0, 0.0), own_pose=own, peer_pose=close, holds_zone=True)
    )
    assert cmd[0] == pytest.approx(0.0)


def test_without_poses_the_peer_seen_by_the_camera_stops_and_turns_the_robot_away():
    # No overhead camera (2026-10-27): the other robot's marker, seen ahead and close.
    params = SafetyParams(require_pose=False)
    cmd, reason = SafetyGate(params).filter(make_input(own_pose=None, peer_seen=(0.1, 0.3)))
    assert cmd == pytest.approx((0.0, 0.0, -params.avoid_turn))  # slightly left: stop and spin right
    assert "peer" in reason
    far, _ = SafetyGate(params).filter(make_input(own_pose=None, peer_seen=(0.0, 1.0)))
    assert far == pytest.approx(FORWARD)
    beside, _ = SafetyGate(params).filter(make_input(own_pose=None, peer_seen=(math.radians(80), 0.3)))
    assert beside == pytest.approx(FORWARD)  # outside the front cone: not in the way


def _slew_gate(**kw):
    return SafetyGate(SafetyParams(require_pose=False, require_heartbeat=False, **kw))


def _slew_inp(cmd, now, carrying=False, line=False, estop=False):
    return SafetyInput(cmd=cmd, line_front=line, own_pose=None, peer_pose=None, now=now, heartbeat_age=None,
                       carrying=carrying, estop=estop)


def test_slew_ramps_up_and_empty_robot_stops_at_once():
    gate = _slew_gate(max_accel=0.4, max_decel_carry=0.3)
    first = gate.filter(_slew_inp((0.08, 0.0, 0.0), 0.0))[0]
    assert 0.0 < first[0] <= 0.4 * 0.1 + 1e-9
    for i in range(1, 4):
        out = gate.filter(_slew_inp((0.08, 0.0, 0.0), 0.1 * i))[0]
    assert out[0] == pytest.approx(0.08)
    assert gate.filter(_slew_inp((0.0, 0.0, 0.0), 0.5))[0] == (0.0, 0.0, 0.0)


def test_carrying_brakes_gradually_but_line_and_estop_stop_at_once():
    gate = _slew_gate(max_accel=1.0, max_decel_carry=0.3)
    for i in range(3):
        gate.filter(_slew_inp((0.08, 0.0, 0.0), 0.1 * i, carrying=True))
    slowed = gate.filter(_slew_inp((0.0, 0.0, 0.0), 0.3, carrying=True))[0]
    assert 0.0 < slowed[0] < 0.08  # a cube in the scoop: no instant stop, it would slide out
    gate = _slew_gate(max_accel=1.0, max_decel_carry=0.3)
    for i in range(3):
        gate.filter(_slew_inp((0.08, 0.0, 0.0), 0.1 * i, carrying=True))
    assert gate.filter(_slew_inp((0.08, 0.0, 0.0), 0.3, carrying=True, line=True))[0][0] == 0.0
    gate = _slew_gate(max_accel=1.0, max_decel_carry=0.3)
    for i in range(3):
        gate.filter(_slew_inp((0.08, 0.0, 0.0), 0.1 * i, carrying=True))
    assert gate.filter(_slew_inp((0.08, 0.0, 0.0), 0.3, carrying=True, estop=True))[0] == (0.0, 0.0, 0.0)


def test_line_does_not_stop_the_delivery_push_when_the_sequencer_says_so():
    gate = SafetyGate(SafetyParams(require_pose=False, require_heartbeat=False))
    base = dict(cmd=(0.1, 0.0, 0.0), line_front=True, own_pose=None, peer_pose=None, now=0.0, heartbeat_age=None)
    assert gate.filter(SafetyInput(**base))[0][0] == 0.0  # the tape stops a normal advance
    gate = SafetyGate(SafetyParams(require_pose=False, require_heartbeat=False))
    assert gate.filter(SafetyInput(**base, delivering=True))[0][0] == pytest.approx(0.1)


def test_the_peer_that_left_the_view_still_blocks_for_a_while():
    gate = SafetyGate(SafetyParams(require_pose=False, require_heartbeat=False, peer_memory=2.0))
    base = dict(line_front=False, own_pose=None, peer_pose=None, heartbeat_age=None)
    gate.filter(SafetyInput(cmd=(0.0, 0.0, 0.0), now=0.0, peer_seen=(0.0, 0.1), **base))  # right ahead and close
    held = gate.filter(SafetyInput(cmd=(0.1, 0.0, 0.0), now=1.0, **base))[0]
    assert held[0] == 0.0  # still blocked from memory
    free = gate.filter(SafetyInput(cmd=(0.1, 0.0, 0.0), now=3.5, **base))[0]
    assert free[0] == pytest.approx(0.1)  # forgotten after peer_memory


def test_a_braking_ramp_never_mixes_translation_and_spin():
    gate = SafetyGate(SafetyParams(require_pose=False, require_heartbeat=False, max_accel=1.0, max_decel_carry=0.3))
    base = dict(line_front=False, own_pose=None, peer_pose=None, heartbeat_age=None, carrying=True)
    for i in range(3):
        gate.filter(SafetyInput(cmd=(0.08, 0.0, 0.0), now=0.1 * i, **base))
    vx, vy, wz = gate.filter(SafetyInput(cmd=(0.0, 0.0, 0.4), now=0.3, **base))[0]
    assert vx > 0.0 and wz == 0.0  # still braking: no spin yet
