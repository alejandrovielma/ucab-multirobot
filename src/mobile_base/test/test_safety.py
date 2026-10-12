import math

import pytest

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


def test_line_blocks_forward_but_allows_turn():
    cmd, reason = SafetyGate(SafetyParams()).filter(
        make_input(cmd=(0.2, 0.0, 0.5), line_front=True)
    )
    assert cmd == pytest.approx((0.0, 0.0, 0.5))
    assert "line" in reason


# The robot has a single FRONT line sensor: any translation that is not forward is blind to the
# track edge. In Webots, strafing to avoid the peer drove both robots off the board.
def test_strafe_is_never_passed_through():
    cmd, _ = SafetyGate(SafetyParams()).filter(make_input(cmd=(0.1, 0.1, 0.0)))
    assert cmd == pytest.approx((0.1, 0.0, 0.0))


def test_reverse_is_never_passed_through():
    cmd, reason = SafetyGate(SafetyParams()).filter(make_input(cmd=(-0.1, 0.0, 0.0)))
    assert cmd == pytest.approx((0.0, 0.0, 0.0))
    assert "forward only" in reason


def test_peer_very_close_in_front_blocks_forward_and_turns_away():
    cmd, reason = SafetyGate(SafetyParams()).filter(make_input(peer_pose=peer_at(0.3, 0.0)))
    assert cmd[0] == pytest.approx(0.0)
    assert cmd[1] == pytest.approx(0.0)
    assert cmd[2] != 0.0
    assert "peer" in reason


def test_peer_in_slow_band_scales_forward_and_turns_away_from_it():
    params = SafetyParams()
    mid = (params.stop_distance + params.slow_distance) / 2
    cmd, _ = SafetyGate(params).filter(make_input(peer_pose=peer_at(mid, 0.05)))
    assert cmd[0] == pytest.approx(FORWARD[0] * 0.5, abs=0.02)
    assert cmd[1] == pytest.approx(0.0)
    assert cmd[2] == pytest.approx(-params.avoid_turn)  # peer slightly left -> turn right


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


def test_carrying_turns_only_while_moving_forward():
    params = SafetyParams()
    cmd, reason = SafetyGate(params).filter(make_input(cmd=(0.0, 0.0, 0.5), carrying=True))
    assert cmd[0] == pytest.approx(params.carry_min_forward)
    assert cmd[2] == pytest.approx(0.5)
    assert "arc" in reason


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


def test_carrying_stops_turning_when_peer_blocks_forward():
    # The robot cannot advance, so an in-place spin would eject the cube.
    cmd, reason = SafetyGate(SafetyParams()).filter(
        make_input(cmd=(0.1, 0.0, 0.5), peer_pose=peer_at(0.3, 0.0), carrying=True)
    )
    assert cmd == pytest.approx((0.0, 0.0, 0.0))
    assert "no in-place turn" in reason


@pytest.mark.parametrize("wz, sign", [(0.5, 1.0), (-0.2, -1.0), (0.0, 1.0)])
def test_carrying_at_the_line_arcs_off_it_instead_of_freezing(wz, sign):
    # Before: frozen forever (no forward over the line, no in-place spin with a cube).
    params = SafetyParams()
    cmd, reason = SafetyGate(params).filter(make_input(cmd=(0.1, 0.0, wz), line_front=True, carrying=True))
    assert cmd == pytest.approx((params.carry_min_forward, 0.0, sign * params.carry_max_turn))
    assert "arc off the line" in reason


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
SOUTH = -math.pi / 2
IN_CORRIDOR = Pose2D(-0.13, -0.05, SOUTH, stamp=10.0)


def test_reverse_allowed_in_corridor_with_lease_and_bounded():
    params = SafetyParams()
    cmd, reason = SafetyGate(params).filter(
        make_input(cmd=(-0.5, 0.0, 0.0), own_pose=IN_CORRIDOR, holds_zone=True)
    )
    assert cmd == pytest.approx((-params.max_reverse, 0.0, 0.0))
    assert "delivery" in reason


@pytest.mark.parametrize(
    "pose, holds_zone, carrying",
    [
        (IN_CORRIDOR, False, False),  # no lease
        (Pose2D(0.4, 0.0, SOUTH, stamp=10.0), True, False),  # outside the corridor
        (Pose2D(-0.13, -0.05, math.pi / 2, stamp=10.0), True, False),  # facing away from the square
        (IN_CORRIDOR, True, True),  # still carrying: reversing would leave the cube outside
    ],
)
def test_reverse_refused_otherwise(pose, holds_zone, carrying):
    cmd, _ = SafetyGate(SafetyParams()).filter(
        make_input(cmd=(-0.04, 0.0, 0.0), own_pose=pose, holds_zone=holds_zone, carrying=carrying)
    )
    assert cmd[0] >= 0.0


def test_square_tape_crossed_with_lease_while_carrying():
    cmd, reason = SafetyGate(SafetyParams()).filter(
        make_input(cmd=(0.05, 0.0, 0.1), line_front=True, own_pose=IN_CORRIDOR, holds_zone=True, carrying=True)
    )
    assert cmd == pytest.approx((0.05, 0.0, 0.1))
    assert "line ahead" not in reason


def test_square_tape_blocks_without_lease():
    cmd, _ = SafetyGate(SafetyParams()).filter(
        make_input(cmd=(0.05, 0.0, 0.0), line_front=True, own_pose=IN_CORRIDOR, holds_zone=False)
    )
    assert cmd[0] == pytest.approx(0.0)
