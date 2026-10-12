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


def test_line_blocks_forward_but_allows_turn_and_strafe():
    cmd, reason = SafetyGate(SafetyParams()).filter(
        make_input(cmd=(0.2, 0.1, 0.5), line_front=True)
    )
    assert cmd == pytest.approx((0.0, 0.1, 0.5))
    assert "line" in reason


def test_line_allows_reverse():
    cmd, _ = SafetyGate(SafetyParams()).filter(make_input(cmd=(-0.1, 0.0, 0.0), line_front=True))
    assert cmd == pytest.approx((-0.1, 0.0, 0.0))


def test_peer_very_close_in_front_blocks_forward():
    cmd, reason = SafetyGate(SafetyParams()).filter(make_input(peer_pose=peer_at(0.2, 0.0)))
    assert cmd[0] == pytest.approx(0.0)
    assert "peer" in reason


def test_peer_in_slow_band_scales_forward_and_strafes_right():
    params = SafetyParams()
    cmd, _ = SafetyGate(params).filter(make_input(peer_pose=peer_at(0.375, 0.0)))
    assert cmd[0] == pytest.approx(FORWARD[0] * 0.5)
    assert cmd[1] == pytest.approx(-params.avoid_strafe)


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


def test_deadlock_robot_without_priority_backs_off():
    params = SafetyParams(has_priority=False)
    cmd, reason = _run_deadlock(SafetyGate(params), 10.0, params.deadlock_time + 0.2)
    assert cmd[0] == pytest.approx(-params.backoff_speed)
    assert "backoff" in reason


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


def test_carrying_stops_turning_when_forward_is_blocked():
    # With the line ahead the robot cannot advance, so an in-place spin would eject the cube.
    cmd, reason = SafetyGate(SafetyParams()).filter(
        make_input(cmd=(0.0, 0.0, 0.5), line_front=True, carrying=True)
    )
    assert cmd == pytest.approx((0.0, 0.0, 0.0))
    assert "line ahead" in reason


def test_not_carrying_keeps_in_place_rotation():
    cmd, _ = SafetyGate(SafetyParams()).filter(make_input(cmd=(0.0, 0.0, 0.5)))
    assert cmd == pytest.approx((0.0, 0.0, 0.5))


def test_backoff_ends_after_backoff_time():
    params = SafetyParams(has_priority=False)
    gate = SafetyGate(params)
    end = 10.0 + params.deadlock_time + 0.2
    _run_deadlock(gate, 10.0, params.deadlock_time + 0.2)
    later = end + params.backoff_time + 0.1
    cmd, _ = gate.filter(
        make_input(now=later, own_pose=Pose2D(0.0, 0.0, 0.0, stamp=later), peer_pose=None)
    )
    assert cmd == pytest.approx(FORWARD)
