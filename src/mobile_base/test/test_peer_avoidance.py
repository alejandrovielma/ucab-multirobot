"""
Tests for peer avoidance: stale poses, slow/stop zones and deadlock escape.
"""

import math

import pytest

from mobile_base.peer_avoidance import (
    PeerAvoidance,
    PeerAvoidanceParams,
    Pose2D,
)


def test_peer_avoidance_stale_poses_keep_command():
    pa = PeerAvoidance()
    cmd = (0.12, 0.0, 0.0)
    own = Pose2D(0.0, 0.0, 0.0, stamp=0.0)
    peer = Pose2D(0.30, 0.0, math.pi, stamp=0.0)

    # Missing poses.
    assert pa.filter(cmd, None, peer, 0.1) == (cmd, "ok")
    assert pa.filter(cmd, own, None, 0.1) == (cmd, "ok")

    # Stale own or peer pose (> 0.5 s).
    assert pa.filter(cmd, own, peer, 0.6) == (cmd, "ok")
    fresh_own = Pose2D(0.0, 0.0, 0.0, stamp=0.5)
    assert pa.filter(cmd, fresh_own, peer, 0.6) == (cmd, "ok")


def test_peer_avoidance_slow_and_stop():
    pa = PeerAvoidance(PeerAvoidanceParams())
    own = Pose2D(0.0, 0.0, 0.0, stamp=1.0)

    # Peer at 0.50 m directly ahead -> half speed and turn right (-0.6).
    peer_slow = Pose2D(0.50, 0.0, math.pi, stamp=1.0)
    (vx, vy, wz), reason = pa.filter((0.10, 0.0, 0.0), own, peer_slow, 1.0)
    assert vx == pytest.approx(0.05)
    assert vy == 0.0
    assert wz == pytest.approx(-0.6)
    assert "slow" in reason

    # Peer at 0.35 m ahead on the right -> stop forward and turn left (+0.6).
    peer_stop = Pose2D(0.35, -0.05, math.pi, stamp=1.0)
    (vx, vy, wz), reason = pa.filter((0.10, 0.0, 0.0), own, peer_stop, 1.0)
    assert vx == 0.0
    assert wz == pytest.approx(0.6)
    assert "stop" in reason


def test_peer_avoidance_deadlock_escape():
    pa = PeerAvoidance(PeerAvoidanceParams())
    own = Pose2D(0.0, 0.0, 0.0, stamp=0.0)
    peer = Pose2D(0.30, 0.05, math.pi, stamp=0.0)

    # Blocked at t=0.0.
    (vx, _, wz), _ = pa.filter((0.10, 0.0, 0.0), own, peer, 0.0)
    assert vx == 0.0
    assert wz == pytest.approx(-0.6)

    # Still blocked at t=2.9 -> not yet escaping.
    own_29 = Pose2D(0.0, 0.0, 0.0, stamp=2.9)
    peer_29 = Pose2D(0.30, 0.05, math.pi, stamp=2.9)
    (vx, _, wz), reason = pa.filter((0.10, 0.0, 0.0), own_29, peer_29, 2.9)
    assert vx == 0.0
    assert wz == pytest.approx(-0.6)
    assert "deadlock" not in reason

    # Blocked for 3.0 s -> yield and rotate away at escape_turn_rad_s (0.8).
    own_30 = Pose2D(0.0, 0.0, 0.0, stamp=3.0)
    peer_30 = Pose2D(0.30, 0.05, math.pi, stamp=3.0)
    (vx, vy, wz), reason = pa.filter((0.10, 0.0, 0.0), own_30, peer_30, 3.0)
    assert (vx, vy, wz) == (0.0, 0.0, -0.8)
    assert "deadlock" in reason

    # Keeps rotating while the peer is still in the +-60 deg cone.
    own_35 = Pose2D(0.0, 0.0, -0.3, stamp=3.5)
    peer_35 = Pose2D(0.30, 0.05, math.pi, stamp=3.5)
    (vx, vy, wz), reason = pa.filter((0.0, 0.0, 0.0), own_35, peer_35, 3.5)
    assert (vx, vy, wz) == (0.0, 0.0, -0.8)
    assert "deadlock" in reason

    # Once the peer is out of the cone, the normal command resumes.
    own_clear = Pose2D(0.0, 0.0, -1.3, stamp=4.0)
    peer_clear = Pose2D(0.30, 0.05, math.pi, stamp=4.0)
    out, reason = pa.filter((0.10, 0.0, 0.0), own_clear, peer_clear, 4.0)
    assert out == (0.10, 0.0, 0.0)
    assert reason == "ok"
