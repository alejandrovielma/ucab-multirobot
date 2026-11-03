"""
Phineas-only pieces on top of Ferb's tool: the far-edge guard of the delivery
push.
"""

from test_autopilot import FAST_PHASES, Seen, StubVision

from teleop_web_lib.autopilot import AutoPilot
from teleop_web_lib.config import AutoConfig

NEAR_M = 0.078
SHORT_FAR_M = 0.121
MIN_SPAN_M = 0.08


def delivery_pilot(**overrides):
    vision = StubVision()
    cfg = AutoConfig(**FAST_PHASES, **overrides)
    pilot = AutoPilot(cfg, vision)
    pilot.carrying = True
    pilot.state = "CARRY"
    return pilot, vision


def test_a_far_edge_too_close_behind_the_near_one_is_ignored():
    pilot, _ = delivery_pilot(deliver_far_min_span_m=MIN_SPAN_M)
    zone = Seen("floor", NEAR_M)
    assert pilot._usable_far_m(zone, SHORT_FAR_M) is None
    assert "far edge ignored" in pilot.drop_report


def test_a_credible_far_edge_is_kept():
    pilot, _ = delivery_pilot(deliver_far_min_span_m=MIN_SPAN_M)
    assert pilot._usable_far_m(Seen("floor", NEAR_M), 0.2) == 0.2


def test_the_far_guard_is_off_by_default_and_none_stays_none():
    pilot, _ = delivery_pilot()
    assert pilot.cfg.deliver_far_min_span_m == 0.0
    assert pilot._usable_far_m(Seen("floor", NEAR_M), SHORT_FAR_M) == (
        SHORT_FAR_M
    )
    pilot, _ = delivery_pilot(deliver_far_min_span_m=MIN_SPAN_M)
    assert pilot._usable_far_m(Seen("floor", NEAR_M), None) is None
    assert pilot.drop_report is None


def push_after_first_step(min_span_m):
    pilot, vision = delivery_pilot(deliver_far_min_span_m=min_span_m)
    vision.zone = Seen("floor", NEAR_M, far_distance_m=SHORT_FAR_M)
    pilot.step(0.1)
    return pilot.step(0.2)


def test_a_truncated_far_edge_does_not_make_the_push_zero_long():
    assert push_after_first_step(MIN_SPAN_M).vy == 1.0


def test_without_the_guard_a_truncated_far_edge_ends_the_push_at_once():
    assert push_after_first_step(0.0).vy == -1.0
