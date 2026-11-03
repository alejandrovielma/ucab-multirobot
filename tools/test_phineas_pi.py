"""
Phineas-only pieces on top of Ferb's tool: the far-edge guard of the delivery
push and the Fusion HAT+ motors driven through the `fusion_hat` library.
"""

import sys
import types

import pytest
from test_autopilot import FAST_PHASES, Seen, StubVision

from teleop_web_lib.autopilot import AutoPilot
from teleop_web_lib.config import AutoConfig
from teleop_web_lib.motors import (
    DRIVER_FUSION_HAT_LIB,
    FusionHatLibMotors,
    build_motors,
    with_start_power,
)

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


@pytest.mark.parametrize(
    ("power", "start", "expected"),
    [
        (0.0, 20.0, 0.0),
        (1.9, 20.0, 0.0),
        (-1.9, 20.0, 0.0),
        (10.0, 20.0, 20.0),
        (-10.0, 20.0, -20.0),
        (50.0, 20.0, 50.0),
        (-50.0, 20.0, -50.0),
        (10.0, 0.0, 10.0),
        (1.0, 0.0, 1.0),
    ],
)
def test_with_start_power(power, start, expected):
    assert with_start_power(power, start) == expected


class FakeMotor:
    """
    Stand-in for fusion_hat.motor.Motor.
    """

    instances = {}

    def __init__(self, port):
        self.port = port
        self.reverse = None
        self.powers = []
        FakeMotor.instances[port] = self

    def set_is_reverse(self, value):
        self.reverse = value

    def power(self, percent):
        self.powers.append(percent)


@pytest.fixture
def fusion_cfg(monkeypatch):
    FakeMotor.instances = {}
    package = types.ModuleType("fusion_hat")
    module = types.ModuleType("fusion_hat.motor")
    module.Motor = FakeMotor
    monkeypatch.setitem(sys.modules, "fusion_hat", package)
    monkeypatch.setitem(sys.modules, "fusion_hat.motor", module)
    return {
        "driver": DRIVER_FUSION_HAT_LIB,
        "fusion_channels": {
            "fl": [4, 5],
            "fr": [11, 10],
            "rl": [6, 7],
            "rr": [9, 8],
        },
        "fl_inverted": False,
        "fr_inverted": True,
        "rl_inverted": False,
        "rr_inverted": True,
        "start_power_pct": 20,
    }


def test_each_wheel_gets_its_port_and_reverse_flag(fusion_cfg):
    FusionHatLibMotors(fusion_cfg)
    reverse = {p: m.reverse for p, m in FakeMotor.instances.items()}
    # FL=M3, FR=M0, RL=M2, RR=M1; the right wheels are inverted.
    assert reverse == {"M3": False, "M0": True, "M2": False, "M1": True}


def test_set_drives_percent_skips_unchanged_and_stops_on_close(fusion_cfg):
    motors = FusionHatLibMotors(fusion_cfg)
    front_left = FakeMotor.instances["M3"]
    motors.set({"fl": 0.5, "fr": -0.1, "rl": 0.0, "rr": 0.0})
    motors.set({"fl": 0.5, "fr": -0.1, "rl": 0.0, "rr": 0.0})
    assert front_left.powers == [0.0, 50.0]
    # -10 % is below the start power: raised to -20 % keeping the sign.
    assert FakeMotor.instances["M0"].powers == [0.0, -20.0]
    motors.close()
    assert front_left.powers[-1] == 0.0


def test_build_motors_picks_the_library_driver(fusion_cfg, tmp_path):
    driver, label = build_motors(fusion_cfg, False, tmp_path / "x.yaml")
    assert isinstance(driver, FusionHatLibMotors)
    assert label == "Fusion HAT+ (library)"
