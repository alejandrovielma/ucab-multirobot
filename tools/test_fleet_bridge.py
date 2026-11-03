"""
Tests for `tools/teleop_web_lib/fleet.py` and `AutoPilot` multi-robot zone
coordination (>30 cm stationary wait, lease retention through backup, and
verified delivery reporting).
"""

from pathlib import Path

from test_autopilot import FAST_PHASES, Seen, StubVision, blind_push_end_s

from teleop_web_lib.autopilot import (
    STATE_CARRY_WAIT_ZONE,
    AutoPilot,
)
from teleop_web_lib.config import (
    SECTION_CAMERA,
    SECTION_FLEET,
    AutoConfig,
    FleetConfig,
    load_config,
)
from teleop_web_lib.fleet import (
    BridgeTracker,
    NullFleetBridge,
    build_fleet_bridge,
    to_behavior_state,
)

TOOL_YAML_PATH = Path(__file__).with_name("teleop_web.yaml")


class StubFleet:
    """
    Fake fleet bridge recording synced states and verified deliveries.
    """

    def __init__(self, is_granted: bool = False):
        self.is_granted = is_granted
        self.last_sync: tuple[str, bool, bool] | None = None
        self.delivered_colors: list[str] = []

    def granted(self) -> bool:
        return self.is_granted

    def sync(self, state: str, carrying: bool, want: bool) -> None:
        self.last_sync = (state, carrying, want)

    def delivered(self, color: str) -> None:
        self.delivered_colors.append(color)


def test_to_behavior_state_maps_autopilot_phases_for_fleet_link():
    assert to_behavior_state("PAUSED") == "PAUSED"
    assert to_behavior_state("MANUAL") == "MANUAL"
    assert to_behavior_state("SEARCH") == "SEARCH"
    assert to_behavior_state("EXPLORE (turn)") == "SEARCH"
    assert to_behavior_state("LINE (back up)") == "LINE_ESCAPE"
    assert to_behavior_state("APPROACH") == "APPROACH"
    assert to_behavior_state("ALIGN") == "APPROACH"
    assert to_behavior_state("CATCH") == "CATCH"
    assert to_behavior_state("CATCH (check)") == "CATCH"
    assert to_behavior_state("CARRY") == "CARRY"
    assert to_behavior_state("CARRY (square)") == "CARRY"
    assert to_behavior_state(STATE_CARRY_WAIT_ZONE) == "WAIT_ZONE"
    assert to_behavior_state("DROP") == "DELIVER"
    assert to_behavior_state("DROP (blind)") == "DELIVER"
    assert to_behavior_state("DROP (back up)") == "RELEASE"
    assert to_behavior_state("DROP (turn)") == "RELEASE"
    assert to_behavior_state("DONE (2 delivered)") == "PAUSED"


def test_bridge_tracker_grant_freshness_and_outgoing_refresh():
    tracker = BridgeTracker(grant_timeout_s=1.0, republish_period_s=0.5)
    assert not tracker.is_granted(0.0)
    tracker.on_granted(True, 1.0)
    assert tracker.is_granted(1.5)
    assert not tracker.is_granted(2.01)

    assert tracker.outgoing("CARRY", True, False, 0.0) == (
        "CARRY",
        True,
        False,
    )
    assert tracker.outgoing("CARRY", True, False, 0.1) == (None, None, None)
    assert tracker.outgoing("CARRY (wait zone)", True, True, 0.2) == (
        "WAIT_ZONE",
        None,
        True,
    )
    assert tracker.outgoing("CARRY (wait zone)", True, True, 0.8) == (
        "WAIT_ZONE",
        True,
        True,
    )


def test_null_fleet_bridge_is_returned_when_disabled_or_fake():
    bridge = build_fleet_bridge(FleetConfig(enabled=False))
    assert isinstance(bridge, NullFleetBridge) and bridge.granted()
    fake_bridge = build_fleet_bridge(FleetConfig(enabled=True), fake=True)
    assert isinstance(fake_bridge, NullFleetBridge) and fake_bridge.granted()


def test_autopilot_waits_still_at_30cm_and_reports_only_after_backup():
    vision = StubVision()
    fleet = StubFleet(is_granted=False)
    pilot = AutoPilot(AutoConfig(**FAST_PHASES), vision, fleet=fleet)
    pilot.state = "SEARCH"

    # Catch a blue cube.
    vision.det = Seen("blue", pilot.cfg.stop_distance_m - 0.01)
    pilot.step(0.0)
    vision.det = None
    pilot.step(1.0)
    assert pilot.carrying and pilot.carrying_color == "blue"

    # At 44 cm (<= 45 cm): asks for the lease while still approaching.
    vision.zone = Seen("floor", 0.44)
    cmd = pilot.step(1.1)
    assert pilot.want_zone is True and cmd.vy > 0.0
    assert fleet.last_sync == ("CARRY", True, True)

    # At 30 cm without grant: stops completely in CARRY (wait zone).
    vision.zone = Seen("floor", 0.30)
    assert pilot.step(1.2).speed == 0.0
    assert pilot.state == STATE_CARRY_WAIT_ZONE

    # Even if the paper flickers out past carry_zone_timeout_s, stays still.
    vision.zone = None
    wait_end_s = 1.2 + pilot.cfg.carry_zone_timeout_s + 5.0
    assert pilot.step(wait_end_s).speed == 0.0
    assert pilot.state == STATE_CARRY_WAIT_ZONE and pilot.carrying

    # Grant arrives: resumes, pushes, backs up, verifies, then releases.
    fleet.is_granted = True
    vision.zone = Seen("floor", 0.11)
    assert pilot.step(wait_end_s + 0.1).vy == 1.0
    assert pilot.state == "DROP" and fleet.delivered_colors == []

    vision.zone = None
    pilot.step(wait_end_s + 0.15)
    end_s = blind_push_end_s(
        pilot.cfg,
        0.11,
        wait_end_s + 0.1,
        wait_end_s + 0.15,
    )
    # Backing up: lease still wanted, delivery not reported yet.
    assert pilot.step(end_s + 0.1).vy == -1.0
    assert pilot.want_zone is True and fleet.delivered_colors == []

    # Backup finished with no cube outside: reports blue once, turns, releases.
    pilot.step(end_s + 0.6)
    assert pilot.state == "DROP (turn)"
    assert fleet.delivered_colors == ["blue"]
    assert pilot.want_zone is True

    pilot.step(end_s + 1.2)
    assert pilot.want_zone is False


def test_autopilot_does_not_report_delivery_when_cube_stayed_outside():
    vision = StubVision()
    fleet = StubFleet(is_granted=True)
    pilot = AutoPilot(AutoConfig(**FAST_PHASES), vision, fleet=fleet)
    pilot.carrying = True
    pilot.carrying_color = "blue"
    pilot.state = "CARRY"

    vision.zone = Seen("floor", 0.11)
    pilot.step(0.0)
    vision.zone = None
    pilot.step(0.05)
    end_s = blind_push_end_s(pilot.cfg, 0.11, 0.0, 0.05)
    pilot.step(end_s + 0.1)

    # Cube is still right in front after backing up.
    vision.det = Seen("blue", pilot.cfg.drop_verify_distance_m - 0.05)
    pilot.step(end_s + 0.6)
    assert fleet.delivered_colors == []
    assert pilot.delivered_count == 0
    assert pilot.want_zone is False


def test_teleop_yaml_targets_blue_and_has_valid_fleet_section():
    cfg = load_config(TOOL_YAML_PATH)
    assert cfg[SECTION_CAMERA]["target_colors"] == ["blue"]
    fleet_cfg = FleetConfig.from_dict(cfg.get(SECTION_FLEET))
    assert fleet_cfg.namespace == "phineas"
