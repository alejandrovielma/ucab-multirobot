"""
Tests for the pure-logic fleet module: state mapping, heartbeat and zone
client.
"""

from mobile_base.fleet import Heartbeat, ZoneClient, to_fleet_state
from mobile_base.states import FleetState, State


def test_to_fleet_state_mapping():
    assert to_fleet_state("SEARCH") == "EXPLORE"
    assert to_fleet_state("APPROACH") == "APPROACH"
    assert to_fleet_state("ARRIVED") == "CAPTURE"
    assert to_fleet_state("CATCH") == "CAPTURE"
    assert to_fleet_state("CARRY") == "CARRY"
    assert to_fleet_state("RELEASE") == "DELIVER"
    assert to_fleet_state("PAUSED") == "STOP"
    assert to_fleet_state("MANUAL") == "STOP"
    assert to_fleet_state("NO_CAMERA") == "STOP"
    assert to_fleet_state("UNKNOWN") == "STOP"


def test_to_fleet_state_returns_plain_str_for_ros_messages():
    assert type(to_fleet_state(State.SEARCH)) is str
    assert to_fleet_state(State.SEARCH) == FleetState.EXPLORE


def test_heartbeat_timeout():
    hb = Heartbeat(timeout_s=2.0)
    assert not hb.ok(0.0)
    hb.beat(1.0)
    assert hb.ok(1.0)
    assert hb.ok(3.0)
    assert not hb.ok(3.01)


def test_zone_client_request_renew_release():
    zc = ZoneClient(robot="ferb", renew_period_s=20.0)
    assert not zc.holds
    assert not zc.should_request(0.0)

    # Wanting the zone while not holding triggers a request.
    zc.update(want=True, occupant="")
    assert zc.should_request(0.0)
    zc.note_requested(0.0)

    # Once granted, no new request before renew_period_s.
    zc.update(occupant="ferb")
    assert zc.holds
    assert not zc.should_request(10.0)
    assert zc.should_request(20.0)
    zc.note_requested(20.0)
    assert not zc.should_request(25.0)

    # Mark delivered and drop want -> release with delivered metadata.
    zc.mark_delivered("blue")
    zc.update(want=False)
    assert zc.should_release()
    delivered, color = zc.pop_release()
    assert delivered is True
    assert color == "blue"
    zc.update(occupant="")
    assert not zc.should_release()
