import json
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(__file__))
from dashboard import Dashboard  # noqa: E402
from teleop import Teleop  # noqa: E402

COLORS = ("red", "orange", "yellow", "green", "blue", "purple")


@pytest.fixture
def dash():
    return Dashboard(Teleop(0.2, 1.0), COLORS)


def call(dash, method, path, obj=None):
    return dash.handle(method, path, json.dumps(obj).encode() if obj is not None else b"")


def test_mission_is_validated_and_does_not_start_by_itself(dash):
    assert call(dash, "POST", "/mission", {"targets": ["blue"], "avoid": ["red"]})[0] == 200
    assert dash.take_change() == ((), ("red",))  # picked, not running: the robot stays still


@pytest.mark.parametrize("body", [{"targets": ["pink"]}, {"targets": "blue"}, {"targets": ["blue"], "avoid": ["blue"]},
                                  {"targets": [1]}])
def test_bad_missions_are_400(dash, body):
    assert call(dash, "POST", "/mission", body)[0] == 400


def test_start_needs_a_target_then_start_and_stop_drive_the_sequencer(dash):
    assert call(dash, "POST", "/start")[0] == 400
    call(dash, "POST", "/mission", {"targets": ["blue", "green"], "avoid": ["red"]})
    dash.take_change()
    assert call(dash, "POST", "/start")[0] == 200
    assert dash.take_change() == (("blue", "green"), ("red",))
    assert dash.take_change() is None  # reported once
    call(dash, "POST", "/stop")
    assert dash.take_change() == ((), ("red",))


def test_manual_and_mission_are_exclusive(dash):
    call(dash, "POST", "/mission", {"targets": ["blue"]})
    call(dash, "POST", "/start")
    dash.take_change()
    call(dash, "POST", "/manual", {"on": True})
    assert dash.manual and dash.take_change() == ((), ())
    call(dash, "POST", "/start")
    assert not dash.manual
    assert call(dash, "POST", "/manual", {"on": "yes"})[0] == 400


def test_emergency_stop_ends_the_mission_and_clearing_it_does_not_restart(dash):
    call(dash, "POST", "/mission", {"targets": ["blue"]})
    call(dash, "POST", "/start")
    dash.take_change()
    call(dash, "POST", "/estop", {"on": True})
    assert dash.teleop.estop and dash.take_change() == ((), ())
    call(dash, "POST", "/estop", {"on": False})
    assert dash.take_change() is None
    status = json.loads(call(dash, "GET", "/status")[1])
    assert status["running"] is False and status["targets"] == ["blue"]


def test_status_has_mission_and_teleop_fields(dash):
    dash.update_status(fsm="EXPLORE", delivered=2, forbidden=0)
    status = json.loads(call(dash, "GET", "/status")[1])
    assert {"estop", "cmd", "targets", "avoid", "running", "manual", "fsm", "delivered", "forbidden"} <= set(status)
    assert call(dash, "GET", "/nope") is None
