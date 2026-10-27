"""The YAML files must match the parameter dataclasses: ROS 2 silently ignores undeclared keys,
so a typo would leave a default in place, and a zone_* mismatch between behavior and gate
would let the behavior plan a reverse the gate refuses (or the other way round)."""

import dataclasses
import pathlib

import pytest
import yaml

from mobile_base.core.behavior import BehaviorParams
from mobile_base.core.safety import SafetyParams

CONFIG_DIR = pathlib.Path(__file__).resolve().parents[1] / "config"
NODE_ONLY = {
    "behavior": {"robot", "my_colors", "avoid_colors", "peer_pose_topic", "rate_hz", "cube_timeout"},
    "safety_gate": {"peer_pose_topic", "rate_hz", "cmd_timeout"},
}
PARAMS = {"behavior": BehaviorParams, "safety_gate": SafetyParams}


def load(robot):
    return yaml.safe_load((CONFIG_DIR / f"{robot}.yaml").read_text())


@pytest.mark.parametrize("robot", ["phineas", "ferb"])
@pytest.mark.parametrize("node", ["behavior", "safety_gate"])
def test_yaml_keys_match_dataclass_fields(robot, node):
    keys = set(load(robot)[f"/{robot}/{node}"]["ros__parameters"])
    fields = {f.name for f in dataclasses.fields(PARAMS[node])} - {"my_colors"}
    assert keys - NODE_ONLY[node] == fields


@pytest.mark.parametrize("robot", ["phineas", "ferb"])
def test_zone_is_identical_in_behavior_and_gate(robot):
    cfg = load(robot)
    behavior = cfg[f"/{robot}/behavior"]["ros__parameters"]
    gate = cfg[f"/{robot}/safety_gate"]["ros__parameters"]
    shared = ["track_horizon", "body_front", "body_rear", "body_half_width"]
    zone_keys = [k for k in behavior if k.startswith("zone_")] + shared
    assert zone_keys
    assert {k: behavior[k] for k in zone_keys} == {k: gate[k] for k in zone_keys}


def test_both_robots_share_the_same_square():
    a, b = load("phineas"), load("ferb")
    za = {k: v for k, v in a["/phineas/behavior"]["ros__parameters"].items() if k.startswith("zone_")}
    zb = {k: v for k, v in b["/ferb/behavior"]["ros__parameters"].items() if k.startswith("zone_")}
    assert za == zb


@pytest.mark.parametrize("robot", ["phineas", "ferb"])
def test_both_robots_start_without_a_mission(robot):
    # The mission (target and forbidden colors) comes from the command interface; until then the
    # robot stays still, so it cannot touch a cube that turns out to be forbidden.
    behavior = load(robot)[f"/{robot}/behavior"]["ros__parameters"]
    assert behavior["my_colors"] == "" and behavior["avoid_colors"] == ""


@pytest.mark.parametrize(("robot", "expected"), [("phineas", 0.14), ("ferb", 0.0)])
def test_line_backoff_reaches_behavior_params(robot, expected):
    behavior = load(robot)[f"/{robot}/behavior"]["ros__parameters"]
    fields = {f.name for f in dataclasses.fields(BehaviorParams)}
    values = {k: v for k, v in behavior.items() if k in fields and k != "my_colors"}
    assert BehaviorParams(my_colors=("red",), **values).line_backoff == pytest.approx(expected)


@pytest.mark.parametrize(("robot", "expected"), [("phineas", 0.55), ("ferb", 0.0)])
def test_square_border_cy_reaches_behavior_params(robot, expected):
    behavior = load(robot)[f"/{robot}/behavior"]["ros__parameters"]
    fields = {f.name for f in dataclasses.fields(BehaviorParams)}
    values = {k: v for k, v in behavior.items() if k in fields and k != "my_colors"}
    assert BehaviorParams(my_colors=("red",), **values).square_border_cy == pytest.approx(expected)
