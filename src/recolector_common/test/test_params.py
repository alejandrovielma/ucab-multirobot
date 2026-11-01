"""
Tests for the dataclass-driven parameter helpers (no ROS needed).
"""

from dataclasses import dataclass, field
from types import SimpleNamespace

import pytest

from recolector_common.params import declare_params, read_params


@dataclass
class Sample:
    """
    Dataclass with one field of each type the helpers coerce.
    """

    rate_hz: float = 2.0
    enabled: bool = True
    colors: list[str] = field(default_factory=lambda: ["red"])
    name: str = "ferb"


class FakeNode:
    """
    Minimal stand-in for rclpy.node.Node: stores parameters in a dict.
    """

    def __init__(self):
        self.values = {}

    def declare_parameter(self, name, default):
        self.values[name] = default

    def get_parameter(self, name):
        return SimpleNamespace(value=self.values[name])


def test_declare_params_uses_field_names_and_defaults():
    node = FakeNode()
    declare_params(node, Sample())
    assert node.values == {
        "rate_hz": 2.0,
        "enabled": True,
        "colors": ["red"],
        "name": "ferb",
    }


def test_declare_params_rejects_non_dataclass():
    with pytest.raises(TypeError):
        declare_params(FakeNode(), {"rate_hz": 2.0})


def test_read_params_round_trip():
    node = FakeNode()
    declare_params(node, Sample())
    assert read_params(node, Sample()) == Sample()


def test_read_params_coerces_to_the_default_type():
    node = FakeNode()
    declare_params(node, Sample())
    node.values["rate_hz"] = 5  # int from YAML into a float field
    node.values["colors"] = ("blue", "green")  # sequence into a list
    result = read_params(node, Sample())
    assert result.rate_hz == 5.0 and isinstance(result.rate_hz, float)
    assert result.colors == ["blue", "green"]


def test_prefix_groups_the_parameter_names():
    node = FakeNode()
    declare_params(node, Sample(), prefix="camera.")
    assert set(node.values) == {
        "camera.rate_hz",
        "camera.enabled",
        "camera.colors",
        "camera.name",
    }
    node.values["camera.name"] = "perry"
    assert read_params(node, Sample(), prefix="camera.").name == "perry"
