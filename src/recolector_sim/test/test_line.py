"""
Simulated line sensor: raw Webots reading -> `black` decision.

rclpy and the ROS message packages are not installed in CI, so the bridge
module is imported against minimal stubs.
"""

import importlib
import sys
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

STUBBED = (
    "rclpy",
    "rclpy.node",
    "rclpy.qos",
    "geometry_msgs",
    "geometry_msgs.msg",
    "sensor_msgs",
    "sensor_msgs.msg",
    "recolector_interfaces",
    "recolector_interfaces.msg",
    "mobile_base",
    "mobile_base.kinematics",
    "recolector_common",
    "recolector_common.params",
    "vision",
    "vision.ros_image",
)


@pytest.fixture
def bridge(monkeypatch):
    for name in STUBBED:
        monkeypatch.setitem(sys.modules, name, MagicMock())
    monkeypatch.setitem(sys.modules, "rclpy.node", SimpleNamespace(Node=object))
    monkeypatch.delitem(sys.modules, "recolector_sim.webots_bridge", False)
    module = importlib.import_module("recolector_sim.webots_bridge")
    yield module
    sys.modules.pop("recolector_sim.webots_bridge", None)


def test_a_reading_below_the_threshold_is_black(bridge):
    below = bridge.LINE_BLACK_BELOW_RAW - 1
    assert bridge.is_black(below) is True


def test_a_reading_at_or_above_the_threshold_is_white(bridge):
    at = bridge.LINE_BLACK_BELOW_RAW
    assert bridge.is_black(at) is False
    assert bridge.is_black(at + 100) is False


def test_nan_before_the_first_valid_reading_counts_as_black(bridge):
    assert bridge.is_black(float("nan")) is True
