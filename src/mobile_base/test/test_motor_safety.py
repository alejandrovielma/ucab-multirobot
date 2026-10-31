"""Motor driver node: a failed stop() must be retried by the watchdog, not forgotten.

rclpy is not installed in CI, so the node module is imported against minimal stubs."""

import importlib
import sys
import types
from types import SimpleNamespace

import pytest


@pytest.fixture
def node_class(monkeypatch):
    stubs = {
        "geometry_msgs": types.ModuleType("geometry_msgs"),
        "geometry_msgs.msg": SimpleNamespace(Twist=object),
        "rclpy": types.ModuleType("rclpy"),
        "rclpy.node": SimpleNamespace(Node=object),
        "mobile_base.nodes.common": SimpleNamespace(declare_dataclass=None, now_sec=None, spin_node=None),
    }
    for name, module in stubs.items():
        monkeypatch.setitem(sys.modules, name, module)
    monkeypatch.delitem(sys.modules, "mobile_base.nodes.hat_motor_driver_node", raising=False)
    cls = importlib.import_module("mobile_base.nodes.hat_motor_driver_node").HatMotorDriverNode
    yield cls
    sys.modules.pop("mobile_base.nodes.hat_motor_driver_node", None)


class FakeBank:
    def __init__(self, results):
        self.results, self.stops = list(results), 0

    def stop(self):
        self.stops += 1
        return self.results.pop(0)

    def apply(self, duties, cap):
        return True


def fake_node(bank):
    log = SimpleNamespace(error=lambda *_: None, warning=lambda *_: None)
    return SimpleNamespace(bank=bank, backend="fusion_hat", cap=0.7, _stopped=False, _last_cmd=None,
                           watchdog_timeout=0.3, get_logger=lambda: log, _apply=lambda d: None)


def test_a_failed_stop_leaves_the_node_not_stopped_so_the_watchdog_retries(node_class, monkeypatch):
    node = fake_node(FakeBank([False, True]))
    node_class.stop(node)
    assert node._stopped is False
    monkeypatch.setattr(sys.modules["mobile_base.nodes.hat_motor_driver_node"], "now_sec", lambda n: 100.0)
    node.stop = lambda: node_class.stop(node)
    node_class._watchdog(node)  # stale (no command ever) and not stopped: stops again
    assert node.bank.stops == 2 and node._stopped is True
