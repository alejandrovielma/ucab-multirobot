"""Checks of the lab tool without hardware: python3 -m pytest tools/lab"""

import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from hardware import WHEELS, Camera, LineSensor, Motors, load_config, open_gpio  # noqa: E402
from lab_robot import vision_params  # noqa: E402

from mobile_base.vision import detect_cubes  # noqa: E402

FIXTURE = pathlib.Path(__file__).resolve().parents[2] / "src/mobile_base/test/fixtures/webots_cube_at_mouth.jpg"


@pytest.mark.parametrize("robot", ["phineas", "ferb"])
def test_config_has_every_section_the_tool_reads(robot):
    cfg = load_config(robot)
    for section in ("hat_motor_driver", "line_sensor", "cube_detector", "behavior", "safety_gate"):
        assert section in cfg
    hat = cfg["hat_motor_driver"]
    assert hat["motor_backend"] == "fusion_hat"
    assert hat["battery_voltage"] == 8.4
    assert len(hat["fusion_ports"]) == len(hat["fusion_reversed"]) == len(WHEELS)
    for wheel in WHEELS:
        assert len(hat[f"pins_{wheel}"]) == 2


def test_motor_duties_never_exceed_the_cap_and_respect_the_speed_cap():
    motors = Motors(load_config("phineas"), open_gpio(None, dry_run=True), max_speed=0.08, dry_run=True)
    duties = motors.drive(1.0, 0.0, 0.0)  # asks 1 m/s: limited to 0.08 m/s
    assert all(abs(d) <= motors.cap + 1e-9 for d in duties)
    slow = motors.drive(0.08, 0.0, 0.0)
    assert duties == pytest.approx(slow)


def test_line_sensor_in_dry_run_reads_white():
    line = LineSensor(load_config("phineas"), open_gpio(None, dry_run=True))
    assert line.raw() == 0 and not line.black()


def test_file_camera_and_detection_with_the_yaml_hue_ranges():
    cam = Camera(f"file:{FIXTURE}")
    dets = detect_cubes(cam.read(), vision_params(load_config("ferb")))
    assert "green" in {d.color for d in dets}


class FakeMotor:
    def __init__(self, port):
        self.port, self.power_log, self.reverse = port, [], None

    def power(self, value):
        self.power_log.append(value)

    def set_is_reverse(self, flag):
        self.reverse = flag

    def stop(self):
        self.power_log.append(0.0)


def test_fusion_backend_drives_the_ports_from_the_yaml(monkeypatch):
    import hardware

    monkeypatch.setattr(hardware, "load_motor_class", lambda log: FakeMotor)
    cfg = load_config("ferb")
    cfg["hat_motor_driver"]["fusion_ports"] = ["M3", "M2", "M1", "M0"]
    cfg["hat_motor_driver"]["fusion_reversed"] = [True, False, False, False]
    motors = Motors(cfg, open_gpio(None, dry_run=True), max_speed=0.10)
    assert not motors.dry_run
    assert motors.port("fl") == "puerto M3 invertido"
    assert motors.bank.motors["fl"].port == "M3" and motors.bank.motors["fl"].reverse is True
    motors.wheel("fr", 1.0)  # full duty asked: the battery cap still applies
    assert motors.bank.motors["fr"].power_log[-1] == pytest.approx(motors.cap * 100.0)
    assert motors.bank.motors["fl"].power_log[-1] == 0.0
    motors.drive(1.0, 0.0, 3.0)
    assert all(abs(m.power_log[-1]) <= motors.cap * 100.0 + 1e-9 for m in motors.bank.motors.values())


def test_fusion_backend_dry_run_never_loads_the_library(monkeypatch):
    import hardware

    def boom(log):
        raise AssertionError("dry run must not import fusion_hat")

    monkeypatch.setattr(hardware, "load_motor_class", boom)
    motors = Motors(load_config("phineas"), open_gpio(None, dry_run=True), max_speed=0.10, dry_run=True)
    assert motors.dry_run and motors.bank is None
    assert motors.port("rr") == "puerto M3"
    motors.drive(0.1, 0.0, 0.0)  # no hardware, no error


def test_battery_override_sets_the_cap():
    cfg = load_config("phineas")
    cfg["hat_motor_driver"]["battery_voltage"] = 7.4
    motors = Motors(cfg, open_gpio(None, dry_run=True), max_speed=0.10, dry_run=True)
    assert motors.cap == pytest.approx(6.0 / 7.4)


def test_cokoino_backend_still_available():
    cfg = load_config("phineas")
    cfg["hat_motor_driver"]["motor_backend"] = "cokoino_drv8833"
    motors = Motors(cfg, open_gpio(None, dry_run=True), max_speed=0.10)
    assert motors.dry_run
    assert motors.port("fl") == "pines IN [17, 27]"
    motors.drive(0.1, 0.0, 0.0)


def test_camera_status_line_reports_the_blue_square():
    import numpy as np

    from lab_robot import camera_status

    frame = np.full((480, 640, 3), 255, np.uint8)
    frame[300:470, 100:540] = (180, 60, 20)  # a big blue region: the delivery square
    text, square = camera_status(frame, vision_params(load_config("phineas")))
    assert square.visible
    assert "cuadro azul" in text
