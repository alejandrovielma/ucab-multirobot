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
    for wheel in WHEELS:
        assert len(cfg["hat_motor_driver"][f"pins_{wheel}"]) == 2


def test_motor_duties_never_exceed_the_cap_and_respect_the_speed_cap():
    motors = Motors(load_config("phineas"), open_gpio(None, dry_run=True), max_speed=0.08)
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
