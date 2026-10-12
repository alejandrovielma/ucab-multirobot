import math
import random
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from multirobot_sim.sim_utils import is_black, noisy_pose, parse_wheel_signs, read_camera_image

PKG = Path(__file__).resolve().parents[1]


class _FakeCamera:
    def __init__(self, result):
        self._result = result

    def getImage(self):
        if isinstance(self._result, Exception):
            raise self._result
        return self._result


def test_read_camera_image_returns_bytes_when_available():
    assert read_camera_image(_FakeCamera(b"\x01\x02")) == b"\x01\x02"


def test_read_camera_image_tolerates_null_image_before_first_frame():
    # Webots' Python wrapper dereferences a NULL pointer until the first frame exists.
    assert read_camera_image(_FakeCamera(ValueError("NULL pointer access"))) is None


def test_read_camera_image_treats_empty_as_missing():
    assert read_camera_image(_FakeCamera(b"")) is None


def test_parse_wheel_signs():
    assert parse_wheel_signs("1,1,1,1") == (1.0, 1.0, 1.0, 1.0)
    assert parse_wheel_signs(" -1, 1 ,-1,1") == (-1.0, 1.0, -1.0, 1.0)
    with pytest.raises(ValueError):
        parse_wheel_signs("1,1,1")
    with pytest.raises(ValueError):
        parse_wheel_signs("1,2,1,1")


def test_is_black():
    assert is_black(20.0, threshold=120.0)
    assert not is_black(240.0, threshold=120.0)


def test_noisy_pose_without_noise_is_exact():
    assert noisy_pose(0.1, -0.2, 0.5, 0.0, 0.0, random.Random(1)) == (0.1, -0.2, 0.5)


def test_noisy_pose_wraps_heading():
    _, _, theta = noisy_pose(0.0, 0.0, math.pi - 1e-6, 0.0, 0.1, random.Random(3))
    assert -math.pi <= theta < math.pi


@pytest.mark.parametrize("robot, name", [("phineas", "Phineas"), ("ferb", "Ferb")])
def test_urdf_points_to_driver_plugin(robot, name):
    root = ET.parse(PKG / "resource" / f"{robot}.urdf").getroot()
    assert root.get("name") == name
    plugin = root.find("webots/plugin")
    assert plugin.get("type") == "multirobot_sim.recolector_driver.RecolectorDriver"
    assert plugin.findtext("robot") == robot


def test_world_robots_are_extern_and_have_sim_sensors():
    world = (PKG / "worlds" / "pista_ucab.wbt").read_text()
    for robot in ("Phineas", "Ferb"):
        assert f'name "{robot}"' in world
    fields = [line.strip() for line in world.splitlines() if not line.lstrip().startswith("#")]
    assert fields.count('controller "<extern>"') == 2
    for device in ('"linea_frente"', '"gps"', '"imu_sim"'):
        assert world.count(device) == 2
    assert 'EXTERNPROTO "../protos/RecolectorUCAB.proto"' in world
    assert (PKG / "protos" / "RecolectorUCAB.proto").is_file()
    assert (PKG / "worlds" / "textures" / "pista_ucab.png").is_file()
