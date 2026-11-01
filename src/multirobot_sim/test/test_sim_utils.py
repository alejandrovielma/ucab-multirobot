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


def test_scoop_color_is_never_classified_as_a_cube_color():
    # In Webots the orange scoop wings were detected as an orange cube, so Phineas chased its
    # own scoop forever. The scoop must stay unsaturated (gray/black) for the HSV classifier.
    import colorsys
    import re

    from mobile_base.core.colors import classify

    proto = (PKG / "protos" / "RecolectorUCAB.proto").read_text()
    match = re.search(r"DEF PLA_AP PBRAppearance \{ baseColor ([\d.]+) ([\d.]+) ([\d.]+)", proto)
    assert match, "scoop appearance PLA_AP not found"
    r, g, b = (float(v) for v in match.groups())
    h, s, v = colorsys.rgb_to_hsv(r, g, b)
    assert classify(h * 179, s * 255, v * 255) is None


def test_gate_motor_matches_the_driver_positions():
    # The driver opens/closes the gate with fixed angles: they must be inside the motor limits.
    from multirobot_sim import recolector_driver as driver_module

    proto = (PKG / "protos" / "RecolectorUCAB.proto").read_text()
    block = proto[proto.index('name "servo_compuerta"') :]
    low = float(block.split("minPosition")[1].split()[0])
    high = float(block.split("maxPosition")[1].split()[0])
    assert low <= driver_module.GATE_OPEN <= high
    assert low <= driver_module.GATE_CLOSED <= high


# ---- track geometry (tools/sim/generate_track.py renders the texture from the measured track)


def _board_size(world: str) -> tuple[float, float]:
    import re

    match = re.search(r"geometry Plane \{ size ([\d.]+) ([\d.]+) \}", world[world.index("DEF PISTA") :])
    return float(match.group(1)), float(match.group(2))


def _png_size(path: Path) -> tuple[int, int]:
    import struct

    data = path.read_bytes()[:24]
    assert data[:8] == b"\x89PNG\r\n\x1a\n"
    return struct.unpack(">II", data[16:24])


@pytest.mark.parametrize("world_name", ["pista_ucab.wbt", "calibracion_pala.wbt"])
def test_track_texture_matches_the_board_aspect(world_name):
    board_x, board_y = _board_size((PKG / "worlds" / world_name).read_text())
    assert (board_x, board_y) == (1.82, 0.94)
    width, height = _png_size(PKG / "worlds" / "textures" / "pista_ucab.png")
    assert width >= 1000 * board_x  # >= 1000 px per meter on the long side
    assert width / height == pytest.approx(board_x / board_y, rel=0.01)


def _track_polygon():
    import yaml

    data = yaml.safe_load((PKG.parent / "mobile_base" / "config" / "track.yaml").read_text())
    flat = data["/**"]["ros__parameters"]["track_polygon"]
    return list(zip(flat[0::2], flat[1::2]))


def test_track_polygon_lies_inside_the_board():
    board_x, board_y = _board_size((PKG / "worlds" / "pista_ucab.wbt").read_text())
    polygon = _track_polygon()
    assert len(polygon) >= 3
    for x, y in polygon:
        assert abs(x) < board_x / 2 and abs(y) < board_y / 2


def _world_items(pattern: str):
    import re

    return re.findall(pattern, (PKG / "worlds" / "pista_ucab.wbt").read_text())


def test_cubes_are_inside_the_track_and_clear_of_the_starts_and_the_delivery_corridor():
    cubes = _world_items(r'CuboMadera \{\s+translation ([-\d.]+) ([-\d.]+) ([-\d.]+)\s+rotation [^\n]+\s+name "(\w+)"')
    assert len(cubes) == 14
    xs = [abs(x) for x, *_ in _track_polygon()]
    ys = [abs(y) for _, y in _track_polygon()]
    inner_x, inner_y = max(xs), max(ys)  # rectangle inner tape edge
    starts = [(0.40, 0.20), (0.40, -0.20)]
    corridor = (-0.585, -0.237, 0.0085 - 0.08, 0.0085 + 0.08)
    colors = {}
    for x, y, z, name in cubes:
        x, y, z = float(x), float(y), float(z)
        assert z == pytest.approx(0.01)  # 2 cm cube resting on the board
        assert abs(x) <= inner_x - 0.06 and abs(y) <= inner_y - 0.06, name
        assert all(math.hypot(x - sx, y - sy) >= 0.15 for sx, sy in starts), name
        dx = max(corridor[0] - x, 0.0, x - corridor[1])
        dy = max(corridor[2] - y, 0.0, y - corridor[3])
        assert math.hypot(dx, dy) >= 0.15, name
        color = name.split("_")[1]
        colors[color] = colors.get(color, 0) + 1
    assert set(colors) == {"rojo", "naranja", "amarillo", "verde", "azul", "morado"}
    assert all(2 <= n <= 3 for n in colors.values())


def test_robots_start_east_facing_west():
    starts = _world_items(r'DEF (PHINEAS|FERB) RecolectorUCAB \{\s+translation ([-\d.]+) ([-\d.]+) 0\s+rotation 0 0 1 ([\d.]+)')
    assert {name: (float(x), float(y)) for name, x, y, _ in starts} == {"PHINEAS": (0.4, 0.2), "FERB": (0.4, -0.2)}
    assert all(float(theta) == pytest.approx(math.pi, abs=1e-4) for *_, theta in starts)


def test_square_marker_is_west_of_the_square_outside_the_tape():
    (x, y), = [(float(a), float(b)) for a, b in _world_items(r"DEF MARCADOR_CUADRADO Solid \{\s+#[^\n]*\s+translation ([-\d.]+) ([-\d.]+) 0")]
    assert x < -0.806 and abs(y - 0.0085) < 0.01  # beyond the square's west tape, on its axis
    assert x > -0.91  # still on the board
