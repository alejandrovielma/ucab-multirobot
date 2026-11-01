import math
from types import SimpleNamespace as NS

import pytest

from mobile_base.core.adapters import cube_from_ferb, cube_to_ferb_fields, line_front_from_sensors

W, H = 640, 480
CAMERA = NS(bearing_distance=lambda u, v: (0.1, 0.3))


@pytest.mark.parametrize("cx,cy,area", [(0.0, 0.5, 2500.0), (-1.0, 0.9, 400.0), (0.6, 0.2, 10000.0)])
def test_round_trip(cx, cy, area):
    fields = cube_to_ferb_fields("red", cx, cy, area, W, H, CAMERA)
    cube = cube_from_ferb(NS(**fields), W, H)
    assert cube.color == "red"
    assert cube.cx_norm == pytest.approx(cx, abs=2 / W)
    assert cube.cy_norm == pytest.approx(cy, abs=2 / H)
    assert cube.area_px == pytest.approx(area, rel=0.05)


def test_fields_use_camera_model_and_bottom_edge():
    f = cube_to_ferb_fields("blue", 0.0, 0.5, 100.0, W, H, CAMERA)
    assert (f["bearing_rad"], f["distance_m"]) == (0.1, 0.3)
    assert f["u"] == 320 and f["v"] == 245 and f["width_px"] == f["height_px"] == 10


def test_zero_size():
    f = cube_to_ferb_fields("red", 0.0, 0.5, 0.0, W, H, CAMERA)
    assert f["width_px"] == f["height_px"] == 0
    assert cube_from_ferb(NS(**f), W, H).area_px == 0.0


def test_above_horizon_has_zero_bearing_distance():
    f = cube_to_ferb_fields("red", 0.0, 0.0, 0.0, W, H, NS(bearing_distance=lambda u, v: None))
    assert f["bearing_rad"] == 0.0 and f["distance_m"] == 0.0


def test_image_edges_and_clamp():
    left = cube_from_ferb(NS(color="red", u=0, v=100, width_px=10, height_px=10), W, H)
    right = cube_from_ferb(NS(color="red", u=W, v=100, width_px=10, height_px=10), W, H)
    assert left.cx_norm == -1.0 and right.cx_norm == 1.0
    f = cube_to_ferb_fields("red", 1.0, 1.0, 400.0, W, H, CAMERA)
    assert f["u"] == W


def test_line():
    assert line_front_from_sensors(NS(black=True)) is True
    assert line_front_from_sensors(NS(black=False)) is False
