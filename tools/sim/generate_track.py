#!/usr/bin/env python3
"""Render the Webots track texture (and the cube layout) from the measured track geometry.

    python3 tools/sim/generate_track.py                # writes src/multirobot_sim/worlds/textures/pista_ucab.png
    python3 tools/sim/generate_track.py --print-cubes  # also prints the CuboMadera nodes for pista_ucab.wbt

Single source of the simulated track (measured on 2026-10-15, tape 1.8 cm wide). World frame:
origin at the center of the taped rectangle, x east (along the 120.6 cm side), y north, meters. The
same numbers are in src/mobile_base/config/track.yaml (body limit) and in the zone_* parameters.

- Rectangle 120.6 x 74 cm at the OUTER tape edge: outer x +-0.603, y +-0.370; inner x +-0.585, y +-0.352.
- Blue delivery square sticking out of the west side, 20.3 x 20.3 cm outer, 26 cm from the north
  corner and 27.8 cm from the south corner: outer x -0.806..-0.603, y -0.093..0.110. Its side toward
  the track has no tape; the blue fills its interior x -0.788..-0.585, y -0.075..0.092 (assumed: up
  to the rectangle's inner tape line, so the rectangle's west tape is interrupted along the opening).
- Board 1.82 x 0.94 m centered on the origin. ASSUMPTION: the white margin around the tape was not
  measured (10 cm left beyond the square on the west side, about 30 cm east, 10 cm north and south).

Image orientation (same as the previous texture, checked against its track polygon): column 0 is the
west edge of the board (x = -BOARD_X / 2), row 0 the north edge (y = +BOARD_Y / 2).
Needs numpy and OpenCV (python3-opencv); no Webots.
"""

import argparse
import math
import pathlib
import random

import cv2
import numpy as np

REPO = pathlib.Path(__file__).resolve().parents[2]
TEXTURE = REPO / "src" / "multirobot_sim" / "worlds" / "textures" / "pista_ucab.png"

BOARD_X, BOARD_Y = 1.82, 0.94  # m (assumed margin, see above)
PX_PER_M = 1000.0  # 1820 x 940 px
TAPE = 0.018
RECT_OUTER = (0.603, 0.370)  # half sizes
RECT_INNER = (RECT_OUTER[0] - TAPE, RECT_OUTER[1] - TAPE)  # body limit (track.yaml)
SQUARE_OUTER = (-0.806, -0.603, -0.093, 0.110)  # x0, x1, y0, y1
SQUARE_BLUE = (-0.788, -0.585, -0.075, 0.092)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
BLUE_RGB = (0.10, 0.25, 0.70)  # same blue as the blue cubes: the square must not be taken for one

# Robots (center x, y, heading) and the delivery corridor (core/delivery.py defaults).
STARTS = {"Phineas": (0.40, 0.20, math.pi), "Ferb": (0.40, -0.20, math.pi)}
ZONE_Y = 0.0085
CORRIDOR = (-0.585, -0.237, ZONE_Y - 0.08, ZONE_Y + 0.08)  # robot centers from the drop to entry + margin

# Cubes: 2 cm, two or three per color (Phineas red/orange/yellow, Ferb green/blue/purple).
CUBE_COLORS = {
    "rojo": (0.78, 0.12, 0.10),
    "naranja": (0.95, 0.45, 0.08),
    "amarillo": (0.95, 0.80, 0.10),
    "verde": (0.08, 0.55, 0.30),
    "azul": (0.10, 0.25, 0.70),
    "morado": (0.40, 0.10, 0.30),
}
CUBE_COUNTS = {"rojo": 3, "naranja": 2, "amarillo": 2, "verde": 3, "azul": 2, "morado": 2}
CUBE_TAPE_MARGIN = 0.06  # m from the inner tape edge to the cube center
CUBE_CLEARANCE = 0.15  # m from the start poses and from the delivery corridor
CUBE_SPACING = 0.10  # m between cubes
CUBE_SEED = 2026


def to_px(x: float, y: float) -> tuple[int, int]:
    return round((x + BOARD_X / 2.0) * PX_PER_M), round((BOARD_Y / 2.0 - y) * PX_PER_M)


def fill(img: np.ndarray, x0: float, x1: float, y0: float, y1: float, color) -> None:
    """Axis-aligned rectangle in meters (x0 < x1, y0 < y1)."""
    (c0, r0), (c1, r1) = to_px(x0, y1), to_px(x1, y0)
    img[r0:r1, c0:c1] = color


def render() -> np.ndarray:
    width, height = round(BOARD_X * PX_PER_M), round(BOARD_Y * PX_PER_M)
    img = np.full((height, width, 3), WHITE, np.uint8)
    blue_bgr = tuple(round(255 * c) for c in reversed(BLUE_RGB))
    fill(img, *SQUARE_OUTER, BLACK)  # the square's three taped sides
    ox, oy = RECT_OUTER
    ix, iy = RECT_INNER
    fill(img, -ox, ox, -oy, oy, BLACK)
    fill(img, -ix, ix, -iy, iy, WHITE)
    fill(img, *SQUARE_BLUE, blue_bgr)  # also opens the rectangle's west tape along the square
    return img


def _distance_to_box(x: float, y: float, box) -> float:
    x0, x1, y0, y1 = box
    return math.hypot(max(x0 - x, 0.0, x - x1), max(y0 - y, 0.0, y - y1))


def cube_ok(x: float, y: float, placed=()) -> bool:
    ix, iy = RECT_INNER
    return (
        abs(x) <= ix - CUBE_TAPE_MARGIN
        and abs(y) <= iy - CUBE_TAPE_MARGIN
        and all(math.hypot(x - sx, y - sy) >= CUBE_CLEARANCE for sx, sy, _ in STARTS.values())
        and _distance_to_box(x, y, CORRIDOR) >= CUBE_CLEARANCE
        and all(math.hypot(x - px, y - py) >= CUBE_SPACING for px, py, *_ in placed)
    )


def cube_layout() -> list[tuple[float, float, float, str]]:
    """Deterministic cube positions: (x, y, yaw, name)."""
    rng = random.Random(CUBE_SEED)
    ix, iy = RECT_INNER
    placed: list[tuple[float, float, float, str]] = []
    for color, count in CUBE_COUNTS.items():
        for k in range(count):
            while True:
                x = round(rng.uniform(-ix, ix), 3)
                y = round(rng.uniform(-iy, iy), 3)
                if cube_ok(x, y, placed):
                    break
            placed.append((x, y, round(rng.uniform(0.0, math.pi / 2), 3), f"cubo_{color}_{k + 1}"))
    return placed


def cube_nodes() -> str:
    out = []
    for x, y, yaw, name in cube_layout():
        r, g, b = CUBE_COLORS[name.split("_")[1]]
        out.append(
            "CuboMadera {\n"
            f"  translation {x} {y} 0.01\n"
            f"  rotation 0 0 1 {yaw}\n"
            f'  name "{name}"\n'
            f"  color {r:.2f} {g:.2f} {b:.2f}\n"
            "}"
        )
    return "\n".join(out)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", type=pathlib.Path, default=TEXTURE)
    parser.add_argument("--print-cubes", action="store_true", help="print the CuboMadera nodes of pista_ucab.wbt")
    args = parser.parse_args()
    img = render()
    if not cv2.imwrite(str(args.out), img):
        raise SystemExit(f"cannot write {args.out}")
    print(f"{args.out}: {img.shape[1]} x {img.shape[0]} px ({PX_PER_M:.0f} px/m)")
    if args.print_cubes:
        print(cube_nodes())


if __name__ == "__main__":
    main()
