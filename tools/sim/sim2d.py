"""2D closed-loop simulation of the two robots on the real track, without Webots or ROS.

The same BehaviorFSM and SafetyGate that run on the robot, fed by a simple camera model (pinhole bearing,
ground-plane height) and a kinematic body: it checks the LOGIC of the cycle (explore, approach, capture,
carry along the axis, push, mission colors, tape, the other robot). It does NOT model mecanum slip, motor
dead zone, lighting or the real camera: the numbers it gives are a smoke test, not a calibration.

    python tools/sim/sim2d.py [--seconds 180] [--seed 1] [--video out.mp4] [--png out.png]

PLACEHOLDERS (not measured): the green and red reference rectangles beside the square, the camera
projection constants, the cube positions.
"""

import argparse
import math
import random
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "src" / "mobile_base"))

from mobile_base.core.behavior import BehaviorFSM, BehaviorInput, BehaviorParams, Cube, State  # noqa: E402
from mobile_base.core.safety import SafetyGate, SafetyInput, SafetyParams  # noqa: E402

DT = 0.1
HFOV = 1.60
INNER = (0.585, 0.352)  # half sizes, inner tape edge
OUTER = (0.603, 0.370)
GAP_Y = (-0.075, 0.092)  # the square's side toward the track has no tape
BLUE = (-0.788, -0.585, -0.075, 0.092)  # x0, x1, y0, y1
SQUARE_C = (-0.6865, 0.0085)
SQUARE_OUTER = (-0.806, -0.603, -0.093, 0.110)
# Reference rectangles flanking the square (PLACEHOLDER geometry: 20 x 4 cm bars along x, 18 cm either side).
REF_GREEN = (-0.70, 0.19)
REF_RED = (-0.70, -0.17)
BOX_X, BOX_HALF = (0.095, 0.20), 0.085  # scoop U footprint in the robot frame
CUBE_COLORS = {"red": (60, 60, 220), "orange": (40, 140, 250), "yellow": (60, 220, 240), "green": (60, 190, 60),
               "purple": (190, 70, 150), "blue": (200, 110, 30)}


def load(robot):
    data = yaml.safe_load((REPO / "src" / "mobile_base" / "config" / f"{robot}.yaml").read_text())
    return {k.split("/")[-1]: v["ros__parameters"] for k, v in data.items()}


def black(x, y):
    """Tape under the point: the rectangle's tape (with the gap at the square) and the square's own."""
    ox, oy = OUTER
    ix, iy = INNER
    in_outer = abs(x) <= ox and abs(y) <= oy
    in_inner = abs(x) < ix and abs(y) < iy
    if in_outer and not in_inner:
        return not (x < -ix and GAP_Y[0] <= y <= GAP_Y[1])
    x0, x1, y0, y1 = SQUARE_OUTER
    if x0 <= x <= x1 and y0 <= y <= y1:
        return not (BLUE[0] <= x <= BLUE[1] and BLUE[2] <= y <= BLUE[3])
    return False


def in_blue(x, y):
    return BLUE[0] <= x <= BLUE[1] and BLUE[2] <= y <= BLUE[3]


def to_robot(r, wx, wy):
    dx, dy = wx - r.x, wy - r.y
    c, s = math.cos(r.th), math.sin(r.th)
    return c * dx + s * dy, -s * dx + c * dy


def to_world(r, xr, yr):
    c, s = math.cos(r.th), math.sin(r.th)
    return r.x + c * xr - s * yr, r.y + s * xr + c * yr


def project(xr, yr, scale=1.0):
    """(cx_norm, cy_norm, distance) of a ground point in front of the robot, or None when out of view."""
    d = math.hypot(xr, yr)
    bearing = math.atan2(yr, xr)
    if xr <= 0.0 or abs(bearing) > HFOV / 2.0 or d < 0.10 or d > 1.2:
        return None
    return -bearing / (HFOV / 2.0), min(0.98, 0.66 * math.sqrt(0.2 / d)), d


@dataclass
class Robot:
    name: str
    x: float
    y: float
    th: float
    fsm: BehaviorFSM
    gate: SafetyGate
    targets: tuple
    avoid: tuple
    path: list = field(default_factory=list)
    touched: set = field(default_factory=set)  # ground-truth forbidden cubes the scoop touched
    tape_ticks: int = 0
    log: list = field(default_factory=list)
    state: str = ""
    trace: list = field(default_factory=list)


def make_robot(name, robot, start, targets, avoid):
    cfg = load(robot)
    fields = BehaviorParams.__dataclass_fields__
    values = {k: v for k, v in cfg["behavior"].items() if k in fields and k != "my_colors"}
    values.update(has_gate=False)
    fsm = BehaviorFSM(BehaviorParams(my_colors=targets, **values), rng=random.Random(sum(map(ord, name))))
    fsm.set_mission(targets, avoid)
    gfields = SafetyParams.__dataclass_fields__
    gate = SafetyGate(SafetyParams(**{**{k: v for k, v in cfg["safety_gate"].items() if k in gfields},
                                      "require_pose": False, "require_heartbeat": False}))
    return Robot(name, *start, fsm, gate, targets, avoid)


def observe(r, other, cubes):
    cam = []
    for c in cubes:
        p = project(*to_robot(r, c["x"], c["y"]))
        if p:
            cam.append(Cube(c["color"], p[0], p[1], 3000.0 * (0.2 / p[2]) ** 2))
    sq = project(*to_robot(r, *SQUARE_C))
    refs = [project(*to_robot(r, *REF_GREEN)), project(*to_robot(r, *REF_RED))]
    in_scoop = {c["color"] for c in cubes if 0.09 <= to_robot(r, c["x"], c["y"])[0] <= 0.19
                and abs(to_robot(r, c["x"], c["y"])[1]) < 0.06}
    scoop = "mixed" if len(in_scoop) > 1 else next(iter(in_scoop), "")  # like vision.scoop_content
    peer = None
    pp = project(*to_robot(r, other.x, other.y))
    if pp:
        peer = (-pp[0] * HFOV / 2.0, pp[2])
    inp = dict(cubes=cam, line_front=black(r.x, r.y), scoop_color=scoop, peer_seen=peer)
    if sq:
        inp.update(square_visible=True, square_cx=sq[0], square_cy=sq[1], square_area=20000.0)
    if all(refs):
        left, right = sorted(refs)  # sorted by cx like vision.detect_refs
        inp.update(refs_visible=True, refs_cx=(left[0] + right[0]) / 2.0, refs_skew=left[1] - right[1])
    return inp


def step_world(r, other, cubes, now, rng):
    obs = observe(r, other, cubes)
    want = r.fsm.step(BehaviorInput(now=now, pose=None, **obs))
    cmd, reason = r.gate.filter(SafetyInput(
        cmd=want, line_front=obs["line_front"], own_pose=None, peer_pose=None, now=now, heartbeat_age=None,
        carrying=r.fsm.carrying, peer_seen=obs["peer_seen"], delivering=r.fsm.state is State.DELIVER))
    r.trace.append((round(now, 1), r.fsm.state.value, tuple(round(v, 3) for v in want), tuple(round(v, 3) for v in cmd), reason,
                    obs["line_front"], round(r.x, 2), round(r.y, 2), round(math.degrees(r.th))))
    del r.trace[:-400]
    vx, vy, wz = cmd
    held = []
    for c in cubes:
        xr, yr = to_robot(r, c["x"], c["y"])
        if BOX_X[0] <= xr <= BOX_X[1] and abs(yr) < BOX_HALF:
            if c["color"] in r.avoid:
                r.touched.add(id(c))
            inside_u = abs(yr) < 0.06  # between the walls: slides in against the back wall, turns with the robot
            pushed = vx > 0.0 and wz == 0.0  # outside the walls: only pushed ahead, a spin sweeps past it
            if inside_u and vx >= 0.0 and abs(wz) <= 0.8:
                held.append((c, min(xr, 0.1395), yr))
            elif pushed:
                held.append((c, xr, yr))
    k = 0.9 + 0.05 * rng.random()  # the wheels deliver less than commanded
    th = r.th + wz * DT * k
    mx, my = (vx * math.cos(r.th) - vy * math.sin(r.th)) * DT * k, (vx * math.sin(r.th) + vy * math.cos(r.th)) * DT * k
    r.x, r.y, r.th = r.x + mx, r.y + my, th
    for c, xr, yr in held:
        c["x"], c["y"] = to_world(r, xr, yr)
    r.tape_ticks += black(r.x, r.y)
    r.path.append((r.x, r.y))
    if r.fsm.state.value != r.state:
        r.state = r.fsm.state.value
        r.log.append((now, r.state, r.fsm.note))


def draw(robots, cubes, t):
    scale = 800.0 / 1.8
    w, h = 1000, 600

    def px(x, y):
        return int(w / 2 + x * scale), int(h / 2 - y * scale)

    img = np.full((h, w, 3), 245, np.uint8)
    import cv2
    cv2.rectangle(img, px(-OUTER[0], OUTER[1]), px(OUTER[0], -OUTER[1]), (30, 30, 30), 3)
    cv2.rectangle(img, px(BLUE[0], BLUE[3]), px(BLUE[1], BLUE[2]), (200, 110, 30), -1)
    cv2.line(img, px(-INNER[0], GAP_Y[0]), px(-INNER[0], GAP_Y[1]), (245, 245, 245), 4)
    for (x, y), col in ((REF_GREEN, (60, 190, 60)), (REF_RED, (60, 60, 220))):
        cv2.rectangle(img, px(x - 0.10, y + 0.02), px(x + 0.10, y - 0.02), col, -1)
    cv2.line(img, px(-0.806, SQUARE_C[1]), px(0.40, SQUARE_C[1]), (190, 190, 190), 1)  # the delivery axis
    for r, col in zip(robots, ((0, 0, 200), (200, 0, 0))):
        pts = np.array([px(*p) for p in r.path[::3]], np.int32)
        if len(pts) > 1:
            cv2.polylines(img, [pts], False, col, 1)
        cv2.circle(img, px(r.x, r.y), 14, col, 2)
        cv2.line(img, px(r.x, r.y), px(*to_world(r, 0.18, 0.0)), col, 3)
        cv2.putText(img, f"{r.name} {r.state}", (px(r.x, r.y)[0] - 40, px(r.x, r.y)[1] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)
    for c in cubes:
        cv2.rectangle(img, (px(c["x"], c["y"])[0] - 6, px(c["x"], c["y"])[1] - 6),
                      (px(c["x"], c["y"])[0] + 6, px(c["x"], c["y"])[1] + 6), CUBE_COLORS[c["color"]], -1)
    cv2.putText(img, f"t={t:5.1f}s", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
    return img


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seconds", type=float, default=180.0)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--video", default=None)
    ap.add_argument("--png", default=None)
    ap.add_argument("--ferb", action="store_true", help="also run Ferb (off by default: Phineas alone)")
    ap.add_argument("--trace", type=int, default=0, help="print the last N ticks of each robot")
    args = ap.parse_args()
    rng = random.Random(args.seed)

    phineas = make_robot("Phineas", "phineas", (0.40, 0.20, math.pi), ("red", "orange"), ("yellow",))
    ferb = make_robot("Ferb", "ferb", (0.40, -0.20, math.pi), ("green",), ("purple",))
    layout = [("red", 0.10, 0.15), ("orange", -0.10, -0.12), ("yellow", 0.20, -0.02), ("green", -0.30, 0.18),
              ("purple", 0.00, 0.00), ("red", 0.30, -0.10), ("green", -0.20, -0.20), ("orange", 0.05, 0.25)]
    cubes = [{"color": c, "x": x + rng.uniform(-0.03, 0.03), "y": y + rng.uniform(-0.03, 0.03)} for c, x, y in layout]
    robots = [phineas, ferb] if args.ferb else [phineas]
    if not args.ferb:
        ferb.x, ferb.y = 9.0, 9.0  # parked far away: Phineas never sees its marker
    writer = None
    if args.video:
        import cv2
        writer = cv2.VideoWriter(args.video, cv2.VideoWriter_fourcc(*"mp4v"), 10, (1000, 600))
    collisions = 0
    steps = int(args.seconds / DT)
    for i in range(steps):
        now = i * DT
        step_world(phineas, ferb, cubes, now, rng)
        if args.ferb:
            step_world(ferb, phineas, cubes, now, rng)
            collisions += math.hypot(phineas.x - ferb.x, phineas.y - ferb.y) < 0.19
        if writer and i % 2 == 0:
            writer.write(draw(robots, cubes, now))
    if writer:
        writer.release()
    if args.png:
        import cv2
        cv2.imwrite(args.png, draw(robots, cubes, steps * DT))

    print(f"== {args.seconds:.0f} s simulados, semilla {args.seed} ==")
    for r in robots:
        inside = sum(1 for c in cubes if c["color"] in r.targets and in_blue(c["x"], c["y"]))
        owned = phineas.targets + ferb.targets if args.ferb else r.targets
        wrong = sum(1 for c in cubes if c["color"] not in owned and in_blue(c["x"], c["y"]))
        print(f"{r.name}: objetivo {r.targets} prohibido {r.avoid}\n  entregados (verdad): {inside}"
              f"  (cuenta del robot: {r.fsm.delivered})  cubos ajenos en el cuadro (NO debe haber): {wrong}"
              f"  prohibidos tocados: {len(r.touched)}  ticks sobre la cinta: {r.tape_ticks}")
        for t, state, note in r.log[:40]:
            print(f"    [{t:6.1f}] {state:8s} {note}")
    for r in robots:
        for row in r.trace[-args.trace:] if args.trace else ():
            print(r.name, *row)
    print(f"ticks con los robots a menos de 19 cm: {collisions}")


if __name__ == "__main__":
    main()
