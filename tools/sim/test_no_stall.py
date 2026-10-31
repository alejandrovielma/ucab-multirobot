"""Closed-loop regression: the real BehaviorFSM + SafetyGate must not freeze in one state.

A stall is more than STALL_S simulated seconds in one non-IDLE state with under 5 cm of extent and
under 20 deg of turn. Run from the repo root: python3 -m pytest tools/sim/test_no_stall.py
"""
import dataclasses
import math
import pathlib
import random
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import sim2d as S  # noqa: E402
from mobile_base.core.cartographer import Cartographer  # noqa: E402
from mobile_base.core.localization import Localizer  # noqa: E402

LAYOUT = [("yellow", 0.10, 0.15), ("yellow", -0.10, -0.12), ("yellow", 0.30, -0.10), ("blue", 0.20, -0.02),
          ("blue", -0.30, 0.18), ("blue", 0.05, 0.25), ("red", -0.20, -0.20), ("green", 0.00, 0.00),
          ("purple", 0.25, 0.20), ("orange", -0.05, 0.0)]
SECONDS = 300
STALL_S = 30.0
IGNORE_LAST_S = 10.0
# Seeds that stalled in CARRY (sidestep jitter) before the dodge was latched and time-bounded.
CASES = [("blue", 29), ("blue", 17), ("blue", 18), ("yellow", 5), ("yellow", 22), ("yellow", 33)]
# Seeds that stalled in EXPLORE (avoid spin vs guided spin alternating) before the guide was switched off.
CASES += [("yellow", 6), ("yellow", 23), ("blue", 20), ("blue", 22)]
# Seed that stalled in EXPLORE at the lab's speed caps (--max-speed 0.05 --max-turn 0.5) before the blind turn
# was timed by angle and the turn away from the estimated tape kept its direction (it rocked in place).
# LAB_CAPS mirrors tools/lab/lab_robot.py lab_speed_caps with its default arguments.
LAB_CASES = [("yellow", 33), ("blue", 15)]  # stall at t = 532 s / 244 s: run the full 600 s
LAB_CAPS = dict(explore_speed=0.05, approach_speed=0.05, capture_speed=0.05, carry_speed=0.05,
                turn_speed=0.5, approach_turn_min=0.4, k_turn=0.5)


def simulate(color, seed, caps=None, seconds=SECONDS):
    rng = random.Random(seed)
    other = "blue" if color == "yellow" else "yellow"
    r = S.make_robot("Phineas", "phineas", (0.40, 0.20, math.pi), (color,), (other,))
    if caps:
        r.fsm.params = dataclasses.replace(r.fsm.params, **caps)
    ferb = S.make_robot("Ferb", "ferb", (9.0, 9.0, 0.0), ("green",), ("purple",))
    cubes = [{"color": c, "x": x + rng.uniform(-.03, .03), "y": y + rng.uniform(-.03, .03)} for c, x, y in LAYOUT]
    r.carto = Cartographer(Localizer(seed=seed, start=None))
    tr = []
    for i in range(int(seconds / S.DT)):
        S.step_world(r, ferb, cubes, i * S.DT, rng)
        tr.append((r.fsm.state.value, r.x, r.y, r.th))
    return tr


def find_stall(tr):
    """(state, start_s) of the first window of STALL_S in one state without moving, or None."""
    w, cut = int(STALL_S / S.DT), len(tr) - int(IGNORE_LAST_S / S.DT)
    seg = 0
    for i in range(1, len(tr) + 1):
        if i < len(tr) and tr[i][0] == tr[seg][0]:
            continue
        if tr[seg][0] != "IDLE":
            for a in range(seg, min(i, cut) - w + 1, int(1 / S.DT)):
                win = tr[a:a + w]
                ext = max(max(t[1] for t in win) - min(t[1] for t in win), max(t[2] for t in win) - min(t[2] for t in win))
                turn = max(abs(math.degrees(math.remainder(t[3] - win[0][3], 2 * math.pi))) for t in win)
                if ext < 0.05 and turn < 20:
                    return tr[seg][0], round(a * S.DT, 1)
        seg = i
    return None


@pytest.mark.parametrize("color,seed", CASES)
def test_no_stall(color, seed):
    assert find_stall(simulate(color, seed)) is None


@pytest.mark.parametrize("color,seed", LAB_CASES)
def test_no_stall_at_the_lab_speed_caps(color, seed):
    assert find_stall(simulate(color, seed, LAB_CAPS, seconds=600)) is None
