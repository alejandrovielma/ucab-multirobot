"""Carry planner: A* over forward-only motion primitives with the body kept inside the track.

With a cube in the fixed scoop the robot can only go straight or arc forward (spins eject the cube,
reversing leaves it behind): it is a Dubins car. Steering greedily toward the goal dead-ends in the
narrow parts of the track (Webots run 13); a search over the three primitives (straight, arc left,
arc right) finds a sequence that keeps every body corner inside. On the Webots track a plan exists
from 72 % of valid poses to the delivery entry, with a median planning time of 38 ms.
"""

import heapq
import math
from collections.abc import Callable
from dataclasses import dataclass

from mobile_base.core.track import Footprint, Track, path_inside, penetration, predict

Pose = tuple[float, float, float]


@dataclass(frozen=True)
class Plan:
    start: Pose
    steps: tuple[tuple[float, float, float], ...]  # (vx, wz, duration), consecutive equal ones merged

    @property
    def duration(self) -> float:
        return sum(step[2] for step in self.steps)

    def command(self, t: float) -> tuple[float, float] | None:
        """(vx, wz) at t seconds after the start, None once finished."""
        for vx, wz, duration in self.steps:
            if t < duration:
                return vx, wz
            t -= duration
        return None

    def expected(self, t: float) -> Pose:
        """Pose the robot should have at t seconds (open-loop replay)."""
        pose = self.start
        for vx, wz, duration in self.steps:
            dt = min(t, duration)
            pose = predict(pose, vx, wz, dt)
            t -= dt
            if t <= 0.0:
                break
        return pose


def _key(pose: Pose, cell: float, heading_bins: int) -> tuple[int, int, int]:
    theta = pose[2] % (2.0 * math.pi)
    return round(pose[0] / cell), round(pose[1] / cell), round(theta / (2.0 * math.pi / heading_bins)) % heading_bins


def plan_arcs(
    track: Track,
    footprint: Footprint,
    start: Pose,
    goal: Callable[[Pose], bool],
    target: tuple[float, float],
    straight_speed: float,
    arc_speed: float | None,
    turn: float,
    step: float = 0.03,
    cell: float = 0.02,
    heading_bins: int = 24,
    max_expansions: int = 30000,
    spin_speed: float | None = None,
    reverse_speed: float | None = None,
) -> Plan | None:
    """Shortest primitive sequence from start to a pose where goal(pose) holds, or None.

    target is only used by the heuristic (straight-line distance). Each primitive covers `step`
    meters; arcs use the tightest radius allowed with a cube (arc_speed / turn). arc_speed None
    leaves the arcs out (the robots now only translate or spin in place: team rule of 2026-10-27);
    spin_speed offers in-place spins (one heading bin each) and reverse_speed a straight reverse.
    """
    # A start already touching the edge (pose noise) is accepted; the plan never goes deeper than it.
    limit = penetration(track, footprint, start)
    primitives = [(straight_speed, 0.0, step / straight_speed)]
    if arc_speed:
        primitives += [(arc_speed, turn, step / arc_speed), (arc_speed, -turn, step / arc_speed)]
    if spin_speed:
        bin_angle = 2.0 * math.pi / heading_bins
        primitives += [(0.0, spin_speed, bin_angle / spin_speed), (0.0, -spin_speed, bin_angle / spin_speed)]
    if reverse_speed:
        primitives.append((-reverse_speed, 0.0, step / reverse_speed))
    tie = 0
    queue = [(0.0, 0.0, tie, start, ())]
    seen = set()
    while queue and len(seen) < max_expansions:
        _, cost, _, pose, path = heapq.heappop(queue)
        key = _key(pose, cell, heading_bins)
        if key in seen:
            continue
        seen.add(key)
        if goal(pose):
            return Plan(start, _merge(path))
        for vx, wz, duration in primitives:
            if not path_inside(track, footprint, pose, vx, wz, duration, steps=3, limit=limit):
                continue
            nxt = predict(pose, vx, wz, duration)
            if _key(nxt, cell, heading_bins) in seen:
                continue
            # Mild preference for straight segments; spins are cheap, reversing is discouraged.
            new_cost = cost + (0.5 if vx == 0.0 else 1.5 if vx < 0.0 else 1.0 + (0.2 if wz else 0.0))
            # Weighted A* (1.5 x heuristic): slightly longer paths, several times fewer expansions.
            h = 1.5 * math.hypot(nxt[0] - target[0], nxt[1] - target[1]) / step
            tie += 1
            heapq.heappush(queue, (new_cost + h, new_cost, tie, nxt, path + ((vx, wz, duration),)))
    return None


def _merge(path) -> tuple[tuple[float, float, float], ...]:
    merged: list[list[float]] = []
    for vx, wz, duration in path:
        if merged and merged[-1][0] == vx and merged[-1][1] == wz:
            merged[-1][2] += duration
        else:
            merged.append([vx, wz, duration])
    return tuple(tuple(m) for m in merged)
