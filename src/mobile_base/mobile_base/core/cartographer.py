"""Cartographer: the robot's memory of the track, built on the particle-filter estimate.

It remembers (only while the estimate is trusted): which cells the camera has looked at, where target
cubes were seen (to come back for them), and how often the tape was hit in each cell (to keep away from
traps). From that it proposes an exploration goal: a remembered target first, otherwise the cell that has
gone longest unseen, away from the tape and from cells with many hits, not too far. The sequencer decides
what to do with the goal; the safety gate and the line sensor still have the last word.
"""

import math
from dataclasses import dataclass

import numpy as np

from mobile_base.core.localization import Estimate, Localizer


@dataclass(frozen=True)
class CartoParams:
    cell: float = 0.05  # m
    view_min: float = 0.12  # m, camera footprint counted as looked at
    view_max: float = 0.50
    view_half_angle: float = 0.6  # rad (a little under the 0.8 rad half field of view)
    tape_margin: float = 0.15  # m, goals at least this far from the tape (body front 0.095 + margin)
    merge: float = 0.08  # m, two sightings closer than this are the same cube
    forget_after: float = 120.0  # s, a remembered cube not confirmed for this long is dropped
    goal_reached: float = 0.10  # m
    distance_cost: float = 30.0  # s of "unseen" one meter of travel is worth
    hit_cost: float = 20.0  # s per tape hit recorded in the goal cell or its neighbors
    unseen_cap: float = 90.0  # s, cells unseen longer than this are all equally attractive


@dataclass
class Sighting:
    color: str
    x: float
    y: float
    t: float


class Cartographer:
    def __init__(self, localizer: Localizer, params: CartoParams = CartoParams()):
        self.loc, self.params = localizer, params
        ix, iy = localizer.tape.inner
        self.nx, self.ny = int(math.ceil(2 * ix / params.cell)), int(math.ceil(2 * iy / params.cell))
        xs = -ix + (np.arange(self.nx) + 0.5) * params.cell
        ys = -iy + (np.arange(self.ny) + 0.5) * params.cell
        self.cx, self.cy = np.meshgrid(xs, ys, indexing="ij")  # cell centers
        self.seen_at = np.full((self.nx, self.ny), -np.inf)
        self.hits = np.zeros((self.nx, self.ny))
        # Cells a goal may be in: far enough from the tape on every side.
        m = params.tape_margin
        self.goal_ok = (np.abs(self.cx) <= ix - m) & (np.abs(self.cy) <= iy - m)
        self.cubes: list[Sighting] = []
        self.goal: tuple[float, float] | None = None
        self.est: Estimate = localizer.estimate()

    # ---- per tick

    def moved(self, cmd: tuple[float, float, float], dt: float) -> None:
        self.loc.predict(cmd, dt)

    def observe(self, now: float, black: bool, landmarks, cubes, targets, carrying: bool = False) -> Estimate:
        """landmarks: [(name, cx, cy)]; cubes: objects with color, cx_norm, cy_norm (the camera's)."""
        self.loc.update_line(black)
        for name, cx, cy in landmarks:
            self.loc.update_landmark(name, cx, cy)
        self.est = est = self.loc.estimate()
        if not est.localized:
            return est
        if black:
            i, j = self._cell(est.x, est.y)
            if i is not None:
                self.hits[i, j] += 1.0
        self._look(now, est, cubes, targets, carrying)
        return est

    def goal_for(self, now: float, targets) -> tuple[float, float] | None:
        """Where exploring should head, or None without a trusted estimate."""
        est = self.est
        if not est.localized:
            return None
        live = [c for c in self.cubes if c.color in targets and now - c.t <= self.params.forget_after]
        self.cubes = live
        if live:
            best = min(live, key=lambda c: math.hypot(c.x - est.x, c.y - est.y))
            self.goal = (best.x, best.y)
            return self.goal
        if self.goal is not None:
            gi, gj = self._cell(*self.goal)
            reached = math.hypot(self.goal[0] - est.x, self.goal[1] - est.y) < self.params.goal_reached
            if gi is not None and not reached and self.seen_at[gi, gj] < now - 1.0:
                return self.goal  # keep it until it is looked at: no dithering between cells
        self.goal = self._frontier(now, est)
        return self.goal

    def tape_ahead(self) -> float | None:
        est = self.est
        return self.loc.tape.distance_ahead(est.x, est.y, est.theta) if est.localized else None

    # ---- internals

    def _cell(self, x: float, y: float):
        ix, iy = self.loc.tape.inner
        i, j = int((x + ix) // self.params.cell), int((y + iy) // self.params.cell)
        return (i, j) if 0 <= i < self.nx and 0 <= j < self.ny else (None, None)

    def _look(self, now: float, est: Estimate, cubes, targets, carrying: bool) -> None:
        p = self.params
        dx, dy = self.cx - est.x, self.cy - est.y
        d = np.hypot(dx, dy)
        rel = np.remainder(np.arctan2(dy, dx) - est.theta + math.pi, 2.0 * math.pi) - math.pi
        view = (d >= p.view_min) & (d <= p.view_max) & (np.abs(rel) <= p.view_half_angle)
        self.seen_at[view] = now
        cam = self.loc.camera
        ix, iy = self.loc.tape.inner
        seen_now = []
        for c in cubes:
            if c.color not in targets:
                continue
            dist, b = cam.distance(c.cy_norm), cam.bearing(c.cx_norm)
            ox, oy = est.x + cam.camera_x * math.cos(est.theta), est.y + cam.camera_x * math.sin(est.theta)
            x, y = ox + dist * math.cos(est.theta + b), oy + dist * math.sin(est.theta + b)
            if abs(x) < ix - 0.02 and abs(y) < iy - 0.02:  # on the track: not delivered, not past the tape
                seen_now.append((c.color, x, y))
        kept = []
        for s in self.cubes:
            sd = math.hypot(s.x - est.x, s.y - est.y)
            srel = math.remainder(math.atan2(s.y - est.y, s.x - est.x) - est.theta, 2.0 * math.pi)
            in_view = p.view_min <= sd <= p.view_max and abs(srel) <= p.view_half_angle
            confirmed = any(col == s.color and math.hypot(x - s.x, y - s.y) < p.merge for col, x, y in seen_now)
            if carrying and sd < 0.25:
                continue  # this is the cube in the scoop
            if in_view and not confirmed:
                continue  # looked at its place and it is not there
            kept.append(s)
        for color, x, y in seen_now:
            match = next((s for s in kept if s.color == color and math.hypot(s.x - x, s.y - y) < p.merge), None)
            if match is None:
                if not (carrying and math.hypot(x - est.x, y - est.y) < 0.25):
                    kept.append(Sighting(color, x, y, now))
            else:
                match.x, match.y, match.t = (match.x + x) / 2.0, (match.y + y) / 2.0, now
        self.cubes = kept

    def _frontier(self, now: float, est: Estimate) -> tuple[float, float] | None:
        p = self.params
        unseen = np.minimum(now - self.seen_at, p.unseen_cap)
        dist = np.hypot(self.cx - est.x, self.cy - est.y)
        # Tape hits near a cell (3x3 neighborhood) make it less attractive: corners that trap the robot.
        padded = np.pad(self.hits, 1)
        near_hits = sum(padded[1 + a:1 + a + self.nx, 1 + b:1 + b + self.ny] for a in (-1, 0, 1) for b in (-1, 0, 1))
        score = unseen - p.distance_cost * dist - p.hit_cost * near_hits
        score = np.where(self.goal_ok & (dist > p.goal_reached), score, -np.inf)
        if not np.isfinite(score).any():
            return None
        i, j = np.unravel_index(int(np.argmax(score)), score.shape)
        return float(self.cx[i, j]), float(self.cy[i, j])
