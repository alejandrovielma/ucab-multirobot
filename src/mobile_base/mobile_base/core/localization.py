"""Where the robot is without the overhead camera: a particle filter (Monte Carlo localization).

Predict: every particle moves with the command the safety gate let through, scaled by its own wheel
efficiency (unknown: the particles whose efficiency matches the real robot survive the resampling, so the
filter calibrates itself) plus noise. Correct: the line sensor against the known tape (black only on the
tape; the blue square reads white) and the camera's landmarks (blue square, green and red rectangles:
bearing from cx, distance from cy through a calibration table). Lost (a black reading that no particle
explains): part of the particles is re-seeded on the tape, where the robot certainly is.

World frame of core/delivery.py: origin at the track center, x east, y north, meters. Track measured on
2026-10-15 (tools/sim/generate_track.py); the reference rectangles and the camera table are PLACEHOLDERS.
"""

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class TapeMap:
    outer: tuple[float, float] = (0.603, 0.370)  # half sizes of the rectangle at the tape's outer edge
    inner: tuple[float, float] = (0.585, 0.352)
    gap_y: tuple[float, float] = (-0.075, 0.092)  # the square's side toward the track has no tape
    square_outer: tuple[float, float, float, float] = (-0.806, -0.603, -0.093, 0.110)  # x0, x1, y0, y1
    blue: tuple[float, float, float, float] = (-0.788, -0.585, -0.075, 0.092)

    def black(self, x, y):
        """Tape under the point(s)."""
        x, y = np.asarray(x, float), np.asarray(y, float)
        (ox, oy), (ix, iy) = self.outer, self.inner
        ring = (np.abs(x) <= ox) & (np.abs(y) <= oy) & ~((np.abs(x) < ix) & (np.abs(y) < iy))
        gap = (x < -ix) & (y >= self.gap_y[0]) & (y <= self.gap_y[1])
        return (ring & ~gap) | (self._in(x, y, self.square_outer) & ~self._in(x, y, self.blue))

    def allowed(self, x, y):
        """Where the robot center can be: inside the outer tape edge or over the square."""
        x, y = np.asarray(x, float), np.asarray(y, float)
        ox, oy = self.outer
        return ((np.abs(x) <= ox) & (np.abs(y) <= oy)) | self._in(x, y, self.square_outer)

    def distance_ahead(self, x: float, y: float, th: float, limit: float = 1.5, step: float = 0.01) -> float:
        """Meters from the point along the heading to the first tape (or out of the board)."""
        d = np.arange(step, limit, step)
        xs, ys = x + d * math.cos(th), y + d * math.sin(th)
        stop = self.black(xs, ys) | ~self.allowed(xs, ys)
        return float(d[np.argmax(stop)]) if stop.any() else limit

    @staticmethod
    def _in(x, y, box):
        x0, x1, y0, y1 = box
        return (x >= x0) & (x <= x1) & (y >= y0) & (y <= y1)


# Landmark centers, world frame. The square is measured; the rectangles are PLACEHOLDERS (unmeasured).
LANDMARKS = {"square": (-0.6865, 0.0085), "green": (-0.70, 0.19), "red": (-0.70, -0.17)}


@dataclass(frozen=True)
class CameraModel:
    """Image -> ground: bearing from cx (pinhole, left positive), distance from cy (calibration table)."""

    hfov: float = 1.60  # rad, IMX219-120
    # (cy_norm, meters) pairs, cy ascending. PLACEHOLDER (the 2D sim's model): measure on the track with
    # a cube at known distances in front of the camera.
    cy_table: tuple[tuple[float, float], ...] = (
        (0.27, 1.195), (0.30, 0.968), (0.35, 0.711), (0.40, 0.545), (0.45, 0.430), (0.50, 0.348),
        (0.60, 0.242), (0.66, 0.200), (0.70, 0.178), (0.80, 0.136), (0.90, 0.108), (0.98, 0.091))
    camera_x: float = 0.0  # m, camera ahead of the robot center (the table already measures from it)

    def bearing(self, cx: float) -> float:
        return -cx * self.hfov / 2.0

    def distance(self, cy: float) -> float:
        cys, ds = zip(*self.cy_table)
        return float(np.interp(cy, cys, ds))


@dataclass(frozen=True)
class LocParams:
    particles: int = 1000  # vectorized: a few ms per tick on the Pi 5
    trans_noise: float = 0.10  # std of the translation, fraction of the commanded one
    rot_noise: float = 0.10  # std of the rotation, fraction of the commanded one
    pos_noise: float = 0.003  # m per step, always (unmodeled pushes)
    heading_noise: float = 0.01  # rad per step, always
    scale_range: tuple[float, float] = (0.6, 1.2)  # wheel efficiency: real motion / commanded motion
    scale_jitter: float = 0.005
    p_black_on_tape: float = 0.9  # line sensor: black when the center is on the tape
    p_black_off_tape: float = 0.02  # black elsewhere (noise, a dark cube)
    bearing_std: float = 0.10  # rad
    range_std: float = 0.25  # fraction of the distance
    outlier: float = 0.02  # floor of a landmark likelihood (misdetections must not kill the filter)
    random_fraction: float = 0.01  # particles always re-drawn uniformly at each resampling
    # Augmented MCL (Thrun et al., Probabilistic Robotics 8.3.5): when the landmarks fit worse lately
    # (fast average) than usual (slow average) the filter may have locked on a wrong pose: re-draw more.
    alpha_slow: float = 0.01
    alpha_fast: float = 0.2
    max_random_fraction: float = 0.2
    # Roughening after resampling, so copies of one particle spread out even when the robot stands still.
    rough_pos: float = 0.008  # m
    rough_heading: float = 0.02  # rad
    reseed_fraction: float = 0.5  # share re-seeded on the tape when a black reading is unexplained
    localized_spread: float = 0.10  # m, position spread under which the estimate is trusted
    localized_heading: float = 0.35  # rad, heading spread under which the estimate is trusted


@dataclass(frozen=True)
class Estimate:
    x: float
    y: float
    theta: float
    spread: float  # m, weighted position std
    heading_spread: float  # rad, circular std
    scale: float  # learned wheel efficiency
    localized: bool


class Localizer:
    def __init__(self, params: LocParams = LocParams(), tape: TapeMap = TapeMap(), camera: CameraModel = CameraModel(),
                 landmarks: dict | None = None, seed: int = 0, start: tuple[float, float, float] | None = None):
        self.params, self.tape, self.camera = params, tape, camera
        self.landmarks = dict(LANDMARKS if landmarks is None else landmarks)
        self.rng = np.random.default_rng(seed)
        self.w_slow = self.w_fast = 0.0
        self.reset(start)

    def reset(self, start: tuple[float, float, float] | None = None) -> None:
        """Uniform over the track, or around a known start (the person placed the robot there)."""
        n = self.params.particles
        if start is None:
            self.p = self._uniform(n)
        else:
            self.p = self._uniform(n)
            self.p[:, 0] = start[0] + self.rng.normal(0.0, 0.03, n)
            self.p[:, 1] = start[1] + self.rng.normal(0.0, 0.03, n)
            self.p[:, 2] = start[2] + self.rng.normal(0.0, 0.1, n)
        self.w = np.full(n, 1.0 / n)

    def predict(self, cmd: tuple[float, float, float], dt: float) -> None:
        p, n, q = self.params, len(self.w), self.p
        vx, vy, wz = cmd
        s = q[:, 3]
        tx = vx * dt * s * (1.0 + self.rng.normal(0.0, p.trans_noise, n))
        ty = vy * dt * s * (1.0 + self.rng.normal(0.0, p.trans_noise, n))
        dth = wz * dt * s * (1.0 + self.rng.normal(0.0, p.rot_noise, n))
        c, si = np.cos(q[:, 2]), np.sin(q[:, 2])
        q[:, 0] += tx * c - ty * si + self.rng.normal(0.0, p.pos_noise, n)
        q[:, 1] += tx * si + ty * c + self.rng.normal(0.0, p.pos_noise, n)
        q[:, 2] = np.remainder(q[:, 2] + dth + self.rng.normal(0.0, p.heading_noise, n) + math.pi,
                               2.0 * math.pi) - math.pi
        q[:, 3] = np.clip(s + self.rng.normal(0.0, p.scale_jitter, n), *p.scale_range)
        outside = ~self.tape.allowed(q[:, 0], q[:, 1])
        if outside.any():
            self._weigh(np.where(outside, 0.01, 1.0))

    def update_line(self, black: bool) -> None:
        p = self.params
        on = self.tape.black(self.p[:, 0], self.p[:, 1])
        if black:
            like = np.where(on, p.p_black_on_tape, p.p_black_off_tape)
            if float(np.dot(self.w, on)) < 0.02:
                self._reseed_on_tape()  # nobody thought we were on the tape: the filter is lost
                on = self.tape.black(self.p[:, 0], self.p[:, 1])
                like = np.where(on, p.p_black_on_tape, p.p_black_off_tape)
        else:
            like = np.where(on, 1.0 - p.p_black_on_tape, 1.0 - p.p_black_off_tape)
        self._weigh(like)

    def update_landmark(self, name: str, cx: float, cy: float) -> None:
        p, q = self.params, self.p
        lx, ly = self.landmarks[name]
        cam_x = q[:, 0] + self.camera.camera_x * np.cos(q[:, 2])
        cam_y = q[:, 1] + self.camera.camera_x * np.sin(q[:, 2])
        dx, dy = lx - cam_x, ly - cam_y
        expected_b = np.arctan2(dy, dx) - q[:, 2]
        err_b = np.remainder(self.camera.bearing(cx) - expected_b + math.pi, 2.0 * math.pi) - math.pi
        expected_d = np.hypot(dx, dy)
        sigma_d = p.range_std * expected_d + 0.02
        err_d = (self.camera.distance(cy) - expected_d) / sigma_d
        # Normalized density (1/sigma): without it the far poses, with a wider sigma, would be favored.
        like = p.outlier + np.exp(-0.5 * (err_b / p.bearing_std) ** 2 - 0.5 * err_d ** 2) * (0.1 / sigma_d)
        fit = float(np.dot(self.w, like))
        self.w_slow = fit if self.w_slow == 0.0 else self.w_slow + p.alpha_slow * (fit - self.w_slow)
        self.w_fast = fit if self.w_fast == 0.0 else self.w_fast + p.alpha_fast * (fit - self.w_fast)
        self._weigh(like)

    def estimate(self) -> Estimate:
        p, q, w = self.params, self.p, self.w
        x, y = float(np.dot(w, q[:, 0])), float(np.dot(w, q[:, 1]))
        spread = math.sqrt(float(np.dot(w, (q[:, 0] - x) ** 2 + (q[:, 1] - y) ** 2)))
        c, s = float(np.dot(w, np.cos(q[:, 2]))), float(np.dot(w, np.sin(q[:, 2])))
        r = min(1.0, math.hypot(c, s))
        heading_spread = math.sqrt(-2.0 * math.log(r)) if r > 1e-9 else math.pi
        return Estimate(x, y, math.atan2(s, c), spread, heading_spread, float(np.dot(w, q[:, 3])),
                        spread < p.localized_spread and heading_spread < p.localized_heading)

    # ---- internals

    def _uniform(self, n: int) -> np.ndarray:
        ix, iy = self.tape.inner
        lo, hi = self.params.scale_range
        return np.column_stack([self.rng.uniform(-ix, ix, n), self.rng.uniform(-iy, iy, n),
                                self.rng.uniform(-math.pi, math.pi, n), self.rng.uniform(lo, hi, n)])

    def _reseed_on_tape(self) -> None:
        n = int(len(self.w) * self.params.reseed_fraction)
        ox, oy = self.tape.outer
        x0 = self.tape.square_outer[0]
        cand = np.column_stack([self.rng.uniform(x0, ox, 40 * n), self.rng.uniform(-oy, oy, 40 * n)])
        cand = cand[self.tape.black(cand[:, 0], cand[:, 1])][:n]
        k = len(cand)
        idx = self.rng.choice(len(self.w), k, replace=False)
        lo, hi = self.params.scale_range
        self.p[idx, 0], self.p[idx, 1] = cand[:, 0], cand[:, 1]
        self.p[idx, 2] = self.rng.uniform(-math.pi, math.pi, k)
        self.p[idx, 3] = self.rng.uniform(lo, hi, k)
        self.w[idx] = 1.0 / len(self.w)
        self.w /= self.w.sum()

    def _weigh(self, like: np.ndarray) -> None:
        w = self.w * like
        total = float(w.sum())
        if total <= 0.0 or not math.isfinite(total):
            self.reset()  # every particle ruled out: start over
            return
        self.w = w / total
        if 1.0 / float(np.dot(self.w, self.w)) < len(self.w) / 2.0:
            self._resample()

    def _resample(self) -> None:
        """Low-variance resampling, plus a few uniform particles to recover from a wrong lock."""
        n = len(self.w)
        positions = (self.rng.random() + np.arange(n)) / n
        idx = np.minimum(np.searchsorted(np.cumsum(self.w), positions), n - 1)
        self.p = self.p[idx].copy()
        p = self.params
        self.p[:, 0] += self.rng.normal(0.0, p.rough_pos, n)
        self.p[:, 1] += self.rng.normal(0.0, p.rough_pos, n)
        self.p[:, 2] += self.rng.normal(0.0, p.rough_heading, n)
        worse = 1.0 - self.w_fast / self.w_slow if self.w_slow > 0.0 else 0.0
        k = int(n * min(p.max_random_fraction, max(p.random_fraction, worse)))
        if k:
            self.p[self.rng.choice(n, k, replace=False)] = self._uniform(k)
        self.w = np.full(n, 1.0 / n)
