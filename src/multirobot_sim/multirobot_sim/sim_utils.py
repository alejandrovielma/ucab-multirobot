"""Pure helpers of the Webots driver plugin (testable without Webots)."""

import random

from mobile_base.core.geometry import wrap_angle


def parse_wheel_signs(text: str) -> tuple[float, float, float, float]:
    """Parse "1,-1,1,1" (order FL, FR, RL, RR) into per-wheel velocity signs."""
    values = tuple(float(v) for v in text.split(","))
    if len(values) != 4 or any(v not in (-1.0, 1.0) for v in values):
        raise ValueError(f"wheel_signs must be four values of 1 or -1, got {text!r}")
    return values


def is_black(raw_value: float, threshold: float) -> bool:
    """Simulated TCRT5000: ~240 on the white board, 0-30 on the black tape."""
    return raw_value < threshold


def noisy_pose(
    x: float, y: float, theta: float, sigma_xy: float, sigma_theta: float, rng: random.Random
) -> tuple[float, float, float]:
    """Add Gaussian noise to emulate the overhead camera measurement."""
    if sigma_xy > 0.0:
        x += rng.gauss(0.0, sigma_xy)
        y += rng.gauss(0.0, sigma_xy)
    if sigma_theta > 0.0:
        theta += rng.gauss(0.0, sigma_theta)
    return x, y, wrap_angle(theta)
