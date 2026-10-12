"""Track interior polygon for a geofence with the overhead pose.

The robot has a single front line sensor, and white looks the same inside and outside the tape:
once the sensor ends beyond the tape (for example after turning in place on it) nothing stops
the robot. With the overhead pose, "is my front inside the track?" has a direct answer.
"""

import math
from collections.abc import Sequence
from dataclasses import dataclass


class Track:
    def __init__(self, flat: Sequence[float] | None):
        values = list(flat or ())
        if len(values) % 2:
            raise ValueError("track_polygon needs x, y pairs (an even number of values)")
        self.vertices = list(zip(values[0::2], values[1::2]))
        self.enabled = len(self.vertices) >= 3
        self.centroid = self._area_centroid() if self.enabled else (0.0, 0.0)

    def contains(self, x: float, y: float) -> bool:
        """Ray casting; a disabled track contains everything (no geofence)."""
        if not self.enabled:
            return True
        inside = False
        n = len(self.vertices)
        for i in range(n):
            x1, y1 = self.vertices[i]
            x2, y2 = self.vertices[(i + 1) % n]
            if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
                inside = not inside
        return inside

    def front_inside(self, x: float, y: float, theta: float, ahead: float) -> bool:
        return self.contains(x + ahead * math.cos(theta), y + ahead * math.sin(theta))

    def _area_centroid(self) -> tuple[float, float]:
        a = cx = cy = 0.0
        n = len(self.vertices)
        for i in range(n):
            x1, y1 = self.vertices[i]
            x2, y2 = self.vertices[(i + 1) % n]
            cross = x1 * y2 - x2 * y1
            a += cross
            cx += (x1 + x2) * cross
            cy += (y1 + y2) * cross
        return cx / (3.0 * a), cy / (3.0 * a)

    def distance_to_edge(self, x: float, y: float) -> float:
        """Distance from (x, y) to the polygon boundary (inf when there is no track)."""
        if not self.enabled:
            return math.inf
        best = math.inf
        n = len(self.vertices)
        for i in range(n):
            x1, y1 = self.vertices[i]
            x2, y2 = self.vertices[(i + 1) % n]
            dx, dy = x2 - x1, y2 - y1
            length2 = dx * dx + dy * dy
            t = 0.0 if length2 == 0.0 else max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / length2))
            best = min(best, math.hypot(x - (x1 + t * dx), y - (y1 + t * dy)))
        return best


@dataclass(frozen=True)
class Footprint:
    """Robot body WITHOUT the scoop (wheels included): the scoop may step on the tape, the body may not."""

    front: float = 0.095  # m ahead of the center (front wheels reach 0.093)
    rear: float = 0.095
    half_width: float = 0.082  # wheels reach 0.0805

    @property
    def sweep_radius(self) -> float:
        """Circle swept by the corners when spinning in place."""
        return math.hypot(max(self.front, self.rear), self.half_width)

    def corners(self, x: float, y: float, theta: float) -> list[tuple[float, float]]:
        c, s = math.cos(theta), math.sin(theta)
        return [
            (x + dx * c - dy * s, y + dx * s + dy * c)
            for dx in (self.front, -self.rear)
            for dy in (self.half_width, -self.half_width)
        ]


def predict(pose, vx: float, wz: float, t: float) -> tuple[float, float, float]:
    """Exact unicycle motion (constant vx, wz) during t seconds."""
    x, y, theta = pose
    if abs(wz) < 1e-9:
        return x + vx * t * math.cos(theta), y + vx * t * math.sin(theta), theta
    r = vx / wz
    theta2 = theta + wz * t
    return x + r * (math.sin(theta2) - math.sin(theta)), y - r * (math.cos(theta2) - math.cos(theta)), theta2


def path_inside(track: "Track", footprint: Footprint, pose, vx: float, wz: float, horizon: float, steps: int = 6) -> bool:
    """True when every body corner stays inside the track along the commanded motion."""
    if not track.enabled:
        return True
    for k in range(1, steps + 1):
        x, y, theta = predict(pose, vx, wz, horizon * k / steps)
        if not all(track.contains(cx, cy) for cx, cy in footprint.corners(x, y, theta)):
            return False
    return True
