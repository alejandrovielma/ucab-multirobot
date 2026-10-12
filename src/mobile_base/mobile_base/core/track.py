"""Track interior polygon for a geofence with the overhead pose.

The robot has a single front line sensor, and white looks the same inside and outside the tape:
once the sensor ends beyond the tape (for example after turning in place on it) nothing stops
the robot. With the overhead pose, "is my front inside the track?" has a direct answer.
"""

import math
from collections.abc import Sequence


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
