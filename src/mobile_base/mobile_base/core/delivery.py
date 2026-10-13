"""Geometry of the delivery square, shared by the behavior (where to go) and the safety gate
(where reversing and crossing the square's tape are allowed).

World frame of the overhead camera / Webots: x east, y north, meters. Values measured on the
Webots track texture (square interior x -0.239..-0.020, y -0.388..-0.180, north tape at
y -0.180..-0.164); measure the real track and override them in the YAML.

The robot enters heading south (the ArUco id 10 marker stands on the south side, outside the
track). Robot positions below are of the robot CENTER; the cube sits cube_ahead meters in front
of it, inside the U of the fixed scoop. Rule: the scoop may step on the tape, the body may not, so
the robot stops with its front (0.095 m) just before the square's north tape and only the scoop
(with the cube) goes in.
"""

import math
from dataclasses import dataclass

from mobile_base.core.geometry import wrap_angle


@dataclass(frozen=True)
class DeliveryZone:
    x: float = -0.13  # square center (and entry axis)
    square_y: float = -0.284
    square_half: float = 0.104  # half side of the interior
    heading: float = -math.pi / 2  # entry heading (south)
    cube_ahead: float = 0.145  # cube center ahead of the robot center (against the U back wall)
    entry_y: float = 0.08  # robot center where it waits for the lease, north of the tape
    drop_y: float = -0.065  # body front at -0.160 (tape starts at -0.164); cube center at -0.210
    exit_y: float = 0.02  # robot center after reversing; the scoop is clear of the cube
    corridor_half_width: float = 0.10
    corridor_margin: float = 0.05
    max_heading_error: float = math.radians(60.0)

    def in_corridor(self, x: float, y: float) -> bool:
        """Robot center inside the strip between the entry point and the drop point."""
        return (
            abs(x - self.x) <= self.corridor_half_width
            and self.drop_y - self.corridor_margin <= y <= self.entry_y + self.corridor_margin
        )

    def aligned(self, theta: float) -> bool:
        return abs(wrap_angle(theta - self.heading)) <= self.max_heading_error


def zone_from(params) -> DeliveryZone:
    """Build the zone from the flat zone_* fields of a ROS parameter dataclass."""
    return DeliveryZone(
        x=params.zone_x,
        square_y=params.zone_square_y,
        entry_y=params.zone_entry_y,
        drop_y=params.zone_drop_y,
        exit_y=params.zone_exit_y,
        corridor_half_width=params.zone_corridor_half_width,
    )
