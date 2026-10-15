"""Geometry of the delivery square, shared by the behavior (where to go) and the safety gate
(where reversing and crossing the square's tape are allowed).

World frame of the overhead camera / Webots: origin at the center of the track rectangle, x east
(along the 120.6 cm side), y north, meters. Real track measured on 2026-10-15: the blue square sticks
out of the west side of the rectangle and its side toward the track has no tape, so the robot
approaches heading WEST (pi). Any approach heading works: every distance below is measured ALONG
the approach axis (the line through the square center with direction `heading`).

The "boundary" is the line the robot body must not cross: the rectangle's inner tape edge where the
square opens (x = -0.585 on the real track). It sits square_depth / 2 before the square center.
Robot positions are of the robot CENTER, given as distances BEFORE the boundary (positive on the
track side). The cube sits cube_ahead meters in front of the robot center, inside the U of the
fixed scoop. Rule: the scoop may enter the square, the body may not, so the robot stops with its
front (0.095 m) just before the boundary and only the scoop (with the cube) goes in.

Real values: drop 0.098 -> robot center x -0.487, body front 3 mm before the boundary, cube center
(2 cm cube against the U back wall, 0.1395 m ahead) at x -0.6265, 4 cm inside the blue. Entry 0.20 m
before the drop (x -0.287). Exit after reversing 0.205 (x -0.38): the scoop tip, 0.20 m ahead of the
center, ends east of the cube.
"""

import math
from dataclasses import dataclass

from mobile_base.core.geometry import wrap_angle


@dataclass(frozen=True)
class DeliveryZone:
    x: float = -0.6865  # square center (blue interior), world frame
    y: float = 0.0085
    heading: float = math.pi  # approach heading: the robot faces it while entering (west)
    square_depth: float = 0.203  # blue interior along the axis; the boundary is depth / 2 before the center
    square_width: float = 0.167  # blue interior across the axis
    entry_offset: float = 0.298  # robot center before the boundary where DELIVER starts (lease held)
    drop_offset: float = 0.098  # robot center before the boundary when the cube is inside
    exit_offset: float = 0.205  # robot center before the boundary after reversing out
    corridor_half_width: float = 0.08
    corridor_margin: float = 0.05
    max_heading_error: float = math.radians(60.0)
    cube_ahead: float = 0.1395  # cube center ahead of the robot center (2 cm cube against the U back wall)

    def along_across(self, x: float, y: float) -> tuple[float, float]:
        """Coordinates of a point on the approach axis: along = signed distance past the boundary
        (negative on the track side), across = distance to the left of the approach direction."""
        c, s = math.cos(self.heading), math.sin(self.heading)
        dx, dy = x - self.x, y - self.y
        return dx * c + dy * s + self.square_depth / 2.0, -dx * s + dy * c

    def before(self, x: float, y: float) -> float:
        """Distance of the point before the boundary along the axis (negative past it)."""
        return -self.along_across(x, y)[0]

    def point(self, before: float, across: float = 0.0) -> tuple[float, float]:
        """World point `before` meters before the boundary (and `across` to the left of the axis)."""
        c, s = math.cos(self.heading), math.sin(self.heading)
        along_center = -before - self.square_depth / 2.0
        return self.x + along_center * c - across * s, self.y + along_center * s + across * c

    def pose(self, before: float, across: float = 0.0) -> tuple[float, float, float]:
        """Robot pose on the axis, facing the square."""
        return (*self.point(before, across), self.heading)

    @property
    def entry_point(self) -> tuple[float, float]:
        return self.point(self.entry_offset)

    @property
    def drop_point(self) -> tuple[float, float]:
        return self.point(self.drop_offset)

    @property
    def exit_point(self) -> tuple[float, float]:
        return self.point(self.exit_offset)

    @property
    def cube_drop_point(self) -> tuple[float, float]:
        """Where the cube center ends when the robot stops at the drop point."""
        return self.point(self.drop_offset - self.cube_ahead)

    def inside_square(self, x: float, y: float, margin: float = 0.0) -> bool:
        """Point inside the square interior, at least margin from its sides."""
        along, across = self.along_across(x, y)
        return margin <= along <= self.square_depth - margin and abs(across) <= self.square_width / 2.0 - margin

    def in_corridor(self, x: float, y: float) -> bool:
        """Robot center inside the strip between the entry point and the drop point."""
        along, across = self.along_across(x, y)
        return (
            abs(across) <= self.corridor_half_width
            and self.drop_offset - self.corridor_margin <= -along <= self.entry_offset + self.corridor_margin
        )

    def aligned(self, theta: float) -> bool:
        return abs(wrap_angle(theta - self.heading)) <= self.max_heading_error


def zone_from(params) -> DeliveryZone:
    """Build the zone from the flat zone_* fields of a ROS parameter dataclass."""
    return DeliveryZone(
        x=params.zone_x,
        y=params.zone_y,
        heading=params.zone_heading,
        square_depth=params.zone_square_depth,
        entry_offset=params.zone_entry_offset,
        drop_offset=params.zone_drop_offset,
        exit_offset=params.zone_exit_offset,
        corridor_half_width=params.zone_corridor_half_width,
    )
