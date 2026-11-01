"""Boundary conversions between recolector_interfaces messages and the core's own types.

Pure and duck-typed (no rclpy): the core keeps its normalized detections (cx_norm in [-1, 1],
cy_norm of the centroid in [0, 1], area_px) and only the ROS nodes call these.
"""

import math

from mobile_base.core.behavior import Cube


def cube_from_ferb(det, image_width: int, image_height: int) -> Cube:
    """recolector_interfaces/CubeDetection -> Cube. Ferb's v is the BOTTOM edge row, ours the centroid."""
    cx = 2.0 * det.u / image_width - 1.0
    cy = (det.v - det.height_px / 2.0) / image_height
    return Cube(det.color, max(-1.0, min(1.0, cx)), max(0.0, min(1.0, cy)), float(det.width_px * det.height_px))


def cube_to_ferb_fields(color, cx_norm, cy_norm, area_px, image_width, image_height, camera) -> dict:
    """Our detection -> CubeDetection fields. `camera` has bearing_distance(u, v) (vision.camera_model).

    shortcut: the box shape is lost in our detection, so it is published as a square of side sqrt(area).
    """
    side = math.sqrt(max(area_px, 0.0))
    u = round((cx_norm + 1.0) / 2.0 * image_width)
    v = round(cy_norm * image_height + side / 2.0)
    bd = camera.bearing_distance(u, v)
    bearing, distance = bd if bd is not None else (0.0, 0.0)
    return dict(color=color, bearing_rad=float(bearing), distance_m=float(distance), u=u, v=v,
                width_px=round(side), height_px=round(side))


def line_front_from_sensors(msg) -> bool:
    """recolector_interfaces/LineSensors -> our line_front flag."""
    return bool(msg.black)
