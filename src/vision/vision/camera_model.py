"""
Pinhole camera model that maps pixels to floor positions. Pure logic, no
rclpy and no OpenCV.

The camera sits at some height, tilted down, with no sideways rotation.
"""

import math
from dataclasses import dataclass

# Camera geometry calibrated on the real robot: the bottom image row is at
# about 16.5 cm, right where the scoop's blind spot starts. The values that
# are actually used live in config/hsv_ranges.yaml; these are the fallbacks.
DEFAULT_HFOV_RAD = 0.94
DEFAULT_HEIGHT_M = 0.10
DEFAULT_TILT_RAD = 0.20

# A ray whose downward component is below this never reaches the floor.
MIN_DOWNWARD_COMPONENT = 1e-6


@dataclass
class CameraParams:
    """
    Camera geometry that does not depend on the image size.

    Field names double as the `camera.*` ROS parameter names.
    """

    hfov_rad: float = DEFAULT_HFOV_RAD
    height_m: float = DEFAULT_HEIGHT_M
    tilt_rad: float = DEFAULT_TILT_RAD  # positive = looking down


@dataclass(frozen=True)
class CameraModel:
    """
    Pinhole camera at some height, tilted down, with no sideways rotation.

    Robot axes: x forward, y to the left. The floor is the plane z = 0.
    """

    width_px: int
    height_px: int
    hfov_rad: float
    height_m: float
    tilt_rad: float  # positive = looking down

    @property
    def focal_px(self) -> float:
        """
        Focal length in pixels, derived from the horizontal field of view.
        """
        return (self.width_px / 2.0) / math.tan(self.hfov_rad / 2.0)

    def ground_point(
        self,
        u: float,
        v: float,
    ) -> tuple[float, float, float] | None:
        """
        Floor point seen at pixel (u, v).

        Returns (x_m, y_m, depth_m): position relative to the camera
        (projected on the floor) and the depth along the optical axis.
        None if the pixel is above the horizon.
        """
        f = self.focal_px
        left = (self.width_px / 2.0 - u) / f
        down = (v - self.height_px / 2.0) / f
        cos_tilt, sin_tilt = math.cos(self.tilt_rad), math.sin(self.tilt_rad)
        forward_w = cos_tilt - sin_tilt * down
        down_w = sin_tilt + cos_tilt * down
        if down_w <= MIN_DOWNWARD_COMPONENT:
            return None
        scale = self.height_m / down_w
        return scale * forward_w, scale * left, scale

    def bearing_distance(
        self,
        u: float,
        v: float,
    ) -> tuple[float, float] | None:
        """
        (bearing_rad, distance_m) of the floor point at (u, v).

        Positive bearing = left. None if the pixel is above the horizon.
        """
        point = self.ground_point(u, v)
        if point is None:
            return None
        x, y, _ = point
        return math.atan2(y, x), math.hypot(x, y)
