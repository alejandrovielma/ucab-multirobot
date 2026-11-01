# Interface integration (Phineas nodes on Ferb's internal messages)

`multirobot_interfaces` stays the external fleet contract (RobotState, RobotPose, RequestZone, ReleaseZone). Phineas' internal topics now use `recolector_interfaces` (ADR 0004).

| Old (multirobot_interfaces) | New (recolector_interfaces) | Topic |
|---|---|---|
| `CubeDetectionArray` (cx_norm, cy_norm, area_px) | `CubeDetectionArray` (header, `CubeDetection[]`: color, bearing_rad, distance_m, u, v, width_px, height_px) | `cubes` |
| array-level `scoop_*`, `square_*` | `Scene` (new, additive) | `cubes/scene` |
| `std_msgs/Bool` | `LineSensors` (header, `black`) | `line_front` -> `line` |

## Conversion

The pure core (behavior, safety, cartographer) is unchanged and still uses normalized detections. `mobile_base/core/adapters.py` converts at the ROS boundary:

- `cube_from_ferb`: `cx_norm = 2*u/W - 1`, `cy_norm = (v - height_px/2)/H` (Ferb's `v` is the bottom edge, ours the centroid), `area_px = width_px*height_px`.
- `cube_to_ferb_fields`: inverse. The box shape is not kept by our detector, so it is published as a square of side `sqrt(area_px)`. `bearing_rad`/`distance_m` come from `vision.camera_model` with the `camera.*` parameters (defaults of Ferb's `CameraParams`; 0 when the pixel is above the horizon).
- `line_front_from_sensors`: `LineSensors.black` -> bool.

`behavior` takes `image_width`/`image_height` parameters (default 640x480). `cube_detector` takes the size from each image. `Scene` carries `scoop_color`, `scoop_fill`, `square_visible`, `square_cx`, `square_cy`, `square_area` with the semantics they had in the old array.

## Pending

`src/multirobot_sim/multirobot_sim/recolector_driver.py` (Webots driver) publishes `LineSensors` on `line` (not run: needs Webots). `camera.hfov_rad` defaults to Ferb's lens (0.94); the IMX219-120 on Phineas needs its own value.
