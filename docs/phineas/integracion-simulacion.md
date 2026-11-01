# Phineas behavior on Ferb's simulator

`ros2 launch bringup phineas_on_ferb_sim.launch.py webots:=headless [targets:=yellow] [avoid:=red]`

Runs Ferb's Webots world and bridge (`recolector_sim/webots_bridge`) with our `phineas_cube_detector -> phineas_behavior -> phineas_safety_gate` instead of Ferb's `cube_detector`, `behavior_fsm` and `obstacle_guard`. The web_viewer is kept. Nodes run un-namespaced so their topics match the bridge (`image_raw`, `range`, `cmd_vel`). Parameters: calibrated values of `mobile_base/config/phineas.yaml` plus `bringup/config/phineas_on_ferb_sim.yaml` (overrides).

Docker: `docker compose run --rm -e SKIP_BUILD=0 dev bash -c 'source /opt/ros/lyrical/setup.bash; source /ws/install/setup.bash; ros2 launch bringup phineas_on_ferb_sim.launch.py webots:=headless'`

## Why our cube detector, not Ferb's
Both publish `cubes`. Only ours publishes `cubes/scene`, which `phineas_behavior` subscribes to (scoop content, blue square). It reads the bridge's `image_raw` through a remap (`camera/image_raw` -> `image_raw`).

## Relaxed parameters (safety_gate)
`require_pose: false` and `require_heartbeat: false`: the sim has no overhead pose, no `/zone/occupant`, no peer pose. These are already the defaults of `phineas.yaml` for the real robots; the YAML only makes it explicit.

## Observed (2026-11-01, Docker recolector:sim, headless, no GPU)
- Webots, bridge, web_viewer, cube_detector, behavior and safety_gate all stay alive (`ros2 node list`).
- `cubes` and `cubes/scene` flow at about 1 Hz (software rendering; camera frames come at ~3-10 Hz). `behavior` leaves IDLE and publishes `state` = EXPLORE.
- `cmd_vel_nav` at 10 Hz (0.12, 0, 0) and `cmd_vel` at 20 Hz through the gate (`safety: ok`).
- The robot moves: camera frames saved at t=10 s and t=40 s show the table edge approaching and then only floor; `cmd_vel` stays 0.12 m/s forward. **It drives off the table in about 30 s.**
- No target cube was ever seen, nothing approached or captured. The only detections are two orange boxes at fixed pixels: they are the robot's own scoop walls (visible in every frame), reported as cubes by our detector. They would be chased if `targets` contained orange.

## Safety finding (resolved)
The gate used to default to `line_front = False` ("no tape") when `line` was silent, so a dead line sensor node let the robot drive over the border (in the sim, off the table in ~30 s). Now the gate fails closed: it records the arrival time of the last `LineSensors` message and, if none arrived within `line_timeout_s` (default 0.5 s, 25 periods of the 50 Hz sensor; never heard = stale), it treats the line as tape under the sensor and logs `safety: line_stale: line ahead`. Forward and other translations toward the assumed tape are blocked; spinning in place is still allowed. `require_line: false` opts out (explicit, WARN at startup); `phineas_on_ferb_sim.yaml` sets it because Ferb's bridge publishes no `line`. `behavior` is unchanged: the gate is the safety layer.

## Limits
- No line (the gate runs with `require_line: false`), pose, zone arbiter or peer: tape backoff, geofence and delivery by pose are not exercised.
- Cubes/scene/line adapters (`core/adapters.py`) run for real for `cubes` and `cubes/scene`; `line` is untested in a loop.
- Hue ranges are ours (real cubes), not Ferb's sim-calibrated `hsv_ranges.yaml`; whether yellow/other cubes are detected in the sim is not verified (none in view).
