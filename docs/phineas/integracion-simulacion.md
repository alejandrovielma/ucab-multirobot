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
The gate used to default to `line_front = False` ("no tape") when `line` was silent, so a dead line sensor node let the robot drive over the border (in the sim, off the table in ~30 s). Now the gate fails closed: it records the arrival time of the last `LineSensors` message and, if none arrived within `line_timeout_s` (default 0.5 s, 25 periods of the 50 Hz sensor; never heard = stale), it treats the line as tape under the sensor and logs `safety: line_stale: line ahead`. Forward and other translations toward the assumed tape are blocked; spinning in place is still allowed. `require_line: false` opts out (explicit, WARN at startup). The sim used to need it because Ferb's bridge published no `line`; it no longer does (see the next section). `behavior` is unchanged: the gate is the safety layer.

## Simulated line sensor (T10)
`webots_bridge` publishes `recolector_interfaces/LineSensors` on `line` (best effort, like `range`; 50 Hz asked, about 25-30 Hz observed in the headless sim because of the 16 ms step and software rendering).

- Feasibility: the world can tell the tape from the table. The proto's `linea_*` are infra-red `DistanceSensor`s (`lookupTable [0 1023 0.01, 0.025 0 0.01]`, 12 mm above the floor) and Webots modulates the reading by the colour of the surface, texture included (`pista_ucab.png` carries the tape): white table reads about 240 raw (235-246), black tape reads 0. Off the table the floor is 0.78 m away, out of range, also 0. So `black` goes true both on the tape and at the table edge, which is what the guard needs.
- Which sensor: the real robot has ONE TCRT5000 between the front wheels (`docs/hardware.md`). The proto has four sensors at the corners (x = +-0.075, y = +-0.035) and none at the centre line. The bridge uses the two front ones (`linea_fl`, `linea_fr`) and reports `black` if either is black: symmetric about the centre line, and 15 mm ahead of the front axle (the real one is under the axle line). It is therefore a bit more conservative than the real sensor. A true centre sensor would need a proto change.
- Threshold: `LINE_BLACK_BELOW_RAW = 120` raw units (the lookupTable's 0..1023 scale, no physical unit), halfway between the two observed levels. NaN (before the first valid reading) counts as black.
- Observed (Docker, headless, 130 s): `black` toggles at the border and always returns to white; the robot backs off at the tape instead of leaving the table (before it drove off in ~30 s); it reached APPROACH on cubes/boxes several times (EXPLORE -> APPROACH -> "target lost"/"orange cube in the way"), no capture seen.
- `phineas_on_ferb_sim.yaml` no longer sets `require_line: false`: the gate runs fail-closed in the sim too; `line_stale` appears only before the first message.

### Bug found on the way: QoS of `line`
`safety_gate_node` and `behavior_node` subscribed to `line` with the default reliable QoS, while Ferb's real `line_sensors` node (and now the bridge) publish best effort. ROS 2 does not connect a reliable subscriber to a best-effort publisher (`offering incompatible QoS ... RELIABILITY`), so the gate never saw a message: with `require_line: true` it reported `line_stale` forever, with `false` it never saw tape. This would have affected the real robot with Ferb's node too. Fixed by subscribing with `qos_profile_sensor_data` (accepts both).

## Limits
- No pose, zone arbiter or peer: geofence and delivery by pose are not exercised. The line guard is, with the simulated sensor below.
- Cubes/scene/line adapters (`core/adapters.py`) run for real in the loop.
- Hue ranges are ours (real cubes), not Ferb's sim-calibrated `hsv_ranges.yaml`; whether yellow/other cubes are detected in the sim is not verified (none in view).
