# Task: talk to Phineas and Perry (fleet link)

Repo: `notoriussss/Ferb-Recolector`, branch `develop`. Work on a new branch `feature/behavior-fleet-link`.
Commit locally with messages `area: what`. **Do not push, do not touch `main`.**
Everything in English (identifiers, comments, logs, docs).

Samuel explicitly asked for this implementation, so the "team writes the code" rule in `AGENTS.md`
does not block it. Keep the repo rules: pure logic without `rclpy` plus tests, every physical value
in YAML with its unit in the name, and velocity always goes through `obstacle_guard`.

## Context

Our robot is **Ferb**. The other collector is **Phineas**, from the repo
https://github.com/alejandrovielma/ucab-multirobot (reference commit `09d0f16`, 2026-10-23).
That repo also contains **Perry**'s nodes. Perry is the arm, and its Pi runs `zone_arbiter` and the
overhead ArUco localizer. Phineas and Perry are already implemented, so **we adopt their interface
as it is**. It replaces the `/fleet/status` and `/fleet/delivery_lock` proposal, which nobody
implemented.

Their contract (read these files in their repo before starting):
- `src/multirobot_interfaces/` holds msg `RobotState`, `RobotPose`, `CubeDetection` and
  `CubeDetectionArray`, plus srv `RequestZone` and `ReleaseZone`.
- `src/coordinator/coordinator/zone_arbiter_node.py` and `core/zone_lease.py` define the services
  `/zone/request` and `/zone/release`. `/zone/occupant` is a `std_msgs/String` heartbeat at 2 Hz;
  an empty string means the square is free. Its QoS is RELIABLE, TRANSIENT_LOCAL, depth 1. The lease
  is 60 s, and a new request from the current holder renews it. The release with `delivered=true`
  increments the official count, which is returned in `delivered_total`.
- `src/mobile_base/mobile_base/core/safety.py` holds the peer avoidance rules (see step 5).
- Each robot runs under namespace `/phineas` or `/ferb`. Perry publishes `/phineas/pose` and
  `/ferb/pose` (`RobotPose`, table frame, meters). Phineas publishes `/phineas/state` (`RobotState`).
- Rule since 2026-10-22: both robots collect **every colour except red**. Red cubes stay on the track.

## Steps

1. **Interfaces.** Copy their `src/multirobot_interfaces/` package as `src/multirobot_interfaces/`
   with the same package name and **byte-identical** `.msg`/`.srv` files, `CMakeLists.txt` and
   `package.xml`. DDS type matching depends on this. Add a `README.md` inside the package that names
   the source repo and commit, and says "do not edit, update from upstream". Keep our
   `recolector_interfaces` for our internal messages. Add the `multirobot_interfaces` dependency to
   `mobile_base/package.xml`.

2. **Namespace.** In `src/bringup/launch/collector.launch.py`, add the launch arg `robot`
   (default `ferb`) and run every node with `namespace=robot`. Check that every topic in our nodes is
   relative: `cmd_vel_raw`, `cmd_vel`, `cubes`, `range`, `behavior/state`, `cmd_vel_teleop` and
   `camera/...`. Check that the web viewer still works under the namespace. The only absolute names
   are `/zone/*` and the peer pose topic.

3. **Pure logic: `src/mobile_base/mobile_base/fleet.py`** (no rclpy, time passed in as a float).
   *Implementation note (ADR 0005): `PeerAvoidance` and `Pose2D` live in `peer_avoidance.py`, and its
   distance fields are `peer_slow_distance_m` / `peer_stop_distance_m`.*
   - `to_fleet_state(local: str) -> str` maps our states to their vocabulary:
     `SEARCH→EXPLORE`, `APPROACH→APPROACH`, `ARRIVED`/`CATCH→CAPTURE`, `CARRY→CARRY`,
     `RELEASE→DELIVER`. Anything else (`PAUSED`, `MANUAL`, `NO_CAMERA`...) maps to `STOP`.
   - `Heartbeat(timeout_s)`: `beat(now)` and `ok(now)`. It is not ok before the first beat.
   - `ZoneClient(robot, renew_period_s)`: input `want: bool` plus the latest occupant string.
     `holds` is true when the occupant equals robot. `should_request(now)` is true when `want` is
     true and either we don't hold the zone or `renew_period_s` has passed since the last request.
     `should_release()` is true on the falling edge of `want` while holding. It remembers whether
     the release is `delivered` and which `color` (set by `mark_delivered(color)`).
   - `PeerAvoidance(params)`: inputs are own pose, peer pose (x, y, theta plus stamp) and now.
     Ignore poses older than `pose_max_age_s` (0.5). Mirror their numbers: a cone of ±60° ahead,
     `slow_distance_m` 0.60 (scale forward speed and turn away at `avoid_turn_rad_s` 0.6),
     `stop_distance_m` 0.40 (no forward motion). If blocked for longer than `deadlock_time_s` (3.0),
     **Ferb is the robot that yields**: rotate in place at `escape_turn_rad_s` 0.8, away from the
     peer, until it is out of the cone. The output is the filtered `(vx, vy, wz)` plus a reason
     string. With no fresh own pose or peer pose, it returns the command unchanged (the ultrasonic
     still protects us).

4. **Node `fleet_link`** (`src/mobile_base/mobile_base/fleet_link.py`, plus an entry point in `setup.py`):
   - Subscribes to `behavior/state` (String), `scoop/has_cube` (Bool, optional, carrying=false if
     absent), local `zone/want` (Bool), local `zone/delivered` (String colour), and `/zone/occupant`
     (String, with the QoS above).
   - Publishes `state` (`multirobot_interfaces/RobotState`, robot=`ferb`, at `state_rate_hz` 2.0;
     `delivered` comes from the last `delivered_total`, `gate_closed` is always false because our
     scoop has no gate). Also publishes `fleet/ok` (Bool, true while the heartbeat is fresh, at
     5 Hz) and `zone/granted` (Bool).
   - Calls `/zone/request` and `/zone/release` **asynchronously** and never blocks the executor.
     If a service is missing, it logs a warning at most once every 5 s.
   - Parameters: `robot_name` (ferb), `heartbeat_timeout_s` (2.0), `state_rate_hz`, and
     `renew_period_s` (20.0).
   - Add it to `collector.launch.py`.

5. **`obstacle_guard`.**
   - Subscribe to `fleet/ok`. If `require_heartbeat` (param, default true) is set and the heartbeat
     is not ok, publish zero. This matches their rule: no arbiter heartbeat for 2 s means stop.
   - Subscribe to `pose` and to `peer_pose_topic` (param, default `/phineas/pose`), then apply
     `PeerAvoidance` after the ultrasonic limits. `require_pose` (param, default **false**) is there
     because we have line sensors and pose is extra.
   - Add launch args `require_heartbeat` and `require_pose` to `collector.launch.py` for bench tests.

6. **`behavior_fsm`.** Delivery is not implemented yet, so only add the hooks: publishers
   `zone/want` and `zone/delivered`, which stay unused for now. Also add a docstring TODO: request
   the zone before crossing into the square, enter only when `zone/granted` is true, publish the
   colour on `zone/delivered` after the drop, and then `zone/want=false`. Do not write CATCH, CARRY
   or RELEASE.

7. **Config** (`src/mobile_base/config/collector.yaml`): add a `fleet_link:` section and the new
   `obstacle_guard` params. Change `behavior_fsm.target_colors` to
   `[orange, yellow, green, blue, purple]` and comment it as "every colour except red, track rule of
   2026-10-22".

8. **Tests:** add `src/mobile_base/test/test_fleet.py` covering the state mapping, the heartbeat
   timeout, request/renew/release, deadlock escape and stale poses. Use plain pytest with no rclpy.
   Run `pytest src/mobile_base/test src/vision/test tools` and
   `ruff check src tools && ruff format --check src tools`. If Docker is available, also run
   `colcon build` there.

9. **Docs:**
   - In `docs/external-interface.md`, fill in "Agreed" with today's date. Source: their repo
     `09d0f16`, adopted by us, pending confirmation with their team. Answer the table rows:
     - Network: same Wi-Fi, same `ROS_DOMAIN_ID` (value to agree), `chrony` between the 3 Pis, and
       `ROS_STATIC_PEERS` if discovery fails.
     - Colours: everything except red.
     - Square: lease via `/zone/*`.
     - Count: Perry's arbiter.
     - Stop: no heartbeat means stop.
     - Markers: ArUco id 1 Phineas, id 2 Ferb, id 10 square.
   - In `docs/architecture.md`, update the topic table and the node diagram.
   - In `docs/collector-plan.md`, replace the `/fleet/*` proposal with this.
   - Add ADR `docs/decisions/0004-adopt-multirobot-interface.md`. It explains that we vendor only
     their interface package, not their code, so it is consistent with ADR 0003.
   - Platform: Ferb's Pi now runs **Ubuntu 26.04**, the same as Phineas and Perry, so we move to
     **ROS 2 Lyrical** (Jazzy has no packages for 26.04). Update `AGENTS.md`, `README.md`,
     `docs/decisions/0001-*`, the Dockerfile and the CI to Lyrical. Write a field test note for the
     first run from our Pi: `ros2 topic echo /zone/occupant` and
     `ros2 service call /zone/request multirobot_interfaces/srv/RequestZone "{robot: ferb}"`.

## Not in scope

`/cubes/claim`, `/world/cubes` and `/<robot>/mission`. These are planned in their
`docs/plan-integracion-hibrida.md` but not implemented yet. Note them in `external-interface.md`
as "future". Also out of scope: CATCH, CARRY, RELEASE, the motor driver and copying any of their
non-interface code.

## Report back

Report the branch name, the commit list, the test and ruff output, and anything that could not be
done or that you had to decide yourself.
