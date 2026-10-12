# Feature: phineas-repo-base

## Objective

Create a modular ROS 2 Jazzy monorepo for the mobile collector robots (Phineas and Ferb share code) with a
first end-to-end autonomous flow runnable in Webots: nodes start, read simulated sensors, and the robot
explores the track without leaving it, approaches cubes of its own colors, and keeps distance from the other
robot using overhead-camera poses.

## Problem and why

The team has a design document but zero code. The hardware is limited (Pi 5, IMX219 camera, 4 TT motors on a
mecanum chassis, Cokoino HAT, ONE front line sensor, 3D-printed fixed scoop). Collision avoidance relies on an
overhead camera (Perry's or a dedicated one) that publishes robot poses. The team needs a structure where the
same decision nodes run in Webots and on the real robot, swapping only the hardware layer.

## Scope (authorized)

- In: repository structure, interfaces package, pure-logic core with unit tests, ROS 2 nodes, Webots
  simulation package (world, protos, URDF, driver plugin, launch), bringup launch and YAML config, Docker
  test environment, docs.
- Out (left for the team, marked TODO with acceptance criteria): transport-to-square and delivery behavior,
  Perry's arm, real overhead ArUco localizer implementation details beyond a skeleton, real camera capture on
  Ubuntu (CSI/libcamera).

## Constraints

- ROS 2 Jazzy, Python 3.12, rclpy. Code, identifiers and comments in English.
- Pi 5 GPIO only through lgpio. HAT pins (BCM): nSLEEP 12, 13 held HIGH; DRV#1 AIN1 17, AIN2 27 (M1 front
  left), BIN1 22, BIN2 23 (M2 rear left); DRV#2 AIN1 24, AIN2 25 (M3 front right), BIN1 26, BIN2 16 (M4 rear
  right). PWM on IN pins (fast decay: PWM on xIN1, xIN2 low = forward). Duty cap = 6 V / V_bat.
- Front line sensor on GPIO5 (digital, active when black).
- Fixed scoop: no servo command; never reverse while carrying a cube.
- No hardware available in this environment: Webots behavior is NOT verified here; only colcon build and
  pytest inside Docker `ros:jazzy-ros-base`.

## Architecture contract

Hardware boundary per robot (namespace `/<robot>`, robot in {phineas, ferb}):

| Topic | Type | Producer (sim / real) | Consumer |
|---|---|---|---|
| `camera/image_raw` | sensor_msgs/Image | Webots plugin / camera_node | cube_detector |
| `line_front` | std_msgs/Bool (true = black line) | Webots plugin / line_sensor_node | safety_gate, behavior |
| `pose` | multirobot_interfaces/RobotPose | Webots plugin (GPS+InertialUnit, emulates overhead camera) / overhead_localization | behavior, safety_gate of both robots |
| `cmd_vel` | geometry_msgs/Twist | safety_gate | Webots plugin / hat_motor_driver |
| `cmd_vel_nav` | geometry_msgs/Twist | behavior | safety_gate |
| `cubes` | multirobot_interfaces/CubeDetectionArray | cube_detector | behavior |
| `state` | multirobot_interfaces/RobotState | behavior | coordinator, other robot |
| `/zone/occupant` | std_msgs/String (2 Hz heartbeat, transient local) | coordinator | behavior, safety_gate |
| `/zone/request`, `/zone/release` | services | coordinator | behavior |

## Tasks

- [x] T1 Scaffold: repo layout, `multirobot_interfaces` (msg/srv), Docker test env, `.gitignore`.
  Route: delegated (writer trigger: 2+ non-trivial files).
- [x] T2 Pure core + tests (RED then GREEN): mecanum kinematics, safety gate, color classifier, zone lease
  arbiter, behavior FSM. Route: delegated.
- [x] T3 ROS nodes, launch and config: mobile_base nodes, coordinator node, cube_detector, bringup.
  Route: delegated.
- [x] T4 Webots package: world (from group model, controllers `<extern>`, front line sensor and GPS/IMU via
  `extensiones`), URDF + plugin driver, sim launch. Route: delegated.
- [x] T5 Docs: README (build, test, run sim on Windows + WSL2, run real), docs/architecture.md,
  docs/team-todo.md. Route: delegated.

## Acceptance criteria

- `colcon build` succeeds in `ros:jazzy-ros-base` for all packages.
- `colcon test` (pytest) passes for all pure-logic modules.
- Every node file is importable (`python3 -c "import ..."`) inside the container without hardware libs.
- Webots launch file and world exist and reference consistent robot names, devices and plugin paths
  (structural check only; runtime unverified).

## Checks

```bash
docker run --rm -v <repo>:/ws -w /ws ros:jazzy-ros-base bash -lc "./scripts/test_in_docker.sh"
```

## Delivery

Strategy: `exception-ok` (local repository without remote; the user decides push/PR later).
Forecast: ~2500 authored lines (scaffold, generated-like packaging and tests).

## Progress and evidence

- Base commit on main: 312b26e (design document).
- T1 b0acf44, T2 76d9c86, T3 ce4b854, T4 6dceffe, T5 8e3bab1 (delegated writer).
- RED observed per package (collection ImportError) before GREEN; details in writer report.
- Verification (parent spot check, `scripts/test_in_docker.ps1 -NoBuild`): 6 packages built, 86 tests, 0 failures; smoke 33/33; robot.launch.py phineas and ferb keep 6 processes alive.
- Not verified: Webots runtime (plugin loading, wheel signs, /clock with two controllers), lgpio API names on the Pi, TCRT5000 active level.
- Review: risk high, consent granted, START refused `lens_context_budget_exceeded` (96 files, 4223 lines); per-commit split unassessable. User chose to continue without this review (2026-10-12); next smaller candidates will be reviewed.

- Follow-up after Webots teleop tests (user, 2026-10-12): strafe works without turning; an in-place spin ejected the cube at ~45 deg even at speed 0.36.
  - 14884c8 measured 3 cm cubes, scoop U widened to 40 mm.
  - b6891b7 carry rules in safety_gate (arc turns only, no strafe, hold still on deadlock) + U depth 48 mm. RED 4 failures -> GREEN 91 tests, 0 failures.
  - Review assess (base 4dde89f): medium, under_budget (148 lines), pending in slice.
  - Validated by user in Webots teleop (2026-10-12): arc turn (W+Q) keeps the cube in the 48 mm pocket; in-place spin still ejects it, as the carry rule assumes.
  - Webots world saved accidentally from Webots: stashed as `stash@{0}`, not committed.

## Next step

Team runs `ros2 launch bringup sim.launch.py` in WSL2 with Webots R2025a, validates wheel signs (docs/team-todo.md item 2), then implements CARRY/delivery (item 1).
