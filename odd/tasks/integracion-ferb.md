# integracion-ferb

Objective: ferb/develop (Ferb-Recolector) is the base; keep what Phineas adds.
Branch: integracion-ferb (cut from ferb/develop). main untouched.
Route: inline (mechanical imports, no new design).

## Tasks
- [x] T1 Import non-colliding trees from main: coordinator, overhead_localization, multirobot_sim, hardware, tools/lab, tools/sim, odd, figuras, docs -> docs/phineas (commit e932ea5)
- [x] T2 Merge mobile_base core/nodes beside Ferb's flat modules; entry points prefixed `phineas_` (427 tests pass in venv)
- [x] T3 Phineas launch files imported as phineas_*.launch.py; executables mapped to phineas_*; launch test passes
- [x] T4 Cube detections on recolector_interfaces CubeDetectionArray (topic cubes) plus new Scene (cubes/scene); core.adapters converts (commit 46417f4)
- [x] T5 Line sensor on recolector_interfaces LineSensors (topic line) (commit d0cc3a6)
- [x] T6 Stall fixes ported into Ferb's FSM (missed-catch ignore-list, stuck-line watchdog) with tests. Deferred: geofence, planner/cartographer (need a pose source). Skipped: carry-delivery (Ferb's zone/WAIT_ZONE/blind push covers it). Optional: camera capture verification (Ferb has FC-51 and TCS34725). Commit: 8ae148f (A and B together)
- [x] T7 Ferb's AO analog reading + BlackConfirm vendored in tools/lab/analog_line.py (opt-in); motor_hardware skipped (ours is a superset); discrepancies in docs/phineas/integracion-hardware.md. Commit: bf39dce

- [x] T8 Phineas behavior chain on Ferb's simulator: `phineas_on_ferb_sim.launch.py` (see docs/phineas/integracion-simulacion.md). Result: chain runs and moves, but without a `line` source the robot drives off the table in ~30 s; no cube approached

- [x] T9 Gate fails closed when `line` is silent: `line_timeout_s` (0.5 s) + `require_line` (true; false in the Ferb sim yaml), reason `line_stale: line ahead`, `line_is_fresh` helper with tests. Verified in Docker sim in both modes (see docs/phineas/integracion-simulacion.md). `behavior_node` unchanged.
## Not verified here (Windows)
tools/lab tests need termios (Linux); tools/sim needs rclpy. Run scripts/test_in_docker.sh.

## Not verified (T4/T5)
rclpy/colcon missing on Windows: nodes only py_compile'd; Scene.msg not built. recolector_driver.py (Webots) now publishes LineSensors on `line` (untested: needs Webots).

## CI lint (ruff)
Ferb's CI runs `ruff check` and `ruff format --check` on src/tools/scripts (0 errors on their base; 2977 errors and 79 files to reformat once our code was imported). Our legacy code is listed in `extend-exclude` in pyproject.toml with a comment; clean it module by module and delete its line. Files of Ferb that we modified (collector.py, behavior_fsm.py, setup.py, test_collector.py) comply with ruff. Check: `ruff check src tools scripts simulation/webots/controllers` and `ruff format --check ...` both pass.

## Docker verification (2026-11-01, recolector:sim, ROS Lyrical)
- colcon build: 10 packages ok (Scene.msg and merged setup.py files build).
- colcon test: 529 tests, 0 failures.
- import smoke test: 82 modules ok. Launch smoke: phineas_robot.launch.py robot:=phineas and robot:=ferb, 6 processes stayed alive each.
- tools tests: 202 passed, 1 failed: tools/test_teleop_web.py::test_each_black_edge_of_the_line_sensor_is_logged_with_its_analog_value. It also fails on an untouched ferb/develop (checked), so it predates the integration; owner: Ferb.
- Not covered: Webots simulation run, real robot (Pi).

## Simulation (2026-11-01, Docker recolector:sim)
- Our `phineas_sim.launch.py` cannot run here: it needs `webots_ros2_driver`, which has no apt package for Lyrical (only webots-ros2-importer and -msgs exist).
- Ferb's `simulation.launch.py webots:=headless` (own bridge, no webots_ros2) runs in the integrated tree once `libsndio7.0` is installed: Webots starts, `cubes` at ~6.6 Hz, behavior_fsm in SEARCH. Without that library `webots-bin` exits 127 (image gap on Ubuntu 26.04; not fixed in docker/Dockerfile yet).
- Ferb's bridge does not simulate the line sensor, so `line` is never published there.
- Not covered: our adapters, `phineas_*` nodes and Scene in a simulated run.
