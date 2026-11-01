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
- [x] T7 Ferb's AO analog reading + BlackConfirm vendored in tools/lab/analog_line.py (opt-in); motor_hardware skipped (ours is a superset); discrepancies in docs/phineas/integracion-hardware.md. Commit: see git log (feat(lab): ...)

## Not verified here (Windows)
tools/lab tests need termios (Linux); tools/sim needs rclpy. Run scripts/test_in_docker.sh.

## Not verified (T4/T5)
rclpy/colcon missing on Windows: nodes only py_compile'd; Scene.msg not built. recolector_driver.py (Webots) now publishes LineSensors on `line` (untested: needs Webots).
