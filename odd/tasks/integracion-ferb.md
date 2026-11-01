# integracion-ferb

Objective: ferb/develop (Ferb-Recolector) is the base; keep what Phineas adds.
Branch: integracion-ferb (cut from ferb/develop). main untouched.
Route: inline (mechanical imports, no new design).

## Tasks
- [x] T1 Import non-colliding trees from main: coordinator, overhead_localization, multirobot_sim, hardware, tools/lab, tools/sim, odd, figuras, docs -> docs/phineas (commit e932ea5)
- [x] T2 Merge mobile_base core/nodes beside Ferb's flat modules; entry points prefixed `phineas_` (427 tests pass in venv)
- [x] T3 Phineas launch files imported as phineas_*.launch.py; executables mapped to phineas_*; launch test passes
- [ ] T4 Unify CubeDetection (propose Ferb's recolector_interfaces: bearing_rad/distance_m)
- [ ] T5 Unify line sensor topic (propose Ferb's LineSensors on `line`)
- [ ] T6 Port geofence, planner, stall fixes, delivery into Ferb's FSM (collector/approach) with tests
- [ ] T7 Adopt Ferb's hardware path (AO analog sensor, motor_hardware) in tools/lab

## Not verified here (Windows)
tools/lab tests need termios (Linux); tools/sim needs rclpy. Run scripts/test_in_docker.sh.
