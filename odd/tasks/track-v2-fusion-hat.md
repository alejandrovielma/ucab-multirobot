# Feature: track-v2-fusion-hat

## Objective
Bring the repository to the real robot and track as measured on 2026-10-15: SunFounder Fusion HAT+
motor board, 2S 7.4 V Li-ion battery, rectangular track with a blue delivery square, 2 x 2 cm cubes.
Code, simulation (no Webots runs: user request to save tokens) and documentation.

## Facts (user, photos 2026-10-15; vendor docs)
- Motor board: SunFounder Fusion HAT+ (not Cokoino). MCU on I2C 0x17 drives 4 motor ports (M0-M3,
  XH2.54) through PWM4-11; Python library `fusion_hat` (`Motor('M0').power(-100..100)`,
  `set_is_reverse`); battery input 6.0-8.4 V powers Pi + HAT; GPIO5 is not used by the HAT.
  Sources: docs.sunfounder.com/projects/fusion-hat (hardware/interfaces, api/fusion_hat.motor).
  Vendor installer targets Raspberry Pi OS; on Ubuntu the kernel driver build needs kernel headers
  (forum.sunfounder.com reports).
- Battery: Li-ion 7.4 V 2000 mAh (2S), 3-wire XH connector (full charge 8.4 V).
- Track (outer tape edges, tape 1.8 cm wide): rectangle 120.6 x 74 cm; blue square sticking out of the
  west side, 20.3 x 20.3 cm outer, 26 cm from the north corner and 27.8 cm from the south corner; its
  side toward the track has no tape (open). Only the scoop may enter the square (Perry works there).
- Camera: mounted at the front of the top plate, looking roughly horizontal (Webots assumes 24 deg down
  at 11 cm). Height, setback and tilt not measured yet: Webots camera left unchanged, documented.

## Geometry (world frame: origin at the rectangle center, x along the 120.6 cm side toward the east /
## right of the photo, y toward the north / far side of the photo; meters)
- Rectangle outer x +-0.603, y +-0.370; inner (body limit) x +-0.585, y +-0.352.
- Square outer x -0.806..-0.603, y -0.093..0.110; blue interior x -0.788..-0.585, y -0.075..0.092
  (assumed: blue fills up to the rectangle's inner tape line), center (-0.6865, 0.0085).
- Delivery heading west (pi). Body front stops at x -0.585 + 0.003 -> robot center -0.487; cube (2 cm,
  against the U back wall, 0.1395 m ahead) at x -0.6265, 4 cm inside the blue.

## Tasks
- [x] T0 Commit pending 2 x 2 cm cube / blue square changes. (route: inline)
- [x] T1 Generalized DeliveryZone (any heading) + new zone/track/wait-point config + tests.
      (route: delegated writer; commit a77cf0b)
- [x] T2 Blue square detection in vision (not a cube) + square fields in the message + behavior
      alignment while entering + tests. (route: delegated writer; commit da0148b)
- [x] T3 Fusion HAT+ motor backend: ROS node and lab tool, battery 8.4 V cap + tests.
      (route: delegated writer; commit 1e10d54)
- [x] T4 Webots world: new track texture/board/cubes/marker/start poses generated from one geometry
      source (no Webots run). (route: delegated writer; commit 978ecb8)
- [x] T5 Docs: README, architecture, team-todo, lab guide (Ubuntu + Fusion HAT+ driver), lab README.
      (route: delegated writer; commit 9c447ac)
- [x] T6 Checks: full suite (WSL ROS Jazzy, Docker daemon not running), tools/lab tests.

## Progress / evidence

Route: one delegated writer for T1-T5 (2+ non-trivial files per task: writer trigger).

- T1 (a77cf0b): core/delivery.py defines the zone by square center (-0.6865, 0.0085), heading pi,
  square_depth 0.203 (boundary x -0.585) and offsets before the boundary: entry 0.298 (x -0.287),
  drop 0.098 (x -0.487), exit 0.205 (x -0.38), corridor half width 0.08, cube_ahead 0.1395.
  Parameters renamed: zone_x, zone_y, zone_heading, zone_square_depth, zone_entry_offset,
  zone_drop_offset, zone_exit_offset, zone_corridor_half_width; gate_entry_x_tolerance ->
  gate_entry_across_tolerance, gate_entry_y_band -> gate_entry_along_band. Wait points phineas
  (0, 0.22), ferb (0, -0.22). track.yaml = inner rectangle x +-0.585, y +-0.352 (4 vertices).
  Tests ported (synthetic south zones keep their old values); new zone-math tests for the west
  approach and a round trip for 5 headings. RED observed only on the ported behavior tests (old API);
  the new zone-math tests were written together with the module. Unit suite: 191 passed.
- T2 (da0148b): vision.detect_square (largest blue region >= square_min_area 12000 px) and
  detect_cubes drops blue blobs that are the square; CubeDetectionArray square_visible/cx/cy/area;
  behavior adds -square_align_gain (0.8) * square_cx while entering, bounded by carry_max_turn.
  RED observed (5 failing tests), then GREEN: unit suite 196 passed.
- T3 (1e10d54): core/fusion_hat.py (wiring_from, duty_to_power, FusionMotorBank, lazy
  load_motor_class); hat_motor_driver motor_backend fusion_hat (default) / cokoino_drv8833,
  fusion_ports [M0, M1, M2, M3], fusion_reversed all false, battery 8.4 (cap 0.71). Lab tool uses the
  same backend, prints the port per wheel; Fusion HAT+ needs no Pi GPIO for motors. RED observed for
  core tests (collection error), lab tests written after the change. Unit 210 passed, lab 9 passed;
  `lab_robot.py motors --dry-run` prints "rueda fl (puerto M0)".
- T4 (978ecb8): tools/sim/generate_track.py renders pista_ucab.png 1820 x 940 px (1000 px/m) for a
  1.82 x 0.94 m board (margin assumed) and a deterministic 14-cube layout (seed 2026); worlds updated
  (board, table 1.8 x 0.92, marker id 10 at (-0.84, 0.0085) facing east, starts (0.40, +-0.20, pi)).
  Webots NOT run. New structural tests in multirobot_sim/test.
- T5 (9c447ac): Spanish docs updated; team-todo tasks 14-18 added (18 = delivery stalls before the
  drop point, see below).
- T6: scripts/test_in_docker.sh run inside WSL Ubuntu-24.04 with ROS 2 Jazzy (Docker Desktop daemon
  was not running): "Summary: 242 tests, 0 errors, 0 failures, 0 skipped", "smoke test: 37 ok, 0
  failed", both launch smoke tests "6 processes started and stayed alive", "== all checks passed".
  tools/lab: 9 passed. Docker run itself: not executed (daemon unavailable).

Finding (pre-existing, not fixed): a kinematic run of behavior + safety gate on the track polygon
stalls in DELIVER 1.3 cm before the drop point (the gate's 0.3 s geofence prediction blocks forward
motion when the front is ~1 cm from the boundary, the drop leaves 3 mm). Reproduced with the previous
geometry too (stopped at y -0.053 vs drop -0.065). Recorded as team-todo task 18.

## Next step
Engram mirror pending (writer had no mirror step). Decide task 18; run Webots with the new world;
lab: Fusion HAT+ driver on Ubuntu Server 26.04 and wheel -> port mapping.

## Parent verification and follow-up (2026-10-15)
- Native review of f7eb90b..a8823ba (cubes 2 x 2 cm + plan): approved by the reliability lens and
  acknowledged (lineage review-bcff5bed2f74d25e); 3 non-blocking findings (2 cm cube retention in the
  40 mm U unproved, detection thresholds not recalibrated, geometry assumptions) tracked in team-todo.
- Parent spot check: src/mobile_base/test 210 passed; tools/lab 9 passed.
- 07dd24e fix: drop_tolerance 0.02 (team-todo #18). Integration test behavior + gate on the real track
  polygon completes entry -> drop -> release; with drop_tolerance 0 it stalls at x -0.477 (RED observed).
- Full suite in WSL with ROS 2 Jazzy (Docker Desktop daemon down): 244 tests, 0 errors, 0 failures;
  smoke 37/37; launch smoke both robots OK.
- Generated texture checked visually against the user's track photo.
