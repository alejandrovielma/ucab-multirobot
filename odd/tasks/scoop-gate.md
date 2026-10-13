# Feature: scoop-gate

## Objective
A servo gate at the mouth of the U retains the cube while carrying, so the robot can spin in place and
reverse with a cube. Removes the arc-only constraint that, with the body rule, cut deliveries to 0-1
per 10 min (feature body-geofence, Webots runs 11-18). User decision 2026-10-12, simulate first.

## Design
- Top-hinged flap in front of the cube (scoop frame x 0.082, hinge at the top of the U walls, axis y).
  Closed (0 rad): hangs down, bottom 7.5 mm above the floor. Open (-pi/2): horizontal forward at 3.75 cm,
  above a 3 cm cube entering. Motor "servo_compuerta": SG90-like limits (0.18 N m, 10.5 rad/s).
- Open while exploring and capturing; close after the camera confirms the capture; open at the drop
  point, reverse, stay open for the next capture.
- Behavior publishes gate_cmd (std_msgs/Bool, true = closed) and RobotState.gate_closed; the safety gate
  relaxes the carry rules (arcs only, no reverse) only while gate_closed.
- Planner gains in-place spin and reverse primitives when the gate is closed.
- has_gate=false keeps the current behavior (fixed scoop).

## Tasks
- [x] T1 Webots: flap + motor in the proto, driver subscribes gate_cmd. Check in calibracion_pala.wbt that
      the closed gate keeps the cube during a spin and a reverse, and re-measure the scoop window.
- [x] T2 Interfaces + behavior (gate sequencing, planner primitives) + safety (relaxed carry rules) + tests.
- [x] T3 Webots runs: deliveries vs runs 11-18; cube inside the square in delivery frames.
- [ ] T4 Real robot servo node (lgpio) and docs (README, architecture, team-todo, technical document).

## Checks
Docker suite; kinematic harness; Webots runs with tools/sim_monitor.py --frames-on-delivery.

## Progress / evidence
- e1e4116 plan; f5c1b2e..: proto + driver; side door replaced the top flap (the open flap hid the cube at
  the mouth from the camera: no detection); adc1296 behavior/safety/planner; 5a322e7 penetration-based
  geofence + precise entry; 482cf82 lease holder not turned away.
- calibracion_pala.wbt: door closed -> cube kept through a 5 s spin and a 12 cm reverse (scoop window
  0.58); door open + reverse -> cube left behind (0.00). Open door is not visible to the camera.
- Planner with the gate: 100 % of 164 test poses reach the precise entry (median 91 ms, max 0.5 s).
- Docker after 5a322e7: 207 tests, 0 failures.
- Webots run 19: 2 deliveries in 1 min, then stuck (protruding track corner, imprecise entry).
  Run 20: 0 (livelock between the lease holder and the waiting robot).
  Run 21 (9.3 min): 3 deliveries (Ferb 2, Phineas 1), all three cubes inside the square in the
  delivery frames; body beyond the tape inner edge at most 1.2 cm in 12/279 and 7/278 samples
  (pose noise 5 mm). Without the gate (runs 11-18): 0-1 per 10 min.
- Native review: not run.

## Open
- User clarification 2026-10-13: the user meant side walls that open to release the cube. Releasing
  already works with the fixed U (reversing leaves the cube); the front door retains it to allow spins.
  Decision pending. T4 (real servo node, docs) not started.
