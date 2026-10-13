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
- [ ] T1 Webots: flap + motor in the proto, driver subscribes gate_cmd. Check in calibracion_pala.wbt that
      the closed gate keeps the cube during a spin and a reverse, and re-measure the scoop window.
- [ ] T2 Interfaces + behavior (gate sequencing, planner primitives) + safety (relaxed carry rules) + tests.
- [ ] T3 Webots runs: deliveries vs runs 11-18; cube inside the square in delivery frames.
- [ ] T4 Real robot servo node (lgpio) and docs (README, architecture, team-todo, technical document).

## Checks
Docker suite; kinematic harness; Webots runs with tools/sim_monitor.py --frames-on-delivery.

## Progress / evidence
(pending)
