# Feature: scoop-shape (option B: passive scoop that keeps the cube while spinning)

## Objective
Find a scoop U shape, without any servo, that keeps the cube during in-place spins and still releases
it by reversing (user request 2026-10-13: avoid the complexity of the servo gate).

## Reasoning before the experiment
In an in-place spin the cube sits ~14.5 cm from the robot center. The side wall pushes it along a
straight tangent while the U keeps rotating; following the circle needs a force toward the robot
center, and the back wall can only push outward. The cube slides toward the mouth at any speed
(user teleop: ejected at ~45 deg even at 0.36 rad/s). A wedge (walls converging toward the mouth)
adds an inward component; a large wedge would block the release.

## Experiment (Webots, calibracion_pala.wbt variants, no gate)
tools/sim/scoop_shape_sweep.sh + tools/sim/scoop_shape_test.py: spin CCW 5 s at 1.2 rad/s, spin CW 5 s,
reverse 3 s; scoop window fill from the robot camera after each (0.60 = cube in the U).
The PROTO now has palaBoca, palaFondoAncho, palaProfundidad and compuerta fields.

| Shape | Mouth | Back | Depth | Start | After CCW spin | After CW | After reverse |
|---|---|---|---|---|---|---|---|
| B | 34 mm | 40 mm | 48 mm | 0.60 | 0.00 | 0.00 | 0.00 |
| C | 32 | 40 | 48 | 0.60 | 0.00 | 0.00 | 0.00 |
| D | 32 | 44 | 48 | 0.60 | 0.00 | 0.00 | 0.00 |
| E | 34 | 40 | 60 | 0.60 | 0.00 | 0.00 | 0.00 |
| F | 32 | 36 | 60 | 0.60 | 0.00 | 0.00 | 0.00 |
| G | 34 | 34 | 60 | 0.60 | 0.00 | 0.00 | 0.00 |
(A, the current 40/40/48 U, had no valid start frame in the final sweep; its loss of the cube on a spin
was already measured by the user with teleop and in the earlier gate test with the gate open.)
The first sweep was invalid: Webots on Windows was not restarted between worlds (fixed with taskkill).

## Result
Option B fails: no passive U shape kept the cube through an in-place spin. Consistent with the
reasoning: without something in front of the cube nothing provides the inward force.

## Next step
User decision: servo gate (feat/scoop-gate, 3 deliveries in 9.3 min with the body rule), relax the body
rule, or keep the fixed scoop with arc-only turns.
