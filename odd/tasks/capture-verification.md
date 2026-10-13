# Feature: capture-verification

## Objective
Count a delivery only when a cube really reached the square: verify with the robot's own camera
that the cube is inside the fixed scoop after CAPTURE, while carrying, and that it left after the drop.

## Problem
Webots run 7: 5 deliveries counted, 1 cube in the square. CAPTURE pushes for 1 s and assumes success.

## Decisions
- Camera instead of buying an FC-51 (user decision, 2026-10-12).
- The simulated camera becomes the real Pi5-IMX219-120: Waveshare lists 120 deg diagonal FOV and a
  1.79 mm lens; pinhole horizontal FOV from the IMX219 active area (3.68 x 2.76 mm) is
  2*atan(1.84/1.79) = 1.60 rad (~92 deg). With the OV5647 FOV (0.94 rad) the captured cube is out of
  the image (bottom ray hits the ground ~0.185 m ahead of the center).
- In-scoop check reuses the cube detections (own color, low and centered in the image), so no new
  topic or message: behavior already receives `cubes`.

## Tasks
- [x] T1 Webots camera FOV 1.60 rad; measure in frames where a cube in the U and a cube at the funnel
      mouth appear (cy, cx, area); set capture_cy and the scoop window from measurements.
- [x] T2 Behavior: CAPTURE -> CARRY only if the cube is seen in the scoop; CARRY/DELIVER give up when
      it is not seen for scoop_lost_timeout; release delivered=true only if the cube left the scoop. Tests first.
- [x] T3 Webots 7 min run: counted deliveries == cubes inside the square (final screenshot).
- [x] T2b Fixes from run 8: border before capture; DELIVER only aligned in the corridor; lead-in.
- [x] T4 Docs: README status, architecture, team-todo #7 and #9.

## Acceptance criteria
Deliveries counted equal cubes in the square at the end of a 7 min Webots run; all Docker tests pass.

## Routes
All tasks inline in the parent (design and measurements already in context); deviation from the
2+ files writer trigger recorded here.

## Progress / evidence
- 8fb51b9 plan; 38cecec IMX219 FOV + calibration world + world arg; ad3ea7a scoop verification;
  b7db40e run-8 fixes; 9f0489f monitor frames on delivery.
- Calibration (calibracion_pala.wbt): window fill 0.60 with a cube in the U, 0.04 with a cube at the
  mouth; mouth cube center at cy 0.655 (pinhole prediction 0.65).
- Docker after b7db40e: 163 tests, 0 failures; import smoke 35/35; launch smoke both robots OK.
- Webots run 8: verification worked (scoop colors reported) but two 5-minute lockups -> T2b.
- Webots run 9 (7 min): both inside the track, Phineas n=3, Ferb n=0.
- Webots run 10 (7 min): both inside the track (min center distance 0.24 m); Phineas n=2 and its
  delivery frames show 1 and then 2 red cubes inside the square: counted == delivered. Ferb n=0:
  7 captures near the north border lost by the geofence while arcing (team-todo #12).
- Native review: not run.

## Next step
Rules decision for cubes captured facing the border (team-todo #12).
