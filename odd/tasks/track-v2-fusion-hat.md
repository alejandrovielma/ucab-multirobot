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
- [ ] T1 Generalized DeliveryZone (any heading) + new zone/track/wait-point config + tests.
- [ ] T2 Blue square detection in vision (not a cube) + square fields in the message + behavior
      alignment while entering + tests.
- [ ] T3 Fusion HAT+ motor backend: ROS node and lab tool, battery 8.4 V cap + tests.
- [ ] T4 Webots world: new track texture/board/cubes/marker/start poses generated from one geometry
      source (no Webots run).
- [ ] T5 Docs: README, architecture, team-todo, lab guide (Ubuntu + Fusion HAT+ driver), lab README.
- [ ] T6 Checks: Docker suite, tools/lab tests.

## Progress / evidence
(pending)
