# Feature: body-geofence

## Objective
Team rule (2026-10-12): the scoop may step on the tape, the robot body may never cross it. Recover the
cubes captured facing the border (team-todo #12, option 1).

## Decisions
- Geofence on the body rectangle (0.19 x 0.164 m, wheels included, scoop excluded) along the predicted
  command (0.3 s), in the safety gate and the behavior (same model, core/track.py).
- Reverse allowed with the overhead pose when the body stays inside (never while carrying).
- Drop point y -0.065: body front stops before the square's tape, only the scoop enters.
- Spins only with room for the whole turn (center >= 12.55 cm from the edge); else back off.
- Border turns aim at the most open of 12 headings (kinematic harness: removed stuck periods).
- Carry planner (core/planner.py): A* over straight / arc left / arc right with the body inside;
  a plan exists from 72 % of valid poses to the entry on the Webots track (median 38 ms).
- Rejected: requiring room to spin before every forward move (run 11: robots pinned; the track is at
  most 0.24 m from its edge); greedy arc steering (run 13: dead-ends in the 0.36 m waist).

## Tasks
- [x] T1 Body footprint, prediction, path check (track.py) + gate + behavior. 33c202c
- [x] T2 Fixes from runs 11-12: no spin-room for forward (d83a5f6), full-turn room + back off (5cdd697).
- [x] T3 Carry planner (0ab7025), open-space border turns + transition reasons (5c60aa2).
- [x] T4 Fixes from runs 15-17: approach near the tape (850f2dc), hold and replan (3bc80e1), planned
      heading for DELIVER and no turn-away in the corridor (b481c23).
- [ ] T5 Acceptance: deliveries comparable to the runs without the body rule (2-3 per 7 min) -> not met.

## Evidence
- Docker after b481c23: 192 tests, 0 failures; launch smoke OK.
- Kinematic harness (no cubes): 8 seeds x 300 s, body never outside, no stuck periods.
- Webots runs 11-18 (7-10 min each): deliveries 0,0,0,0,0,1,0,1. Run 18 reasons: 63 captures skipped
  (no plan to the square from the post-capture pose), 25 approaches abandoned at the border, 5 cubes
  dropped for lack of plan. Body corner beyond the tape inner edge: at most 1.2 cm in 9/284 and 16/284
  samples (includes pose noise). Run 18 delivery frame: the cube rests on the square's north tape.
- Native review: not run.

## Next step
Decision pending with the user: a servo gate that RETAINS the cube in the U would allow in-place turns
and reversing with a cube, removing the arc-only constraint that limits deliveries.
