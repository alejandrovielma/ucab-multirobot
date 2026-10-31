# Test plan: delivery with the camera, the legos' midpoint and the pink + blue area

Scope: the browser tool (`tools/teleop_web.py`) delivering yellow cubes into the
temporary area (pale blue sheet, pink line where it starts), located only with
the Pi camera.

## Decisions

- The bottom TCRT5000 is used only for the black line. Measured over white,
  pink and blue it reads the same (AO 285 / 290 / 299 counts; black 3383), so
  it cannot trigger on the area. The AO readout in the page is a diagnostic;
  `line_sensor.adc_channel: null` turns it off.
- The camera does the rest: the two legos (midpoint = gate) and the area
  (`area_blue` + `area_pink`, see `camera.area_hsv` in `tools/teleop_web.yaml`).
  The area wins when seen; the gate is the fallback.
- The lime green / cyan variant of the area is out of scope until its colours
  are measured.

## Phase 0. Preparation

1. Battery charged; no `Undervoltage` in `dmesg`.
2. Deploy the files together (code + YAML + `src/vision/vision/zone.py`),
   check the hashes and restart `teleop_web.py` (Auto off).
3. Check `state=PAUSED`, `auto=false`, `camera=true` and no errors in the log.
4. Keep the ethernet cable off the arena during moving tests (or use Wi-Fi).
5. Log every run: state changes (`monitor.py`), a frame every 2 s of the Pi
   stream and the phones, and a note with time, battery and light.

## Phase 1. Static vision (robot parked)

| # | Test | Pass if |
|---|---|---|
| 1.1 | Area at 30, 40, 50, 70, 100 cm, centred and 15 deg left/right | Detected 30-70 cm; distance error <= 5 cm |
| 1.2 | No area on the white table, the sign paper, the claw, the dark wall, the power supply | 0 false detections in 20 frames each |
| 1.3 | Repeat 1.1 with other light and facing the dark wall (white balance shift) | Still detected, no false area |
| 1.4 | Legos' midpoint at 40, 60, 80, 100 cm, with both legos and with one | Distance error <= 5 cm; no jumps (saw 171 and 136 cm) |
| 1.5 | Area and gate both in view, then the area covered | Uses the area when seen, the gate otherwise, no flicker |
| 1.6 | Cube at 20, 15, 10 cm | Seen at 20 and 15, not at 10; visible after the 0.6 s back-up |

Tune: the `area_hsv` ranges, `floor_min_area_px`, `floor_max_area_px`.

## Phase 2. Arrival and delivery calibration (manual driving)

| # | Test | Measure |
|---|---|---|
| 2.1 | Forward at 0.42 and backward at 0.35, 1 s over a ruler, 5 times | m/s of each |
| 2.2 | Drive slowly to the area; note when the pink line reaches row 445 or 0.19 m | Where the line really is relative to the scoop |
| 2.3 | Push a cube 0.4, 0.6, 0.8 s from the arrival point, 5 times each | cm the cube ends inside the blue |
| 2.4 | Back up 0.8, 1.0, 1.2 s after the push | Robot clears the area without touching the back |

Goal: cube 6-10 cm inside the blue. Sets `floor_deliver_push_s` and
`floor_deliver_backup_s` (0.6 s and 1.0 s are estimates).

## Phase 3. One-cube Auto cycles

One yellow cube; centre, left and right starts (5 each) plus one start with the
cube ~10 cm from the scoop. Pass: 13 of 15 delivered inside the blue with no
help, counter correct, no re-pick, cycle under 30 s.

## Phase 4. Several cubes and distractors

Three yellow cubes and one blue distractor, three runs of 3 minutes. Pass: the
blue cube untouched, counters match, no stop over 5 s without reason, no cube
lost beyond recovery.

## Phase 5. Failure cases

| Case | Expected |
|---|---|
| Cube at or behind the black border | Not chased in a loop: ignored 8 s, then explores |
| Far cube that flickers | Keeps heading toward it 1.5 s, no back-up |
| Cube touching the scoop at the start | Backs up 0.6 s and sees it |
| Yellow tape on the power supply | Not taken for a cube |
| Near the claw | Not taken for the area, no contact |
| Facing the dark wall | No false area, no stall |
| No cubes for 20 s | Stops cleanly (`NO CUBES` / `DONE`) |
| Restart after a hand nudge | Backs up and searches, not stuck |

## Acceptance

- Never leaves the track.
- No false-area stall or turn without reason.
- At least 85 % of cubes delivered inside the blue without help.
- No block over 5 s outside `NO CUBES`.
- Same result in three sessions with different light.
