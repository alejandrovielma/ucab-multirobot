# Feature: fsm-stall-fixes

## Objective
Make the robot logic (BehaviorFSM + SafetyGate + the lab loops) never stall or die in the middle of a
run, so that it can be placed on the track and always keeps doing its job: explore, turn back at the
line, go to a cube of its color, capture, carry, deliver.

## Why
A closed-loop simulation with the real FSM and gate (80 runs, 600 s each, 2026-10-31) stalled in
25/40 yellow and 30/40 blue runs (more than 30 s in one state without moving 5 cm or turning 20 deg).
A static read of behavior.py found the same patterns plus others. The user's goal on 2026-10-31:
"fix them, rerun the simulation, verify everything works, review the code in depth, do not stop until
it is confirmed working to put it on the track".

## Honest limit (do not oversell)
The simulation does not model the HAT start-power floor, camera misdetections, the real scoop
retention, the line sensor at the robot center or battery/Wi-Fi failures. "Zero stalls in the
simulation" is necessary, not sufficient: only floor runs with logs prove the real robot.

## Facts (evidence)
- Stall patterns (sim): (a) CARRY sidestep jitter, commanded strafe flips sign every tick and the pose
  freezes; `_dodges` counts episodes not time (behavior.py:951-957), so `dodge_limit` never fires;
  (b) EXPLORE spin in place alternating +-1.2 rad/s with a non-target cube reported in the scoop.
- Static risks: APPROACH<->CAPTURE rebound with no counter (792-802, 819-825); APPROACH strafe with no
  timeout (788-791); shed loop with no counter (408-433); border cube re-approached with no ignore
  (747-753); a stuck-black line sensor loops reverse-and-turn forever.
- Harness: scratchpad stall_check/ (harness.py imports tools/sim/sim2d make_robot/step_world; stall =
  >30 s in a state, <5 cm and <20 deg, IDLE and the last 10 s ignored).
- Real robot (2026-10-31): commanded speed == real speed (motors test, 15 cm in 3 s at 0.05 m/s).

## Delivery strategy
Direct commits to `main` (user instruction 2026-10-23, memory push-directo-a-main). Each commit runs
the full suite and, when the native assessment says due, the native review; push after approval.

## Tasks
- [x] T1 Regression test in the repo: deterministic closed-loop no-stall test over a few seeds per
      color (RED first with the seeds that stall today).
- [x] T2 CARRY sidestep: root cause, direction latch and a time bound (abandon the transport).
- [x] T3 EXPLORE spin-in-place with a non-target cube in the scoop: root cause and a bound.
- [ ] T4 APPROACH<->CAPTURE rebound counter (ignore the cube after repeated failed captures).
- [ ] T5 APPROACH strafe timeout.
- [ ] T6 Border cube: do not re-approach a target on or beyond the tape.
- [ ] T7 Stuck line sensor watchdog: stop and report instead of looping forever.
- [ ] T8 Runtime robustness of the lab loops: a failing tick stops the motors and the run continues;
      repeated failures abort safely (fail-safe), camera/sensor read errors never leave the motors on.
- [ ] T9 Deep code review of behavior.py, safety.py and lab_robot.py; fix the confirmed findings.
- [ ] T11 Efficiency: 72 % of approaches end without a capture (1373 of 1902 EXPLORE>APPROACH in the
      80-run harness, deliveries ~0.9 per 600 s): classify why and fix what is a bug, not a trade-off.
- [ ] T10 Final verification: full suite, harness with 100 seeds per color: 0 stalled runs; residual
      risks written down.

## Acceptance
- Harness: 0 stalled runs in at least 100 seeds per mission color (600 s), no dwell above 60 s outside
  IDLE, DELIVER reached and completed in most runs.
- Full workspace suite green except the known OpenCV-4.6 test (`generateImageMarker`).
- Every fix has a test that reproduced the problem first.
- No code path leaves the motors powered after an error or a signal (SIGKILL cannot be caught: written
  down as residual risk).

## Progress
- 2026-10-31: document created; no source change yet.
- 2026-10-31: T1+T2 done, commit bc6bb28 (writer, delegated: 2+ non-trivial files + mapping trigger).
  Root cause: two non-target cubes flank the robot, one sits at the avoid_cx 0.3 edge, each 2 cm strafe
  tick flips which one is nearest, so the sign of the sidestep flipped every tick; `_dodges` counted
  episodes, not time. Fix: direction latched per episode + `dodge_time` 6.0 s. RED: tools/sim
  test_no_stall 6/6 failed, test_stall_fixes 2/3. GREEN observed by the parent: 68 passed (new tests +
  test_delivery + test_config); full suite 368 tests, 1 known OpenCV-4.6 failure. Harness seeds 1-40
  (600 s): CARRY stalls 13->0 yellow, 16->0 blue; stalled runs yellow 25->22, blue 30->22 (runs that
  froze in CARRY now continue and reach the EXPLORE pattern, T3). Assessed medium/under_budget: no
  native review due; reviewed boundary stays at e76031b.
- 2026-10-31: T3 done, commit 50480b7. Diagnosis (writer recorded every stall tick): ALL 76 EXPLORE
  stall episodes were one pattern: the avoid spin (turns away from a non-target cube inside
  avoid_cx 0.3) and the guided spin (toward the cartographer goal, which lies behind the cube) alternate
  every ~0.5 s; `_avoid_since` was reset whenever the cube left the path so `avoid_timeout` never
  accumulated. NOT a sim artifact (scoop empty, spin 1.2 rad/s). Fix: after `avoid_cycles` 3
  engagements within `avoid_window` 6 s the guide is off for `guide_off_time` 4 s (drive straight).
  Parent verified: 81 passed (stall, config, behavior tests + tools/sim); harness seeds 1-40, 600 s,
  rerun by the parent: stalled runs 0/40 yellow, 0/40 blue (before 25/30; after T2 22/22); deliveries
  mean 0.95 yellow / 0.9 blue (before 0.5 / 0.3). Remaining: APPROACH<->EXPLORE flip bursts in blue
  seeds 16 and 20 (not stalls by definition). Assessed medium/under_budget (255 accumulated lines).
