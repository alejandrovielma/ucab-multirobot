# Open issues of the Phineas proof of concept

Phineas was built as a lab proof of concept, not as a product. This file lists what was left
unfinished or unverified at the end of the project (November 2026), so the presentation can stay
on what was shown and the next team knows where to start.

## Not tested yet

- **Joint run with Ferb.** The zone exclusion rule (ask at 0.70 m, wait at 0.55 m, keep away while
  the other holds the zone, keep the turn until out, max 5 s, 3 s request hold) is implemented and
  covered by tests, but was never run with both robots moving on the track.
- **Sweep of a forbidden cube.** It triggered in real runs, but nobody measured whether the scoop
  touched the cube. Its distances are provisional: the scoop's width and reach were never measured
  (`auto.foreign_*` in `tools/teleop_web.yaml`).
- **Scoring rule.** Whether touching a forbidden cube is penalised is not confirmed against the
  official rule. If it is, sliding around should come before sweeping.

## Not working yet

- **Peer avoidance.** `tools/teleop_web_lib/peer.py` and `PeerAvoidance` run in observe mode only.
  Nobody publishes `/phineas/pose` or `/ferb/pose`: Perry's camera at 70 cm covers about
  55 × 40 cm, only the delivery area. It needs an overhead camera that sees the whole track, and the
  heading must be validated before it brakes for real (a 180° error steers towards the peer).
- **Releasing the turn on shutdown.** On a signal, ROS shuts down before our cleanup, so the release
  call does not go out. The lease then lasts until it expires (60 s). Avoid restarting with a turn
  granted.

## Weak spots

- **Network.** Ping between Phineas and Ferb was 560–870 ms in two measurements, while poses expire
  after 0.5 s. Perry dropped off the network several times. Measure with real traffic and consider a
  dedicated router instead of a hotspot.
- **Perry is a single point of failure.** If it goes down, nobody delivers; the robots wait with the
  cube by agreement.
- **Beacon filter.** The 75 % expected-height threshold is provisional; recalibrate it under the
  final track lighting.
- **Few measurements.** The turning result (75 → 44 % of the time) is one 60 s run before and one
  after; the runs of 2026-11-05 are not comparable with each other (mission and cube layout
  changed). No success rate can be claimed from them.
- **Battery.** The reading drops under load. A run started at 13 % lost power and left three files
  corrupted at the same size: compare hashes after any power cut.
- **Camera calibration.** The 2026-11-03 fit (rms 6.8 %, ±3 cm at 45 cm) used hand-measured points;
  re-measure, and check the far range (60 cm and beyond).
