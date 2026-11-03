# Interface with the other robots

The other robots on the track (the other collector, **Phineas**, and the claw arm, **Perry**) are projects by other teams from repository [alejandrovielma/ucab-multirobot](https://github.com/alejandrovielma/ucab-multirobot). This file records **what is agreed and contracted with them**. None of their application code goes in this repository; only the interface package `multirobot_interfaces` is vendored.

Status: **Adopted from upstream (2026-10-24)**, based on reference commit `09d0f16` (2026-10-23), pending final physical confirmation with their team.

---

## Specification summary

| Topic | Question | Agreed specification |
|---|---|---|
| **Network** | Do we use a teammate's hotspot or a router? Which `ROS_DOMAIN_ID`? | Same Wi-Fi network (phone hotspot or router). Same `ROS_DOMAIN_ID` across all robots (value to confirm, default 0). `chrony` configured across all 3 Raspberry Pis for time sync. If multicast discovery fails, configure `ROS_STATIC_PEERS` with fixed IPs. |
| **Colours** | Does each collector have reserved colours? Which are ours? | **Both collectors collect every colour except red** (track rule of 2026-10-22). Target colours: orange, yellow, green, blue, purple. Red cubes stay on the track and are ignored. |
| **Square** | How do we avoid entering the square at the same time as the other collector, or while the claw is working? | Arbitrated by Perry via `/zone/*` services. A robot requests turn via `/zone/request` and enters only when `/zone/occupant` equals its name. Lease duration is 60 s (auto-renews if requested again). On exit, releases via `/zone/release`. Waiting robot waits >30 cm away. In case of deadlock or tie, **Ferb yields**. |
| **Count** | Who keeps the official count: each robot or the claw? | **Perry's `zone_arbiter` keeps the official count.** When a collector calls `/zone/release` with `delivered=true` (and cube `color`), Perry increments the official count and returns `delivered_total`. |
| **Notices** | Do we have to announce over ROS when we drop a cube? With which topic and type? | Cube drop is signaled via `/zone/release` (`multirobot_interfaces/srv/ReleaseZone` with `robot`, `delivered=true`, `color`). Robot state is broadcast at 2 Hz on `/<robot>/state` (`multirobot_interfaces/msg/RobotState`). |
| **Stop** | Is there a shared emergency stop? | **Heartbeat stop:** `/zone/occupant` heartbeat arrives at 2 Hz. If no heartbeat is received for 2.0 s, robots must stop forward motion immediately (`obstacle_guard` enforces zero velocity). In addition, each robot has a physical battery master switch. |
| **Markers** | Is a marker (ArUco) allowed next to the square or on top of the robots? | Yes. ArUco markers tracked by Perry's overhead camera: **ID 1** = Phineas, **ID 2** = Ferb, **ID 10** = Delivery square. |

---

## Agreed contract (2026-10-24)

Source: Adopted from `alejandrovielma/ucab-multirobot` (commit `09d0f16`, 2026-10-23).

### 1. Namespaces and node isolation
- Ferb runs all local nodes under namespace `/ferb`.
- Phineas runs all local nodes under namespace `/phineas`.
- Perry runs nodes in the global namespace or `/perry`.
- Only `/zone/*` and peer pose topics are absolute. All local collector topics (`cmd_vel`, `cubes`, `range`, `state`, etc.) are relative to the namespace.

### 2. Topics and Services (`multirobot_interfaces`)

| Interface | Name | Type | Rate / QoS | Description |
|---|---|---|---|---|
| Heartbeat & Turn | `/zone/occupant` | `std_msgs/msg/String` | 2 Hz, RELIABLE, TRANSIENT_LOCAL, depth 1 | Current occupant of the square (`""` if free, `"phineas"` or `"ferb"`). Also serves as fleet heartbeat. |
| Service | `/zone/request` | `multirobot_interfaces/srv/RequestZone` | Client call | Request entry lease to delivery square (`robot: string`). Returns `granted: bool`, `reason: string`. Lease lasts 60 s. |
| Service | `/zone/release` | `multirobot_interfaces/srv/ReleaseZone` | Client call | Release square lease (`robot: string`, `delivered: bool`, `color: string`). Returns `ok: bool`, `delivered_total: uint32`. |
| Own Pose | `/ferb/pose` | `multirobot_interfaces/msg/RobotPose` | ~10–20 Hz | Published by Perry's overhead tracker in table coordinates (meters, theta rad). |
| Peer Pose | `/phineas/pose` | `multirobot_interfaces/msg/RobotPose` | ~10–20 Hz | Published by Perry's overhead tracker. Used by Ferb's `obstacle_guard` / `PeerAvoidance`. |
| Own State | `/ferb/state` | `multirobot_interfaces/msg/RobotState` | 2 Hz | Published by Ferb's `fleet_link`. States: `STOP`, `EXPLORE`, `APPROACH`, `CAPTURE`, `CARRY`, `DELIVER`. |
| Peer State | `/phineas/state` | `multirobot_interfaces/msg/RobotState` | 2 Hz | Published by Phineas. |

### 3. Peer Avoidance Rules
From `mobile_base/core/safety.py` upstream, mirrored in Ferb's `PeerAvoidance`:
- Uses overhead pose estimates; ignores stamps older than `pose_max_age_s` = 0.5 s.
- Forward detection cone: ±60° ahead.
- `slow_distance_m` = 0.60 m: scale forward speed and steer away at `avoid_turn_rad_s` = 0.6 rad/s.
- `stop_distance_m` = 0.40 m: stop forward motion entirely.
- `deadlock_time_s` = 3.0 s: if blocked for >3 s, **Ferb is the robot that yields**: rotate in place at `escape_turn_rad_s` = 0.8 rad/s away from peer until peer leaves the cone.

---

## Future extensions (not in current scope)

Planned in upstream `docs/plan-integracion-hibrida.md` but not yet implemented:
- `/cubes/claim`: Claiming specific cubes to prevent both collectors targeting the same cube.
- `/world/cubes`: Global cube map tracked by Perry's overhead camera.
- `/<robot>/mission`: High-level mission dispatch from an orchestrator.

These remain marked as **future** and are not required for current fleet operations.
