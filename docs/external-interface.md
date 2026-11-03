# Interface with the other robots

This repository implements the **Phineas** collector (`/phineas`). The other robots on the track — the peer collector **Ferb** ([`notoriussss/Ferb-Recolector`](https://github.com/notoriussss/Ferb-Recolector), `/ferb`) and the sorting claw arm **Perry** ([`Kobalt09/Robot_Claw`](https://github.com/Kobalt09/Robot_Claw), `/perry`) — are maintained in their own repositories. This file records **what is agreed and contracted across the three teams**. None of their application code goes in this repository; only the shared interface package `multirobot_interfaces` is vendored.

Status: **Updated for the Definitive Multi-Robot Integration Plan (2026-11-02)**.

---

## Specification summary

| Topic | Question | Agreed specification |
|---|---|---|
| **Network** | Which Wi-Fi network and `ROS_DOMAIN_ID` do we use? | Dedicated team router or phone hotspot (not university Wi-Fi). **`ROS_DOMAIN_ID=0`** across all 3 Raspberry Pis (Phineas Pi 5 native ROS 2, Ferb Pi 5 native ROS 2, Perry Pi 4 Docker `network_mode: host`). `chrony` time sync; optional `ROS_STATIC_PEERS` if multicast discovery is blocked. |
| **Colours** | Which colours does each collector bring to the zone? | Perry sorts into **`yellow`** and **`blue`** bins. To decouple trajectories in the open arena so the two collectors never chase the same cube head-on: **Phineas collects only `blue`** (`target_colors: [blue]`) and **Ferb collects only `yellow`**. Red and all other colours stay on the track (`avoid: [red]`). |
| **Square & >30 cm Wait** | How do we avoid colliding at the delivery square entrance or with the claw? | Arbitrated by Perry via `/zone/*` services. Phineas requests the lease (`/zone/request`) early at **~45 cm** (`zone_request_margin_m: 0.45` / visual row `zone_request_v_px: 255`, or `zone_request_distance_m: 0.45` in `teleop_web`). Without a valid grant (`zone/granted == false`), Phineas stops completely at **30–35 cm** before the square (`zone_wait_margin_m: 0.30` / visual row `zone_wait_v_px: 300`, `WAIT_ZONE` / `CARRY (wait zone)`), leaving >30 cm clear for the active collector to push (12–19.5 cm from Perry's axis), reverse out, and spin away. The lease is held until the reverse exit completes before calling `/zone/release`. |
| **Flow Control (`gate_closed`)** | How does Perry pause incoming deliveries when the zone is full or blocked? | Perry publishes `/zone/occupant = "perry"` (keeping the 2 Hz heartbeat alive) and `/perry/state` with **`gate_closed: true`** whenever >=2 cubes are waiting in the zone, a cluster is `BLOCKED`, or the operator presses `STOP`. While `gate_closed` is `true` and Phineas does not already hold the lease, `fleet_link` (`ZoneClient`) suppresses `/zone/request` calls (`should_request() == false`) and Phineas waits >30 cm away; if Phineas already holds the lease mid-delivery, it finishes reversing out and releases the zone cleanly. |
| **Count** | Who keeps the official count: each robot or the claw? | **Perry's `zone_arbiter` keeps the official count.** When a collector calls `/zone/release` with `delivered=true` and `color="blue"` (Phineas) or `"yellow"` (Ferb), Perry increments the official count, enters its 2.0 s `ASENTANDO` (`wait_still`) check, and returns `delivered_total`. |
| **Stop** | Is there a shared emergency stop? | **Heartbeat stop:** `/zone/occupant` arrives at 2 Hz. If no heartbeat is received for `2.0 s`, `fleet_link` sets `fleet/ok = false` and `zone/granted = false`, and `obstacle_guard` commands zero velocity (`require_heartbeat: true`). Each robot also has a physical battery master switch. |
| **Poses, Markers & Ultrasonic** | Is there an overhead ceiling camera, ArUco tags on the cars, or an ultrasonic sensor? | **No (`require_pose: false`).** Perry's camera only views the delivery area (no ceiling tracker, no ArUco tags on top of the cars), and the physical collectors carry only a front CSI camera and a bottom TCRT5000 line sensor (`ultrasonic` runs with `fake: true`). |

---

## Agreed contract (2026-11-02)

### 1. Namespaces and node isolation
- **Phineas (this repo)** runs all local nodes under namespace `/phineas` (`require_pose:=false`, `require_heartbeat:=true`, `use_lease:=true` / `require_zone_grant:=true`, `targets:=[blue]`, `avoid:=[red]`).
- **Ferb** runs all local nodes under namespace `/ferb` (`target_colors: [yellow]`).
- **Perry** runs `zone_arbiter` and `PerryFleetNode` (`ros_node.py`), publishing `/zone/occupant` and `/perry/state` and serving `/zone/request` and `/zone/release`.
- Only `/zone/*`, `/perry/state`, and optional `/*/pose` topics are absolute. All collector-internal topics (`cmd_vel`, `cubes`, `zone/seen`, `zone/want`, `zone/granted`, `state`, etc.) resolve relative to `/phineas`.

### 2. Topics and Services (`multirobot_interfaces`)

| Interface | Name | Type | Rate / QoS | Description |
|---|---|---|---|---|
| Heartbeat & Turn | `/zone/occupant` | `std_msgs/msg/String` | 2 Hz, RELIABLE, TRANSIENT_LOCAL, depth 1 | Current occupant of the square (`""` if free, `"phineas"`, `"ferb"`, or `"perry"`). Also serves as the 2 Hz fleet heartbeat. |
| Claw Flow Control | `/perry/state` | `multirobot_interfaces/msg/RobotState` | 2 Hz | Published by Perry. When `gate_closed` is `true`, collectors that do not already hold the lease suppress `/zone/request` and wait >30 cm outside the square. |
| Service | `/zone/request` | `multirobot_interfaces/srv/RequestZone` | Client call | Request (or renew every 20 s) entry lease (`robot: "phineas"`). Returns `granted: bool`, `holder: string`, `lease_seconds: float32` (60 s lease). |
| Service | `/zone/release` | `multirobot_interfaces/srv/ReleaseZone` | Client call | Release square lease after backing out (`robot: "phineas"`, `delivered: bool`, `color: "blue"`). Returns `ok: bool`, `delivered_total: uint32`. |
| Own State | `/phineas/state` | `multirobot_interfaces/msg/RobotState` | 2 Hz | Published by Phineas's `fleet_link`. States: `STOP`, `EXPLORE`, `APPROACH`, `CAPTURE`, `CARRY`, `DELIVER`. |
| Peer State | `/ferb/state` | `multirobot_interfaces/msg/RobotState` | 2 Hz | Published by Ferb's `fleet_link`. |
| Own Pose (optional) | `/phineas/pose` | `multirobot_interfaces/msg/RobotPose` | ~10–20 Hz (unused on track) | Optional planar pose (`require_pose: false` on physical track without ceiling camera). |
| Peer Pose (optional) | `/ferb/pose` | `multirobot_interfaces/msg/RobotPose` | ~10–20 Hz (unused on track) | Optional peer pose used by `obstacle_guard` / `PeerAvoidance` only when an overhead tracker is running. |

### 3. Delivery-Zone Approach & Wait Thresholds (>30 cm Rule)
To guarantee Phineas never blocks the square entrance in camera-only mode (`require_pose: false`):
- **Early lease request (~45 cm):**
  - `CollectorFsm` (`src/mobile_base/mobile_base/collector.py`): activates `zone_want = true` when distance to the stop point is `<= zone_request_margin_m` (`0.45 m`) **or** when the floor paper's near edge reaches visual row `v_px >= zone_request_v_px` (`255 px`).
  - `AutoPilot` (`tools/teleop_web_lib/autopilot.py` + `FleetBridge`): activates `want = true` at `zone_request_distance_m` (`0.45 m`).
- **Stationary wait outside the square (>30 cm):**
  - `CollectorFsm`: if `require_zone_grant` is `true` and `zone_granted` is `false`, transitions to `WAIT_ZONE` (`Command(0.0, 0.0)`) as soon as distance to the stop point is `<= zone_wait_margin_m` (`0.30 m`) **or** visual row `v_px >= zone_wait_v_px` (`300 px`, well before `zone_arrive_v_px = 445 px`).
  - `AutoPilot`: if `not fleet.granted()`, enters `CARRY (wait zone)` at `zone_wait_distance_m` (`0.30 m`) and remains completely still (even if the paper flickers in the camera) until `zone/granted` becomes `true`.

### 4. Optional Overhead Peer Avoidance (`PeerAvoidance`)
Retained in `obstacle_guard` (`peer_avoidance.py`) for simulation or overhead-camera setups (inactive when no `/phineas/pose` and `/ferb/pose` are published):
- Ignores pose stamps older than `pose_max_age_s` = `0.5 s`.
- Forward detection cone: `±60°` ahead; slows at `peer_slow_distance_m` = `0.60 m` (`avoid_turn_rad_s` = `0.6 rad/s`), stops forward motion at `peer_stop_distance_m` = `0.40 m`, and rotates away after `deadlock_time_s` = `3.0 s` (`escape_turn_rad_s` = `0.8 rad/s`).
