# 0004. Adopt upstream multirobot interface (multirobot_interfaces)

**Status:** accepted  
**Date:** 2026-10-24

## Context

Our robot (**Ferb**) shares the arena with **Phineas** (collector 1) and **Perry** (sorting arm, zone arbiter, and overhead localizer). The original project plan envisioned custom `/fleet/status` and `/fleet/delivery_lock` topics. However, Phineas and Perry were already implemented by the other team in [alejandrovielma/ucab-multirobot](https://github.com/alejandrovielma/ucab-multirobot) (commit `09d0f16`, 2026-10-23) using a concrete ROS 2 package: `multirobot_interfaces`.

We needed to establish fleet interoperability without abandoning our architectural decisions—most notably ADR 0003, which establishes a single, decoupled repository for our collector robot rather than a monorepo or code duplication.

## Decision

1. **Vendor only the interface package:**  
   We copy `src/multirobot_interfaces/` byte-for-byte from `ucab-multirobot` (commit `09d0f16`) into our workspace (`src/multirobot_interfaces/`). The package name, `.msg` and `.srv` definitions, `CMakeLists.txt`, and `package.xml` remain identical to guarantee DDS type-hash compatibility across nodes.
2. **Do not vendor application code:**  
   We do **not** copy any node implementations, controller logic, or internal modules from `ucab-multirobot`. All node logic (such as `fleet_link`, `PeerAvoidance` in `peer_avoidance.py`, and safety hooks in `obstacle_guard`) is implemented independently in this repository according to our clean architecture (pure logic without `rclpy`, unit tests with pytest, and parameterization via YAML).
3. **Adopt upstream arbitration and localization contracts:**  
   - Zone arbitration via `/zone/request` and `/zone/release` (`multirobot_interfaces/RequestZone` and `ReleaseZone`).
   - Lease monitoring and fleet heartbeat via `/zone/occupant` (`std_msgs/String` at 2 Hz; missing for >2.0 s stops the robot).
   - Multi-robot namespaces: `/ferb` for our nodes and `/phineas` for the other collector.
   - Overhead tracking poses: `/ferb/pose` and `/phineas/pose` (`multirobot_interfaces/RobotPose`).
   - Fleet state publishing: `/ferb/state` (`multirobot_interfaces/RobotState` at 2 Hz).
4. **Internal messages remain isolated:**  
   Our internal robot topics and messages remain under `recolector_interfaces` (`LineSensors`, internal `CubeDetectionArray`).

## Consequences

- **Compatibility:** Full DDS and ROS 2 service/topic compatibility with Phineas and Perry out of the box.
- **Independence (ADR 0003 preserved):** Our build, tests, and CI remain self-contained. No external git submodules or intertwined codebases.
- **Maintenance:** If the other team changes `multirobot_interfaces`, we must re-sync the interface definitions manually. Internal application changes in either repository will not break the other as long as the interface contract is preserved.
