# 0001. Python + ROS 2 Lyrical on Ubuntu 26.04

**Status:** accepted (updated 2026-10-24 from Ubuntu 24.04 / ROS 2 Jazzy)

## Context
The class whiteboard sets the stack Pi 5 → HAT → OS → ROS 2 → application. We are 4 people with different skill levels and there is little time. The robot runs control at 20–50 Hz, vision at 15 Hz and no heavy computation.

Initially, Ubuntu 24.04 and ROS 2 Jazzy were selected. However, as of 2026-10-24, Ferb's Raspberry Pi 5 runs Ubuntu 26.04 to align with the rest of the fleet (Phineas and Perry). Because Jazzy has no binary packages for Ubuntu 26.04, the platform moves to ROS 2 Lyrical.

## Decision
- Ubuntu 26.04 LTS, because it is the version running on the fleet's Raspberry Pi 5 boards.
- ROS 2 Lyrical (supported distribution on Ubuntu 26.04).
- All code in Python 3.12 with `rclpy`. Custom messages in an `ament_cmake` package (the only way to generate interfaces).
- OpenCV for vision and `lgpio` for GPIO.

## Consequences
- The whole fleet (Ferb, Phineas, Perry) shares the exact same OS and ROS distribution, preventing DDS wire incompatibilities and compiler/glibc mismatches.
- The whole team can read and review any package.
- If a node turns out to be slow on the Pi (for example vision), just that node can be rewritten in C++ without touching the rest, because the topics do not change.
- RPi.GPIO does not work on the Pi 5; `lgpio` or `gpiozero` must be used.
