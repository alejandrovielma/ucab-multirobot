# 0002. Webots for simulation

**Status:** accepted

## Context
We need to test the logic without draining batteries or risking the robots. Gazebo was ruled out for the course. We need to simulate mecanum wheels, a camera and line sensors.

## Decision
Webots R2025a. There is already a world faithful to the track (`simulation/webots/`) with the `RecolectorUCAB` PROTO: mecanum with friction at 45°, no encoders, servo-driven scoop, camera, 4 line sensors, ultrasonic, IR, colour sensor and IMU. The next step is `webots_ros2` so the same ROS nodes drive the simulated robot.

## Consequences
- On Windows, Webots runs natively and ROS 2 runs in WSL2.
- Simulated friction is not the same as real friction; turns are calibrated on the robot.
