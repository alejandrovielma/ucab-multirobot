# Feature: ros2-lyrical

## Objective
Move every machine of the project (robot Pis, Perry's Pi, the simulation host) from ROS 2 Jazzy to
ROS 2 Lyrical Luth, so the Pis run ROS natively on Ubuntu Server 26.04 without Docker.

## Why
- The Pis were flashed with Ubuntu Server 26.04 (the CSI camera works there). Jazzy only ships
  binaries for Ubuntu 24.04; Lyrical is the LTS whose Tier 1 platform is Ubuntu 26.04 amd64/arm64
  (supported until May 2031): https://docs.ros.org/en/jazzy/Releases/lyrical/supported-platforms.html
- Mixing distros between machines over DDS is not guaranteed, so all machines move together
  (user decision 2026-10-20: "Lyrical en todos los equipos").

## Facts
- webots_ros2 for Lyrical: `ros-lyrical-webots-ros2` (apt) or source build:
  https://docs.ros.org/en/lyrical/Tutorials/Advanced/Simulators/Webots/Installation-Ubuntu.html
  Compatibility of that release with Webots R2025a is not verified yet.
- Docker images `ros:lyrical-ros-base` exist for amd64 and arm64:
  https://discourse.openrobotics.org/t/lyrical-luth-test-and-tutorial-party-instructions/54427/14
- WSL: the exact Ubuntu 26.04 distro name must be read from `wsl --list --online`.

## Scope
Repository: Dockerfile, test scripts, launch docstring, sim sweep script, README, lab plan.
Environment (needs user permission, not done by the repository change): WSL Ubuntu 26.04 + Lyrical on
the host; Lyrical on the three Pis (team, in the lab).

## Tasks
- [x] T1 Switch repository references from Jazzy to Lyrical; test scripts source the container's
      `$ROS_DISTRO` so they keep working on the old Jazzy WSL during the transition. (route: inline,
      mechanical edits in 7 files; structural check: `git grep -i jazzy` outside odd/ is empty)
- [ ] T2 Verify: build the `ros:lyrical-ros-base` test image and run the full suite (colcon build +
      test, module imports, robot.launch smoke). (route: per-action)
      BLOCKED 2026-10-20: the image build fails in `apt-get install` (HTTP 500 / unexpected EOF
      through the host proxy 127.0.0.1:8888 for archive.ubuntu.com); the base image pulled on the
      second attempt. Not a code failure; rerun `scripts/test_in_docker.ps1` when the network allows.
- [ ] T3 Host: WSL Ubuntu 26.04 + `ros-lyrical-ros-base` + `ros-lyrical-webots-ros2`, then one sim
      launch check. (needs user permission)
- [ ] T4 Pis: install Lyrical natively, `robot.launch.py` bench test (team, lab).

## Checks
- Full suite in the Lyrical container: 0 failures.
- `git grep -i jazzy` outside odd/ only in historical notes.

## Progress
- 2026-10-20: branch feat/ros2-lyrical created from feat/track-v2-fusion-hat (a1ee8ae).
- 2026-10-20: T1 done; T2 blocked by the host network proxy (no test result yet).
