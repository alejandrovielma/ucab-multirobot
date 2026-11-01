# recolector-ucab

Collector robot for UCAB Robotics. It is a mecanum robot that picks up wooden cubes inside the area marked by the track's black line and pushes them, one at a time with its scoop, to the delivery area (blue and green paper in a gap of the line). It keeps count of the cubes it delivers.

Other robots also work on the track (another collector and the claw that sorts the cubes from the square). **They are separate projects, by other teams.** This repository is only our robot; anything that must be agreed with them is recorded in [`docs/external-interface.md`](docs/external-interface.md).

> The code is empty on purpose: each file says what it must do and carries a `TODO`. The team writes it.
> A first piece is already there: colour vision and approaching a cube (without the scoop), tested in simulation.

## Technology

| Layer | Choice |
|---|---|
| Computer | Raspberry Pi 5, 8 GB |
| Operating system | Ubuntu 26.04 LTS (Server on the Pi; Desktop or WSL2 on the laptops) |
| Middleware | ROS 2 Lyrical |
| Language | **Python 3.12** with `rclpy` |
| Vision | OpenCV (HSV) |
| GPIO | `lgpio` / `gpiozero` (RPi.GPIO does not work on the Pi 5) |
| Parameters | YAML |
| Simulation | Webots R2025a, later `webots_ros2` |
| Quality | `ruff`, `pytest`, GitHub Actions |

Why: [`docs/decisions/`](docs/decisions/).

## Structure

```
recolector-ucab/
├── AGENTS.md                 rules for AI assistants (Claude, Copilot, Codex…)
├── CLAUDE.md                 points to AGENTS.md
├── agents/                   per-area guides for AI assistants and for reviewing PRs
├── docs/                     architecture, code style, external interface, team, hardware, plan and decisions
├── src/                      ROS 2 workspace (colcon)
│   ├── recolector_interfaces/  custom messages
│   ├── recolector_common/      helpers shared by the packages (ROS parameters, node boilerplate)
│   ├── mobile_base/            motors, line, ultrasonic, scoop, colour and state machine
│   ├── vision/                 camera, cube detection and web viewer
│   ├── recolector_sim/         Webots ↔ ROS 2 bridge (simulation only)
│   └── bringup/                launch files (real robot and simulation)
├── simulation/webots/        track world and robot model
├── docker/                   development image (ROS 2 + Webots); used with compose.yaml
├── tools/                    HSV calibration, ROS world generation, direct HAT control (teleop_web_lib/)
├── scripts/                  Pi and laptop setup, HAT test
└── .github/                  CI, PR and issue templates, CODEOWNERS
```

## Getting started

```bash
git clone <repo-url> ~/recolector-ucab
cd ~/recolector-ucab
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Simulation: open `simulation/webots/worlds/pista_ucab.wbt` in Webots R2025a ([instructions](simulation/webots/README.md)).

### With Docker (without installing ROS)

```bash
docker compose build
docker compose up sim        # Webots + nodes; open http://localhost:8080: camera and WASD control
```

The simulated robot searches, approaches the nearest cube of the chosen colours and stops
before the scoop. Details and other modes in [`docker/README.md`](docker/README.md). To
take it to the Raspberry Pi: [`docs/ssh-deployment.md`](docs/ssh-deployment.md).

## How we work

- `main` always builds; nothing is pushed directly to `main`.
- Branches `feature/…`, `fix/…`, `docs/…`; commits `area: what`.
- Every PR is reviewed by another team member.
- Everything technical in English: code, comments, logs and documentation.

Details in [`CONTRIBUTING.md`](CONTRIBUTING.md). Roles and milestones in [`docs/team.md`](docs/team.md).

## Documents

- [Architecture](docs/architecture.md)
- [Interface with the other robots](docs/external-interface.md)
- [Field test: fleet link & arbiter](docs/field-test-fleet-link.md)
- [Team, roles and milestones](docs/team.md)
- [Hardware and wiring](docs/hardware.md)
- [Deploying the code to the robot over SSH](docs/ssh-deployment.md)
- [Original collector plan](docs/collector-plan.md) (includes ideas about the other robots that are no longer part of this repo)
