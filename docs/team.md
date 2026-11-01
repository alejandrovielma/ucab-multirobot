# Team, roles and milestones

We are 4 people for one robot. Each person **owns** an area: they make decisions, review the PRs that touch it and maintain its guide in `agents/`. Everyone can code anywhere.

When names are assigned, also update `.github/CODEOWNERS`.

| Role | Person | Responsibilities | Guide |
|---|---|---|---|
| **1. Mobile base and hardware** | _to be assigned_ | `motor_driver`, `line_sensors`, `ultrasonic`, `scoop_driver`; assembly and wiring; `scripts/setup_pi.sh`, `scripts/test_hat.py`; `docs/hardware.md` | `agents/mobile-base.md` |
| **2. Vision and colour** | _to be assigned_ | `vision` (`camera_node`, `cube_detector`); `color_sensor`; `tools/hsv_tuner.py` | `agents/vision.md` |
| **3. Behaviour** | _to be assigned_ | `behavior_fsm`, `obstacle_guard`, `recolector_interfaces`; `docs/architecture.md`; agreements with the other teams (`docs/external-interface.md`) | `agents/behavior.md` |
| **4. Simulation, integration and quality** | _to be assigned_ | `simulation/`, `bringup`, `scripts/setup_dev.sh`, CI; final review of `main` | `agents/simulation-integration.md` |

## Team rules

- Every PR is reviewed by someone other than its author.
- One task = one issue = one branch = one PR.
- Nothing is tested on the real robot without first going through the simulation or a test on the laptop, except pure hardware tests.

## Milestones

| Milestone | Demo |
|---|---|
| **M0. Repo ready** | `colcon build` passes on the 4 laptops and in CI |
| **M1. It moves** | the robot (real and simulated) moves by keyboard in every direction |
| **M2. It stays in** | it drives around inside the area without crossing the line and stops in front of a hand |
| **M3. It sees the cubes** | the camera marks the cubes by colour on the real track; the scoop sensor confirms the colour |
| **M4. It delivers one** | it finds a cube, catches it with the scoop and leaves it in the square; the count goes up |
| **M5. Test day** | it works on its own for several minutes on the track with the other robots, without crashing |
