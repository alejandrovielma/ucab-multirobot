# AGENTS.md

Instructions for any AI assistant working in this repository (Claude Code, Copilot, Codex, Cursor, etc.).

## What this project is

A single robot: the UCAB Robotics collector. It is mecanum, picks up 2.5 cm cubes inside the black-line area and pushes them with its scoop to the delivery square, counting the ones it delivers. The other robots on the track are other teams' projects and do **not** go in this repo. Context is in `README.md` and `docs/`.

## The team writes the code

This repo is a class project. Code files start empty, with a docstring and a `TODO`.
- Do not write a file's full implementation unless someone explicitly asked you to.
- If asked for help, prefer explaining, reviewing or suggesting; let the person write.
- Do not fill in other areas' files all at once.

## Before touching anything

1. Read your area's guide in `agents/`.
2. If you change a topic or a message, update `docs/architecture.md` first.
3. Do not invent hardware measurements: if they are not in `docs/hardware.md` or in a YAML, leave a `# TODO measure`.

## Stack and commands

- Ubuntu 26.04, ROS 2 Lyrical, Python 3.12, `rclpy`.
- Build: `colcon build --symlink-install`
- Test: `colcon test && colcon test-result --verbose`
- Lint and format: `ruff check src tools scripts simulation/webots/controllers && ruff format --check src tools scripts simulation/webots/controllers` (`ruff format` fixes the layout)
- Tools tests (the browser tool is not a ROS package): `python3 -m pytest tools`

## Code rules

- Everything technical in English: identifiers, comments, docstrings, logs, states, config values and UI text.
- Every physical parameter (pins, speeds, thresholds, times) goes in YAML, with the unit in the name (`cmd_timeout_s`).
- Pure logic (kinematics, colour classification, state machine) goes in modules without `rclpy`, with tests in `test/`.
- Hardware access goes behind a class with a fake version, so it can be tested on the laptop.
- **Read [`docs/code-style.md`](docs/code-style.md) before writing code.** It is the reference for the rules below, with examples. Match the style of the file you edit; do not reformat or restructure code you were not asked to touch.
- No magic numbers or strings: a named constant (`UPPER_SNAKE_CASE`, unit as suffix, a comment saying why), or the YAML if it is tunable. Topic names live in `topics.py`, states in `states.py`. A default in the code equals the YAML value.
- Parameters are declared once, as dataclass fields (`recolector_common/params.py`); the field name is the ROS parameter name. Never `declare_parameter("...")` / `get_parameter("...")` with a string in a node.
- Every module, class and function has a docstring with the quotes on their own lines and the text at the same indentation (one-liners too):

  ```python
  def clamp(value, low, high):
      """
      Limit `value` to the closed interval [low, high].
      """
  ```
- Layout: at most 80 columns, and a call or literal that does not fit goes one item per line with a trailing comma (prefer the vertical form from three arguments up).
- Modularity: nodes are thin wiring (subscribe, call pure logic, publish). One responsibility per module and per function; complexity at most 10. A decision that does not need `rclpy` goes in its own module, with a test.
- Do not use Nav2, SLAM, Gazebo or MoveIt (course decision). RPi.GPIO does not work on the Pi 5.

## Robot safety

- Velocity goes through `obstacle_guard` before reaching `motor_driver`.
- `motor_driver` brakes if it stops receiving commands.
- 2S battery at most: the HAT handles 10.8 V.

## Git

- Branches `feature/<area>-<topic>`, `fix/…`, `docs/…`; never push to `main`.
- Commits `area: what`. Small PR with the template filled in.
- No `build/`, `install/`, `log/`, heavy photos or credentials.
