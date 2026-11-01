# 0005. Shared `recolector_common` package and code layout rules

**Status:** accepted (24/10/2026)

## Context
An audit of the code found three problems that made it hard to read and to change safely:

- Some files mixed many responsibilities (`tools/teleop_web.py` had 875 lines and eight of them; `web_viewer.py`, `color_detection.py` and `fleet.py` were also oversized).
- Values were hardcoded: topic names, states, rates, device names, tuning times and thresholds. Some node defaults did not match the YAML (`camera.height_m`, `camera.tilt_rad`).
- The same boilerplate was repeated in every node (`main()`, the clock helper, `declare_parameter` followed by `get_parameter`).

## Decision
- A new ament_python package, `recolector_common`, holds what more than one package needs:
  - `params.py`: pure logic (no `rclpy`) that declares and reads ROS parameters from a dataclass. The field name is the parameter name; an optional `prefix` gives grouped names such as `camera.hfov_rad`.
  - `node_utils.py`: `now_s(node)` and `spin_node(node_factory)`.
- `mobile_base` and `vision` depend on it. It depends on nothing of ours, so there are no cycles.
- Code layout rules (also in `AGENTS.md`, enforced by `ruff` where possible):
  - One responsibility per module and per function; complexity at most 10 (C901).
  - No magic numbers or strings: named constants, or the YAML when tunable (PLR2004 for comparisons). Topic names live in each package's `topics.py` and states in `mobile_base/states.py`.
  - A node's default must equal the YAML value or come from the same constant.
  - 80 columns, one argument or item per line once it does not fit, trailing commas.
  - Docstrings with the quotes on their own lines and the text at the same indentation, for every module, class and function.
- `tools/teleop_web.py` stays as the entry point; its code moved to `tools/teleop_web_lib/`.

## Consequences
- A parameter is named once. Adding one means adding a dataclass field and a YAML line.
- `PeerAvoidance` moved from `fleet.py` to `peer_avoidance.py`: `obstacle_guard` and `fleet_link` each use a disjoint half of the old file. Its distance fields are now `peer_slow_distance_m` and `peer_stop_distance_m`, the same names as the YAML.
- A new ROS package must be listed as an `exec_depend` by its users, and `rosdep install` picks it up from `src/`.
- Every `ament_python` package declares `extras_require={"test": ["pytest"]}` in `setup.py` (and `python3-pytest` as `test_depend`). Do not use `tests_require`: recent setuptools drops it, colcon then does not see pytest and falls back to `unittest`, which finds no tests and exits with code 5 (this broke `colcon test` in CI).
- `tools/teleop_web_lib/` still has its own small kinematics module: that tool works without ROS and uses normalised ratios, not rad/s. It is meant to disappear when `motor_driver` exists.
