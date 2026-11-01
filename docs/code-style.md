# Code style

How we write code in this repository. It applies to everyone: the team, new contributors and AI assistants. `AGENTS.md` has the short version; this is the reference. The reasoning is in [ADR 0005](decisions/0005-shared-common-package-and-code-layout.md).

The goals, in order: **readable on a small screen**, **every fact written in one place**, **small pieces with one job each**.

## Checklist before you open a PR

- [ ] No number or string is buried in the code: it is a named constant or a YAML value.
- [ ] Every function, class and module has a docstring in the repo format.
- [ ] No line is longer than 80 columns; long calls and literals are one item per line.
- [ ] Each function does one thing; each module has one responsibility.
- [ ] Logic that does not need ROS or hardware is in its own module, with a test.
- [ ] Comments, docstrings, logs and names are in English.
- [ ] `ruff check` and `ruff format --check` are clean (see [Tooling](#tooling)).

## 1. Constants: nothing hardcoded

A value that appears in the code has a **name** and a **home**.

| It is... | It goes in |
|---|---|
| A physical or tuning value (speed, distance, time, threshold, pin) | the YAML, with the unit in the name, read through a parameter dataclass |
| A fixed fact of the module (a register address, a protocol field, a tolerance) | a constant at the top of that module |
| A topic, service or frame name | `topics.py` of the package |
| A state or a vocabulary word shared between nodes | `states.py` (an `Enum`, not loose strings) |
| A hardware device name (Webots, I2C address) | a constant next to the code that opens it |
| A value used by two packages | `recolector_common`, or the package that owns the concept |

**Rules**

- Names are `UPPER_SNAKE_CASE`, with the unit as a suffix: `_m`, `_s`, `_ms`, `_hz`, `_rad`, `_rad_s`, `_m_s`, `_px`. A comment says *why* the value is what it is when that is not obvious.
- Group constants at the top of the module under a short heading comment.
- `0`, `1`, `-1` and `2` are fine when they are plain arithmetic (identity, sign, halving). Anything else needs a name. `ruff` enforces this for comparisons (`PLR2004`); reviewers enforce the rest.
- A default that exists in the code and in the YAML must be the same value. The YAML wins at runtime; the code constant is only the fallback.
- Never invent a hardware measurement. If it is not in `docs/hardware.md` or a YAML, write `# TODO measure`.

```python
# Bad: what is 0.17? what is 0.5?
if cube.distance_m <= 0.17:
    vx *= 1.0 - error * 0.5

# Good
DEFAULT_STOP_DISTANCE_M = 0.17  # cube this far from the camera: stop
MAX_ALIGN_SLOWDOWN_FRACTION = 0.5  # at the alignment limit, lose half the speed

if cube.distance_m <= params.stop_distance_m:
    vx *= 1.0 - error * MAX_ALIGN_SLOWDOWN_FRACTION
```

## 2. Parameters: declared once

A ROS parameter is a **field of a dataclass**. The field name is the parameter name, the field value is the default. `recolector_common.params` declares and reads them, so the name is never typed twice.

```python
@dataclass
class ObstacleGuardNodeParams:
    """
    Node-level parameters (thresholds live in GuardParams).

    Field names double as ROS parameter names (see recolector_common.params).
    """

    range_timeout_s: float = 0.5
    require_heartbeat: bool = True


declare_params(self, ObstacleGuardNodeParams())
params = read_params(self, ObstacleGuardNodeParams())
```

- Grouped names (`camera.hfov_rad`) use `prefix="camera."`.
- To add a parameter: add the field, add the line to the YAML, done. No `declare_parameter("...")` or `get_parameter("...")` with a string in node code.
- Parameters that can change at runtime are reloaded in a post-set callback, not read on every tick.
- The one accepted exception to "no string names" is a family of parameters whose names depend on data, such as `hsv.<colour>` in `cube_detector`. Build the name from a constant prefix (`f"{HSV_PREFIX}{name}"`), never from a bare literal.

## 3. Modularization: where code goes

Think in three layers. Code only calls downward.

```
node file         wires ROS: topics, timers, parameters, logging        (needs rclpy)
pure logic        decisions, math, state machines, parsing              (no rclpy, no hardware)
hardware access   motors, camera, sensors, behind a class + a fake      (lazy import of the library)
```

- **Node files are thin.** A node subscribes, calls pure logic, publishes. If a node file has a decision that can be written without `rclpy`, move it out and test it.
- **One responsibility per module.** Example: `fleet.py` talks about the zone arbiter, `peer_avoidance.py` about the other robot; each node imports only the half it uses.
- **One job per function.** `ruff` caps complexity at 10 (`C901`). As a guide, a function that does not fit on one screen should be split; a name that needs "and" is two functions.
- **Name the condition.** Extract `_is_vision_stale(now)` instead of a three-line `if`.
- **Return early** instead of nesting.
- **Hardware behind a class with a fake**, so the node runs on the laptop (`FakeMotors`, `fake=True`). Import the hardware library inside the class that needs it (`from gpiozero import ...`), because it only exists on the Pi.
- **Threads:** shared state is guarded by one lock, and the lock is held for the shortest time possible.
- **No module-level state** other than constants.
- Prefer the standard library; add a dependency only for something a few lines cannot do.

Where things live today is in [`docs/architecture.md`](architecture.md#code-layout).

## 4. Multi-line layout

Lines are **at most 80 columns**, so the code reads on a small screen and side by side with a second file. `ruff format` does the wrapping; you control it with the **trailing comma**: when a call, signature or literal does not fit, or you want it vertical, put one item per line and end with a comma. The formatter keeps it exploded.

```python
# Bad: one long line, or a bunch of arguments packed together
self._pub = self.create_publisher(Twist, TOPIC_CMD_VEL_RAW, DEFAULT_QOS_DEPTH)

# Good
self._pub = self.create_publisher(
    Twist,
    TOPIC_CMD_VEL_RAW,
    DEFAULT_QOS_DEPTH,
)
```

- Prefer the vertical form when a call has three or more arguments, or when an argument is itself a call, dict or list.
- Long conditions get a named variable or a helper, not a wrapped `if a and b or c`:

  ```python
  at_zone = (
      zone.distance_m <= cfg.zone_stop_distance_m
      or zone.v >= cfg.zone_arrive_v_px
  )
  if at_zone and self._is_aligned(zone.bearing_rad):
      ...
  ```
- Long strings are split with implicit concatenation, one piece per line. Log messages stay f-strings (readable) in the node, but the fixed text of a topic or name is a constant.
- Imports are sorted by `ruff` (`I`); do not fight it.

## 5. Docstrings and comments

**Every module, class and function has a docstring**, including private ones and one-liners. The quotes go on their own lines and the text sits at the same indentation as the quotes:

```python
"""
Mecanum inverse kinematics (X-roller layout). Pure logic, no rclpy.

REP 103 convention: vx > 0 forward, vy > 0 left, wz > 0 counter-clockwise.
"""


class ZoneClient:
    """
    Client-side state for requesting, renewing and releasing the delivery
    square.
    """

    def should_request(self, now: float) -> bool:
        """
        True when the zone must be requested (or its lease renewed).
        """
```

- First line: what it is or does, in the third person ("Return...", "True when...", or a noun phrase). Then, after a blank line, whatever the reader needs: units, ranges, side effects, why.
- Say the unit and the convention of anything physical (`m/s`, positive = left).
- `__init__` says what the object starts as (`Start with no heartbeat received.`).
- Tests do not need docstrings: the test name says what it checks (`ruff` exempts them). Helpers and fakes in tests do get one when it is not obvious.
- **Comments explain why, not what.** `# Without a recent range, forward is blocked` is useful; `# increment i` is not. Keep them in English and under 80 columns.
- Use `# TODO (role N): ...` for pending work and `# TODO measure` for a missing measurement.

## 6. Tests

- Pure logic has a test in the package's `test/` (or `tools/test_*.py`). Fakes replace hardware and ROS, so tests run on the laptop with no robot.
- One behaviour per test, named for the behaviour (`test_stops_at_stop_distance_and_stays`).
- A bug fix comes with the test that fails without it.
- Tests may compare with literal numbers; that is what they are for.

## Tooling

```bash
ruff check src tools scripts simulation/webots/controllers
ruff format --check src tools scripts simulation/webots/controllers
colcon build --symlink-install && colcon test && colcon test-result --verbose
python3 -m pytest tools          # the browser tool is not a ROS package
```

`ruff format src tools scripts simulation/webots/controllers` fixes the layout for you. CI runs all of the above.

| What | Who checks it |
|---|---|
| 80 columns, layout, import order | `ruff format` / `ruff check` (`E`, `I`) |
| Docstring present and in the repo format | `ruff` (`D100`-`D107`, `D207`-`D210`, `D213`) |
| Magic numbers in comparisons | `ruff` (`PLR2004`) |
| Function complexity | `ruff` (`C901`, max 10) |
| Other magic values, names, constants in the right module | reviewer (`agents/reviewer.md`) |
| One responsibility per module, thin nodes | reviewer |

Do not turn `D200` on: it asks one-line docstrings to fit on one line, the opposite of our format.

## Adding something: the short path

*A new tunable value in a node* (for example a second timeout):

1. Add a field with the unit in its name to the node's parameter dataclass, with a `DEFAULT_...` constant.
2. Add the same value to the package's YAML.
3. Use `params.<field>` where you need it.

*A new node:*

1. Put the decision logic in a module without `rclpy` and test it.
2. Add `topics.py` entries for new topic names and `states.py` entries for new states.
3. Write the node as wiring: parameters from a dataclass, subscriptions that store, a timer that calls the logic and publishes.
4. Add the entry point to `setup.py` and the node to the launch file. A new `ament_python` package declares `extras_require={"test": ["pytest"]}` (not `tests_require`) so `colcon test` runs pytest. If a topic or message changed, update `docs/architecture.md` first.
