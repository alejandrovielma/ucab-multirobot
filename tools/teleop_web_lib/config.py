"""
YAML loading and typed settings for the browser tool. No hardware.

The real values live in tools/teleop_web.yaml. The defaults below are only
the fallback when a key is missing, so a typo cannot silently change the
robot's behaviour: unknown keys raise at startup.
"""

from dataclasses import dataclass, fields
from pathlib import Path

# --- Auto mode defaults (real values: tools/teleop_web.yaml, section auto) ----
DEFAULT_STOP_DISTANCE_M = 0.125
DEFAULT_SLOW_DOWN_DISTANCE_M = 0.45
DEFAULT_FORWARD_SPEED = 0.42
DEFAULT_ROAM_FORWARD_SPEED = 0.35
DEFAULT_TURN_SPEED = 0.32
DEFAULT_SEARCH_TURN_SPEED = 0.25
DEFAULT_ALIGN_TOLERANCE_RAD = 0.22
DEFAULT_LOST_TIMEOUT_S = 0.6
DEFAULT_NO_CUBES_STOP_S = 60.0
DEFAULT_SCOOP_PUSH_S = 0.5
DEFAULT_LINE_BACKUP_S = 0.5
DEFAULT_LINE_TURN_S = 1.2
# Brake pause as soon as the black border is seen, before backing up.
DEFAULT_LINE_STOP_S = 0.2
# Longest time backing up to get off the line (avoids sliding along the border).
DEFAULT_LINE_BACKUP_MAX_S = 0.9
# How recent the last movement must be for the line recovery to undo it.
DEFAULT_LINE_MOTION_MEMORY_S = 1.5
DEFAULT_LINE_SPEED = 0.35
# Only the landmarks in view: slow down this far from them (no delivery).
DEFAULT_ZONE_STOP_DISTANCE_M = 0.59
# Area paper in view: arrived when its near edge is this close or this low.
DEFAULT_FLOOR_ARRIVE_V_PX = 436
DEFAULT_DELIVER_TURN_S = 2.0
# Image row (px) from which a cube counts as at the scoop threshold.
DEFAULT_SCOOP_ARRIVE_V_PX = 436
# The catch / drop alignment tolerance is align_tolerance_rad times this.
DEFAULT_CATCH_ALIGN_FACTOR = 1.4
# Roaming: drive forward for `roam_forward_s` out of every `roam_period_s`,
# and turn to scan the rest of the time.
DEFAULT_ROAM_PERIOD_S = 8.0
DEFAULT_ROAM_FORWARD_S = 1.0
# Largest sideways correction while driving toward a target (0-1).
DEFAULT_STEER_LIMIT = 0.7
# Slowest approach speed, as a fraction of forward_speed.
DEFAULT_MIN_SPEED_FRACTION = 0.35
# Smallest slow-down span (m), avoids dividing by a near-zero distance.
DEFAULT_MIN_SLOW_SPAN_M = 0.05
# A cube seen and then lost from view may be under the scoop (blind spot):
# back up this long to uncover it. 0 turns it off. Measured: 0.6 s at
# line_speed backs up ~13 cm; a cube at 10 cm from the scoop was invisible
# and showed up whole afterwards (it is lost from view below ~12 cm).
DEFAULT_REVEAL_BACKUP_S = 0.6
# Back up to uncover a lost cube only if it was last seen this close (m): a
# far cube that flickers out of view is not hidden under the scoop.
DEFAULT_REVEAL_MAX_DISTANCE_M = 0.35
# After losing sight of a cube, keep heading where it was last seen for this
# long before going back to roaming.
DEFAULT_REACQUIRE_S = 1.5
# A line hit while going for a cube or carrying one means the cube is at the
# border: ignore cubes for this long so the robot goes elsewhere. TODO measure
DEFAULT_LINE_CUBE_BAN_S = 8.0
# Closer than this to its target, the alignment tolerance is widened by the
# factor below (a few cm sideways is already a big angle up close). Keep the
# factor under catch_align_factor, or the robot would drive up without
# ever being allowed to catch. TODO measure
DEFAULT_CLOSE_ALIGN_DISTANCE_M = 0.30
DEFAULT_CLOSE_ALIGN_FACTOR = 1.3
# Delivery into the area paper (blue sheet ~17 cm deep, entered across the
# pink line): the arrival is when its near edge is ~19 cm from the camera and
# the cube sits ~12 cm ahead of it, so ~15 cm of push puts the cube inside.
# Forward at 0.42 is ~0.26 m/s (backing at 0.35 measured ~13 cm in 0.6 s), so
# that is ~0.6 s; back up about as far and a little more. The landmark gate
# keeps the longer deliver_* times. TODO measure
DEFAULT_FLOOR_DELIVER_BACKUP_S = 1.0
# Once turning to align, keep turning until the bearing error is below this
# fraction of the tolerance; otherwise the error hovering around the tolerance
# flips between turning and driving every tick (seen: 12 flips in 6 s).
DEFAULT_ALIGN_EXIT_FRACTION = 0.5
# Delivery needs the robot to face the area squarely: legos stand at both
# sides of it and a crooked push knocks them down. TODO measure
DEFAULT_DROP_ALIGN_TOLERANCE_RAD = 0.12
# Carrying a cube without ever seeing the delivery gate for this long: assume
# the cube slipped out of the scoop. TODO measure
DEFAULT_CARRY_ZONE_TIMEOUT_S = 15.0
# Without a cube in view, do not roam forward with the gate this close.
# TODO measure
DEFAULT_GATE_AVOID_DISTANCE_M = 0.35
# While carrying, the area out of view for less than this (s) only stops the
# robot: it flickers, and turning right away makes the robot turn back later.
DEFAULT_CARRY_ZONE_HOLD_S = 0.0
# Cubes the mission says to avoid: drive around one that is closer than
# foreign_avoid_distance_m (m) and within foreign_clearance_m (m) of the
# straight path. Once dodging, keep sliding until it is foreign_exit_margin_m
# (m) farther out, so the robot does not flicker at the limit.
DEFAULT_FOREIGN_AVOID_DISTANCE_M = 0.30
DEFAULT_FOREIGN_CLEARANCE_M = 0.12
DEFAULT_FOREIGN_EXIT_MARGIN_M = 0.03
# Delivery: with the paper's near edge this close (m), face the pink entry
# line squarely (within entry_square_tolerance_rad), slide sideways at
# strafe_speed until the paper's centre is within drop_align_tolerance_rad,
# then push the cube straight in, deliver_insert_m past that edge. Forward at
# forward_speed covers forward_speed_mps; the cube's centre sits
# cube_ahead_m ahead of the lens. TODO measure
DEFAULT_DELIVER_COMMIT_DISTANCE_M = 0.20
DEFAULT_DELIVER_INSERT_M = 0.07
DEFAULT_FORWARD_SPEED_MPS = 0.31
DEFAULT_CUBE_AHEAD_M = 0.022
DEFAULT_ENTRY_SQUARE_TOLERANCE_RAD = 0.10
DEFAULT_STRAFE_SPEED = 0.40
# A cube this close (m) after backing up from a delivery was left outside
# the area: the delivery does not count. TODO measure
DEFAULT_DROP_VERIFY_DISTANCE_M = 0.30
# ... and within this bearing (rad) of straight ahead: a cube off to a side
# is another one on the table, not the delivered one. TODO measure
DEFAULT_DROP_VERIFY_BEARING_RAD = 0.2
# After the blind push into the scoop, stop this long (s) and check: a cube
# still in view this close (m) did not go in, so it is not counted.
# TODO measure
DEFAULT_CATCH_CHECK_S = 0.3
DEFAULT_CATCH_CHECK_DISTANCE_M = 0.20
# Delivery push guided by the paper's near edge: drive while it is farther
# than deliver_blind_edge_m (m) and seen within deliver_edge_timeout_s (s),
# at most deliver_push_max_s (s); then the blind rest, timed with the speed
# measured over at least deliver_speed_min_s (s) of the edge coming closer.
# TODO measure
DEFAULT_DELIVER_BLIND_EDGE_M = 0.12
# Squaring up at the paper, stay committed while it reads up to this much
# (m) beyond deliver_commit_distance_m, and wait still this long (s) when it
# drops out of view instead of turning to the landmarks. TODO measure
DEFAULT_DELIVER_COMMIT_HYSTERESIS_M = 0.08
DEFAULT_CARRY_PAPER_HOLD_S = 1.0
# Squaring up from a diagonal: with the paper's centre farther than this
# (rad) off straight ahead, slide toward it before turning (turning in place
# pushed it out of view). If it drops out anyway, slide toward where it was
# last seen for up to carry_paper_slide_s (s) after the hold. TODO measure
DEFAULT_SQUARE_MAX_BEARING_RAD = 0.30
DEFAULT_CARRY_PAPER_SLIDE_S = 2.0
# Delivery push: stop once the area's far edge is this close (m); 0 = do not
# use the far edge. TODO measure
DEFAULT_DELIVER_FAR_STOP_M = 0.142
DEFAULT_DELIVER_FAR_MIN_SPAN_M = 0.0
DEFAULT_ASSUME_IN_SCOOP_DISTANCE_M = 0.0
# With the paper's near edge this close (m) the scoop hides the pink entry
# line: deliver without squaring instead of waiting for it. TODO measure
DEFAULT_ENTRY_HIDDEN_DISTANCE_M = 0.15
DEFAULT_DELIVER_EDGE_TIMEOUT_S = 0.3
DEFAULT_DELIVER_PUSH_MAX_S = 3.0
DEFAULT_DELIVER_SPEED_MIN_S = 0.3

# --- Vision filter used by this tool (section `detection` overrides) ----------
# Looser than the ROS cube_detector because this tool's camera is not
# calibrated as tightly and the scoop is allowed in view.
DEFAULT_DETECTION_FILTER = {
    "min_area_px": 80,
    "min_size_ratio": 0.50,
    "max_size_ratio": 2.10,
    "ignore_touching_bottom": False,
    "max_distance_m": 1.2,
}

# The vision package is plain Python: it is imported straight from the repo,
# without building the ROS workspace.
REPO_ROOT = Path(__file__).resolve().parents[2]
VISION_PACKAGE_DIR = REPO_ROOT / "src" / "vision"
HSV_CONFIG_PATH = VISION_PACKAGE_DIR / "config" / "hsv_ranges.yaml"
HSV_NODE_NAME = "cube_detector"

# Key of the parameters of a node inside a ROS parameter file.
ROS_PARAMS_KEY = "ros__parameters"

# Section names of tools/teleop_web.yaml.
SECTION_MOTORS = "motors"
SECTION_CAMERA = "camera"
SECTION_SERVER = "server"
SECTION_LINE_SENSOR = "line_sensor"
SECTION_AUTO = "auto"
SECTION_DETECTION = "detection"
SECTION_OVERHEAD = "overhead"
SECTION_OVERHEAD_2 = "overhead_2"


@dataclass(frozen=True)
class AutoConfig:
    """
    Settings of the Auto mode; field names are the YAML keys.
    """

    enabled: bool = False
    stop_distance_m: float = DEFAULT_STOP_DISTANCE_M
    slow_down_distance_m: float = DEFAULT_SLOW_DOWN_DISTANCE_M
    forward_speed: float = DEFAULT_FORWARD_SPEED
    roam_forward_speed: float = DEFAULT_ROAM_FORWARD_SPEED
    turn_speed: float = DEFAULT_TURN_SPEED
    search_turn_speed: float = DEFAULT_SEARCH_TURN_SPEED
    align_tolerance_rad: float = DEFAULT_ALIGN_TOLERANCE_RAD
    lost_timeout_s: float = DEFAULT_LOST_TIMEOUT_S
    no_cubes_stop_s: float = DEFAULT_NO_CUBES_STOP_S
    scoop_push_s: float = DEFAULT_SCOOP_PUSH_S
    line_backup_s: float = DEFAULT_LINE_BACKUP_S
    line_turn_s: float = DEFAULT_LINE_TURN_S
    line_stop_s: float = DEFAULT_LINE_STOP_S
    line_backup_max_s: float = DEFAULT_LINE_BACKUP_MAX_S
    line_motion_memory_s: float = DEFAULT_LINE_MOTION_MEMORY_S
    line_speed: float = DEFAULT_LINE_SPEED
    zone_stop_distance_m: float = DEFAULT_ZONE_STOP_DISTANCE_M
    floor_arrive_v_px: int = DEFAULT_FLOOR_ARRIVE_V_PX
    deliver_turn_s: float = DEFAULT_DELIVER_TURN_S
    scoop_arrive_v_px: int = DEFAULT_SCOOP_ARRIVE_V_PX
    catch_align_factor: float = DEFAULT_CATCH_ALIGN_FACTOR
    roam_period_s: float = DEFAULT_ROAM_PERIOD_S
    roam_forward_s: float = DEFAULT_ROAM_FORWARD_S
    steer_limit: float = DEFAULT_STEER_LIMIT
    min_speed_fraction: float = DEFAULT_MIN_SPEED_FRACTION
    min_slow_span_m: float = DEFAULT_MIN_SLOW_SPAN_M
    reveal_backup_s: float = DEFAULT_REVEAL_BACKUP_S
    reveal_max_distance_m: float = DEFAULT_REVEAL_MAX_DISTANCE_M
    reacquire_s: float = DEFAULT_REACQUIRE_S
    line_cube_ban_s: float = DEFAULT_LINE_CUBE_BAN_S
    align_exit_fraction: float = DEFAULT_ALIGN_EXIT_FRACTION
    drop_align_tolerance_rad: float = DEFAULT_DROP_ALIGN_TOLERANCE_RAD
    floor_deliver_backup_s: float = DEFAULT_FLOOR_DELIVER_BACKUP_S
    close_align_distance_m: float = DEFAULT_CLOSE_ALIGN_DISTANCE_M
    close_align_factor: float = DEFAULT_CLOSE_ALIGN_FACTOR
    carry_zone_timeout_s: float = DEFAULT_CARRY_ZONE_TIMEOUT_S
    gate_avoid_distance_m: float = DEFAULT_GATE_AVOID_DISTANCE_M
    carry_zone_hold_s: float = DEFAULT_CARRY_ZONE_HOLD_S
    foreign_avoid_distance_m: float = DEFAULT_FOREIGN_AVOID_DISTANCE_M
    foreign_clearance_m: float = DEFAULT_FOREIGN_CLEARANCE_M
    foreign_exit_margin_m: float = DEFAULT_FOREIGN_EXIT_MARGIN_M
    deliver_commit_distance_m: float = DEFAULT_DELIVER_COMMIT_DISTANCE_M
    deliver_insert_m: float = DEFAULT_DELIVER_INSERT_M
    forward_speed_mps: float = DEFAULT_FORWARD_SPEED_MPS
    cube_ahead_m: float = DEFAULT_CUBE_AHEAD_M
    entry_square_tolerance_rad: float = DEFAULT_ENTRY_SQUARE_TOLERANCE_RAD
    strafe_speed: float = DEFAULT_STRAFE_SPEED
    drop_verify_distance_m: float = DEFAULT_DROP_VERIFY_DISTANCE_M
    drop_verify_bearing_rad: float = DEFAULT_DROP_VERIFY_BEARING_RAD
    catch_check_s: float = DEFAULT_CATCH_CHECK_S
    catch_check_distance_m: float = DEFAULT_CATCH_CHECK_DISTANCE_M
    deliver_blind_edge_m: float = DEFAULT_DELIVER_BLIND_EDGE_M
    deliver_commit_hysteresis_m: float = DEFAULT_DELIVER_COMMIT_HYSTERESIS_M
    carry_paper_hold_s: float = DEFAULT_CARRY_PAPER_HOLD_S
    square_max_bearing_rad: float = DEFAULT_SQUARE_MAX_BEARING_RAD
    carry_paper_slide_s: float = DEFAULT_CARRY_PAPER_SLIDE_S
    deliver_far_stop_m: float = DEFAULT_DELIVER_FAR_STOP_M
    deliver_far_min_span_m: float = DEFAULT_DELIVER_FAR_MIN_SPAN_M
    assume_in_scoop_distance_m: float = DEFAULT_ASSUME_IN_SCOOP_DISTANCE_M
    entry_hidden_distance_m: float = DEFAULT_ENTRY_HIDDEN_DISTANCE_M
    deliver_edge_timeout_s: float = DEFAULT_DELIVER_EDGE_TIMEOUT_S
    deliver_push_max_s: float = DEFAULT_DELIVER_PUSH_MAX_S
    deliver_speed_min_s: float = DEFAULT_DELIVER_SPEED_MIN_S

    @classmethod
    def from_dict(cls, values: dict | None) -> "AutoConfig":
        """
        Build the settings from the YAML `auto` section.

        Missing keys keep their default; unknown keys raise ValueError.
        """
        values = values or {}
        defaults = cls()
        known = {field.name for field in fields(cls)}
        unknown = sorted(set(values) - known)
        if unknown:
            raise ValueError(f"unknown settings in section auto: {unknown}")
        return cls(
            **{
                name: _coerce_like(getattr(defaults, name), value)
                for name, value in values.items()
            }
        )


def _coerce_like(default, value):
    """
    Convert `value` to the type of `default` (bool, int or float).
    """
    return type(default)(value)


def load_config(path: Path) -> dict:
    """
    Read the YAML file into a dictionary.
    """
    # Imported here: PyYAML is only needed when the tool actually runs.
    import yaml

    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def read_node_params(path: Path, node_name: str) -> dict:
    """
    The `ros__parameters` of `node_name` in a ROS parameter YAML.

    The node key may carry a namespace wildcard (`/**/cube_detector`), so it
    is matched by its last path segment.
    """
    document = load_config(path)
    for key, value in document.items():
        if key.rsplit("/", 1)[-1] == node_name:
            return value[ROS_PARAMS_KEY]
    raise KeyError(f"no section for node {node_name!r} in {path}")
