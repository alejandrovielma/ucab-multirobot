"""
Launches all collector nodes on the Raspberry Pi.

Camera, detector, web viewer, behavior_fsm, obstacle_guard, fleet_link and
the hardware nodes (motor_driver, line_sensors, ultrasonic, scoop_driver,
color_sensor).
For safety, behavior_fsm starts paused by default (`enabled:=false`): start it
with the "Start" button in the web viewer, with `enabled:=true`, or through
`phineas_robot.launch.py`.

Operational fleet defaults for Phineas:
  robot:=phineas
  require_pose:=false
  require_heartbeat:=true
  use_lease:=true (alias of require_zone_grant:=true)
  targets:=[blue]
  avoid:=[red]
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE_VISION = "vision"
PACKAGE_MOBILE_BASE = "mobile_base"
VISION_CONFIG_FILE = "hsv_ranges.yaml"
BASE_CONFIG_FILE = "collector.yaml"
CONFIG_DIR = "config"
# Nodes that only need collector.yaml.
HARDWARE_NODES = (
    "motor_driver",
    "line_sensors",
    "ultrasonic",
    "scoop_driver",
    "color_sensor",
)

# Launch arguments and their defaults.
DEFAULT_ROBOT = "phineas"
DEFAULT_DEVICE = "0"
DEFAULT_PORT = "8080"
# Starts paused; see the module docstring.
DEFAULT_ENABLED = "false"
DEFAULT_REQUIRE_HEARTBEAT = "true"
DEFAULT_REQUIRE_POSE = "false"
DEFAULT_REQUIRE_ZONE_GRANT = "true"
# Optional alias of require_zone_grant (empty = defer to require_zone_grant).
DEFAULT_USE_LEASE = ""
DEFAULT_TARGETS = "[blue]"
DEFAULT_AVOID = "[red]"

TRUE_VALUES = ("true", "1", "yes", "on")


def _config_path(package: str, filename: str) -> str:
    """
    Path of a YAML installed in a package's config directory.
    """
    share = get_package_share_directory(package)
    return os.path.join(share, CONFIG_DIR, filename)


def _parse_bool(raw: str) -> bool:
    """
    Convert a launch argument string into a boolean.
    """
    return raw.strip().lower() in TRUE_VALUES


def _parse_colors(raw: str) -> list[str]:
    """
    Parse a bracketed or comma-separated colour list such as `[blue]` or
    `blue,yellow`.
    """
    cleaned = raw.strip().strip("[]")
    items = (part.strip().strip("'\"") for part in cleaned.split(","))
    return [item for item in items if item]


def _resolve_targets(targets_raw: str, avoid_raw: str) -> list[str]:
    """
    Build the `target_colors` list after removing any colours in `avoid`.
    """
    avoided = set(_parse_colors(avoid_raw))
    return [c for c in _parse_colors(targets_raw) if c not in avoided]


def _resolve_lease(require_grant_raw: str, use_lease_raw: str) -> bool:
    """
    Resolve `require_zone_grant`, allowing `use_lease` as a CLI alias.
    """
    if use_lease_raw.strip():
        return _parse_bool(use_lease_raw)
    return _parse_bool(require_grant_raw)


def _node(package: str, executable: str, namespace: str, params: list) -> Node:
    """
    A node in the robot namespace that logs to the screen.
    """
    return Node(
        package=package,
        executable=executable,
        namespace=namespace,
        parameters=params,
        output="screen",
    )


def _setup(context):
    """
    Resolve the launch arguments and construct all collector nodes.
    """
    vision_cfg = _config_path(PACKAGE_VISION, VISION_CONFIG_FILE)
    base_cfg = _config_path(PACKAGE_MOBILE_BASE, BASE_CONFIG_FILE)
    robot = LaunchConfiguration("robot").perform(context)
    device = LaunchConfiguration("device").perform(context)
    port = int(LaunchConfiguration("port").perform(context))
    enabled = _parse_bool(LaunchConfiguration("enabled").perform(context))
    require_hb = _parse_bool(
        LaunchConfiguration("require_heartbeat").perform(context),
    )
    require_pose = _parse_bool(
        LaunchConfiguration("require_pose").perform(context),
    )
    require_grant = _resolve_lease(
        LaunchConfiguration("require_zone_grant").perform(context),
        LaunchConfiguration("use_lease").perform(context),
    )
    target_colors = _resolve_targets(
        LaunchConfiguration("targets").perform(context),
        LaunchConfiguration("avoid").perform(context),
    )
    return [
        _node(
            PACKAGE_VISION,
            "camera_node",
            robot,
            [vision_cfg, {"device": str(device)}],
        ),
        _node(PACKAGE_VISION, "cube_detector", robot, [vision_cfg]),
        _node(
            PACKAGE_VISION,
            "web_viewer",
            robot,
            [vision_cfg, {"port": port}],
        ),
        _node(
            PACKAGE_MOBILE_BASE,
            "behavior_fsm",
            robot,
            [
                base_cfg,
                {
                    "enabled": enabled,
                    "require_zone_grant": require_grant,
                    "target_colors": target_colors,
                },
            ],
        ),
        _node(
            PACKAGE_MOBILE_BASE,
            "obstacle_guard",
            robot,
            [
                base_cfg,
                {
                    "require_heartbeat": require_hb,
                    "require_pose": require_pose,
                },
            ],
        ),
        _node(
            PACKAGE_MOBILE_BASE,
            "fleet_link",
            robot,
            [base_cfg, {"robot_name": str(robot)}],
        ),
        *[
            _node(PACKAGE_MOBILE_BASE, executable, robot, [base_cfg])
            for executable in HARDWARE_NODES
        ],
    ]


def generate_launch_description():
    """
    Declare the launch arguments and every node of the collector.
    """
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot", default_value=DEFAULT_ROBOT),
            DeclareLaunchArgument("device", default_value=DEFAULT_DEVICE),
            DeclareLaunchArgument("port", default_value=DEFAULT_PORT),
            DeclareLaunchArgument("enabled", default_value=DEFAULT_ENABLED),
            DeclareLaunchArgument(
                "require_heartbeat",
                default_value=DEFAULT_REQUIRE_HEARTBEAT,
            ),
            DeclareLaunchArgument(
                "require_pose",
                default_value=DEFAULT_REQUIRE_POSE,
            ),
            DeclareLaunchArgument(
                "require_zone_grant",
                default_value=DEFAULT_REQUIRE_ZONE_GRANT,
            ),
            DeclareLaunchArgument("use_lease", default_value=DEFAULT_USE_LEASE),
            DeclareLaunchArgument("targets", default_value=DEFAULT_TARGETS),
            DeclareLaunchArgument("avoid", default_value=DEFAULT_AVOID),
            OpaqueFunction(function=_setup),
        ]
    )
