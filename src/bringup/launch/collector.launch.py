"""
Launches all collector nodes on the Raspberry Pi.

Camera, detector, web viewer, behavior_fsm, obstacle_guard, fleet_link and
the hardware nodes (motor_driver, line_sensors, ultrasonic, scoop_driver,
color_sensor).
For safety, behavior_fsm starts paused: start it with the "Start" button in
the web viewer or with enabled:=true.

Hardware not wired yet (pins at -1 in config/collector.yaml) runs as a fake
and warns. Without the HC-SR04 pins, obstacle_guard blocks forward motion.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

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
DEFAULT_ROBOT = "ferb"
DEFAULT_DEVICE = "0"
DEFAULT_PORT = "8080"
# Starts paused; see the module docstring.
DEFAULT_ENABLED = "false"
DEFAULT_REQUIRE_HEARTBEAT = "true"
DEFAULT_REQUIRE_POSE = "false"


def _config_path(package: str, filename: str) -> str:
    """
    Path of a YAML installed in a package's config directory.
    """
    share = get_package_share_directory(package)
    return os.path.join(share, CONFIG_DIR, filename)


def _bool_arg(name: str) -> ParameterValue:
    """
    A launch argument passed to a node as a boolean parameter.
    """
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def _node(package: str, executable: str, namespace, parameters: list) -> Node:
    """
    A node in the robot namespace that logs to the screen.
    """
    return Node(
        package=package,
        executable=executable,
        namespace=namespace,
        parameters=parameters,
        output="screen",
    )


def generate_launch_description():
    """
    Declare the launch arguments and every node of the collector.
    """
    vision_cfg = _config_path(PACKAGE_VISION, VISION_CONFIG_FILE)
    base_cfg = _config_path(PACKAGE_MOBILE_BASE, BASE_CONFIG_FILE)
    robot = LaunchConfiguration("robot")
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
            _node(
                PACKAGE_VISION,
                "camera_node",
                robot,
                [
                    vision_cfg,
                    {
                        "device": ParameterValue(
                            LaunchConfiguration("device"),
                            value_type=str,
                        )
                    },
                ],
            ),
            _node(PACKAGE_VISION, "cube_detector", robot, [vision_cfg]),
            _node(
                PACKAGE_VISION,
                "web_viewer",
                robot,
                [vision_cfg, {"port": LaunchConfiguration("port")}],
            ),
            _node(
                PACKAGE_MOBILE_BASE,
                "behavior_fsm",
                robot,
                [base_cfg, {"enabled": _bool_arg("enabled")}],
            ),
            _node(
                PACKAGE_MOBILE_BASE,
                "obstacle_guard",
                robot,
                [
                    base_cfg,
                    {
                        "require_heartbeat": _bool_arg("require_heartbeat"),
                        "require_pose": _bool_arg("require_pose"),
                    },
                ],
            ),
            _node(
                PACKAGE_MOBILE_BASE,
                "fleet_link",
                robot,
                [
                    base_cfg,
                    {"robot_name": ParameterValue(robot, value_type=str)},
                ],
            ),
            *[
                _node(PACKAGE_MOBILE_BASE, executable, robot, [base_cfg])
                for executable in HARDWARE_NODES
            ],
        ]
    )
