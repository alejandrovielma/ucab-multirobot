"""
Vision only on the real robot: camera, detector and web viewer. Does not move
the robot.

Useful to validate colour detection on the Pi before letting the robot move.

  ros2 launch bringup vision.launch.py            # http://<pi-ip>:8080
  ros2 launch bringup vision.launch.py device:=1  # camera /dev/video1
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PACKAGE_VISION = "vision"
VISION_CONFIG_FILE = "hsv_ranges.yaml"
CONFIG_DIR = "config"

DEFAULT_DEVICE = "0"
DEFAULT_PORT = "8080"


def _node(executable: str, parameters: list) -> Node:
    """
    A vision node that logs to the screen.
    """
    return Node(
        package=PACKAGE_VISION,
        executable=executable,
        parameters=parameters,
        output="screen",
    )


def generate_launch_description():
    """
    Declare the launch arguments and the three vision nodes.
    """
    share = get_package_share_directory(PACKAGE_VISION)
    cfg = os.path.join(share, CONFIG_DIR, VISION_CONFIG_FILE)
    return LaunchDescription(
        [
            DeclareLaunchArgument("device", default_value=DEFAULT_DEVICE),
            DeclareLaunchArgument("port", default_value=DEFAULT_PORT),
            _node(
                "camera_node",
                [
                    cfg,
                    {
                        "device": ParameterValue(
                            LaunchConfiguration("device"),
                            value_type=str,
                        )
                    },
                ],
            ),
            _node("cube_detector", [cfg]),
            _node(
                "web_viewer",
                [cfg, {"port": LaunchConfiguration("port")}],
            ),
        ]
    )
