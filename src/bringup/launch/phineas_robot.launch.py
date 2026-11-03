"""
Launches the Phineas collector ready for multi-robot track operation.

Thin wrapper around `collector.launch.py` with `enabled:=true` and the agreed
track defaults:
  robot:=phineas
  require_pose:=false
  require_heartbeat:=true
  use_lease:=true
  targets:=[blue]
  avoid:=[red]
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

COLLECTOR_LAUNCH_FILE = "collector.launch.py"

DEFAULT_ROBOT = "phineas"
DEFAULT_DEVICE = "0"
DEFAULT_PORT = "8080"
DEFAULT_ENABLED = "true"
DEFAULT_REQUIRE_POSE = "false"
DEFAULT_REQUIRE_HEARTBEAT = "true"
DEFAULT_USE_LEASE = "true"
DEFAULT_TARGETS = "[blue]"
DEFAULT_AVOID = "[red]"

FORWARDED_ARGS = (
    "robot",
    "device",
    "port",
    "enabled",
    "require_pose",
    "require_heartbeat",
    "use_lease",
    "targets",
    "avoid",
)


def _collector_launch_path() -> str:
    """
    Absolute path to `collector.launch.py` next to this file.
    """
    here = os.path.dirname(os.path.realpath(__file__))
    return os.path.join(here, COLLECTOR_LAUNCH_FILE)


def generate_launch_description():
    """
    Declare Phineas track arguments and forward them to `collector.launch.py`.
    """
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot", default_value=DEFAULT_ROBOT),
            DeclareLaunchArgument("device", default_value=DEFAULT_DEVICE),
            DeclareLaunchArgument("port", default_value=DEFAULT_PORT),
            DeclareLaunchArgument("enabled", default_value=DEFAULT_ENABLED),
            DeclareLaunchArgument(
                "require_pose",
                default_value=DEFAULT_REQUIRE_POSE,
            ),
            DeclareLaunchArgument(
                "require_heartbeat",
                default_value=DEFAULT_REQUIRE_HEARTBEAT,
            ),
            DeclareLaunchArgument("use_lease", default_value=DEFAULT_USE_LEASE),
            DeclareLaunchArgument("targets", default_value=DEFAULT_TARGETS),
            DeclareLaunchArgument("avoid", default_value=DEFAULT_AVOID),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(_collector_launch_path()),
                launch_arguments={
                    name: LaunchConfiguration(name) for name in FORWARDED_ARGS
                }.items(),
            ),
        ]
    )
