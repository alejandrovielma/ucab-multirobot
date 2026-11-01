"""
Launches the collector nodes connected to Webots.

Instead of webots_ros2 we use our own bridge (recolector_sim/webots_bridge),
which is the <extern> controller of the "Ferb" robot (us) and publishes the
same topics as the real robot.

Arguments:
  webots      headless (default, no window, for Docker), gui or none
              (none = Webots is already open, for example on Windows)
  webots_url  which Webots to connect to: ipc://1234/Ferb (same machine) or
              tcp://<ip>:1234/Ferb (Webots on another machine or outside
              Docker)
  world       world to open
              (default simulation/webots/worlds/pista_ucab_ros.wbt)
  port        web viewer port (default 8080)

Example:
  ros2 launch bringup simulation.launch.py webots:=gui
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE_VISION = "vision"
PACKAGE_MOBILE_BASE = "mobile_base"
PACKAGE_SIM = "recolector_sim"
VISION_CONFIG_FILE = "hsv_ranges.yaml"
BASE_CONFIG_FILE = "collector.yaml"
CONFIG_DIR = "config"

# --- Launch argument values ---------------------------------------------------
MODE_HEADLESS = "headless"
MODE_NONE = "none"
DEFAULT_WEBOTS_URL = "ipc://1234/Ferb"
DEFAULT_PORT = "8080"
WORLD_FILE = "pista_ucab_ros.wbt"

# --- Webots installation ------------------------------------------------------
DEFAULT_WEBOTS_HOME = "/usr/local/webots"
CONTROLLER_LIB_DIR = ("lib", "controller")
CONTROLLER_PYTHON_DIR = "python"

# --- Webots command line ------------------------------------------------------
WEBOTS_COMMAND = [
    "webots",
    "--batch",
    "--mode=realtime",
    "--stdout",
    "--stderr",
]
HEADLESS_FLAGS = ["--no-rendering", "--minimize"]
XVFB_COMMAND = ["xvfb-run", "-a", "-s", "-screen 0 1280x720x24"]


def _default_world() -> str:
    """
    Default world file inside the repository.
    """
    # With --symlink-install this file points into the repo:
    # src/bringup/launch/ -> repo root.
    here = os.path.dirname(os.path.realpath(__file__))
    repo = os.path.realpath(os.path.join(here, "../../.."))
    return os.path.join(repo, "simulation", "webots", "worlds", WORLD_FILE)


def _config_path(package: str, filename: str) -> str:
    """
    Path of a YAML installed in a package's config directory.
    """
    share = get_package_share_directory(package)
    return os.path.join(share, CONFIG_DIR, filename)


def _bridge_env(webots_url: str) -> dict:
    """
    Environment the bridge needs to load the Webots controller library.
    """
    webots_home = os.environ.get("WEBOTS_HOME", DEFAULT_WEBOTS_HOME)
    controller_lib = os.path.join(webots_home, *CONTROLLER_LIB_DIR)
    python_path = os.environ.get("PYTHONPATH", "")
    library_path = os.environ.get("LD_LIBRARY_PATH", "")
    return {
        "PYTHONPATH": os.pathsep.join(
            p
            for p in (
                os.path.join(controller_lib, CONTROLLER_PYTHON_DIR),
                python_path,
            )
            if p
        ),
        "LD_LIBRARY_PATH": os.pathsep.join(
            p for p in (controller_lib, library_path) if p
        ),
        "WEBOTS_CONTROLLER_URL": webots_url,
        "WEBOTS_HOME": webots_home,
    }


def _webots_actions(mode: str, world: str) -> list:
    """
    Actions that start Webots (and shut everything down when it exits), or
    nothing when Webots is already open.
    """
    if mode == MODE_NONE:
        return []
    if not os.path.isfile(world):
        raise RuntimeError(
            f"world {world} does not exist; pass it with world:=<path>"
        )
    command = list(WEBOTS_COMMAND)
    if mode == MODE_HEADLESS:
        command = [*XVFB_COMMAND, *command, *HEADLESS_FLAGS]
    webots = ExecuteProcess(
        cmd=[*command, world], name="webots", output="screen"
    )
    return [
        webots,
        # If Webots closes, everything shuts down.
        RegisterEventHandler(
            OnProcessExit(
                target_action=webots,
                on_exit=[EmitEvent(event=Shutdown())],
            )
        ),
    ]


def _collector_nodes(port: int, bridge_env: dict) -> list:
    """
    The bridge plus the same collector nodes that run on the real robot.
    """
    vision_cfg = _config_path(PACKAGE_VISION, VISION_CONFIG_FILE)
    base_cfg = _config_path(PACKAGE_MOBILE_BASE, BASE_CONFIG_FILE)
    return [
        Node(
            package=PACKAGE_SIM,
            executable="webots_bridge",
            output="screen",
            additional_env=bridge_env,
        ),
        Node(
            package=PACKAGE_VISION,
            executable="cube_detector",
            parameters=[vision_cfg],
            output="screen",
        ),
        Node(
            package=PACKAGE_VISION,
            executable="web_viewer",
            parameters=[vision_cfg, {"port": port}],
            output="screen",
        ),
        Node(
            package=PACKAGE_MOBILE_BASE,
            executable="behavior_fsm",
            # No zone arbiter in the simulation: deliver without a turn.
            parameters=[base_cfg, {"require_zone_grant": False}],
            output="screen",
        ),
        Node(
            package=PACKAGE_MOBILE_BASE,
            executable="obstacle_guard",
            # No fleet_link or zone arbiter in the simulation.
            parameters=[base_cfg, {"require_heartbeat": False}],
            output="screen",
        ),
    ]


def _setup(context):
    """
    Resolve the launch arguments and build every action.
    """
    mode = LaunchConfiguration("webots").perform(context)
    world = LaunchConfiguration("world").perform(context)
    url = LaunchConfiguration("webots_url").perform(context)
    port = int(LaunchConfiguration("port").perform(context))
    return [
        *_webots_actions(mode, world),
        *_collector_nodes(port, _bridge_env(url)),
    ]


def generate_launch_description():
    """
    Declare the launch arguments; the rest is built once they are known.
    """
    return LaunchDescription(
        [
            DeclareLaunchArgument("webots", default_value=MODE_HEADLESS),
            DeclareLaunchArgument(
                "webots_url",
                default_value=DEFAULT_WEBOTS_URL,
            ),
            DeclareLaunchArgument("world", default_value=_default_world()),
            DeclareLaunchArgument("port", default_value=DEFAULT_PORT),
            OpaqueFunction(function=_setup),
        ]
    )
