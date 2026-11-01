"""
Phineas behavior chain (phineas_behavior -> phineas_safety_gate) on Ferb's
Webots simulation, replacing only Ferb's behavior_fsm and obstacle_guard.

Reuses Ferb's Webots launcher, bridge and web_viewer from simulation.launch.py.
Our phineas_cube_detector replaces Ferb's cube_detector (both publish `cubes`;
only ours also publishes `cubes/scene`), reading the bridge's `image_raw`.

SAFETY LIMITS of this setup (simulation only, never copy to a real robot):
  * The bridge simulates the line sensor from the floor sensors at the front
    (see docs/phineas/integracion-simulacion.md): black tape and the table
    edge read as `black`. It is only a stand-in for the single TCRT5000 of
    the real robot, not a measurement of it.
  * No `pose` (overhead camera), no /zone/occupant (arbiter), no peer pose:
    require_pose and require_heartbeat are set to false (see the YAML).

Arguments: webots, webots_url, world, port (as simulation.launch.py), plus
targets (default yellow) and avoid (default red), comma separated colors.
"""

import importlib.util
import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

HERE = os.path.dirname(os.path.realpath(__file__))
_spec = importlib.util.spec_from_file_location(
    "ferb_simulation", os.path.join(HERE, "simulation.launch.py")
)
ferb = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(ferb)


def _phineas_params(node: str) -> dict:
    """
    Calibrated parameters of /phineas/<node>, without the namespace.
    """
    path = ferb._config_path("mobile_base", "phineas.yaml")
    with open(path) as f:
        return yaml.safe_load(f)[f"/phineas/{node}"]["ros__parameters"]


def _setup(context):
    """
    Ferb's simulator plus our nodes, once the arguments are known.
    """
    mode = LaunchConfiguration("webots").perform(context)
    world = LaunchConfiguration("world").perform(context)
    url = LaunchConfiguration("webots_url").perform(context)
    port = int(LaunchConfiguration("port").perform(context))
    vision_cfg = ferb._config_path("vision", ferb.VISION_CONFIG_FILE)
    overrides = ferb._config_path("bringup", "phineas_on_ferb_sim.yaml")
    mission = {
        "my_colors": ParameterValue(
            LaunchConfiguration("targets"), value_type=str
        ),
        "avoid_colors": ParameterValue(
            LaunchConfiguration("avoid"), value_type=str
        ),
    }

    def ours(executable, node, extra=None, **kwargs):
        return Node(
            package="mobile_base",
            executable="phineas_" + executable,
            name=node,
            parameters=[_phineas_params(node), overrides, *(extra or [])],
            output="screen",
            **kwargs,
        )

    return [
        *ferb._webots_actions(mode, world),
        Node(
            package=ferb.PACKAGE_SIM,
            executable="webots_bridge",
            output="screen",
            additional_env=ferb._bridge_env(url),
        ),
        Node(
            package=ferb.PACKAGE_VISION,
            executable="web_viewer",
            parameters=[vision_cfg, {"port": port}],
            output="screen",
        ),
        ours(
            "cube_detector",
            "cube_detector",
            remappings=[("camera/image_raw", "image_raw")],
        ),
        ours("behavior", "behavior", [mission]),
        ours("safety_gate", "safety_gate"),
    ]


def generate_launch_description():
    """
    Declare the launch arguments; the rest is built once they are known.
    """
    return LaunchDescription(
        [
            DeclareLaunchArgument("webots", default_value=ferb.MODE_HEADLESS),
            DeclareLaunchArgument(
                "webots_url", default_value=ferb.DEFAULT_WEBOTS_URL
            ),
            DeclareLaunchArgument("world", default_value=ferb._default_world()),
            DeclareLaunchArgument("port", default_value=ferb.DEFAULT_PORT),
            DeclareLaunchArgument("targets", default_value="yellow"),
            DeclareLaunchArgument("avoid", default_value="red"),
            OpaqueFunction(function=_setup),
        ]
    )
