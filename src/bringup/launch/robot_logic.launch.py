"""Hardware-independent nodes of ONE robot: cube_detector, behavior, safety_gate.

Included by robot.launch.py (real robot) and sim.launch.py (Webots).
Args: robot (phineas|ferb), use_sim_time, require_pose, require_heartbeat, targets and avoid (the
mission: comma-separated colors; without targets the robot stays still until one is set with
`ros2 param set /<robot>/behavior my_colors "blue"` or the command interface).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot = LaunchConfiguration("robot")
    config = PathJoinSubstitution([FindPackageShare("mobile_base"), "config", [robot, ".yaml"]])
    # Track polygon for the pose geofence, shared by both robots.
    track = PathJoinSubstitution([FindPackageShare("mobile_base"), "config", "track.yaml"])
    sim_time = {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)}
    gate_overrides = {
        "require_pose": ParameterValue(LaunchConfiguration("require_pose"), value_type=bool),
        "require_heartbeat": ParameterValue(LaunchConfiguration("require_heartbeat"), value_type=bool),
    }

    mission = {
        "my_colors": ParameterValue(LaunchConfiguration("targets"), value_type=str),
        "avoid_colors": ParameterValue(LaunchConfiguration("avoid"), value_type=str),
    }

    def logic_node(executable, extra=None):
        return Node(
            package="mobile_base",
            executable=executable,
            name=executable,
            namespace=robot,
            parameters=[config, track, sim_time, *(extra or [])],
            output="screen",
        )

    return LaunchDescription([
        DeclareLaunchArgument("robot", description="phineas or ferb"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("require_pose", default_value="true"),
        DeclareLaunchArgument("require_heartbeat", default_value="true"),
        DeclareLaunchArgument("targets", default_value="", description="target colors, e.g. blue,green"),
        DeclareLaunchArgument("avoid", default_value="", description="forbidden colors, e.g. red"),
        logic_node("cube_detector"),
        logic_node("behavior", [mission]),
        logic_node("safety_gate", [gate_overrides]),
    ])
