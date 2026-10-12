"""Webots simulation: the track world, Phineas and Ferb, their logic nodes and the coordinator.

ros2 launch bringup sim.launch.py
Requires webots_ros2_driver (sudo apt install ros-jazzy-webots-ros2) and Webots R2025a.
The Webots plugin replaces the hardware nodes (motors, line sensor, camera) and emulates
the overhead camera pose with GPS + InertialUnit, so the safety gate requires pose and heartbeat.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import WebotsLauncher

ROBOTS = {"phineas": "Phineas", "ferb": "Ferb"}  # ROS namespace -> Webots robot name


def generate_launch_description():
    sim_share = get_package_share_directory("multirobot_sim")
    bringup_share = get_package_share_directory("bringup")

    webots = WebotsLauncher(world=os.path.join(sim_share, "worlds", "pista_ucab.wbt"))
    actions = [
        DeclareLaunchArgument(
            "logic", default_value="true", description="false = only Webots and drivers (manual cmd_vel tests)"
        ),
        webots,
    ]

    for robot, webots_name in ROBOTS.items():
        actions.append(
            WebotsController(
                robot_name=webots_name,
                parameters=[{"robot_description": os.path.join(sim_share, "resource", f"{robot}.urdf")}],
            )
        )
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "robot_logic.launch.py")),
                launch_arguments={
                    "robot": robot,
                    "use_sim_time": "true",
                    "require_pose": "true",
                    "require_heartbeat": "true",
                }.items(),
                condition=IfCondition(LaunchConfiguration("logic")),
            )
        )

    actions.append(
        Node(
            package="coordinator",
            executable="zone_arbiter",
            name="zone_arbiter",
            parameters=[{"use_sim_time": True}],
            output="screen",
        )
    )
    actions.append(
        RegisterEventHandler(
            OnProcessExit(target_action=webots, on_exit=[EmitEvent(event=Shutdown())])
        )
    )
    return LaunchDescription(actions)
