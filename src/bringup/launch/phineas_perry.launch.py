"""Perry's Pi: zone arbiter (coordinator) plus the overhead ArUco localizer."""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    overhead_config = PathJoinSubstitution([FindPackageShare("overhead_localization"), "config", "overhead.yaml"])
    return LaunchDescription([
        Node(package="coordinator", executable="zone_arbiter", name="zone_arbiter", output="screen"),
        Node(package="overhead_localization", executable="aruco_localizer", name="aruco_localizer",
             parameters=[overhead_config], output="screen"),
    ])
