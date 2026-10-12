"""Real robot: hardware nodes plus the shared logic, under namespace /<robot>.

ros2 launch bringup robot.launch.py robot:=phineas
The coordinator (zone_arbiter) and the overhead localizer run on Perry's Pi, not here.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot = LaunchConfiguration("robot")
    config = PathJoinSubstitution([FindPackageShare("mobile_base"), "config", [robot, ".yaml"]])

    def hw_node(executable):
        return Node(
            package="mobile_base",
            executable=executable,
            name=executable,
            namespace=robot,
            parameters=[config],
            output="screen",
        )

    logic = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("bringup"), "launch", "robot_logic.launch.py"])
        ),
        launch_arguments={
            "robot": robot,
            "use_sim_time": "false",
            "require_pose": LaunchConfiguration("require_pose"),
            "require_heartbeat": LaunchConfiguration("require_heartbeat"),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("robot", default_value="phineas", choices=["phineas", "ferb"]),
        DeclareLaunchArgument("require_pose", default_value="true",
                              description="false only for bench tests without the overhead camera"),
        DeclareLaunchArgument("require_heartbeat", default_value="true",
                              description="false only for bench tests without the coordinator"),
        hw_node("hat_motor_driver"),
        hw_node("line_sensor"),
        hw_node("camera"),
        logic,
    ])
