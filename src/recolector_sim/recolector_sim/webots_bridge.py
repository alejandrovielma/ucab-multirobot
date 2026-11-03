"""
webots_bridge node: Webots <extern> controller for the simulated robot.

In simulation it does what camera_node, ultrasonic and motor_driver do on the
real robot, with the same topics:
  publishes   image_raw (sensor_msgs/Image, bgr8) from the camera device
  publishes   range (sensor_msgs/Range) from the ultrasonic device
  subscribes  cmd_vel (geometry_msgs/Twist) and drives the 4 mecanum motors

If cmd_vel stops arriving for more than `cmd_timeout_s`, it stops (same as
motor_driver).

It needs the Webots controller library: simulation.launch.py sets up
PYTHONPATH, LD_LIBRARY_PATH and WEBOTS_CONTROLLER_URL from WEBOTS_HOME.
"""

import sys
from dataclasses import dataclass

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Range

from mobile_base.kinematics import (
    WHEELS,
    MecanumGeometry,
    scale_to_limit,
    wheel_speeds,
)
from recolector_common.params import declare_params, read_params
from vision.ros_image import bgr_to_image

try:
    from controller import Robot
except ImportError:  # reported in main()
    Robot = None

NODE_NAME = "webots_bridge"

# --- Topics and frames --------------------------------------------------------
TOPIC_IMAGE_RAW = "image_raw"
TOPIC_RANGE = "range"
TOPIC_CMD_VEL = "cmd_vel"
DEFAULT_QOS_DEPTH = 10
CAMERA_FRAME_ID = "camera"
ULTRASONIC_FRAME_ID = "ultrasonic"

# --- Device names in RecolectorUCAB.proto (Spanish, set by the Webots model) --
MOTOR_DEVICE_FORMAT = "motor_{wheel}"
SCOOP_SERVO_DEVICE = "servo_pala"
CAMERA_DEVICE = "camara"
ULTRASONIC_DEVICE = "ultrasonido"

# --- Parameter defaults: RecolectorUCAB PROTO measurements (simulation) -------
DEFAULT_WHEEL_RADIUS_M = 0.0325
DEFAULT_HALF_WHEELBASE_M = 0.0605
DEFAULT_HALF_TRACK_M = 0.0655
# 10.5 rad/s x 70 % (2S battery).
DEFAULT_MAX_WHEEL_SPEED_RAD_S = 7.35
DEFAULT_CMD_TIMEOUT_S = 0.5
DEFAULT_CAMERA_PERIOD_MS = 64
DEFAULT_RANGE_PERIOD_MS = 32

# --- Simulated hardware -------------------------------------------------------
SCOOP_DOWN_RAD = 0.0
# Simulated HC-SR04 geometry.
ULTRASONIC_FOV_RAD = 0.26
ULTRASONIC_MIN_RANGE_M = 0.02
ULTRASONIC_MAX_RANGE_M = 4.0
# Webots camera images are BGRA: 4 bytes per pixel, the first 3 are BGR.
WEBOTS_IMAGE_CHANNELS = 4
BGR_CHANNELS = 3
MS_PER_S = 1000.0
# Time stamp older than any real one, so the first period always elapses.
NEVER_MS = -1e9


@dataclass
class WebotsBridgeParams:
    """
    Parameters of the bridge (the defaults are the PROTO measurements).

    Field names double as ROS parameter names (see recolector_common.params).
    """

    wheel_radius_m: float = DEFAULT_WHEEL_RADIUS_M
    half_wheelbase_m: float = DEFAULT_HALF_WHEELBASE_M
    half_track_m: float = DEFAULT_HALF_TRACK_M
    max_wheel_speed_rad_s: float = DEFAULT_MAX_WHEEL_SPEED_RAD_S
    cmd_timeout_s: float = DEFAULT_CMD_TIMEOUT_S
    camera_period_ms: int = DEFAULT_CAMERA_PERIOD_MS
    range_period_ms: int = DEFAULT_RANGE_PERIOD_MS


class WebotsBridge(Node):
    """
    Connects the ROS topics of the real robot to a simulated Webots robot.
    """

    def __init__(self, robot):
        """
        Declare the parameters and connect to the Webots devices.
        """
        super().__init__(NODE_NAME)
        declare_params(self, WebotsBridgeParams())
        self._params = read_params(self, WebotsBridgeParams())

        self._robot = robot
        self._step_ms = int(robot.getBasicTimeStep())
        self._geo = MecanumGeometry(
            self._params.wheel_radius_m,
            self._params.half_wheelbase_m,
            self._params.half_track_m,
        )
        self._max_wheel = self._params.max_wheel_speed_rad_s
        self._cmd_timeout_s = self._params.cmd_timeout_s

        self._motors = self._setup_motors()
        self._setup_sensors()

        self._pub_image = self.create_publisher(
            Image,
            TOPIC_IMAGE_RAW,
            qos_profile_sensor_data,
        )
        self._pub_range = self.create_publisher(
            Range,
            TOPIC_RANGE,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Twist,
            TOPIC_CMD_VEL,
            self._on_cmd,
            DEFAULT_QOS_DEPTH,
        )

        self._cmd = Twist()
        self._cmd_time_s: float | None = None
        self._last_cam_ms = NEVER_MS
        self._last_range_ms = NEVER_MS
        self.get_logger().info(f"connected to Webots robot '{robot.getName()}'")

    def _setup_motors(self) -> dict:
        """
        Put the 4 wheel motors in velocity mode and raise the scoop.
        """
        motors = {}
        for wheel in WHEELS:
            motor = self._robot.getDevice(
                MOTOR_DEVICE_FORMAT.format(wheel=wheel)
            )
            motor.setPosition(float("inf"))
            motor.setVelocity(0.0)
            motors[wheel] = motor
        self._robot.getDevice(SCOOP_SERVO_DEVICE).setPosition(SCOOP_DOWN_RAD)
        return motors

    def _setup_sensors(self) -> None:
        """
        Enable the camera and the ultrasonic sensor at their periods (never
        faster than the simulation step).
        """
        self._cam_period_ms = max(
            self._step_ms,
            self._params.camera_period_ms,
        )
        self._camera = self._robot.getDevice(CAMERA_DEVICE)
        self._camera.enable(self._cam_period_ms)
        self._range_period_ms = max(
            self._step_ms,
            self._params.range_period_ms,
        )
        self._sonar = self._robot.getDevice(ULTRASONIC_DEVICE)
        self._sonar.enable(self._range_period_ms)

    def _on_cmd(self, msg: Twist):
        """
        Store the latest velocity command with the simulation time.
        """
        self._cmd = msg
        self._cmd_time_s = self._robot.getTime()

    def step(self) -> bool:
        """
        One simulation step. False when Webots closes or resets.
        """
        if self._robot.step(self._step_ms) == -1:
            return False
        t_ms = self._robot.getTime() * MS_PER_S
        self._apply_cmd()
        if t_ms - self._last_cam_ms >= self._cam_period_ms:
            self._last_cam_ms = t_ms
            self._publish_image()
        if t_ms - self._last_range_ms >= self._range_period_ms:
            self._last_range_ms = t_ms
            self._publish_range()
        return True

    def _apply_cmd(self):
        """
        Drive the wheels with the latest command, or stop it if it is stale.
        """
        fresh = (
            self._cmd_time_s is not None
            and self._robot.getTime() - self._cmd_time_s <= self._cmd_timeout_s
        )
        if fresh:
            cmd = self._cmd
            speeds = wheel_speeds(
                cmd.linear.x,
                cmd.linear.y,
                cmd.angular.z,
                self._geo,
            )
            speeds = scale_to_limit(speeds, self._max_wheel)
        else:
            speeds = dict.fromkeys(WHEELS, 0.0)
        for wheel, speed in speeds.items():
            self._motors[wheel].setVelocity(speed)

    def _publish_image(self):
        """
        Publish the simulated camera image as bgr8.
        """
        try:
            raw = self._camera.getImage()
        except ValueError:  # over TCP, the first steps arrive without an image
            return
        if not raw:
            return
        height, width = self._camera.getHeight(), self._camera.getWidth()
        pixels = np.frombuffer(raw, dtype=np.uint8).reshape(
            height,
            width,
            WEBOTS_IMAGE_CHANNELS,
        )
        msg = bgr_to_image(pixels[:, :, :BGR_CHANNELS])
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = CAMERA_FRAME_ID
        self._pub_image.publish(msg)

    def _publish_range(self):
        """
        Publish the simulated ultrasonic reading.
        """
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ULTRASONIC_FRAME_ID
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = ULTRASONIC_FOV_RAD
        msg.min_range = ULTRASONIC_MIN_RANGE_M
        msg.max_range = ULTRASONIC_MAX_RANGE_M
        msg.range = float(self._sonar.getValue())
        self._pub_range.publish(msg)


def main(args=None):
    """
    Entry point of the webots_bridge node: step the simulation and spin ROS
    in the same loop.
    """
    if Robot is None:
        print(
            "[webots_bridge] Webots 'controller' module not found. "
            "Launch it with simulation.launch.py or add "
            "$WEBOTS_HOME/lib/controller/python to PYTHONPATH.",
            file=sys.stderr,
        )
        sys.exit(1)
    rclpy.init(args=args)
    # Waits for Webots to accept the connection (WEBOTS_CONTROLLER_URL).
    robot = Robot()
    node = WebotsBridge(robot)
    try:
        while rclpy.ok() and node.step():
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
