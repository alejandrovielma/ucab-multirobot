"""webots_ros2_driver Python plugin for the RecolectorUCAB robot (Phineas or Ferb).

Loaded by WebotsController from resource/<robot>.urdf. It is the simulated hardware layer
and exposes exactly the same topics as the real robot nodes (namespace /<robot>):
  sub  cmd_vel            -> mecanum kinematics -> motor_fl/fr/rl/rr velocities
  pub  line_front         <- DistanceSensor "linea_frente" below line_threshold
  pub  camera/image_raw   <- Camera "camara" (bgra8)
  pub  pose               <- GPS "gps" + InertialUnit "imu_sim" (emulates the overhead camera)
The scoop servo is held at 0 (the real scoop is fixed).

Properties (child tags of <plugin> in the URDF): robot, line_threshold, pose_noise_xy,
pose_noise_theta, wheel_signs. TODO(team): validate wheel_signs with the strafe test
(docs/team-todo.md); "1,1,1,1" matches the original teleop controller.
Webots types are reached only through webots_node.robot: the `controller` module is never
imported here, so the file stays importable without Webots.
"""

import math
import random

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from mobile_base.core.kinematics import MecanumGeometry, normalize, wheel_speeds
from multirobot_interfaces.msg import RobotPose
from multirobot_sim.sim_utils import is_black, noisy_pose, parse_wheel_signs

MOTORS = ("motor_fl", "motor_fr", "motor_rl", "motor_rr")
MAX_MOTOR_SPEED = 10.5  # rad/s, velMaxMotor of RecolectorUCAB.proto
CAMERA_PERIOD_MS = 64
POSE_PERIOD_S = 0.1
CMD_TIMEOUT_S = 0.5


def _sim_time(seconds: float) -> Time:
    sec = int(seconds)
    return Time(sec=sec, nanosec=int((seconds - sec) * 1e9))


class RecolectorDriver:
    def init(self, webots_node, properties):
        self._robot = webots_node.robot
        self._name = properties.get("robot", "phineas")
        self._line_threshold = float(properties.get("line_threshold", "120"))
        self._noise_xy = float(properties.get("pose_noise_xy", "0.0"))
        self._noise_theta = float(properties.get("pose_noise_theta", "0.0"))
        self._signs = parse_wheel_signs(properties.get("wheel_signs", "1,1,1,1"))
        self._rng = random.Random()
        self._geom = MecanumGeometry()
        timestep = int(self._robot.getBasicTimeStep())

        self._motors = [self._robot.getDevice(name) for name in MOTORS]
        for motor in self._motors:
            motor.setPosition(math.inf)
            motor.setVelocity(0.0)
        self._robot.getDevice("servo_pala").setPosition(0.0)

        self._line = self._robot.getDevice("linea_frente")
        self._line.enable(timestep)
        self._camera = self._robot.getDevice("camara")
        self._camera.enable(CAMERA_PERIOD_MS)
        self._gps = self._robot.getDevice("gps")
        self._gps.enable(timestep)
        self._imu = self._robot.getDevice("imu_sim")
        self._imu.enable(timestep)

        if not rclpy.ok():
            rclpy.init(args=None)
        self._node = rclpy.create_node(f"{self._name}_webots_driver")
        ns = f"/{self._name}"
        self._node.create_subscription(Twist, f"{ns}/cmd_vel", self._on_cmd_vel, 1)
        self._line_pub = self._node.create_publisher(Bool, f"{ns}/line_front", 10)
        self._image_pub = self._node.create_publisher(Image, f"{ns}/camera/image_raw", 1)
        self._pose_pub = self._node.create_publisher(RobotPose, f"{ns}/pose", 10)

        self._cmd = (0.0, 0.0, 0.0)
        self._cmd_time = -math.inf
        self._last_image = -math.inf
        self._last_pose = -math.inf

    def _on_cmd_vel(self, msg: Twist):
        self._cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        self._cmd_time = self._robot.getTime()

    def step(self):
        rclpy.spin_once(self._node, timeout_sec=0)
        now = self._robot.getTime()
        self._drive(now)
        self._line_pub.publish(Bool(data=is_black(self._line.getValue(), self._line_threshold)))
        if now - self._last_image >= CAMERA_PERIOD_MS / 1000.0:
            self._last_image = now
            self._publish_image(now)
        if now - self._last_pose >= POSE_PERIOD_S:
            self._last_pose = now
            self._publish_pose(now)

    def _drive(self, now: float):
        cmd = self._cmd if now - self._cmd_time <= CMD_TIMEOUT_S else (0.0, 0.0, 0.0)
        speeds = normalize(wheel_speeds(*cmd, self._geom), MAX_MOTOR_SPEED)
        for motor, speed, sign in zip(self._motors, speeds, self._signs):
            motor.setVelocity(speed * sign)

    def _publish_image(self, now: float):
        data = self._camera.getImage()
        if not data:
            return
        msg = Image()
        msg.header.stamp = _sim_time(now)
        msg.header.frame_id = f"{self._name}_camera"
        msg.width = self._camera.getWidth()
        msg.height = self._camera.getHeight()
        msg.encoding = "bgra8"
        msg.step = msg.width * 4
        msg.data = bytes(data)
        self._image_pub.publish(msg)

    def _publish_pose(self, now: float):
        x, y, _ = self._gps.getValues()
        yaw = self._imu.getRollPitchYaw()[2]
        if any(math.isnan(v) for v in (x, y, yaw)):
            return
        x, y, theta = noisy_pose(x, y, yaw, self._noise_xy, self._noise_theta, self._rng)
        msg = RobotPose(robot=self._name, x=x, y=y, theta=theta, confidence=1.0)
        msg.stamp = _sim_time(now)
        self._pose_pub.publish(msg)
