"""
motor_driver node: receives cmd_vel and drives the 4 mecanum motors through
the HAT (Fusion HAT+ or Cokoino 2x DRV8833, parameter `driver`).

cmd_vel (REP 103, m/s and rad/s) goes through the mecanum inverse
kinematics; each wheel gets a ratio of its top speed (`max_wheel_speed_m_s`
at the rim). If cmd_vel stops arriving for `cmd_timeout_s`, it brakes.

With `driver: fake` it runs on the laptop and only logs.
"""

from dataclasses import dataclass

from geometry_msgs.msg import Twist
from rclpy.node import Node

from mobile_base.kinematics import MecanumGeometry, wheel_ratios
from mobile_base.motor_hardware import (
    DRIVER_FAKE,
    MotorHardwareParams,
    build_motors,
)
from mobile_base.topics import DEFAULT_QOS_DEPTH, TOPIC_CMD_VEL
from recolector_common.node_utils import now_s, spin_node
from recolector_common.params import declare_params, read_params

NODE_NAME = "motor_driver"

# --- Node parameter defaults (real values: config/collector.yaml) ---------
DEFAULT_CMD_TIMEOUT_S = 0.5
DEFAULT_RATE_HZ = 20.0
# 65 mm wheels (docs/hardware.md).
DEFAULT_WHEEL_RADIUS_M = 0.0325
# From the RecolectorUCAB PROTO; TODO measure on the real chassis.
DEFAULT_HALF_WHEELBASE_M = 0.0605
DEFAULT_HALF_TRACK_M = 0.0655
# Rim speed of a wheel at full duty; the robot's maximum is ~0.24 m/s.
DEFAULT_MAX_WHEEL_SPEED_M_S = 0.24


@dataclass
class MotorDriverParams:
    """
    Node-level parameters (the wiring lives in MotorHardwareParams).

    Field names double as ROS parameter names (see recolector_common.params).
    """

    cmd_timeout_s: float = DEFAULT_CMD_TIMEOUT_S
    rate_hz: float = DEFAULT_RATE_HZ
    wheel_radius_m: float = DEFAULT_WHEEL_RADIUS_M
    half_wheelbase_m: float = DEFAULT_HALF_WHEELBASE_M
    half_track_m: float = DEFAULT_HALF_TRACK_M
    max_wheel_speed_m_s: float = DEFAULT_MAX_WHEEL_SPEED_M_S


class MotorDriverNode(Node):
    """
    cmd_vel -> wheel ratios -> HAT, with a brake watchdog.
    """

    def __init__(self):
        """
        Declare the parameters, open the motors and subscribe to cmd_vel.
        """
        super().__init__(NODE_NAME)
        declare_params(self, MotorDriverParams())
        declare_params(self, MotorHardwareParams())
        self._p = read_params(self, MotorDriverParams())
        hardware = read_params(self, MotorHardwareParams())
        self._motors = build_motors(hardware)
        self._geo = MecanumGeometry(
            wheel_radius_m=self._p.wheel_radius_m,
            half_wheelbase_m=self._p.half_wheelbase_m,
            half_track_m=self._p.half_track_m,
        )
        self._max_wheel_rad_s = (
            self._p.max_wheel_speed_m_s / self._p.wheel_radius_m
        )
        self._cmd = Twist()
        self._cmd_stamp_s: float | None = None
        self.create_subscription(
            Twist,
            TOPIC_CMD_VEL,
            self._on_cmd,
            DEFAULT_QOS_DEPTH,
        )
        self.create_timer(1.0 / self._p.rate_hz, self._tick)
        if hardware.driver == DRIVER_FAKE:
            self.get_logger().warning("fake motors: the robot will not move")
        self.get_logger().info(f"motor driver: {hardware.driver}")

    def _on_cmd(self, msg: Twist):
        """
        Store the latest command with its arrival time.
        """
        self._cmd = msg
        self._cmd_stamp_s = now_s(self)

    def _is_stale(self, now: float) -> bool:
        """
        True when no command arrived within `cmd_timeout_s`.
        """
        if self._cmd_stamp_s is None:
            return True
        return now - self._cmd_stamp_s > self._p.cmd_timeout_s

    def _tick(self):
        """
        Apply the latest command, or brake when it is too old.
        """
        if self._is_stale(now_s(self)):
            self._motors.stop()
            return
        self._motors.set(
            wheel_ratios(
                self._cmd.linear.x,
                self._cmd.linear.y,
                self._cmd.angular.z,
                self._geo,
                self._max_wheel_rad_s,
            )
        )

    def destroy_node(self):
        """
        Brake and release the HAT before shutting down.
        """
        self._motors.close()
        super().destroy_node()


def main(args=None):
    """
    Entry point of the motor_driver node.
    """
    spin_node(MotorDriverNode, args)


if __name__ == "__main__":
    main()
