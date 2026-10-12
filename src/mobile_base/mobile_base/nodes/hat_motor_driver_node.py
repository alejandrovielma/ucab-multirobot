"""Cokoino 4WD HAT (2x DRV8833) motor driver for the real robot.

Subscribes cmd_vel, applies mecanum inverse kinematics, caps the duty to protect the
6 V TT motors from the 2S battery, and drives PWM on the IN pins through lgpio
(fast decay: PWM on IN1 with IN2 low = forward). nSLEEP pins are held high.
Stops all motors when no cmd_vel arrives for watchdog_timeout seconds.
TODO(team): bench test on the HAT (see docs/team-todo.md) before running on the floor.
"""

from geometry_msgs.msg import Twist
from rclpy.node import Node

from mobile_base.core.kinematics import MecanumGeometry, duty_cap, hbridge_pwm, normalize, to_duty, wheel_speeds
from mobile_base.gpio import Gpio
from mobile_base.nodes.common import declare_dataclass, now_sec, spin_node

WHEELS = ("fl", "fr", "rl", "rr")
# BCM pins [IN1, IN2]: M1 front left (DRV#1 A), M3 front right (DRV#2 A),
# M2 rear left (DRV#1 B), M4 rear right (DRV#2 B). Overridable from YAML.
DEFAULT_PINS = {"fl": [17, 27], "fr": [24, 25], "rl": [22, 23], "rr": [26, 16]}


class HatMotorDriverNode(Node):
    def __init__(self):
        super().__init__("hat_motor_driver")
        p = self.declare_parameter
        self.geom = MecanumGeometry(**declare_dataclass(self, MecanumGeometry))
        self.max_wheel_speed = p("max_wheel_speed", 10.5).value
        self.cap = duty_cap(p("battery_voltage", 8.4).value, p("motor_max_voltage", 6.0).value)
        self.pwm_frequency = p("pwm_frequency", 1000.0).value
        self.watchdog_timeout = p("watchdog_timeout", 0.3).value
        self.signs = p("wheel_signs", [1, 1, 1, 1]).value
        nsleep_pins = p("nsleep_pins", [12, 13]).value
        self.pins = {w: p(f"pins_{w}", DEFAULT_PINS[w]).value for w in WHEELS}

        self.gpio = Gpio(p("gpio_chip", 4).value, self.get_logger(), p("dry_run", False).value)
        for in1, in2 in self.pins.values():
            self.gpio.claim_output(in1, 0)
            self.gpio.claim_output(in2, 0)
        for pin in nsleep_pins:
            self.gpio.claim_output(pin, 1)  # nSLEEP high = drivers awake

        self._last_cmd: float | None = None
        self._stopped = True
        self.create_subscription(Twist, "cmd_vel", self._on_cmd, 10)
        self.create_timer(self.watchdog_timeout / 3.0, self._watchdog)
        self.get_logger().info(f"duty cap {self.cap:.2f}, dry run {self.gpio.dry_run}")

    def _on_cmd(self, msg: Twist):
        speeds = normalize(
            wheel_speeds(msg.linear.x, msg.linear.y, msg.angular.z, self.geom), self.max_wheel_speed
        )
        signed = tuple(s * sign for s, sign in zip(speeds, self.signs))
        self._apply(to_duty(signed, self.max_wheel_speed, self.cap))
        self._last_cmd = now_sec(self)
        self._stopped = False

    def _apply(self, duties):
        for wheel, duty in zip(WHEELS, duties):
            in1, in2 = self.pins[wheel]
            pwm1, pwm2 = hbridge_pwm(duty)
            self.gpio.pwm(in1, self.pwm_frequency, pwm1)
            self.gpio.pwm(in2, self.pwm_frequency, pwm2)

    def _watchdog(self):
        stale = self._last_cmd is None or now_sec(self) - self._last_cmd > self.watchdog_timeout
        if stale and not self._stopped:
            self.get_logger().warn("cmd_vel watchdog: stopping motors")
            self.stop()

    def stop(self):
        self._apply((0.0, 0.0, 0.0, 0.0))
        self._stopped = True

    def destroy_node(self):
        self.stop()
        self.gpio.close()
        return super().destroy_node()


def main(args=None):
    spin_node(HatMotorDriverNode, args)


if __name__ == "__main__":
    main()
