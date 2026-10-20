"""Motor driver of the real robot: cmd_vel -> four mecanum wheels.

Subscribes cmd_vel, applies mecanum inverse kinematics and caps the duty to protect the 6 V TT
motors from the 2S battery (8.4 V full -> cap 0.71). Two backends (motor_backend):
- fusion_hat (default): SunFounder Fusion HAT+. Its MCU generates the PWM; each wheel goes to a port
  M0-M3 (fusion_ports, order fl, fr, rl, rr) with an optional direction flip (fusion_reversed), and
  the capped duty becomes a power percent (core/fusion_hat.py). Without the fusion_hat library the
  node logs an error and runs dry.
- cokoino_drv8833: the previous Cokoino 4WD HAT (2x DRV8833) through lgpio (fast decay: PWM on IN1
  with IN2 low = forward); nSLEEP pins are held high.
Stops all motors when no cmd_vel arrives for watchdog_timeout seconds.
TODO(team): map wheels to ports in the lab (tools/lab/lab_robot.py motors) before running on the floor.
"""

from geometry_msgs.msg import Twist
from rclpy.node import Node

from mobile_base.core.fusion_hat import DEFAULT_PORTS, FusionMotorBank, load_motor_class, wiring_from
from mobile_base.core.kinematics import MecanumGeometry, duty_cap, hbridge_pwm, normalize, to_duty, wheel_speeds
from mobile_base.gpio import Gpio
from mobile_base.nodes.common import declare_dataclass, now_sec, spin_node

WHEELS = ("fl", "fr", "rl", "rr")
BACKENDS = ("fusion_hat", "cokoino_drv8833")
# Cokoino BCM pins [IN1, IN2]: M1 front left (DRV#1 A), M3 front right (DRV#2 A),
# M2 rear left (DRV#1 B), M4 rear right (DRV#2 B). Overridable from YAML.
DEFAULT_PINS = {"fl": [17, 27], "fr": [24, 25], "rl": [22, 23], "rr": [26, 16]}


class HatMotorDriverNode(Node):
    def __init__(self):
        super().__init__("hat_motor_driver")
        p = self.declare_parameter
        self.geom = MecanumGeometry(**declare_dataclass(self, MecanumGeometry))
        self.max_wheel_speed = p("max_wheel_speed", 10.5).value
        self.cap = duty_cap(p("battery_voltage", 8.4).value, p("motor_max_voltage", 6.0).value)
        self.watchdog_timeout = p("watchdog_timeout", 0.3).value
        self.signs = p("wheel_signs", [1, 1, 1, 1]).value
        self.backend = p("motor_backend", "fusion_hat").value
        dry_run = p("dry_run", False).value
        # Fusion HAT+ parameters.
        ports = p("fusion_ports", list(DEFAULT_PORTS)).value
        reversed_ = p("fusion_reversed", [False, False, False, False]).value
        # Cokoino parameters.
        self.pwm_frequency = p("pwm_frequency", 1000.0).value
        nsleep_pins = p("nsleep_pins", [12, 13]).value
        self.pins = {w: p(f"pins_{w}", DEFAULT_PINS[w]).value for w in WHEELS}
        gpio_chip = p("gpio_chip", 4).value
        if self.backend not in BACKENDS:
            raise ValueError(f"motor_backend {self.backend!r}: use one of {BACKENDS}")

        self.gpio = None
        self.bank = None
        if self.backend == "fusion_hat":
            wiring = wiring_from(ports, reversed_)
            motor_class = None if dry_run else load_motor_class(self.get_logger())
            if motor_class is not None:
                self.bank = FusionMotorBank(motor_class, wiring)
            dry = self.bank is None
            ports_text = ", ".join(f"{w} {port}{' (reversed)' if flip else ''}" for w, (port, flip) in wiring.items())
            self.get_logger().info(f"Fusion HAT+ ports: {ports_text}")
        else:
            self.gpio = Gpio(gpio_chip, self.get_logger(), dry_run)
            for in1, in2 in self.pins.values():
                self.gpio.claim_output(in1, 0)
                self.gpio.claim_output(in2, 0)
            for pin in nsleep_pins:
                self.gpio.claim_output(pin, 1)  # nSLEEP high = drivers awake
            dry = self.gpio.dry_run

        self._last_cmd: float | None = None
        self._stopped = True
        self.create_subscription(Twist, "cmd_vel", self._on_cmd, 10)
        self.create_timer(self.watchdog_timeout / 3.0, self._watchdog)
        self.get_logger().info(f"backend {self.backend}, duty cap {self.cap:.2f}, dry run {dry}")

    def _on_cmd(self, msg: Twist):
        speeds = normalize(
            wheel_speeds(msg.linear.x, msg.linear.y, msg.angular.z, self.geom), self.max_wheel_speed
        )
        signed = tuple(s * sign for s, sign in zip(speeds, self.signs))
        self._apply(to_duty(signed, self.max_wheel_speed, self.cap))
        self._last_cmd = now_sec(self)
        self._stopped = False

    def _apply(self, duties):
        if self.backend == "fusion_hat":
            if self.bank is not None:
                self.bank.apply(duties, self.cap)
            return
        for wheel, duty in zip(WHEELS, duties):
            in1, in2 = self.pins[wheel]
            pwm1, pwm2 = hbridge_pwm(duty)
            self.gpio.pwm(in1, self.pwm_frequency, pwm1)
            self.gpio.pwm(in2, self.pwm_frequency, pwm2)

    def _watchdog(self):
        stale = self._last_cmd is None or now_sec(self) - self._last_cmd > self.watchdog_timeout
        if stale and not self._stopped:
            self.get_logger().warning("cmd_vel watchdog: stopping motors")
            self.stop()

    def stop(self):
        self._apply((0.0, 0.0, 0.0, 0.0))
        if self.bank is not None:
            self.bank.stop()
        self._stopped = True

    def destroy_node(self):
        self.stop()
        if self.gpio is not None:
            self.gpio.close()
        return super().destroy_node()


def main(args=None):
    spin_node(HatMotorDriverNode, args)


if __name__ == "__main__":
    main()
