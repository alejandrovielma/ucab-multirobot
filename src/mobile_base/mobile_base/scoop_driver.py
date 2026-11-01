"""
scoop_driver node: raises and lowers the scoop (SG90 servo) and reports a
cube in it (FC-51).

Subscribes scoop/lower (true = down, pushing; false = up, releasing the
cube) and publishes scoop/has_cube. It starts with the scoop down.

Either part can be missing: with `servo_pin: -1` the scoop is fixed and the
commands are only logged; with `presence_pin: -1` scoop/has_cube is not
published and behavior_fsm trusts its timed push instead.
"""

from dataclasses import dataclass

from rclpy.node import Node
from std_msgs.msg import Bool

from mobile_base.sensor_hardware import (
    NO_PIN,
    DigitalSensor,
    FakeServoMotor,
    ServoMotor,
)
from mobile_base.topics import (
    DEFAULT_QOS_DEPTH,
    TOPIC_SCOOP_HAS_CUBE,
    TOPIC_SCOOP_LOWER,
)
from recolector_common.node_utils import spin_node
from recolector_common.params import declare_params, read_params

NODE_NAME = "scoop_driver"

# --- Node parameter defaults (real values: config/collector.yaml) ---------
# docs/hardware.md: 0 deg down, about 40 deg up (TODO measure).
DEFAULT_DOWN_ANGLE_DEG = 0.0
DEFAULT_UP_ANGLE_DEG = 40.0
# SG90 datasheet pulse range for 0..180 deg (TODO measure on the servo).
DEFAULT_MIN_PULSE_WIDTH_S = 0.0005
DEFAULT_MAX_PULSE_WIDTH_S = 0.0024
DEFAULT_MAX_ANGLE_DEG = 180.0
# docs/hardware.md: the FC-51 reads 1 when a cube is in the scoop.
DEFAULT_PRESENCE_ACTIVE_VALUE = 1
DEFAULT_RATE_HZ = 20.0


@dataclass
class ScoopDriverParams:
    """
    Parameters of the scoop_driver node.

    Field names double as ROS parameter names (see recolector_common.params).
    """

    servo_pin: int = NO_PIN  # TODO measure
    down_angle_deg: float = DEFAULT_DOWN_ANGLE_DEG
    up_angle_deg: float = DEFAULT_UP_ANGLE_DEG
    min_pulse_width_s: float = DEFAULT_MIN_PULSE_WIDTH_S
    max_pulse_width_s: float = DEFAULT_MAX_PULSE_WIDTH_S
    max_angle_deg: float = DEFAULT_MAX_ANGLE_DEG
    presence_pin: int = NO_PIN  # TODO measure
    presence_active_value: int = DEFAULT_PRESENCE_ACTIVE_VALUE
    rate_hz: float = DEFAULT_RATE_HZ


class ScoopDriverNode(Node):
    """
    Moves the scoop servo and reports the presence sensor.
    """

    def __init__(self):
        """
        Declare the parameters, open the hardware and lower the scoop.
        """
        super().__init__(NODE_NAME)
        declare_params(self, ScoopDriverParams())
        self._p = read_params(self, ScoopDriverParams())
        self._servo = self._open_servo()
        self._presence = None
        self._lowered: bool | None = None
        self._move(lower=True)
        self.create_subscription(
            Bool,
            TOPIC_SCOOP_LOWER,
            self._on_lower,
            DEFAULT_QOS_DEPTH,
        )
        if self._p.presence_pin == NO_PIN:
            self.get_logger().warning("FC-51 pin not set: no scoop/has_cube")
            return
        self._presence = DigitalSensor(self._p.presence_pin)
        self._pub_has_cube = self.create_publisher(
            Bool,
            TOPIC_SCOOP_HAS_CUBE,
            DEFAULT_QOS_DEPTH,
        )
        self.create_timer(1.0 / self._p.rate_hz, self._tick)

    def _open_servo(self):
        """
        The real servo, or a fake one when its pin is not set.
        """
        p = self._p
        if p.servo_pin == NO_PIN:
            self.get_logger().warning("scoop servo pin not set: fixed scoop")
            return FakeServoMotor()
        return ServoMotor(
            p.servo_pin,
            p.min_pulse_width_s,
            p.max_pulse_width_s,
            p.max_angle_deg,
        )

    def _move(self, lower: bool):
        """
        Move the scoop down or up, only when that changes.
        """
        if lower == self._lowered:
            return
        self._lowered = lower
        angle_deg = self._p.down_angle_deg if lower else self._p.up_angle_deg
        self._servo.move_to(angle_deg)
        self.get_logger().info(f"scoop {'down' if lower else 'up'}")

    def _on_lower(self, msg: Bool):
        """
        Apply a scoop/lower command.
        """
        self._move(lower=bool(msg.data))

    def _tick(self):
        """
        Publish whether a cube is in the scoop.
        """
        has_cube = self._presence.read() == self._p.presence_active_value
        self._pub_has_cube.publish(Bool(data=has_cube))

    def destroy_node(self):
        """
        Release the servo and the sensor before shutting down.
        """
        self._servo.close()
        if self._presence is not None:
            self._presence.close()
        super().destroy_node()


def main(args=None):
    """
    Entry point of the scoop_driver node.
    """
    spin_node(ScoopDriverNode, args)


if __name__ == "__main__":
    main()
