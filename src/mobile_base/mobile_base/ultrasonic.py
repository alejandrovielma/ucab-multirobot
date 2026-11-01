"""
ultrasonic node: measures the distance ahead with the HC-SR04 and publishes
range (sensor_msgs/Range).

Readings are capped at `max_range_m`, which means "nothing ahead" (kept
finite: obstacle_guard ignores non-finite ranges). With the pins not wired
(`-1`) it publishes nothing, so obstacle_guard blocks forward motion;
`fake: true` publishes the maximum range instead, for the laptop.
"""

from dataclasses import dataclass

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range

from mobile_base.sensor_hardware import (
    NO_PIN,
    FakeUltrasonicSensor,
    UltrasonicSensor,
)
from mobile_base.topics import TOPIC_RANGE
from recolector_common.node_utils import spin_node
from recolector_common.params import declare_params, read_params

NODE_NAME = "ultrasonic"
FRAME_ID = "ultrasonic"

# --- Node parameter defaults (real values: config/collector.yaml) ---------
DEFAULT_RATE_HZ = 15.0
# HC-SR04 datasheet: 2 cm to 4 m, 15 deg cone. Beyond 2 m is not useful on
# the track and makes every ping slower.
DEFAULT_MIN_RANGE_M = 0.02
DEFAULT_MAX_RANGE_M = 2.0
DEFAULT_FIELD_OF_VIEW_RAD = 0.26
# Pings averaged by gpiozero for each reading.
DEFAULT_QUEUE_LEN = 3


@dataclass
class UltrasonicParams:
    """
    Parameters of the ultrasonic node.

    Field names double as ROS parameter names (see recolector_common.params).
    """

    trigger_pin: int = NO_PIN  # TODO measure
    echo_pin: int = NO_PIN  # TODO measure
    rate_hz: float = DEFAULT_RATE_HZ
    min_range_m: float = DEFAULT_MIN_RANGE_M
    max_range_m: float = DEFAULT_MAX_RANGE_M
    field_of_view_rad: float = DEFAULT_FIELD_OF_VIEW_RAD
    queue_len: int = DEFAULT_QUEUE_LEN
    fake: bool = False


class UltrasonicNode(Node):
    """
    Polls the HC-SR04 and publishes the range.
    """

    def __init__(self):
        """
        Declare the parameters, open the sensor and start polling.
        """
        super().__init__(NODE_NAME)
        declare_params(self, UltrasonicParams())
        self._p = read_params(self, UltrasonicParams())
        self._sensor = self._open_sensor()
        self._pub = self.create_publisher(
            Range,
            TOPIC_RANGE,
            qos_profile_sensor_data,
        )
        if self._sensor is not None:
            self.create_timer(1.0 / self._p.rate_hz, self._tick)

    def _open_sensor(self):
        """
        The real sensor, the fake one, or None when the pins are not set.
        """
        p = self._p
        if p.fake:
            return FakeUltrasonicSensor(p.max_range_m)
        if NO_PIN in (p.trigger_pin, p.echo_pin):
            self.get_logger().warning(
                "HC-SR04 pins not set: no range, forward motion blocked"
            )
            return None
        return UltrasonicSensor(
            p.trigger_pin,
            p.echo_pin,
            p.max_range_m,
            p.queue_len,
        )

    def _tick(self):
        """
        Read the distance and publish it.
        """
        p = self._p
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = FRAME_ID
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = p.field_of_view_rad
        msg.min_range = p.min_range_m
        msg.max_range = p.max_range_m
        msg.range = min(self._sensor.distance_m(), p.max_range_m)
        self._pub.publish(msg)

    def destroy_node(self):
        """
        Release the pins before shutting down.
        """
        if self._sensor is not None:
            self._sensor.close()
        super().destroy_node()


def main(args=None):
    """
    Entry point of the ultrasonic node.
    """
    spin_node(UltrasonicNode, args)


if __name__ == "__main__":
    main()
