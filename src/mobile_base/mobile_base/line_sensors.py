"""
line_sensors node: reads the bottom TCRT5000 (digital DO output, between
the front wheels) and publishes line.

`black` is true over the black tape, the border of the area. The delivery
area is blue and green paper in a gap of the line, so the sensor must read
it as white (adjust the module's trimpot; docs/hardware.md).

With `pin: -1` (not wired) it publishes "white" forever and warns: the
robot will not see the border.
"""

from dataclasses import dataclass

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from mobile_base.sensor_hardware import NO_PIN, DigitalSensor, FakeDigitalSensor
from mobile_base.topics import TOPIC_LINE
from recolector_common.node_utils import spin_node
from recolector_common.params import declare_params, read_params
from recolector_interfaces.msg import LineSensors

NODE_NAME = "line_sensors"
FRAME_ID = "line_sensor"

# --- Node parameter defaults (real values: config/collector.yaml) ---------
# GPIO5 (pin 29), as wired in tools/teleop_web.yaml.
DEFAULT_PIN = 5
# DO value over the black tape (the usual module gives 1 on black).
DEFAULT_BLACK_VALUE = 1
# Fast enough to stop on a 19 mm tape at the robot's top speed.
DEFAULT_RATE_HZ = 50.0
# What the fake reads: white (not the black value).
FAKE_WHITE_VALUE = 0


@dataclass
class LineSensorsParams:
    """
    Parameters of the line_sensors node.

    Field names double as ROS parameter names (see recolector_common.params).
    """

    pin: int = DEFAULT_PIN
    black_value: int = DEFAULT_BLACK_VALUE
    rate_hz: float = DEFAULT_RATE_HZ


class LineSensorsNode(Node):
    """
    Polls the line sensor and publishes the reading.
    """

    def __init__(self):
        """
        Declare the parameters, open the sensor and start polling.
        """
        super().__init__(NODE_NAME)
        declare_params(self, LineSensorsParams())
        self._p = read_params(self, LineSensorsParams())
        if self._p.pin == NO_PIN:
            self.get_logger().warning(
                "line sensor pin not set: border not seen"
            )
            self._sensor = FakeDigitalSensor(FAKE_WHITE_VALUE)
        else:
            self._sensor = DigitalSensor(self._p.pin)
        self._pub = self.create_publisher(
            LineSensors,
            TOPIC_LINE,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0 / self._p.rate_hz, self._tick)

    def _tick(self):
        """
        Read the sensor and publish whether it is over the black tape.
        """
        msg = LineSensors()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = FRAME_ID
        msg.black = self._sensor.read() == self._p.black_value
        self._pub.publish(msg)

    def destroy_node(self):
        """
        Release the GPIO pin before shutting down.
        """
        self._sensor.close()
        super().destroy_node()


def main(args=None):
    """
    Entry point of the line_sensors node.
    """
    spin_node(LineSensorsNode, args)


if __name__ == "__main__":
    main()
