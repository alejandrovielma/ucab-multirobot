"""Front line sensor (TCRT5000 digital output on GPIO5) for the real robot -> line_front."""

from rclpy.node import Node
from std_msgs.msg import Bool

from mobile_base.gpio import Gpio
from mobile_base.nodes.common import spin_node


class LineSensorNode(Node):
    def __init__(self):
        super().__init__("line_sensor")
        p = self.declare_parameter
        self.pin = p("pin", 5).value
        # GPIO level that means "black line under the sensor"; verify with the real module.
        self.active_level = p("active_level", 1).value
        rate_hz = p("rate_hz", 50.0).value
        self.gpio = Gpio(p("gpio_chip", 4).value, self.get_logger(), p("dry_run", False).value)
        self.gpio.claim_input(self.pin)
        self._pub = self.create_publisher(Bool, "line_front", 10)
        self.create_timer(1.0 / rate_hz, self._tick)

    def _tick(self):
        self._pub.publish(Bool(data=self.gpio.read(self.pin) == self.active_level))

    def destroy_node(self):
        self.gpio.close()
        return super().destroy_node()


def main(args=None):
    spin_node(LineSensorNode, args)


if __name__ == "__main__":
    main()
