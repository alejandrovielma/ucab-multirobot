"""Front line sensor (TCRT5000 digital output on GPIO5) for the real robot -> line (recolector_interfaces/LineSensors)."""

from rclpy.node import Node
from recolector_interfaces.msg import LineSensors

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
        self._pub = self.create_publisher(LineSensors, "line", 10)
        self.create_timer(1.0 / rate_hz, self._tick)

    def _tick(self):
        msg = LineSensors(black=self.gpio.read(self.pin) == self.active_level)
        msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(msg)

    def destroy_node(self):
        self.gpio.close()
        return super().destroy_node()


def main(args=None):
    spin_node(LineSensorNode, args)


if __name__ == "__main__":
    main()
