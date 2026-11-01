"""
color_sensor node: reads the TCS34725 on the scoop and publishes the colour
of the caught cube on scoop/color (empty string = no cube / unclear).

The classification is in scoop_color.py; its calibration (reference hues,
white balance, thresholds) is in config/collector.yaml. If the sensor is
not found it warns and publishes nothing: behavior_fsm then trusts the
camera's colour. `fake: true` publishes "" for the laptop.
"""

from dataclasses import dataclass

from rclpy.node import Node
from std_msgs.msg import String

from mobile_base.scoop_color import ScoopColorParams, classify_rgbc
from mobile_base.sensor_hardware import FakeTcs34725, Tcs34725
from mobile_base.topics import DEFAULT_QOS_DEPTH, TOPIC_SCOOP_COLOR
from recolector_common.node_utils import spin_node
from recolector_common.params import declare_params, read_params

NODE_NAME = "color_sensor"

# --- Node parameter defaults (real values: config/collector.yaml) ---------
DEFAULT_I2C_BUS = 1
DEFAULT_INTEGRATION_TIME_MS = 24.0
DEFAULT_GAIN = 4
DEFAULT_RATE_HZ = 10.0


@dataclass
class ColorSensorParams:
    """
    Node-level parameters (the calibration lives in ScoopColorParams).

    Field names double as ROS parameter names (see recolector_common.params).
    """

    i2c_bus: int = DEFAULT_I2C_BUS
    integration_time_ms: float = DEFAULT_INTEGRATION_TIME_MS
    gain: int = DEFAULT_GAIN
    rate_hz: float = DEFAULT_RATE_HZ
    fake: bool = False


class ColorSensorNode(Node):
    """
    Polls the TCS34725 and publishes the classified colour.
    """

    def __init__(self):
        """
        Declare the parameters, open the sensor and start polling.
        """
        super().__init__(NODE_NAME)
        declare_params(self, ColorSensorParams())
        declare_params(self, ScoopColorParams())
        self._p = read_params(self, ColorSensorParams())
        self._color_params = read_params(self, ScoopColorParams())
        self.add_post_set_parameters_callback(self._on_params_changed)
        self._sensor = self._open_sensor()
        self._pub = self.create_publisher(
            String,
            TOPIC_SCOOP_COLOR,
            DEFAULT_QOS_DEPTH,
        )
        if self._sensor is not None:
            self.create_timer(1.0 / self._p.rate_hz, self._tick)

    def _open_sensor(self):
        """
        The real sensor, the fake one, or None when it is not found.
        """
        p = self._p
        if p.fake:
            return FakeTcs34725()
        try:
            return Tcs34725(p.i2c_bus, p.integration_time_ms, p.gain)
        except (OSError, ImportError) as exc:
            self.get_logger().warning(f"no TCS34725: {exc}")
            return None

    def _on_params_changed(self, _params):
        """
        Reload the calibration after a runtime change.
        """
        self._color_params = read_params(self, ScoopColorParams())

    def _tick(self):
        """
        Read the counts and publish the colour.
        """
        color = classify_rgbc(*self._sensor.read_rgbc(), self._color_params)
        self._pub.publish(String(data=color))

    def destroy_node(self):
        """
        Power the sensor down before shutting down.
        """
        if self._sensor is not None:
            self._sensor.close()
        super().destroy_node()


def main(args=None):
    """
    Entry point of the color_sensor node.
    """
    spin_node(ColorSensorNode, args)


if __name__ == "__main__":
    main()
