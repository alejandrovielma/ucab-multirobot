"""
Sensor and servo hardware for the base nodes, each with a fake for the
laptop: digital inputs (TCRT5000, FC-51), the HC-SR04, the SG90 servo and
the TCS34725 colour sensor.

gpiozero (lgpio on the Pi 5) and smbus2 are imported inside the classes
because they only exist on the Pi. A pin set to NO_PIN means "not wired":
the node then uses the fake and warns.
"""

import time

# A pin parameter with this value is not wired yet.
NO_PIN = -1

# --- TCS34725 (I2C) -----------------------------------------------------------
TCS_I2C_ADDRESS = 0x29
# Every register access sets the command bit; auto-increment reads a block.
TCS_COMMAND = 0x80
TCS_AUTO_INCREMENT = 0x20
TCS_REG_ENABLE = 0x00
TCS_REG_ATIME = 0x01
TCS_REG_CONTROL = 0x0F
TCS_REG_ID = 0x12
TCS_REG_CDATAL = 0x14
TCS_ENABLE_PON = 0x01
TCS_ENABLE_AEN = 0x02
# ID register of the TCS34725 / TCS34727.
TCS_VALID_IDS = (0x44, 0x4D)
# Analog gain (x) -> CONTROL register value.
TCS_GAINS = {1: 0x00, 4: 0x01, 16: 0x02, 60: 0x03}
# ATIME = 256 - integration time / 2.4 ms.
TCS_ATIME_STEP_MS = 2.4
TCS_ATIME_CYCLES = 256
# Datasheet: 2.4 ms warm-up after power on before enabling the ADC.
TCS_POWER_ON_DELAY_S = 0.003
# C, R, G, B as little-endian 16-bit words.
TCS_RGBC_BYTES = 8
BYTE_BITS = 8


class DigitalSensor:
    """
    Digital output of a sensor module (TCRT5000 DO, FC-51 OUT) read with
    gpiozero. Power the modules from 3.3 V so DO never puts 5 V on a GPIO.
    """

    def __init__(self, pin: int):
        """
        Open the GPIO pin as an input without pull resistor.
        """
        # Only available on the Pi.
        from gpiozero import DigitalInputDevice

        self._dev = DigitalInputDevice(pin, pull_up=False)

    def read(self) -> int:
        """
        Raw digital output (0 or 1).
        """
        return int(self._dev.value)

    def close(self) -> None:
        """
        Release the GPIO pin.
        """
        self._dev.close()


class FakeDigitalSensor:
    """
    Fake digital input that always reads the same value.
    """

    def __init__(self, value: int):
        """
        Always read `value`.
        """
        self.value = value

    def read(self) -> int:
        """
        The fixed value.
        """
        return self.value

    def close(self) -> None:
        """
        Nothing to release.
        """


class UltrasonicSensor:
    """
    HC-SR04 read with gpiozero's DistanceSensor (it averages `queue_len`
    pings). The 5 V echo needs a divider to 3.3 V.
    """

    def __init__(
        self,
        trigger_pin: int,
        echo_pin: int,
        max_range_m: float,
        queue_len: int,
    ):
        """
        Open the trigger and echo pins.
        """
        # Only available on the Pi.
        from gpiozero import DistanceSensor

        self._dev = DistanceSensor(
            echo=echo_pin,
            trigger=trigger_pin,
            max_distance=max_range_m,
            queue_len=queue_len,
        )

    def distance_m(self) -> float:
        """
        Distance to the nearest object (m), capped at the maximum range.
        """
        return float(self._dev.distance)

    def close(self) -> None:
        """
        Release the pins.
        """
        self._dev.close()


class FakeUltrasonicSensor:
    """
    Fake HC-SR04 that never sees anything (always the maximum range).
    """

    def __init__(self, max_range_m: float):
        """
        Always report `max_range_m`.
        """
        self._max_range_m = max_range_m

    def distance_m(self) -> float:
        """
        The maximum range: nothing ahead.
        """
        return self._max_range_m

    def close(self) -> None:
        """
        Nothing to release.
        """


class ServoMotor:
    """
    SG90 hobby servo driven with gpiozero's AngularServo.

    On the Pi 5 gpiozero uses software PWM, so the servo may jitter a
    little while it holds a position.
    """

    def __init__(
        self,
        pin: int,
        min_pulse_width_s: float,
        max_pulse_width_s: float,
        max_angle_deg: float,
    ):
        """
        Open the servo pin; 0 deg is `min_pulse_width_s`.
        """
        # Only available on the Pi.
        from gpiozero import AngularServo

        self._dev = AngularServo(
            pin,
            min_angle=0.0,
            max_angle=max_angle_deg,
            min_pulse_width=min_pulse_width_s,
            max_pulse_width=max_pulse_width_s,
        )

    def move_to(self, angle_deg: float) -> None:
        """
        Move the horn to `angle_deg`.
        """
        self._dev.angle = angle_deg

    def close(self) -> None:
        """
        Release the pin (the servo goes limp).
        """
        self._dev.close()


class FakeServoMotor:
    """
    Fake servo: only remembers the last angle.
    """

    def __init__(self):
        """
        Start with no angle commanded.
        """
        self.angle_deg: float | None = None

    def move_to(self, angle_deg: float) -> None:
        """
        Remember the angle.
        """
        self.angle_deg = angle_deg

    def close(self) -> None:
        """
        Nothing to release.
        """


class Tcs34725:
    """
    TCS34725 RGB + clear light sensor on I2C (address 0x29).
    """

    def __init__(self, bus: int, integration_time_ms: float, gain: int):
        """
        Open the bus, check the chip ID, set timing and gain, and power up.
        """
        if gain not in TCS_GAINS:
            raise ValueError(f"gain must be one of {sorted(TCS_GAINS)}")
        # Only available on the Pi.
        from smbus2 import SMBus

        self._bus = SMBus(bus)
        chip_id = self._read_byte(TCS_REG_ID)
        if chip_id not in TCS_VALID_IDS:
            self._bus.close()
            raise OSError(f"no TCS34725 at 0x29 (id 0x{chip_id:02x})")
        cycles = round(integration_time_ms / TCS_ATIME_STEP_MS)
        self._write_byte(TCS_REG_ATIME, TCS_ATIME_CYCLES - cycles)
        self._write_byte(TCS_REG_CONTROL, TCS_GAINS[gain])
        self._write_byte(TCS_REG_ENABLE, TCS_ENABLE_PON)
        time.sleep(TCS_POWER_ON_DELAY_S)
        self._write_byte(TCS_REG_ENABLE, TCS_ENABLE_PON | TCS_ENABLE_AEN)

    def _read_byte(self, register: int) -> int:
        """
        Read one register.
        """
        return self._bus.read_byte_data(
            TCS_I2C_ADDRESS,
            TCS_COMMAND | register,
        )

    def _write_byte(self, register: int, value: int) -> None:
        """
        Write one register.
        """
        self._bus.write_byte_data(
            TCS_I2C_ADDRESS,
            TCS_COMMAND | register,
            value,
        )

    def read_rgbc(self) -> tuple[int, int, int, int]:
        """
        Raw (red, green, blue, clear) counts of the last integration.
        """
        data = self._bus.read_i2c_block_data(
            TCS_I2C_ADDRESS,
            TCS_COMMAND | TCS_AUTO_INCREMENT | TCS_REG_CDATAL,
            TCS_RGBC_BYTES,
        )
        clear, red, green, blue = (
            data[i] | (data[i + 1] << BYTE_BITS)
            for i in range(0, TCS_RGBC_BYTES, 2)
        )
        return red, green, blue, clear

    def close(self) -> None:
        """
        Power the chip down and close the bus.
        """
        self._write_byte(TCS_REG_ENABLE, 0)
        self._bus.close()


class FakeTcs34725:
    """
    Fake colour sensor that sees nothing (all counts zero).
    """

    def read_rgbc(self) -> tuple[int, int, int, int]:
        """
        No light: (0, 0, 0, 0).
        """
        return 0, 0, 0, 0

    def close(self) -> None:
        """
        Nothing to release.
        """
