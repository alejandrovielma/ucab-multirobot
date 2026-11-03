"""
Motor hardware for motor_driver: a fake for the laptop and the two HATs
(Cokoino 2x DRV8833 through gpiozero, SunFounder Fusion HAT+ through I2C).

Every driver takes wheel ratios in [-1, 1] (`set`) and releases the
hardware in `close`. The hardware libraries are imported inside the classes
because they only exist on the Pi. These are the same drivers as
tools/teleop_web_lib/motors.py, which is not a ROS package.
"""

from dataclasses import dataclass, field

from mobile_base.kinematics import WHEELS
from mobile_base.mathutil import clamp

# --- Driver names (parameter `driver`) ----------------------------------------
DRIVER_FAKE = "fake"
DRIVER_GPIO = "gpio"
DRIVER_FUSION_HAT = "fusion_hat"

# Duty cycles are rounded to this many decimals so tiny changes do not
# rewrite the PWM outputs.
DUTY_DECIMALS = 3

# --- SunFounder Fusion HAT+ (MCU on I2C) --------------------------------------
FUSION_I2C_BUS = 1
FUSION_I2C_ADDRESS = 0x17
# PWM timer: 72 MHz / (prescaler + 1) / (period + 1) = 100 Hz.
FUSION_PWM_PERIOD = 39999
FUSION_PWM_PRESCALER = 17
# Register banks; the timer or channel number is added to the base.
FUSION_REG_PERIOD = 0x50
FUSION_REG_PRESCALER = 0x40
FUSION_REG_DUTY = 0x60
# Channels P4-P7 share timer 1 and P8-P11 share timer 2.
FUSION_CHANNELS_PER_TIMER = 4
WORD_MASK = 0xFFFF
BYTE_MASK = 0xFF
BYTE_BITS = 8

# --- Defaults (real values: config/collector.yaml) ----------------------------
DEFAULT_DRIVER = DRIVER_FUSION_HAT
DEFAULT_PWM_FREQUENCY_HZ = 200
DEFAULT_MAX_DUTY_RATIO = 0.9


@dataclass
class MotorHardwareParams:
    """
    Wiring of the 4 motors (order fl, fr, rl, rr) for both HATs.

    Field names double as ROS parameter names (see recolector_common.params).
    """

    driver: str = DEFAULT_DRIVER
    # Fusion HAT+: PWM channels (a, b) per wheel; a > 0 forward.
    fl_channels: list[int] = field(default_factory=lambda: [11, 10])
    fr_channels: list[int] = field(default_factory=lambda: [9, 8])
    rl_channels: list[int] = field(default_factory=lambda: [4, 5])
    rr_channels: list[int] = field(default_factory=lambda: [6, 7])
    # Cokoino HAT: GPIO (BCM) IN1 / IN2 per wheel and both nSLEEP pins.
    fl_pins: list[int] = field(default_factory=lambda: [24, 25])
    fr_pins: list[int] = field(default_factory=lambda: [26, 16])
    rl_pins: list[int] = field(default_factory=lambda: [17, 27])
    rr_pins: list[int] = field(default_factory=lambda: [22, 23])
    nsleep_pins: list[int] = field(default_factory=lambda: [12, 13])
    # A wheel that spins backwards is inverted here, not in the wiring.
    fl_inverted: bool = True
    fr_inverted: bool = False
    rl_inverted: bool = True
    rr_inverted: bool = False
    pwm_frequency_hz: int = DEFAULT_PWM_FREQUENCY_HZ
    # Duty cycle cap (0-1).
    max_duty_ratio: float = DEFAULT_MAX_DUTY_RATIO


def to_duty(ratio: float, inverted: bool, max_duty_ratio: float) -> float:
    """
    Signed duty cycle in [-max_duty_ratio, max_duty_ratio] for a wheel
    ratio in [-1, 1], flipped when the wheel is inverted.
    """
    signed = -ratio if inverted else ratio
    return round(clamp(signed, -1.0, 1.0) * max_duty_ratio, DUTY_DECIMALS)


class Motors:
    """
    Shared part of every driver: inversion, duty cap and change detection.
    """

    def __init__(self, p: MotorHardwareParams):
        """
        Start with every wheel unknown, so the first `set` writes them all.
        """
        self._p = p
        self._last: dict[str, float | None] = dict.fromkeys(WHEELS)

    def set(self, ratios: dict[str, float]) -> None:
        """
        Apply the wheel ratios, writing only the wheels that changed.
        """
        for wheel in WHEELS:
            inverted = getattr(self._p, f"{wheel}_inverted")
            duty = to_duty(ratios[wheel], inverted, self._p.max_duty_ratio)
            if duty != self._last[wheel]:
                self._last[wheel] = duty
                self._write(wheel, duty)

    def stop(self) -> None:
        """
        Brake every wheel.
        """
        self.set(dict.fromkeys(WHEELS, 0.0))

    def close(self) -> None:
        """
        Brake and release the hardware.
        """
        self.stop()

    def _write(self, wheel: str, duty: float) -> None:
        """
        Send one wheel's signed duty cycle to the hardware.
        """
        raise NotImplementedError


class FakeMotors(Motors):
    """
    Fake motors for the laptop: they only remember the last duty cycles.
    """

    def __init__(self, p: MotorHardwareParams):
        """
        Start with every wheel stopped.
        """
        super().__init__(p)
        self.duties = dict.fromkeys(WHEELS, 0.0)

    def _write(self, wheel: str, duty: float) -> None:
        """
        Remember the duty cycle instead of driving a motor.
        """
        self.duties[wheel] = duty


class GpioMotors(Motors):
    """
    4 motors on the Cokoino HAT (2x DRV8833) via gpiozero (lgpio on the
    Pi 5).
    """

    def __init__(self, p: MotorHardwareParams):
        """
        Enable both H-bridges and set up one PWM motor per wheel.
        """
        super().__init__(p)
        # Only available on the Pi.
        from gpiozero import Motor, OutputDevice

        self._nsleep = [
            OutputDevice(pin, active_high=True, initial_value=True)
            for pin in p.nsleep_pins
        ]
        self._motors = {}
        for wheel in WHEELS:
            in1, in2 = getattr(p, f"{wheel}_pins")
            motor = Motor(forward=in1, backward=in2, pwm=True)
            motor.forward_device.frequency = p.pwm_frequency_hz
            motor.backward_device.frequency = p.pwm_frequency_hz
            self._motors[wheel] = motor

    def _write(self, wheel: str, duty: float) -> None:
        """
        Set the gpiozero motor value (-1 backward .. 1 forward).
        """
        self._motors[wheel].value = duty

    def close(self) -> None:
        """
        Stop and release the motors and the nSLEEP pins.
        """
        for motor in self._motors.values():
            motor.stop()
            motor.close()
        for pin in self._nsleep:
            pin.off()
            pin.close()


class FusionHatMotors(Motors):
    """
    4 motors on the SunFounder Fusion HAT+ (MCU at I2C 0x17, PWM through
    registers). Each motor uses two channels (a, b): a forward, b backward.
    """

    def __init__(self, p: MotorHardwareParams):
        """
        Open the I2C bus, configure the PWM timers and stop the wheels.
        """
        super().__init__(p)
        # Only available on the Pi.
        from smbus2 import SMBus

        self._bus = SMBus(FUSION_I2C_BUS)
        self._channels = {w: getattr(p, f"{w}_channels") for w in WHEELS}
        timers = {
            channel // FUSION_CHANNELS_PER_TIMER
            for pair in self._channels.values()
            for channel in pair
        }
        for timer in sorted(timers):
            self._write_word(FUSION_REG_PERIOD + timer, FUSION_PWM_PERIOD)
            self._write_word(
                FUSION_REG_PRESCALER + timer,
                FUSION_PWM_PRESCALER,
            )
        self.stop()

    def _write_word(self, register: int, value: int) -> None:
        """
        Write a 16-bit word to a register (the MCU expects it byte-swapped).
        """
        value &= WORD_MASK
        swapped = ((value & BYTE_MASK) << BYTE_BITS) | (value >> BYTE_BITS)
        self._bus.write_word_data(FUSION_I2C_ADDRESS, register, swapped)

    def _write(self, wheel: str, duty: float) -> None:
        """
        Put the duty cycle on channel a (forward) or b (backward).
        """
        channel_a, channel_b = self._channels[wheel]
        counts = int(abs(duty) * FUSION_PWM_PERIOD)
        self._write_word(
            FUSION_REG_DUTY + channel_a,
            counts if duty > 0 else 0,
        )
        self._write_word(
            FUSION_REG_DUTY + channel_b,
            counts if duty < 0 else 0,
        )

    def close(self) -> None:
        """
        Stop the motors and close the I2C bus.
        """
        self.stop()
        self._bus.close()


DRIVERS = {
    DRIVER_FAKE: FakeMotors,
    DRIVER_GPIO: GpioMotors,
    DRIVER_FUSION_HAT: FusionHatMotors,
}


def build_motors(p: MotorHardwareParams) -> Motors:
    """
    The driver named by `p.driver`.
    """
    if p.driver not in DRIVERS:
        raise ValueError(f"unknown motor driver {p.driver!r}")
    return DRIVERS[p.driver](p)
