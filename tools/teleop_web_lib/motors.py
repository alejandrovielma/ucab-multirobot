"""
Motor drivers for the browser tool: a fake one for the laptop and the two
real HATs (Cokoino via gpiozero, SunFounder Fusion HAT+ via I2C).

Every driver has the same interface: `set(ratios)` with wheel ratios in
[-1, 1], and `close()`. The hardware libraries are imported lazily because
they only exist on the Pi.
"""

import logging
from pathlib import Path

from teleop_web_lib.kinematics import WHEELS, clamp

log = logging.getLogger("teleop_web")

# --- Config values ---------------------------------------------------------
DRIVER_FUSION_HAT = "fusion_hat"
DRIVER_GPIO = "gpio"

LABEL_FAKE = "fake"
LABEL_FAKE_MISSING_PINS = "fake (missing pins)"
LABEL_FUSION_HAT = "Fusion HAT+"
LABEL_GPIO_HAT = "HAT"

# nSLEEP pins of both DRV8833 (must be HIGH to enable the H-bridges).
DEFAULT_NSLEEP_PINS = (12, 13)

# Ratios are rounded to this many decimals so tiny changes do not rewrite
# the PWM registers.
RATIO_DECIMALS = 3
FAKE_LOG_DECIMALS = 2

# --- SunFounder Fusion HAT+ (MCU on I2C) -----------------------------------
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


class FakeMotors:
    """
    Fake motors: only logs what it would do.
    """

    def __init__(self):
        """
        Start with nothing logged yet.
        """
        self._last = None

    def set(self, ratios: dict[str, float]) -> None:
        """
        Log the ratios when they change.
        """
        rounded = tuple(round(ratios[w], FAKE_LOG_DECIMALS) for w in WHEELS)
        if rounded != self._last:
            self._last = rounded
            log.info("motors (fake) fl=%.2f fr=%.2f rl=%.2f rr=%.2f", *rounded)

    def close(self) -> None:
        """
        Nothing to release.
        """


class HatMotors:
    """
    4 motors on the Cokoino HAT (2x DRV8833) via gpiozero; uses lgpio on the
    Pi 5.
    """

    def __init__(self, cfg: dict):
        """
        Set up the nSLEEP pins and one PWM motor per wheel.
        """
        # Only available on the Pi.
        from gpiozero import Motor, OutputDevice

        nsleep_pins = cfg.get("nsleep_pins", DEFAULT_NSLEEP_PINS)
        self._nsleep = [
            OutputDevice(pin, active_high=True, initial_value=True)
            for pin in nsleep_pins
            if pin is not None
        ]
        self._motors = {}
        self._inverted = {}
        self._last = {}
        frequency_hz = cfg["pwm_frequency_hz"]
        for wheel in WHEELS:
            in1, in2 = cfg[f"{wheel}_pins"]
            motor = Motor(forward=in1, backward=in2, pwm=True)
            motor.forward_device.frequency = frequency_hz
            motor.backward_device.frequency = frequency_hz
            self._motors[wheel] = motor
            self._inverted[wheel] = bool(cfg[f"{wheel}_inverted"])
            self._last[wheel] = None

    def set(self, ratios: dict[str, float]) -> None:
        """
        Apply the wheel ratios, writing only the wheels that changed.
        """
        for wheel, motor in self._motors.items():
            ratio = -ratios[wheel] if self._inverted[wheel] else ratios[wheel]
            value = clamp(round(ratio, RATIO_DECIMALS))
            if value != self._last[wheel]:
                self._last[wheel] = value
                motor.value = value

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


class FusionHatMotors:
    """
    4 motors on the SunFounder Fusion HAT+ (MCU at I2C 0x17, PWM through
    registers).

    Each motor uses two PWM channels (a, b): a > 0 forward, b > 0 backward.
    """

    def __init__(self, cfg: dict):
        """
        Open the I2C bus and configure the PWM timers.
        """
        # Only available on the Pi.
        from smbus2 import SMBus

        self._bus = SMBus(FUSION_I2C_BUS)
        self._channels = {w: tuple(cfg["fusion_channels"][w]) for w in WHEELS}
        self._inverted = {w: bool(cfg[f"{w}_inverted"]) for w in WHEELS}
        self._last = dict.fromkeys(WHEELS)
        timers = {
            channel // FUSION_CHANNELS_PER_TIMER
            for pair in self._channels.values()
            for channel in pair
        }
        for timer in sorted(timers):
            self._write(FUSION_REG_PERIOD + timer, FUSION_PWM_PERIOD)
            self._write(FUSION_REG_PRESCALER + timer, FUSION_PWM_PRESCALER)
        self.set(dict.fromkeys(WHEELS, 0.0))

    def _write(self, register: int, value: int) -> None:
        """
        Write a 16-bit word to a register (the MCU expects it byte-swapped).
        """
        value &= WORD_MASK
        swapped = ((value & BYTE_MASK) << BYTE_BITS) | (value >> BYTE_BITS)
        self._bus.write_word_data(FUSION_I2C_ADDRESS, register, swapped)

    def set(self, ratios: dict[str, float]) -> None:
        """
        Apply the wheel ratios, writing only the wheels that changed.
        """
        for wheel, (channel_a, channel_b) in self._channels.items():
            ratio = -ratios[wheel] if self._inverted[wheel] else ratios[wheel]
            value = clamp(round(ratio, RATIO_DECIMALS))
            if value == self._last[wheel]:
                continue
            self._last[wheel] = value
            duty = int(abs(value) * FUSION_PWM_PERIOD)
            self._write(FUSION_REG_DUTY + channel_a, duty if value > 0 else 0)
            self._write(FUSION_REG_DUTY + channel_b, duty if value < 0 else 0)

    def close(self) -> None:
        """
        Stop the motors and close the I2C bus.
        """
        self.set(dict.fromkeys(WHEELS, 0.0))
        self._bus.close()


def pins_configured(cfg: dict) -> bool:
    """
    True when the selected driver has everything it needs to run.
    """
    if cfg.get("driver") == DRIVER_FUSION_HAT:
        return True
    return all(p is not None for w in WHEELS for p in cfg[f"{w}_pins"])


def build_motors(
    cfg: dict,
    fake: bool,
    config_path: Path,
) -> tuple[FakeMotors | HatMotors | FusionHatMotors, str]:
    """
    Pick the motor driver; returns (driver, label shown in the page).

    Falls back to the fake driver with a warning when the pins are not
    configured yet.
    """
    if fake:
        return FakeMotors(), LABEL_FAKE
    if not pins_configured(cfg):
        log.warning("missing pins in %s: motors in fake mode", config_path)
        return FakeMotors(), LABEL_FAKE_MISSING_PINS
    if cfg.get("driver") == DRIVER_FUSION_HAT:
        return FusionHatMotors(cfg), LABEL_FUSION_HAT
    return HatMotors(cfg), LABEL_GPIO_HAT
