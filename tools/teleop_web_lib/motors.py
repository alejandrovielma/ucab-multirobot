"""
Motor drivers for the browser tool: a fake one for the laptop and the two
real HATs (Cokoino via gpiozero, SunFounder Fusion HAT+ via its library).

Every driver has the same interface: `set(ratios)` with wheel ratios in
[-1, 1], and `close()`. The hardware libraries are imported lazily because
they only exist on the Pi.
"""

import logging
import math
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

# --- SunFounder Fusion HAT+ (fusion_hat library) ----------------------------
# PWM channel pair (a, b) of each motor port.
FUSION_PORTS = {(11, 10): "M0", (9, 8): "M1", (6, 7): "M2", (4, 5): "M3"}
FUSION_MAX_POWER = 100
# Below this many percent the command is noise, not a move.
FUSION_DEADZONE_PCT = 2.0


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


def with_start_power(
    power: float, start_power: float, deadzone: float = FUSION_DEADZONE_PCT
) -> float:
    """
    The power (percent) raised to start_power when it is too weak to turn the
    wheel (TT motors do not turn below ~20 %), keeping its sign.

    start_power 0 = off; a power under the dead zone is noise and becomes 0.
    """
    if not start_power:
        return power
    if abs(power) < deadzone:
        return 0.0
    return math.copysign(max(abs(power), start_power), power)


class FusionHatMotors:
    """
    4 motors on the SunFounder Fusion HAT+, driven through the `fusion_hat`
    library (its kernel driver owns the I2C chip, so raw I2C access fails on
    Ubuntu with Python 3.14).

    Each wheel is the port that matches its (a, b) PWM channel pair.
    """

    def __init__(self, cfg: dict):
        """
        Open the four motor ports, flipping the inverted wheels.
        """
        # Only available on the Pi.
        from fusion_hat.motor import Motor

        self._motors = {}
        for wheel in WHEELS:
            motor = Motor(FUSION_PORTS[tuple(cfg["fusion_channels"][wheel])])
            motor.set_is_reverse(bool(cfg[f"{wheel}_inverted"]))
            self._motors[wheel] = motor
        self._start_power = float(cfg.get("start_power_pct", 0.0))
        self._last = dict.fromkeys(WHEELS)
        self.set(dict.fromkeys(WHEELS, 0.0))

    def set(self, ratios: dict[str, float]) -> None:
        """
        Apply the wheel ratios, writing only the wheels that changed.
        """
        for wheel, motor in self._motors.items():
            value = clamp(round(ratios[wheel], RATIO_DECIMALS))
            if value == self._last[wheel]:
                continue
            self._last[wheel] = value
            motor.power(with_start_power(value * FUSION_MAX_POWER, self._start_power))

    def close(self) -> None:
        """
        Stop the motors.
        """
        self.set(dict.fromkeys(WHEELS, 0.0))


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
