"""
Bottom line sensor (TCRT5000) for the browser tool.
"""

import logging
from pathlib import Path

log = logging.getLogger("teleop_web")

# DO value over the black tape: the usual module gives 1 on black, 0 on white.
DEFAULT_BLACK_VALUE = 1
# Consecutive black reads needed to report black (1 = no filtering).
DEFAULT_CONFIRM_READS = 1

# --- Analog output (AO) read through the Fusion HAT+ ADC ----------------------
# The HAT's MCU (I2C 0x17) has four 12-bit inputs A0-A3; input N is the word
# at ADC_REG_BASE + 2 * N, high byte first (from SunFounder's fusion-hat
# driver, v1 branch; confirmed here only by reading plausible counts).
ADC_I2C_ADDRESS = 0x17
ADC_REG_BASE = 0x10
ADC_REG_STRIDE = 2
ADC_I2C_BUS = 1
ADC_MAX_COUNTS = 4095
ADC_REFERENCE_MV = 3300
BYTE_BITS = 8
BYTE_MASK = 0xFF


def word_to_counts(smbus_word: int) -> int:
    """
    The ADC counts in an SMBus word, which arrives low byte first while the
    MCU sends the high byte first.
    """
    return ((smbus_word & BYTE_MASK) << BYTE_BITS) | (smbus_word >> BYTE_BITS)


def counts_to_millivolts(counts: int) -> int:
    """
    The input voltage (mV) for 12-bit ADC counts with a 3.3 V reference.
    """
    return round(counts * ADC_REFERENCE_MV / (ADC_MAX_COUNTS + 1))


def reads_black(
    do_value: int,
    black_value: int,
    ao_mv: int | None,
    black_mv: int | None,
) -> bool:
    """
    True when one read is over the black tape: the DO output says so, or
    the AO voltage is at least `black_mv` (when both are known). The DO
    threshold (the potentiometer) sat right at the tape's level, so the
    robot crossed the border without seeing it.
    """
    if do_value == black_value:
        return True
    return ao_mv is not None and black_mv is not None and ao_mv >= black_mv


class BlackConfirm:
    """
    Reports black only after `confirm_reads` black reads in a row: a single
    black read while moving is the sensor lifting off the table, not tape.
    """

    def __init__(self, confirm_reads: int):
        """
        Start on white.
        """
        self._confirm_reads = max(1, int(confirm_reads))
        self.streak = 0  # black reads in a row so far

    def update(self, raw_black: bool) -> bool:
        """
        Feed one read; True once the black streak is long enough.
        """
        if raw_black:
            self.streak += 1
        elif self.streak:
            # Run lengths tune confirm_reads: tape lasts ~3 reads at full
            # speed (19 mm at ~0.31 m/s, 50 Hz), a lift-off blip fewer.
            log.info(
                "line sensor black run: %d reads (%s)",
                self.streak,
                "line" if self.streak >= self._confirm_reads else "ignored",
            )
            self.streak = 0
        return self.streak >= self._confirm_reads


class LineSensor:
    """
    Bottom line sensor (TCRT5000, digital DO output) read with gpiozero.

    Power it from 3.3 V: at 5 V the DO output puts 5 V on the Pi's GPIO.
    """

    def __init__(self, cfg: dict):
        """
        Open the GPIO pin of the sensor.
        """
        # Only available on the Pi.
        from gpiozero import DigitalInputDevice

        self._dev = DigitalInputDevice(cfg["pin"], pull_up=False)
        self._black_value = int(cfg.get("black_value", DEFAULT_BLACK_VALUE))
        self._confirm = BlackConfirm(
            cfg.get("confirm_reads", DEFAULT_CONFIRM_READS),
        )
        self._adc_channel = cfg.get("adc_channel")
        self._black_mv = cfg.get("black_mv")
        self._bus = None
        if self._adc_channel is not None:
            from smbus2 import SMBus

            self._bus = SMBus(ADC_I2C_BUS)

    def is_black(self) -> bool:
        """
        True when the sensor is over the black line (confirmed over
        `confirm_reads` calls; call it once per control iteration).
        """
        ao_mv = None
        if self._black_mv is not None:
            counts = self.analog_counts()
            if counts is not None:
                ao_mv = counts_to_millivolts(counts)
        return self._confirm.update(
            reads_black(
                self._dev.value, self._black_value, ao_mv, self._black_mv
            ),
        )

    def raw_value(self) -> int:
        """
        Raw digital output (0 or 1) from the DO pin.
        """
        return int(self._dev.value)

    def analog_counts(self) -> int | None:
        """
        ADC counts (0-4095) of the module's AO output, or None when AO is
        not wired (no `adc_channel`) or the read fails.
        """
        if self._bus is None:
            return None
        register = ADC_REG_BASE + ADC_REG_STRIDE * int(self._adc_channel)
        try:
            word = self._bus.read_word_data(ADC_I2C_ADDRESS, register)
        except OSError:
            return None
        return word_to_counts(word)

    def close(self) -> None:
        """
        Release the GPIO pin and the I2C bus.
        """
        self._dev.close()
        if self._bus is not None:
            self._bus.close()


def build_line_sensor(
    cfg: dict | None,
    fake: bool,
    config_path: Path,
) -> LineSensor | None:
    """
    The line sensor, or None when running fake or when no pin is set.
    """
    if fake:
        return None
    cfg = cfg or {}
    if cfg.get("pin") is None:
        log.warning(
            "line_sensor.pin is empty in %s: black line not detected",
            config_path,
        )
        return None
    return LineSensor(cfg)
