"""Analog (AO) read of the TCRT5000 through the Fusion HAT+ ADC, and the black-run confirm filter.

Vendored from Ferb's tools/teleop_web_lib/line_sensor.py (same constants and logic, no gpiozero) so the
lab reads the same ADC the browser tool does. The register map comes from SunFounder's fusion-hat
driver; it is confirmed by Ferb's measured counts in docs/hardware.md (white ~285-299, black ~3383).
"""

import logging

log = logging.getLogger("lab")

ADC_I2C_ADDRESS = 0x17
ADC_REG_BASE = 0x10
ADC_REG_STRIDE = 2
ADC_I2C_BUS = 1
ADC_CHANNELS = 4  # A0-A3
ADC_MAX_COUNTS = 4095
ADC_REFERENCE_MV = 3300


def word_to_counts(smbus_word: int) -> int:
    """ADC counts in an SMBus word (arrives low byte first; the MCU sends the high byte first)."""
    return ((smbus_word & 0xFF) << 8) | (smbus_word >> 8)


def counts_to_millivolts(counts: int) -> int:
    """Input voltage (mV) for 12-bit counts with a 3.3 V reference."""
    return round(counts * ADC_REFERENCE_MV / (ADC_MAX_COUNTS + 1))


class BlackConfirm:
    """Reports black only after `confirm_reads` black reads in a row (a single read while moving is the
    sensor lifting off the table, not tape). 1 = no filtering."""

    def __init__(self, confirm_reads: int):
        self._confirm_reads = max(1, int(confirm_reads))
        self.streak = 0

    def update(self, raw_black: bool) -> bool:
        if raw_black:
            self.streak += 1
        elif self.streak:
            log.info("line sensor black run: %d reads (%s)", self.streak,
                     "line" if self.streak >= self._confirm_reads else "ignored")
            self.streak = 0
        return self.streak >= self._confirm_reads


class AnalogReader:
    """AO of the module on Fusion HAT+ input A<channel>. `bus` is an smbus2.SMBus (or a fake)."""

    def __init__(self, bus, channel: int):
        if not 0 <= int(channel) < ADC_CHANNELS:
            raise ValueError(f"adc_channel must be 0-{ADC_CHANNELS - 1}, got {channel}")
        self._bus = bus
        self._register = ADC_REG_BASE + ADC_REG_STRIDE * int(channel)

    def counts(self) -> int | None:
        """Counts 0-4095, or None when the read fails."""
        try:
            return word_to_counts(self._bus.read_word_data(ADC_I2C_ADDRESS, self._register))
        except OSError:
            return None
