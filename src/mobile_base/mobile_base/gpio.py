"""Thin lgpio wrapper for the Raspberry Pi 5 with a dry-run fallback.

lgpio is imported lazily so every module imports on machines without it (CI, laptops).
In dry-run mode writes are ignored and reads return 0.
"""


class Gpio:
    def __init__(self, chip: int, logger, dry_run: bool = False):
        self._lgpio = None
        self._handle = None
        self._claimed: list[int] = []
        if dry_run:
            logger.warning("GPIO dry run: no hardware access")
            return
        try:
            import lgpio  # noqa: PLC0415 - lazy on purpose
        except ImportError:
            logger.error("lgpio not available: falling back to GPIO dry run")
            return
        self._lgpio = lgpio
        self._handle = lgpio.gpiochip_open(chip)

    @property
    def dry_run(self) -> bool:
        return self._lgpio is None

    def claim_output(self, pin: int, level: int = 0) -> None:
        if self._lgpio:
            self._lgpio.gpio_claim_output(self._handle, pin, level)
            self._claimed.append(pin)

    def claim_input(self, pin: int) -> None:
        if self._lgpio:
            self._lgpio.gpio_claim_input(self._handle, pin)
            self._claimed.append(pin)

    def read(self, pin: int) -> int:
        if self._lgpio:
            return self._lgpio.gpio_read(self._handle, pin)
        return 0

    def pwm(self, pin: int, frequency: float, duty_percent: float) -> None:
        if self._lgpio:
            self._lgpio.tx_pwm(self._handle, pin, frequency, duty_percent)

    def close(self) -> None:
        if self._lgpio and self._handle is not None:
            for pin in self._claimed:
                self._lgpio.gpio_free(self._handle, pin)
            self._lgpio.gpiochip_close(self._handle)
            self._handle = None
