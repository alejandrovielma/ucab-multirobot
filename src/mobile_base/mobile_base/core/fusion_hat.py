"""SunFounder Fusion HAT+ motor backend: wheel -> port mapping and duty -> power percent.

The HAT's MCU (I2C 0x17) generates the PWM of its four motor ports M0-M3 (100 Hz by default); the
Python library `fusion_hat` drives them with `Motor('M0').power(value)`, value in [-100, 100]
(the vendor example calls `set_power(value)` instead: both are supported) and flips a port with
`set_is_reverse(bool)`. The battery (2S Li-ion, 8.4 V full) feeds the ports directly, so the duty
cap of core/kinematics.duty_cap still keeps the 6 V TT motors at their rating: duty in
[-cap, cap] -> percent = duty * 100.

The library is imported lazily (load_motor_class) so this module, and everything importing it,
works on machines without the HAT (CI, laptops, Webots).
"""

from collections.abc import Callable, Mapping, Sequence

WHEELS = ("fl", "fr", "rl", "rr")
PORTS = ("M0", "M1", "M2", "M3")
DEFAULT_PORTS = ("M0", "M1", "M2", "M3")  # fl, fr, rl, rr: wiring not verified yet (lab)

Wiring = dict[str, tuple[str, bool]]  # wheel -> (port, reversed)


def wiring_from(ports: Sequence[str], reversed_: Sequence[bool]) -> Wiring:
    """Wheel -> (port, reversed) from the fusion_ports / fusion_reversed lists (order fl, fr, rl, rr)."""
    ports, reversed_ = list(ports), list(reversed_)
    if len(ports) != len(WHEELS) or len(reversed_) != len(WHEELS):
        raise ValueError(f"fusion_ports and fusion_reversed need {len(WHEELS)} entries ({', '.join(WHEELS)})")
    unknown = [port for port in ports if port not in PORTS]
    if unknown:
        raise ValueError(f"unknown Fusion HAT+ motor port(s) {unknown}: use {', '.join(PORTS)}")
    if len(set(ports)) != len(ports):
        raise ValueError(f"two wheels on the same port: {ports}")
    return {wheel: (port, bool(flag)) for wheel, port, flag in zip(WHEELS, ports, reversed_)}


def duty_to_power(duty: float, cap: float) -> float:
    """Signed duty (already capped by to_duty) -> Fusion HAT+ power percent, never past the cap
    nor past the MCU range [-100, 100]."""
    limit = max(0.0, min(1.0, cap))
    return max(-limit, min(limit, duty)) * 100.0


class FusionMotorBank:
    """The four motors of the HAT. motor_class is fusion_hat.motor.Motor (or a fake in tests)."""

    def __init__(self, motor_class: Callable, wiring: Mapping[str, tuple[str, bool]], logger=None):
        self.wiring = dict(wiring)
        self.logger = logger
        self.motors = {}
        for wheel in WHEELS:
            port, flipped = self.wiring[wheel]
            motor = motor_class(port)
            if hasattr(motor, "set_is_reverse"):
                motor.set_is_reverse(flipped)
            self.motors[wheel] = motor

    def set_power(self, wheel: str, percent: float) -> None:
        motor = self.motors[wheel]
        if callable(getattr(motor, "power", None)):
            motor.power(percent)
        else:
            motor.set_power(percent)

    def _guarded(self, wheel: str, action: Callable[[], None]) -> bool:
        # An I2C fault must not escape into the ROS callback: the executor would stop servicing the
        # cmd_vel watchdog that exists to stop the wheels.
        try:
            action()
            return True
        except Exception as exc:  # noqa: BLE001 - any library/bus error, logged
            if self.logger is not None:
                self.logger.error(f"Fusion HAT+ motor {wheel} ({self.wiring[wheel][0]}) failed: {exc}")
            return False

    def apply(self, duties: Sequence[float], cap: float) -> bool:
        """False when a port failed; the other motors are then stopped (best effort)."""
        ok = all([self._guarded(wheel, lambda w=wheel, d=duty: self.set_power(w, duty_to_power(d, cap)))
                  for wheel, duty in zip(WHEELS, duties)])
        if not ok:
            self.stop()
        return ok

    def stop(self) -> bool:
        def stop_one(wheel, motor):
            if callable(getattr(motor, "stop", None)):
                motor.stop()
            else:
                self.set_power(wheel, 0.0)

        return all([self._guarded(wheel, lambda w=wheel, m=motor: stop_one(w, m)) for wheel, motor in self.motors.items()])


def open_bank(motor_class: Callable | None, wiring: Mapping[str, tuple[str, bool]], logger):
    """FusionMotorBank, or None (logged: the caller runs dry) without the library or when the HAT
    does not answer while the motors are created."""
    if motor_class is None:
        return None
    try:
        return FusionMotorBank(motor_class, wiring, logger)
    except Exception as exc:  # noqa: BLE001 - I2C error, HAT absent or not powered
        logger.error(f"Fusion HAT+ not answering ({exc}): motors in dry run")
        return None


def load_motor_class(logger):
    """fusion_hat.motor.Motor, or None (logged) when the library is not installed."""
    try:
        from fusion_hat.motor import Motor  # noqa: PLC0415 - lazy on purpose
    except ImportError as exc:
        logger.error(f"fusion_hat not available ({exc}): motors in dry run")
        return None
    return Motor
