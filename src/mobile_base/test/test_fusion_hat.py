"""SunFounder Fusion HAT+ motor backend: wheel -> port mapping and duty -> power percent.

The HAT MCU takes a power in [-100, 100] per port (M0-M3); the 2S battery (8.4 V full) must not
push the 6 V TT motors past their rating, so the duty cap of core/kinematics.py still applies."""

import pytest

from mobile_base.core.fusion_hat import DEFAULT_PORTS, FusionMotorBank, duty_to_power, wiring_from
from mobile_base.core.kinematics import duty_cap

CAP = duty_cap(8.4, 6.0)  # 0.714


def test_default_wiring_is_m0_to_m3_in_wheel_order_without_reverse():
    wiring = wiring_from(DEFAULT_PORTS, (False, False, False, False))
    assert wiring == {"fl": ("M0", False), "fr": ("M1", False), "rl": ("M2", False), "rr": ("M3", False)}


def test_wiring_from_config_lists():
    wiring = wiring_from(["M2", "M3", "M0", "M1"], [True, False, True, False])
    assert wiring["fl"] == ("M2", True)
    assert wiring["rr"] == ("M1", False)


@pytest.mark.parametrize(
    "ports, reversed_",
    [
        (["M0", "M1", "M2"], [False] * 3),  # one wheel missing
        (["M0", "M1", "M2", "M4"], [False] * 4),  # no such port
        (["M0", "M0", "M2", "M3"], [False] * 4),  # two wheels on one port
        (["M0", "M1", "M2", "M3"], [False] * 3),
    ],
)
def test_bad_wiring_is_rejected(ports, reversed_):
    with pytest.raises(ValueError):
        wiring_from(ports, reversed_)


def test_duty_maps_to_percent_with_the_battery_cap():
    assert CAP == pytest.approx(0.714, abs=1e-3)
    assert duty_to_power(CAP, CAP) == pytest.approx(71.43, abs=0.01)
    assert duty_to_power(-0.5, CAP) == pytest.approx(-50.0)
    assert duty_to_power(0.0, CAP) == 0.0
    assert duty_to_power(0.9, CAP) == pytest.approx(CAP * 100.0)  # clamped to the cap


def test_start_power_spreads_non_zero_duties_over_start_to_cap():
    # A TT motor stalls below its start power: 0.12 m/s (25 % power) would barely move the robot.
    top = CAP * 100.0
    assert duty_to_power(CAP, CAP, start=30.0) == pytest.approx(top)  # the cap is unchanged
    assert duty_to_power(CAP / 2, CAP, start=30.0) == pytest.approx(30.0 + (top - 30.0) / 2)
    assert duty_to_power(-CAP / 2, CAP, start=30.0) == pytest.approx(-(30.0 + (top - 30.0) / 2))
    assert duty_to_power(0.0, CAP, start=30.0) == 0.0  # stopped stays stopped


def test_tiny_duties_are_zero_instead_of_jumping_to_the_start_power():
    assert duty_to_power(0.01 * CAP, CAP, start=30.0) == 0.0
    assert duty_to_power(0.05 * CAP, CAP, start=30.0) > 30.0


def test_trim_shortens_one_wheel_span_and_never_passes_the_cap():
    top = CAP * 100.0
    assert duty_to_power(CAP, CAP, start=30.0, trim=0.8) == pytest.approx(30.0 + (top - 30.0) * 0.8)
    assert duty_to_power(CAP, CAP, start=95.0) == pytest.approx(top)  # a start above the cap is clamped


@pytest.mark.parametrize(
    "start, trim",
    [
        ([30.0] * 3, [1.0] * 4),  # one wheel missing
        ([30.0, 30.0, 30.0, -1.0], [1.0] * 4),
        ([30.0, 30.0, 30.0, 100.0], [1.0] * 4),
        ([30.0] * 4, [1.0, 1.0, 1.0, 1.2]),  # trim may only slow a wheel down
        ([30.0] * 4, [1.0, 1.0, 1.0, 0.0]),
    ],
)
def test_bad_calibration_is_rejected(start, trim):
    with pytest.raises(ValueError):
        FusionMotorBank(PowerMotor, wiring_from(DEFAULT_PORTS, (False,) * 4), start_power=start, trim=trim)


def test_bank_applies_each_wheel_start_power_and_trim():
    bank = FusionMotorBank(PowerMotor, wiring_from(DEFAULT_PORTS, (False,) * 4),
                           start_power=(20.0, 30.0, 25.0, 35.0), trim=(1.0, 0.9, 1.0, 1.0))
    bank.apply((CAP / 2,) * 4, CAP)
    top = CAP * 100.0
    assert bank.motors["fl"].calls[-1] == ("power", pytest.approx(20.0 + (top - 20.0) / 2))
    assert bank.motors["fr"].calls[-1] == ("power", pytest.approx(30.0 + (top - 30.0) * 0.9 / 2))
    assert bank.motors["rr"].calls[-1] == ("power", pytest.approx(35.0 + (top - 35.0) / 2))


def test_open_bank_raises_on_a_bad_calibration_instead_of_running_dry():
    from mobile_base.core.fusion_hat import open_bank

    with pytest.raises(ValueError):
        open_bank(PowerMotor, wiring_from(DEFAULT_PORTS, (False,) * 4), Log(), start_power=(30.0,) * 3)


@pytest.mark.parametrize("duty", [-5.0, -1.0, 1.0, 3.0])
def test_power_never_exceeds_100_percent(duty):
    assert -100.0 <= duty_to_power(duty, 1.0) <= 100.0
    assert -100.0 <= duty_to_power(duty, 2.0) <= 100.0  # a wrong cap still cannot exceed the MCU range


class PowerMotor:
    """Fake of fusion_hat.motor.Motor with the documented power(value) method."""

    def __init__(self, port):
        self.port = port
        self.calls = []

    def power(self, value):
        self.calls.append(("power", value))

    def set_is_reverse(self, flag):
        self.calls.append(("reverse", flag))

    def stop(self):
        self.calls.append(("stop",))


class SetPowerMotor(PowerMotor):
    """The vendor example calls set_power(value) instead."""

    power = None

    def set_power(self, value):
        self.calls.append(("set_power", value))


def test_bank_sets_reverse_flags_and_drives_each_port():
    bank = FusionMotorBank(PowerMotor, wiring_from(DEFAULT_PORTS, (False, True, False, False)))
    assert bank.motors["fr"].calls == [("reverse", True)]
    bank.apply((CAP, -0.2, 0.0, 0.9), CAP)
    assert bank.motors["fl"].port == "M0"
    assert bank.motors["fl"].calls[-1] == ("power", pytest.approx(71.43, abs=0.01))
    assert bank.motors["fr"].calls[-1] == ("power", pytest.approx(-20.0))
    assert bank.motors["rr"].calls[-1] == ("power", pytest.approx(CAP * 100.0))


def test_bank_falls_back_to_set_power_and_stops_every_motor():
    bank = FusionMotorBank(SetPowerMotor, wiring_from(DEFAULT_PORTS, (False,) * 4))
    bank.apply((0.3, 0.3, 0.3, 0.3), CAP)
    assert bank.motors["rl"].calls[-1] == ("set_power", pytest.approx(30.0))
    bank.stop()
    assert all(m.calls[-1] == ("stop",) for m in bank.motors.values())


def test_missing_library_means_dry_run_with_an_error(monkeypatch):
    import sys

    from mobile_base.core.fusion_hat import load_motor_class

    errors = []

    class Logger:
        def error(self, msg):
            errors.append(msg)

    monkeypatch.setitem(sys.modules, "fusion_hat", None)  # import fails as if not installed
    assert load_motor_class(Logger()) is None
    assert errors and "fusion_hat" in errors[0]


class Log:
    def __init__(self):
        self.errors = []

    def error(self, msg):
        self.errors.append(msg)


class DeadHat(PowerMotor):
    """The library is installed but the HAT does not answer on I2C."""

    def __init__(self, port):
        raise OSError("[Errno 121] Remote I/O error")


class FlakyMotor(PowerMotor):
    """A transient I2C fault on one power() call."""

    def power(self, value):
        if self.port == "M1" and value != 0.0:
            raise OSError("[Errno 5] Input/output error")
        super().power(value)


def test_a_hat_that_does_not_answer_means_dry_run_with_an_error():
    from mobile_base.core.fusion_hat import open_bank

    log = Log()
    assert open_bank(DeadHat, wiring_from(DEFAULT_PORTS, (False,) * 4), log) is None
    assert log.errors and "dry run" in log.errors[0]


def test_an_i2c_fault_while_driving_is_logged_and_stops_the_motors_instead_of_raising():
    from mobile_base.core.fusion_hat import open_bank

    log = Log()
    bank = open_bank(FlakyMotor, wiring_from(DEFAULT_PORTS, (False,) * 4), log)
    assert bank.apply((0.3, 0.3, 0.3, 0.3), CAP) is False  # M1 (fr) failed, no exception
    assert log.errors and "fr" in log.errors[0]
    assert all(m.calls[-1] == ("stop",) for w, m in bank.motors.items() if w != "fr")
    assert bank.apply((0.0, 0.0, 0.0, 0.0), CAP) is True
