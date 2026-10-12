"""Mecanum inverse kinematics (X roller layout, Taheri et al. 2015).

Conventions: wheel order FL, FR, RL, RR; vx forward, vy positive to the left,
wz positive counter-clockwise. Wheel speeds are in rad/s.
"""

from dataclasses import dataclass

WheelSpeeds = tuple[float, float, float, float]


@dataclass(frozen=True)
class MecanumGeometry:
    wheel_radius: float = 0.0325  # m, 65 mm Hiwonder mecanum wheel
    lx: float = 0.0605  # m, half wheelbase
    ly: float = 0.0655  # m, half track


def wheel_speeds(vx: float, vy: float, wz: float, geom: MecanumGeometry = MecanumGeometry()) -> WheelSpeeds:
    k = (geom.lx + geom.ly) * wz
    r = geom.wheel_radius
    return (
        (vx - vy - k) / r,
        (vx + vy + k) / r,
        (vx + vy - k) / r,
        (vx - vy + k) / r,
    )


def normalize(speeds: WheelSpeeds, max_speed: float) -> WheelSpeeds:
    """Scale all wheels by the same factor so none exceeds max_speed (keeps the motion direction)."""
    peak = max(abs(s) for s in speeds)
    if peak <= max_speed or peak == 0.0:
        return tuple(speeds)
    factor = max_speed / peak
    return tuple(s * factor for s in speeds)


def duty_cap(v_battery: float, v_motor_max: float = 6.0) -> float:
    """Largest PWM duty that keeps the average motor voltage under v_motor_max."""
    if v_battery <= 0.0:
        raise ValueError("battery voltage must be positive")
    return min(1.0, v_motor_max / v_battery)


def to_duty(speeds: WheelSpeeds, max_wheel_speed: float, duty_cap: float) -> WheelSpeeds:
    """Map wheel speeds (rad/s) to signed duties in [-duty_cap, duty_cap].

    max_wheel_speed is the wheel speed reached at duty_cap (motor at its rated voltage).
    """
    def one(speed: float) -> float:
        duty = speed / max_wheel_speed * duty_cap
        return max(-duty_cap, min(duty_cap, duty))

    return tuple(one(s) for s in speeds)


def hbridge_pwm(duty: float) -> tuple[float, float]:
    """DRV8833 fast-decay drive: (IN1 %, IN2 %). Forward = PWM on IN1 with IN2 low."""
    percent = min(1.0, abs(duty)) * 100.0
    if duty > 0.0:
        return (percent, 0.0)
    if duty < 0.0:
        return (0.0, percent)
    return (0.0, 0.0)
