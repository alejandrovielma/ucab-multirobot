"""
Manual control of the collector for testing the model in Webots.

This is not the robot's logic: it is used to check that motors, scoop and
sensors respond like the real robot before writing the ROS 2 nodes.

Keys (click on the 3D view first):
  W / S        forward / backward
  A / D        strafe left / right (mecanum wheels)
  Q / E        turn left / right
  P            raise / lower the scoop
  + / -        more / less speed
  space        stop
"""

import colorsys
import math
import sys
from typing import NamedTuple

from controller import Keyboard, Robot

# --- Hiwonder chassis measurements (see RecolectorUCAB.proto) -----------------
WHEEL_RADIUS_M = 0.0325
HALF_WHEELBASE_M = 0.0605  # half the wheelbase
HALF_TRACK_M = 0.0655  # half the track width
MAX_MOTOR_SPEED_RAD_S = 10.5  # TT 1:120 at ~6 V
# With a 2S battery, limit to ~70 % so the 6 V TT motors are not overdriven.
LIMIT_2S = 0.7

# --- Scoop --------------------------------------------------------------------
SCOOP_DOWN_RAD = 0.0
SCOOP_UP_RAD = -0.7  # ~40 degrees
# Scoop IR reading above this means a cube is inside.
SCOOP_IR_THRESHOLD = 0.5

# --- Simulated line sensor (TCRT5000): white ~240, black line ~0-30 -----------
LINE_THRESHOLD = 120

# --- Devices in RecolectorUCAB.proto (Spanish names set by the Webots model) --
WHEELS = ("fl", "fr", "rl", "rr")
MOTOR_DEVICE_FORMAT = "motor_{wheel}"
LINE_DEVICE_FORMAT = "linea_{wheel}"
SCOOP_SERVO_DEVICE = "servo_pala"
ULTRASONIC_DEVICE = "ultrasonido"
SCOOP_IR_DEVICE = "ir_pala"
SCOOP_COLOR_DEVICE = "color_pala"
CAMERA_DEVICE = "camara"
ACCELEROMETER_DEVICE = "acelerometro"
GYRO_DEVICE = "giroscopio"
# Slow sensors are read every this many simulation steps.
SLOW_SENSOR_STEPS = 4
GYRO_YAW_INDEX = 2
MS_PER_S = 1000.0

# --- Speed control ------------------------------------------------------------
INITIAL_SPEED_SCALE = 0.5
SPEED_STEP = 0.01
MIN_SPEED_SCALE = 0.1
MAX_SPEED_SCALE = 1.0

# --- Keys: letters (matched upper case) and special keys ----------------------
ASCII_LIMIT = 128
ASCII_MASK = 0xFF
# Axis index in a KeyInput (vx, vy, wz) and direction, per key. Letters are
# matched upper case.
AXIS_VX, AXIS_VY, AXIS_WZ = 0, 1, 2
CHAR_AXES = {
    "W": (AXIS_VX, 1.0),
    "S": (AXIS_VX, -1.0),
    "A": (AXIS_VY, 1.0),
    "D": (AXIS_VY, -1.0),
    "Q": (AXIS_WZ, 1.0),
    "E": (AXIS_WZ, -1.0),
}
SPECIAL_KEY_AXES = {
    Keyboard.UP: (AXIS_VX, 1.0),
    Keyboard.DOWN: (AXIS_VX, -1.0),
    Keyboard.LEFT: (AXIS_WZ, 1.0),
    Keyboard.RIGHT: (AXIS_WZ, -1.0),
}
KEY_SCOOP = "P"
KEY_FASTER = "+"
KEY_SLOWER = "-"

# --- Console report -----------------------------------------------------------
REPORT_PERIOD_S = 0.5
COLORS_ARG_PREFIX = "--colors="
NO_COLOR = "none"
NO_CUBE_LABEL = "-"

# --- Hue classification, only for console output ------------------------------
# Largest value of an 8-bit colour channel.
CHANNEL_MAX = 255
MIN_SATURATION = 0.25
MIN_VALUE = 0.15
DEGREES_PER_TURN = 360
RED_HUE_BELOW_DEG = 15
RED_HUE_FROM_DEG = 330
# Red is bright; the same hue when dark is shown as purple.
RED_MIN_VALUE = 0.45
# (upper hue bound in degrees, name), checked in order.
HUE_BANDS = (
    (40, "orange"),
    (70, "yellow"),
    (170, "green"),
    (260, "blue"),
)
LAST_HUE_BAND = "purple"


class KeyInput(NamedTuple):
    """
    What the pressed keys ask for in one simulation step.
    """

    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    scoop_pressed: bool = False


def read_colors(args) -> list[str]:
    """
    The colours given as --colors=a,b,c on the controller command line.
    """
    for arg in args:
        if arg.startswith(COLORS_ARG_PREFIX):
            return arg.split("=", 1)[1].split(",")
    return []


def color_from_hue(r: float, g: float, b: float) -> str:
    """
    Simple hue-based classification (0-255 channels), only for console output.
    """
    hue, saturation, value = colorsys.rgb_to_hsv(
        r / CHANNEL_MAX,
        g / CHANNEL_MAX,
        b / CHANNEL_MAX,
    )
    if saturation < MIN_SATURATION or value < MIN_VALUE:
        return NO_COLOR
    hue_deg = hue * DEGREES_PER_TURN
    if hue_deg < RED_HUE_BELOW_DEG or hue_deg >= RED_HUE_FROM_DEG:
        return "red" if value > RED_MIN_VALUE else "purple"
    for upper_deg, name in HUE_BANDS:
        if hue_deg < upper_deg:
            return name
    return LAST_HUE_BAND


class TeleopController:
    """
    Reads the keyboard and drives the simulated robot; prints its sensors.
    """

    def __init__(self, robot: Robot, my_colors: list[str]):
        """
        Connect to the robot's devices and start at the default speed.
        """
        self._robot = robot
        self._step_ms = int(robot.getBasicTimeStep())
        self._name = robot.getName()
        self._my_colors = my_colors
        self._scale = INITIAL_SPEED_SCALE
        self._scoop_up = False
        self._scoop_was_pressed = False
        self._heading_rad = (
            0.0  # integrated from the gyro: drifts like the MPU6050
        )
        self._last_report_s = 0.0
        self._setup_devices()

    def _setup_devices(self) -> None:
        """
        Get and enable every device the controller uses.
        """
        robot, step = self._robot, self._step_ms
        slow_step = step * SLOW_SENSOR_STEPS
        self._motors = {}
        for wheel in WHEELS:
            motor = robot.getDevice(MOTOR_DEVICE_FORMAT.format(wheel=wheel))
            motor.setPosition(float("inf"))
            motor.setVelocity(0.0)
            self._motors[wheel] = motor
        self._servo = robot.getDevice(SCOOP_SERVO_DEVICE)
        self._servo.setPosition(SCOOP_DOWN_RAD)

        self._lines = {}
        for wheel in WHEELS:
            sensor = robot.getDevice(LINE_DEVICE_FORMAT.format(wheel=wheel))
            sensor.enable(step)
            self._lines[wheel] = sensor
        self._sonar = self._enabled(ULTRASONIC_DEVICE, step)
        self._scoop_ir = self._enabled(SCOOP_IR_DEVICE, step)
        self._scoop_color = self._enabled(SCOOP_COLOR_DEVICE, slow_step)
        self._enabled(CAMERA_DEVICE, slow_step)
        self._enabled(ACCELEROMETER_DEVICE, step)
        self._gyro = self._enabled(GYRO_DEVICE, step)

        self._keyboard = Keyboard()
        self._keyboard.enable(step)

    def _enabled(self, device_name: str, period_ms: int):
        """
        Get a device and enable it with the given sampling period.
        """
        device = self._robot.getDevice(device_name)
        device.enable(period_ms)
        return device

    # ---------- main loop ----------
    def run(self) -> None:
        """
        Step the simulation until Webots closes.
        """
        colors = ", ".join(self._my_colors) or "unassigned"
        print(f"[{self._name}] ready. My colours: {colors}")
        while self._robot.step(self._step_ms) != -1:
            keys = self._read_keys()
            self._update_scoop(keys.scoop_pressed)
            self._drive(keys)
            self._heading_rad += (
                self._gyro.getValues()[GYRO_YAW_INDEX]
                * self._step_ms
                / MS_PER_S
            )
            now_s = self._robot.getTime()
            if now_s - self._last_report_s >= REPORT_PERIOD_S:
                self._last_report_s = now_s
                self._report(now_s)

    # ---------- input ----------
    def _read_keys(self) -> KeyInput:
        """
        Drain the keyboard queue; also applies the +/- speed keys.
        """
        axes = [0.0, 0.0, 0.0]
        scoop_pressed = False
        key = self._keyboard.getKey()
        while key != -1:
            char = self._key_char(key)
            axis = CHAR_AXES.get(char) or SPECIAL_KEY_AXES.get(key)
            if axis is not None:
                axes[axis[0]] = axis[1]
            elif char == KEY_SCOOP:
                scoop_pressed = True
            elif char == KEY_FASTER:
                self._scale = min(MAX_SPEED_SCALE, self._scale + SPEED_STEP)
            elif char == KEY_SLOWER:
                self._scale = max(MIN_SPEED_SCALE, self._scale - SPEED_STEP)
            key = self._keyboard.getKey()
        return KeyInput(*axes, scoop_pressed)

    @staticmethod
    def _key_char(key: int) -> str:
        """
        Upper-case character of a key code, or "" for special keys.
        """
        code = key & ASCII_MASK
        return chr(code).upper() if code < ASCII_LIMIT else ""

    def _update_scoop(self, pressed: bool) -> None:
        """
        Toggle the scoop on the rising edge of the P key.
        """
        if pressed and not self._scoop_was_pressed:
            self._scoop_up = not self._scoop_up
            target = SCOOP_UP_RAD if self._scoop_up else SCOOP_DOWN_RAD
            self._servo.setPosition(target)
        self._scoop_was_pressed = pressed

    # ---------- motion ----------
    def _drive(self, keys: KeyInput) -> None:
        """
        Mecanum inverse kinematics (X-roller layout). vx, vy in m/s, wz in
        rad/s.
        """
        linear_m_s = (
            MAX_MOTOR_SPEED_RAD_S * LIMIT_2S * WHEEL_RADIUS_M * self._scale
        )
        k = HALF_WHEELBASE_M + HALF_TRACK_M
        vx = keys.vx * linear_m_s
        vy = keys.vy * linear_m_s
        wz = keys.wz * linear_m_s / k
        wheels = {
            "fl": (vx - vy - k * wz) / WHEEL_RADIUS_M,
            "fr": (vx + vy + k * wz) / WHEEL_RADIUS_M,
            "rl": (vx + vy - k * wz) / WHEEL_RADIUS_M,
            "rr": (vx - vy + k * wz) / WHEEL_RADIUS_M,
        }
        cap = MAX_MOTOR_SPEED_RAD_S * LIMIT_2S
        for wheel, speed in wheels.items():
            self._motors[wheel].setVelocity(max(-cap, min(cap, speed)))

    # ---------- console report ----------
    def _scoop_rgb(self) -> tuple[int, int, int]:
        """
        Mean colour of the scoop colour-sensor image (0-255 per channel).
        """
        sensor = self._scoop_color
        image = sensor.getImage()
        if not image:
            return 0, 0, 0
        width, height = sensor.getWidth(), sensor.getHeight()
        red = green = blue = 0
        for y in range(height):
            for x in range(width):
                red += sensor.imageGetRed(image, width, x, y)
                green += sensor.imageGetGreen(image, width, x, y)
                blue += sensor.imageGetBlue(image, width, x, y)
        pixels = width * height
        return red // pixels, green // pixels, blue // pixels

    def _report(self, now_s: float) -> None:
        """
        Print one status line with every sensor.
        """
        black = [
            w for w, s in self._lines.items() if s.getValue() < LINE_THRESHOLD
        ]
        has_cube = self._scoop_ir.getValue() > SCOOP_IR_THRESHOLD
        color = (
            color_from_hue(*self._scoop_rgb()) if has_cube else NO_CUBE_LABEL
        )
        mine = "yes" if color in self._my_colors else "no"
        print(
            f"[{self._name}] t={now_s:5.1f}s speed={self._scale:.2f} "
            f"scoop={'up' if self._scoop_up else 'down'} "
            f"black_line={black or 'none'} "
            f"sonar={self._sonar.getValue():.2f} m "
            f"cube_in_scoop={'yes' if has_cube else 'no'} color={color}"
            + (f" (mine: {mine})" if has_cube else "")
            + f" gyro_heading={math.degrees(self._heading_rad):6.1f} deg"
        )


def main() -> None:
    """
    Entry point of the Webots controller.
    """
    TeleopController(Robot(), read_colors(sys.argv[1:])).run()


if __name__ == "__main__":
    main()
