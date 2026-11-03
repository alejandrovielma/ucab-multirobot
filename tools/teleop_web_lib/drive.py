"""
Control thread of the browser tool: keeps the last manual command, brakes
when commands stop arriving, and runs the Auto mode when it is on.

This tool talks straight to the HAT, so it does NOT go through obstacle_guard.
Use it with the wheels off the ground or a hand near the switch.
"""

import logging
import threading
import time

from teleop_web_lib.autopilot import (
    PREFIX_LINE,
    STOP,
    AutoPilot,
    DriveCommand,
    Vision,
)
from teleop_web_lib.config import AutoConfig
from teleop_web_lib.kinematics import WHEELS, clamp, mecanum_wheel_ratios
from teleop_web_lib.line_sensor import counts_to_millivolts

log = logging.getLogger("teleop_web")

# Pause between two control iterations.
LOOP_PERIOD_S = 0.02
# How long close() waits for the control thread to finish.
JOIN_TIMEOUT_S = 1.0

# vx, vy and wz are the first three fields of a DriveCommand (speed is last).
AXIS_COUNT = 3
SPEED_MIN = 0.0
SPEED_MAX = 1.0

LINE_LABEL_NO_SENSOR = "no sensor"
LINE_LABEL_BLACK = "BLACK"
LINE_LABEL_WHITE = "white"


class Drive:
    """
    Applies the manual command, brakes after `cmd_timeout_s` without a new
    one, and delegates to the autopilot while Auto is on.
    """

    def __init__(
        self,
        motors,
        max_duty_ratio: float,
        cmd_timeout_s: float,
        auto_cfg: AutoConfig | None = None,
        line_sensor=None,
    ):
        """
        Start the control thread.
        """
        self._motors = motors
        self._max_duty = max_duty_ratio
        self._timeout_s = cmd_timeout_s
        self._line = line_sensor  # None = no line sensor
        self._pilot = AutoPilot(auto_cfg or AutoConfig())
        self.line_black = False
        self.line_raw: int | None = None
        self.line_counts: int | None = None  # AO through the ADC, if wired
        self._was_black = False
        self._logged_state = None
        self._lock = threading.Lock()
        self._cmd = DriveCommand()  # last manual command
        self._stamp = 0.0
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    # ---------- called from the HTTP threads ----------
    def set_vision(self, vision: Vision) -> None:
        """
        Give the autopilot its source of detections.
        """
        with self._lock:
            self._pilot.vision = vision

    def set_auto(
        self,
        enabled: bool | None = None,
        reset: bool = False,
    ) -> None:
        """
        Switch Auto on or off and/or reset the counters.
        """
        with self._lock:
            self._pilot.configure(time.monotonic(), enabled, reset)

    def command(self, vx: float, vy: float, wz: float, speed: float) -> None:
        """
        Store a manual command; it expires after `cmd_timeout_s`.
        """
        with self._lock:
            self._cmd = DriveCommand(
                clamp(vx),
                clamp(vy),
                clamp(wz),
                clamp(speed, SPEED_MIN, SPEED_MAX),
            )
            self._stamp = time.monotonic()

    def status(self) -> dict:
        """
        Snapshot of the robot state for the page.
        """
        with self._lock:
            pilot = self._pilot
            return {
                "auto": pilot.enabled,
                "state": pilot.state,
                "target": pilot.target_info,
                "collected": pilot.collected_count,
                "delivered": pilot.delivered_count,
                "line": self._line_label(),
                "line_black": self.line_black,
                "line_raw": self.line_raw,
                "line_counts": self.line_counts,
                "line_mv": (
                    None
                    if self.line_counts is None
                    else counts_to_millivolts(self.line_counts)
                ),
            }

    def _line_label(self) -> str:
        """
        Text describing the line sensor reading.
        """
        if self._line is None:
            return LINE_LABEL_NO_SENSOR
        return LINE_LABEL_BLACK if self.line_black else LINE_LABEL_WHITE

    # ---------- control thread ----------
    def _loop(self) -> None:
        """
        Read the line sensor, decide the command and drive the motors.
        """
        while self._running:
            now = time.monotonic()
            if self._line is not None:
                self.line_raw = self._line.raw_value()
                self.line_black = self._line.is_black()
                self.line_counts = self._line.analog_counts()
                self._log_line_edge()
            else:
                self.line_raw = None
                self.line_black = False
                self.line_counts = None
            command = self._next_command(now)
            with self._lock:
                self._pilot.note_motion(now, command)
                state = self._pilot.state
            if state != self._logged_state:
                log.info("state %s -> %s", self._logged_state, state)
                self._logged_state = state
            scale = command.speed * self._max_duty
            # DriveCommand: vy forward, vx strafe right; the kinematics take
            # (forward, left, ccw), so the wheel names in the config are the
            # real wheels.
            ratios = mecanum_wheel_ratios(
                command.vy,
                -command.vx,
                command.wz,
            )
            self._motors.set({w: r * scale for w, r in ratios.items()})
            time.sleep(LOOP_PERIOD_S)

    def _log_line_edge(self) -> None:
        """
        Log every time the line sensor turns black, with the analog value and
        the state: real tape reads ~3300 counts or more and lasts a few
        iterations; a false trigger (the sensor lifting off the floor when the
        robot pitches or turns) reads something else.
        """
        if self.line_black and not self._was_black:
            with self._lock:
                state = self._pilot.state
            log.info(
                "line sensor black: ao=%s state=%s",
                self.line_counts,
                state,
            )
        self._was_black = self.line_black

    def _next_command(self, now: float) -> DriveCommand:
        """
        Manual command if one is active, else the autopilot, else stop.
        """
        with self._lock:
            manual = self._cmd
            expired = now - self._stamp > self._timeout_s
            manual_active = not expired and any(manual[:AXIS_COUNT])
            if self._pilot.state.startswith(PREFIX_LINE) or (
                self.line_black and (self._pilot.enabled or manual_active)
            ):
                return self._pilot.step(now, self.line_black)
            if manual_active:
                self._pilot.mark_manual()
                return manual
            if self._pilot.enabled:
                return self._pilot.step(now, self.line_black)
            self._pilot.mark_paused()
            return STOP

    def close(self) -> None:
        """
        Stop the thread, brake and release the motors.
        """
        self._running = False
        self._thread.join(timeout=JOIN_TIMEOUT_S)
        self._motors.set(dict.fromkeys(WHEELS, 0.0))
        self._motors.close()
