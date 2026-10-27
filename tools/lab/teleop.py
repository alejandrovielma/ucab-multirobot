"""Pure logic of the web teleop: validates the browser's commands, keeps the dead-man watchdog and the
latched emergency stop. No hardware here so it can be tested anywhere; lab_robot.py wires it up."""

import json
import math
import threading
import time

STATUS_DEFAULTS = {"reason": "", "line": False, "scoop": "", "peer": None, "cubes": [], "square": False, "fps": 0.0}


class Teleop:
    def __init__(self, max_speed: float, max_turn: float, timeout: float = 0.4, clock=time.monotonic):
        self.max_speed, self.max_turn, self.timeout, self._clock = max_speed, max_turn, timeout, clock
        self._lock = threading.Lock()
        self._cmd = (0.0, 0.0, 0.0)
        self._cmd_at = -math.inf
        self._estop = False
        self._status = dict(STATUS_DEFAULTS)

    @property
    def estop(self) -> bool:
        with self._lock:
            return self._estop

    def current(self) -> tuple[float, float, float]:
        """Command to execute now: zero on estop or when the last command is older than the timeout."""
        with self._lock:
            if self._estop or self._clock() - self._cmd_at > self.timeout:
                return (0.0, 0.0, 0.0)
            return self._cmd

    def update_status(self, **fields) -> None:
        with self._lock:
            self._status.update(fields)

    def handle(self, method: str, path: str, body: bytes):
        if (method, path) == ("POST", "/cmd"):
            return self._cmd_post(body)
        if (method, path) == ("POST", "/estop"):
            return self._estop_post(body)
        if (method, path) == ("GET", "/status"):
            return self._status_get()
        return None

    @staticmethod
    def _json(body: bytes):
        try:
            data = json.loads(body or b"null")
        except (ValueError, UnicodeDecodeError):
            return None
        return data if isinstance(data, dict) else None

    @staticmethod
    def _bad(msg: str):
        return 400, json.dumps({"error": msg}).encode()

    def _cmd_post(self, body: bytes):
        data = self._json(body)
        if data is None:
            return self._bad("expected a JSON object")
        vals = []
        for key in ("vx", "vy", "wz"):
            v = data.get(key)
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
                return self._bad(f"{key} must be a finite number")
            vals.append(float(v))
        vx, vy, wz = (max(-lim, min(lim, v)) for v, lim in zip(vals, (self.max_speed, self.max_speed, self.max_turn)))
        with self._lock:
            self._cmd, self._cmd_at = (vx, vy, wz), self._clock()
        return 200, b"{}"

    def _estop_post(self, body: bytes):
        data = self._json(body)
        if data is None or not isinstance(data.get("on"), bool):
            return self._bad('expected {"on": bool}')
        with self._lock:
            self._estop = data["on"]
            self._cmd = (0.0, 0.0, 0.0)
        return 200, b"{}"

    def _status_get(self):
        cmd = self.current()
        with self._lock:
            out = {**self._status, "estop": self._estop, "cmd": list(cmd)}
        return 200, json.dumps(out).encode()
