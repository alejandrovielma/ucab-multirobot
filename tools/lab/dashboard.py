"""Pure logic of the robot dashboard: the mission buttons (target and forbidden colors, start / stop) and
the manual mode, on top of the teleop (WASD, emergency stop). No hardware here; lab_robot.py wires it up.

This is the deliberative layer of the hybrid architecture: a person picks the mission with buttons (team
decision 2026-10-30: no language model), the sequencer executes it and the safety gate filters it."""

import json
import threading

from teleop import Teleop


class Dashboard:
    def __init__(self, teleop: Teleop, known_colors):
        self.teleop = teleop
        self.known = tuple(known_colors)
        self._lock = threading.Lock()
        self._targets: tuple[str, ...] = ()
        self._avoid: tuple[str, ...] = ()
        self._running = False
        self._manual = False
        self._changed = False

    @property
    def manual(self) -> bool:
        with self._lock:
            return self._manual

    def take_change(self):
        """(targets, avoid) the sequencer must adopt, or None. No targets = stay still (IDLE)."""
        with self._lock:
            if not self._changed:
                return None
            self._changed = False
            return (self._targets, self._avoid) if self._running else ((), self._avoid)

    def update_status(self, **fields) -> None:
        self.teleop.update_status(**fields)

    def handle(self, method: str, path: str, body: bytes):
        routes = {("POST", "/mission"): self._mission, ("POST", "/start"): self._start, ("POST", "/stop"): self._stop,
                  ("POST", "/manual"): self._set_manual, ("GET", "/status"): self._status}
        if (method, path) == ("POST", "/estop"):
            answer = self.teleop.handle(method, path, body)
            if answer and answer[0] == 200 and self.teleop.estop:
                with self._lock:  # clearing the stop must not restart the mission by itself
                    self._running, self._changed = False, True
            return answer
        route = routes.get((method, path))
        return route(body) if route else self.teleop.handle(method, path, body)

    @staticmethod
    def _bad(msg: str):
        return 400, json.dumps({"error": msg}).encode()

    def _colors(self, data, key):
        value = data.get(key, [])
        if not isinstance(value, list) or not all(isinstance(c, str) for c in value):
            raise ValueError(f"{key} must be a list of color names")
        unknown = [c for c in value if c not in self.known]
        if unknown:
            raise ValueError(f"unknown color(s) {unknown}: use {', '.join(self.known)}")
        return tuple(dict.fromkeys(value))

    def _mission(self, body: bytes):
        data = Teleop._json(body)
        if data is None:
            return self._bad("expected a JSON object")
        try:
            targets, avoid = self._colors(data, "targets"), self._colors(data, "avoid")
        except ValueError as exc:
            return self._bad(str(exc))
        both = set(targets) & set(avoid)
        if both:
            return self._bad(f"color(s) {sorted(both)} both target and forbidden")
        with self._lock:
            self._targets, self._avoid, self._changed = targets, avoid, True
        return 200, b"{}"

    def _start(self, _body: bytes):
        with self._lock:
            if not self._targets:
                return self._bad("pick at least one target color first")
            self._running, self._manual, self._changed = True, False, True
        return 200, b"{}"

    def _stop(self, _body: bytes):
        with self._lock:
            self._running, self._changed = False, True
        return 200, b"{}"

    def _set_manual(self, body: bytes):
        data = Teleop._json(body)
        if data is None or not isinstance(data.get("on"), bool):
            return self._bad('expected {"on": bool}')
        with self._lock:
            self._manual = data["on"]
            if self._manual:
                self._running, self._changed = False, True  # manual and mission are exclusive
        return 200, b"{}"

    def _status(self, _body: bytes):
        status = json.loads(self.teleop.handle("GET", "/status", b"")[1])
        with self._lock:
            status.update(targets=list(self._targets), avoid=list(self._avoid), running=self._running,
                          manual=self._manual)
        return 200, json.dumps(status).encode()
