#!/usr/bin/env python3
"""Lab tool for the real robot without ROS (Raspberry Pi 5 + Fusion HAT+ + IMX219 + line sensor).

    python3 tools/lab/lab_robot.py motors              # each port/wheel forward then backward (wiring check)
    python3 tools/lab/lab_robot.py threshold --delay 15   # power at which the robot starts moving (camera)
    python3 tools/lab/lab_robot.py drive               # keyboard teleop over SSH
    python3 tools/lab/lab_robot.py line                # line sensor level
    python3 tools/lab/lab_robot.py camera              # camera + cube detection, view at http://<ip>:8080/
    python3 tools/lab/lab_robot.py chase --target blue --avoid red   # autonomous, with a mission

Common options: --robot phineas|ferb, --dry-run (no GPIO), --chip N, --camera auto|picamera2|gstreamer|
v4l2|file:<img>, --port 8080, --max-speed 0.05 m/s, --max-turn 0.5 rad/s (lab caps). Ctrl+C always stops the motors. See tools/lab/README.md.
"""

import argparse
import math
import os
import secrets
import select
import signal
import socket
import sys
import termios
import time
import tty
from pathlib import Path

from hardware import LOG, Camera, LineSensor, Motors, WHEELS, load_config, open_gpio

from mobile_base.core.kinematics import wheel_speeds  # noqa: E402
from stream import FrameBuffer, serve
from dashboard import Dashboard
from supervisor import run_loop
from teleop import Teleop

from mobile_base.core.cartographer import Cartographer  # noqa: E402
from mobile_base.core.localization import Localizer  # noqa: E402
from mobile_base.core.behavior import BehaviorFSM, BehaviorInput, BehaviorParams, Cube, State  # noqa: E402
from mobile_base.core.colors import DEFAULT_HUE_RANGES, check_targets, parse_colors, ranges_from_flat  # noqa: E402
from mobile_base.core.safety import SafetyGate, SafetyInput, SafetyParams  # noqa: E402
from mobile_base.vision import VisionParams, detect_cubes, detect_peer, detect_refs, detect_square, landmark_sightings, scoop_content  # noqa: E402

COLORS_BGR = {"red": (0, 0, 255), "orange": (0, 140, 255), "yellow": (0, 230, 230), "green": (0, 200, 0),
              "blue": (255, 80, 0), "purple": (200, 0, 160)}


# ------------------------------------------------------------------ helpers


def vision_params(cfg: dict) -> VisionParams:
    det = dict(cfg["cube_detector"])
    hue = ranges_from_flat(det.pop("hue_ranges"))
    fields = VisionParams.__dataclass_fields__
    return VisionParams(hue_ranges=hue, **{k: v for k, v in det.items() if k in fields})


def annotate(frame, detections, params: VisionParams, lines: list[str], square=None):
    import cv2

    h, w = frame.shape[:2]
    out = frame.copy()
    x0 = int((1 - params.scoop_x_half) / 2 * w)
    x1 = int((1 + params.scoop_x_half) / 2 * w)
    cv2.rectangle(out, (x0, int(params.scoop_y_min * h)), (x1, int(params.scoop_y_max * h)), (200, 200, 200), 1)
    if square is not None and square.visible:
        sx, sy = int((square.cx_norm + 1) / 2 * w), int(square.cy_norm * h)
        cv2.drawMarker(out, (sx, sy), (255, 0, 0), cv2.MARKER_CROSS, 40, 3)
        cv2.putText(out, f"cuadro azul cx={square.cx_norm:+.2f}", (sx - 60, sy - 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
    for d in detections:
        cx, cy = int((d.cx_norm + 1) / 2 * w), int(d.cy_norm * h)
        size = int(math.sqrt(d.area_px) / 2)
        color = COLORS_BGR.get(d.color, (255, 255, 255))
        cv2.rectangle(out, (cx - size, cy - size), (cx + size, cy + size), color, 2)
        cv2.putText(out, f"{d.color} cx={d.cx_norm:+.2f} cy={d.cy_norm:.2f}", (cx - size, cy - size - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
    for i, text in enumerate(lines):
        cv2.putText(out, text, (8, 20 + 20 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
        cv2.putText(out, text, (8, 20 + 20 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
    return out


class RawKeys:
    """Single key presses from the terminal (works over SSH), restored on exit."""

    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def get(self, timeout: float) -> str | None:
        ready, _, _ = select.select([sys.stdin], [], [], timeout)
        return sys.stdin.read(1) if ready else None

    def __exit__(self, *exc):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)


def open_motor_gpio(args, cfg):
    """The Fusion HAT+ drives its motors over I2C: the Pi GPIO is only needed by the Cokoino HAT."""
    if cfg["hat_motor_driver"].get("motor_backend", "fusion_hat") == "fusion_hat":
        return open_gpio(None, dry_run=True)
    return open_gpio(args.chip, args.dry_run)


# ------------------------------------------------------------------ modes


def robot_steps(v: float, seconds: float, star: bool) -> list[tuple[str, tuple[float, float, float]]]:
    """Whole-robot steps of the motors mode. --star: the 8 translations and both spins of the team's
    motion rule, in pairs that bring the robot back, so the leftover offset shows the drift."""
    if not star:
        return [("avanzar", (v, 0.0, 0.0)), ("lateral izquierda", (0.0, v, 0.0)),
                ("girar antihorario", (0.0, 0.0, 10.0 * v))]  # 0.8 rad/s at 0.08 m/s, as before
    d = v * math.sqrt(0.5)
    w = (math.pi / 2.0) / seconds  # a quarter turn per step
    return [("adelante", (v, 0.0, 0.0)), ("atras", (-v, 0.0, 0.0)),
            ("izquierda", (0.0, v, 0.0)), ("derecha", (0.0, -v, 0.0)),
            ("diagonal adelante-izquierda", (d, d, 0.0)), ("diagonal atras-derecha", (-d, -d, 0.0)),
            ("diagonal adelante-derecha", (d, -d, 0.0)), ("diagonal atras-izquierda", (-d, d, 0.0)),
            ("girar antihorario 90 grados", (0.0, 0.0, w)), ("girar horario 90 grados", (0.0, 0.0, -w))]


def mode_motors(args, cfg):
    motors = Motors(cfg, open_motor_gpio(args, cfg), max(args.max_speed, args.speed), args.dry_run)
    if args.delay > 0.0:
        # Time to unplug the Ethernet cable used for SSH: a dragging cable makes the robot drift.
        print(f"empieza en {args.delay:.0f} s", flush=True)
        time.sleep(args.delay)
    try:
        order = [w.strip() for w in args.wheels.split(",")]
        if any(w not in WHEELS for w in order):
            raise SystemExit(f"--wheels: usar {','.join(WHEELS)} separadas por comas, no {args.wheels!r}")
        for wheel in () if args.only_robot else order:
            for direction in (1.0, -1.0):
                label = "adelante" if direction > 0 else "atras"
                print(f"rueda {wheel} ({motors.port(wheel)}): {label} {args.seconds:.1f} s "
                      f"(duty {args.duty:.2f} del tope)", flush=True)
                motors.wheel(wheel, direction * args.duty)
                time.sleep(args.seconds)
                motors.stop()
                time.sleep(0.5)
        for name, cmd in () if args.only_wheels else robot_steps(args.speed, args.seconds, args.star):
            print(f"robot: {name} {args.seconds:.1f} s", flush=True)
            motors.drive(*cmd)
            time.sleep(args.seconds)
            motors.stop()
            time.sleep(0.5)
    finally:
        motors.stop()
        motors.gpio.close()


# Whole-robot motions for the threshold mode: every wheel gets the same power, with the signs of the
# mecanum kinematics (a spin is a differential: left side back, right side forward).
THRESHOLD_MOTIONS = [
    ("girar antihorario", (0.0, 0.0, 1.0)), ("girar horario", (0.0, 0.0, -1.0)),
    ("adelante", (1.0, 0.0, 0.0)), ("atras", (-1.0, 0.0, 0.0)),
    ("izquierda", (0.0, 1.0, 0.0)), ("derecha", (0.0, -1.0, 0.0)),
]


def motion_signs(vx: float, vy: float, wz: float) -> tuple[float, ...]:
    """Sign of each wheel (fl, fr, rl, rr) for a body motion, from the mecanum kinematics."""
    return tuple(math.copysign(1.0, s) if abs(s) > 1e-9 else 0.0 for s in wheel_speeds(vx, vy, wz))


def frame_motion(previous, frame) -> float:
    """Mean absolute difference of two frames (gray, 160 x 120, blurred): grows when the view moves."""
    import cv2

    def small(img):
        return cv2.GaussianBlur(cv2.cvtColor(cv2.resize(img, (160, 120)), cv2.COLOR_BGR2GRAY), (5, 5), 0)

    return float(cv2.absdiff(small(previous), small(frame)).mean())


def moving_since(levels: list[float], still_mean: float, still_std: float, steps: int = 2) -> int | None:
    """Index of the first of `steps` consecutive levels well above the still camera noise, or None."""
    limit = still_mean + max(4.0 * still_std, 1.5)
    run = 0
    for i, level in enumerate(levels):
        run = run + 1 if level > limit else 0
        if run >= steps:
            return i - steps + 1
    return None


def mode_threshold(args, cfg):
    """Power at which the robot starts moving, for each whole-robot motion (fusion_start_power).

    The power of all four wheels rises by --step percent every --step-time seconds, up to --max-power.
    The camera tells when the robot moves (frame difference well above its still noise for two steps),
    so it runs without anyone at the keyboard (unplug the SSH cable during --delay). It stops at once
    when it detects motion or the line sensor sees the tape, and logs the level of every step."""
    gpio = open_gpio(args.chip, args.dry_run)
    motors = Motors(cfg, gpio, args.max_speed, args.dry_run)
    line = LineSensor(cfg, gpio)
    cam = Camera(args.camera)
    if motors.bank is None:
        print("sin la Fusion HAT+ no hay nada que medir", flush=True)
        return
    top = min(motors.cap * 100.0, args.max_power)
    found = {}

    def frames_for(seconds):
        """Frame-difference levels over `seconds`."""
        levels, previous, end = [], cam.read(), time.monotonic() + seconds
        while time.monotonic() < end:
            frame = cam.read()
            if frame is not None and previous is not None:
                levels.append(frame_motion(previous, frame))
            previous = frame if frame is not None else previous
        return levels

    if args.delay > 0.0:
        print(f"empieza en {args.delay:.0f} s: poner el robot en el centro, mirando a algo con detalle "
              "(cinta, cubos), y desconectar el cable", flush=True)
        time.sleep(args.delay)
    try:
        for name, cmd in THRESHOLD_MOTIONS:
            signs = motion_signs(*cmd)
            still = frames_for(1.5)
            mean = sum(still) / len(still) if still else 0.0
            std = (sum((x - mean) ** 2 for x in still) / len(still)) ** 0.5 if still else 0.0
            print(f"{name}: ruido quieto {mean:.2f} +- {std:.2f}", flush=True)
            levels, powers, hit, why = [], [], None, "no se movio hasta el tope"
            power = args.start_power
            while power <= top + 1e-9:
                for wheel, sign in zip(WHEELS, signs):
                    motors.bank.set_power(wheel, sign * power)
                step = frames_for(args.step_time)
                levels.append(max(step) if step else 0.0)
                powers.append(power)
                print(f"  {power:5.1f} %  nivel {levels[-1]:.2f}", flush=True)
                if line.black():
                    why = "linea negra: parado"
                    break
                first = moving_since(levels, mean, std)
                if first is not None:
                    hit, why = powers[first], "en movimiento"
                    break
                power = round(power + args.step, 1)
            motors.stop()
            found[name] = hit
            print(f"{name}: {'%.0f %%' % hit if hit is not None else '-'} ({why})", flush=True)
            time.sleep(1.0)
    finally:
        motors.stop()
        gpio.close()
    print("resultado: " + ", ".join(f"{k} {'-' if v is None else f'{v:.0f} %'}" for k, v in found.items()),
          flush=True)
    measured = [v for v in found.values() if v is not None]
    if measured:
        start = max(measured) + args.step  # the slowest motion to start, plus one step of margin
        print(f"fusion_start_power: [{start:.1f}, {start:.1f}, {start:.1f}, {start:.1f}]", flush=True)


def mode_drive(args, cfg):
    motors = Motors(cfg, open_motor_gpio(args, cfg), args.max_speed, args.dry_run)
    # Star motion (team rule 2026-10-27): translations in 8 directions or a spin in place, never both.
    d = math.sqrt(0.5)
    keys = {"w": (1, 0, 0), "s": (-1, 0, 0), "a": (0, 1, 0), "d": (0, -1, 0), "q": (0, 0, 1), "e": (0, 0, -1),
            "u": (d, d, 0), "o": (d, -d, 0), "j": (-d, d, 0), "l": (-d, -d, 0)}
    speed, turn = args.max_speed, 1.0
    print("w/s avanzar/retroceder, a/d lateral, u/o diagonal adelante izq/der, j/l diagonal atras izq/der, "
          "q/e girar en el sitio, espacio parar, +/- velocidad, x salir", flush=True)
    cmd = (0.0, 0.0, 0.0)
    last_key = time.monotonic()
    try:
        with RawKeys() as raw:
            while True:
                key = raw.get(0.1)
                now = time.monotonic()
                if key:
                    last_key = now
                    if key == "x":
                        break
                    if key == " ":
                        cmd = (0.0, 0.0, 0.0)
                    elif key in "+=":
                        speed = min(args.max_speed, speed + 0.02)
                    elif key == "-":
                        speed = max(0.02, speed - 0.02)
                    elif key in keys:
                        fx, fy, fw = keys[key]
                        cmd = (fx * speed, fy * speed, fw * turn)
                    print(f"\rcmd vx={cmd[0]:+.2f} vy={cmd[1]:+.2f} wz={cmd[2]:+.2f} (vel {speed:.2f})   ", end="",
                          flush=True)
                if now - last_key > args.latch_timeout:
                    cmd = (0.0, 0.0, 0.0)  # no key for a while: stop (operator distracted, SSH dropped)
                motors.drive(*cmd)
    finally:
        motors.stop()
        motors.gpio.close()
        print()


def mode_line(args, cfg):
    line = LineSensor(cfg, open_gpio(args.chip, args.dry_run))
    print(f"GPIO{line.pin}, nivel activo {line.active_level} = linea negra. Ctrl+C para salir.", flush=True)
    while True:
        raw = line.raw()
        print(f"\rnivel={raw}  {'NEGRO (linea)' if line.black() else 'blanco'}      ", end="", flush=True)
        time.sleep(0.1)


def camera_status(frame, params: VisionParams):
    """One console line (scoop window, blue square, cubes) and the square view, for the camera mode."""
    dets = detect_cubes(frame, params)
    scoop, fill = scoop_content(frame, params)
    square = detect_square(frame, params)
    found = ", ".join(f"{d.color}(cx {d.cx_norm:+.2f}, cy {d.cy_norm:.2f}, {d.area_px:.0f}px)" for d in dets)
    sq = (f"cuadro azul (cx {square.cx_norm:+.2f}, cy {square.cy_norm:.2f}, {square.area_px:.0f}px)"
          if square.visible else "sin cuadro")
    return f"pala={scoop or '-'} {fill:.2f} | {sq} | {found or 'sin cubos'}", square


def mode_camera(args, cfg):
    cam = Camera(args.camera)
    params = vision_params(cfg)
    frames = FrameBuffer()
    serve(frames, args.port)
    print(f"video en http://<ip-del-robot>:{args.port}/  (Ctrl+C para salir)", flush=True)
    count, t0, last_print, last_save = 0, time.monotonic(), 0.0, 0.0
    saves = 0
    if args.save_every > 0:
        import cv2

        os.makedirs(args.save_dir, exist_ok=True)
        print(f"guardando una imagen cada {args.save_every:.0f} s en {args.save_dir}/", flush=True)
    while True:
        frame = cam.read()
        if frame is None:
            LOG.warn("frame vacio")
            time.sleep(0.1)
            continue
        dets = detect_cubes(frame, params)
        scoop, fill = scoop_content(frame, params)
        text, square = camera_status(frame, params)
        count += 1
        now = time.monotonic()
        fps = count / (now - t0)
        shown = annotate(frame, dets, params, [f"{cam.backend} {fps:.1f} fps", f"pala: {scoop or '-'} {fill:.2f}"], square)
        frames.put(shown)
        if now - last_print >= 1.0:
            last_print = now
            print(f"{fps:4.1f} fps | {text}", flush=True)
        if args.save_every > 0 and saves < args.save_max and now - last_save >= args.save_every:
            last_save = now
            saves += 1
            stamp = time.strftime("%H%M%S")
            ok = cv2.imwrite(os.path.join(args.save_dir, f"cam_{stamp}.png"), frame)  # raw, for calibration
            ok = cv2.imwrite(os.path.join(args.save_dir, f"cam_{stamp}_marcada.jpg"), shown) and ok
            if not ok:  # imwrite returns False (no exception) on a full or read-only disk
                LOG.warn(f"no se pudo guardar la imagen en {args.save_dir}/ (disco lleno o sin permiso?)")


class MapRunner:
    """The cartographer (particle filter + memory) in a lab loop. The command sent at one tick moved the
    robot until the next one: it is applied there, with the real elapsed time."""

    def __init__(self, enabled: bool):
        self.carto = Cartographer(Localizer()) if enabled else None
        self._sent = None  # (cmd, time)

    def inputs(self, now, black, square, refs, cubes, fsm) -> dict:
        if self.carto is None:
            return {}
        if self._sent is not None:
            cmd, t = self._sent
            self.carto.moved(cmd, max(0.0, now - t))
        est = self.carto.observe(now, black, landmark_sightings(square, refs), cubes, fsm.targets,
                                 carrying=fsm.carrying)
        goal = self.carto.goal_for(now, fsm.targets)
        if not est.localized:
            return {}
        return dict(est_pose=(est.x, est.y, est.theta), explore_goal=goal, tape_ahead=self.carto.tape_ahead())

    def sent(self, cmd, now) -> None:
        self._sent = (cmd, now)

    def text(self) -> str:
        if self.carto is None:
            return "sin mapa"
        e = self.carto.est
        where = f"({e.x:+.2f}, {e.y:+.2f}) {math.degrees(e.theta):+4.0f} grados +-{e.spread * 100:.0f} cm"
        return ("en " if e.localized else "buscando, cerca de ") + where + f", ruedas {e.scale:.2f}"


def lab_speed_caps(values: dict, args) -> dict:
    """Cap the behavior's speeds for the lab: translation by --max-speed and spins by --max-turn.
    The HAT's start power maps even a small command to 25-43 % power, so an uncapped 1.2 rad/s spin
    was the fastest thing the robot did."""
    values.update(has_gate=False,
                  explore_speed=min(values.get("explore_speed", 0.12), args.max_speed),
                  approach_speed=min(values.get("approach_speed", 0.10), args.max_speed),
                  turn_speed=min(values.get("turn_speed", 1.2), args.max_turn),
                  approach_turn_min=min(values.get("approach_turn_min", 0.4), args.max_turn),
                  # centering a cube spins at k_turn * |cx| (up to k_turn rad/s): cap the gain too
                  k_turn=min(values.get("k_turn", 1.5), args.max_turn),
                  capture_speed=min(values.get("capture_speed", 0.08), args.max_speed),
                  carry_speed=min(values.get("carry_speed", 0.08), args.max_speed))
    return values


def stop_on_signals() -> None:
    """kill and a dropped SSH session send SIGTERM/SIGHUP, which end Python without running `finally`
    and leave the motors at their last command. Turn them into SystemExit so every mode stops them.
    (SIGKILL cannot be caught.)"""
    for sig in (signal.SIGTERM, getattr(signal, "SIGHUP", None)):  # no SIGHUP on Windows
        if sig is not None:
            signal.signal(sig, lambda signum, _frame: sys.exit(128 + signum))


def mission_colors(args, bcfg):
    """(targets, avoid) from --target/--avoid or the YAML; a bad color name exits with a message, not a traceback."""
    try:
        return (check_targets(parse_colors(args.target if args.target is not None else bcfg.get("my_colors", ""))),
                parse_colors(args.avoid if args.avoid is not None else bcfg.get("avoid_colors", "")))
    except ValueError as exc:
        raise SystemExit(f"mision invalida: {exc}") from None


def mode_chase(args, cfg):
    gpio = open_gpio(args.chip, args.dry_run)
    motors = Motors(cfg, gpio, args.max_speed, args.dry_run)
    line = LineSensor(cfg, gpio)
    cam = Camera(args.camera)
    vparams = vision_params(cfg)
    bcfg = cfg["behavior"]
    fields = BehaviorParams.__dataclass_fields__
    values = {k: v for k, v in bcfg.items() if k in fields and k != "my_colors"}
    targets, avoid = mission_colors(args, bcfg)
    if not targets:
        raise SystemExit("sin mision: pasar --target (por ejemplo --target blue --avoid red)")
    lab_speed_caps(values, args)
    if args.capture_cy is not None:
        values["capture_cy"] = args.capture_cy
    fsm = BehaviorFSM(BehaviorParams(my_colors=targets, **values))
    try:
        fsm.set_mission(targets, avoid)
    except ValueError as exc:
        raise SystemExit(f"mision invalida: {exc}") from None
    gcfg = cfg["safety_gate"]
    gfields = SafetyParams.__dataclass_fields__
    gate = SafetyGate(SafetyParams(**{**{k: v for k, v in gcfg.items() if k in gfields},
                                      "require_pose": False, "require_heartbeat": False}))
    frames = FrameBuffer()
    serve(frames, args.port)
    print(f"objetivo {list(targets)}, prohibidos {list(avoid)}; video en http://<ip-del-robot>:{args.port}/ ; Ctrl+C para parar",
          flush=True)
    mapper = MapRunner(not args.no_map)
    t0 = time.monotonic()
    previous = None

    def tick(frame, now):
        nonlocal previous
        dets = detect_cubes(frame, vparams) if frame is not None else []
        cubes = [Cube(d.color, d.cx_norm, d.cy_norm, d.area_px) for d in dets]
        # The line sensor reads the blue square as white: the camera keeps the body out of it.
        square = detect_square(frame, vparams) if frame is not None else None
        black = line.black()
        scoop = scoop_content(frame, vparams)[0] if frame is not None else ""
        peer = detect_peer(frame, vparams) if frame is not None else None  # the other robot's marker
        refs = detect_refs(frame, vparams) if frame is not None else None  # green and red rectangles
        where = mapper.inputs(now, black, square, refs, cubes, fsm)
        want = fsm.step(BehaviorInput(
            **where, now=now, cubes=cubes, line_front=black, scoop_color=scoop,
            refs_visible=refs is not None and refs.visible, peer_seen=peer,
            refs_cx=refs.center_cx if refs is not None else 0.0,
            refs_skew=refs.skew if refs is not None else 0.0,
            square_visible=square is not None and square.visible,
            square_cx=square.cx_norm if square is not None else 0.0,
            square_cy=square.cy_norm if square is not None else 0.0,
            square_area=square.area_px if square is not None else 0.0))
        cmd, reason = gate.filter(SafetyInput(cmd=want, line_front=black, own_pose=None, peer_pose=None, now=now,
                                               heartbeat_age=None, carrying=fsm.carrying, peer_seen=peer,
                                               delivering=fsm.state is State.DELIVER))
        if fsm.state is not previous:
            note = f" ({fsm.note})" if fsm.note else ""
            print(f"[{now:6.1f}] {previous.value if previous else '-'} -> {fsm.state.value}{note}", flush=True)
            previous = fsm.state
        if fsm.state is State.CAPTURE and args.stop_at_capture:
            motors.stop()
            print(f"[{now:6.1f}] cubo en la boca de la pala: parado (--stop-at-capture). Ctrl+C para salir.", flush=True)
            while True:
                f = cam.read()
                if f is not None:
                    frames.put(annotate(f, detect_cubes(f, vparams), vparams, ["CAPTURE: parado"]))
                time.sleep(0.1)
        motors.drive(*cmd)
        mapper.sent(cmd, now)
        if frame is not None:
            frames.put(annotate(frame, dets, vparams,
                                [mapper.text(),
                                 f"{fsm.state.value} linea={int(black)} pala={scoop or '-'} "
                                 f"prohibidos tocados={fsm.forbidden_touches} entregados={fsm.delivered}"
                                 + (f" otro robot a {peer[1]:.2f} m" if peer else ""),
                                 f"cmd vx={cmd[0]:+.2f} vy={cmd[1]:+.2f} wz={cmd[2]:+.2f} ({reason})"]))
    try:
        run_loop(cam.read, tick, motors.stop)
    finally:
        motors.stop()
        motors.gpio.close()


def mode_dashboard(args, cfg):
    """Mission buttons, camera view and manual WASD in the browser; every command through the safety gate.

    Deliberative layer = the person (buttons), sequencer = BehaviorFSM, reactive layer = SafetyGate."""
    gpio = open_gpio(args.chip, args.dry_run)
    motors = Motors(cfg, gpio, args.max_speed, args.dry_run)
    line = LineSensor(cfg, gpio)
    cam = Camera(args.camera)
    vparams = vision_params(cfg)
    gcfg = cfg["safety_gate"]
    gfields = SafetyParams.__dataclass_fields__
    gate = SafetyGate(SafetyParams(**{**{k: v for k, v in gcfg.items() if k in gfields},
                                      "require_pose": False, "require_heartbeat": False}))
    bcfg = cfg["behavior"]
    values = {k: v for k, v in bcfg.items() if k in BehaviorParams.__dataclass_fields__ and k != "my_colors"}
    lab_speed_caps(values, args)
    fsm = BehaviorFSM(BehaviorParams(my_colors=(), **values))  # no mission: IDLE until INICIAR
    teleop = Teleop(args.max_speed, args.max_turn)
    dash = Dashboard(teleop, DEFAULT_HUE_RANGES)
    token = secrets.token_urlsafe(8)
    page = (Path(__file__).parent / "dashboard.html").read_bytes()
    frames = FrameBuffer()
    serve(frames, args.port, page=page, api=dash.handle, token=token)
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("10.255.255.255", 1))  # no packet is sent: only picks the outgoing interface
            ip = s.getsockname()[0]
    except OSError:
        ip = "<ip-del-robot>"
    print(f"control: http://{ip}:{args.port}/?t={token}   (Ctrl+C o pkill -INT para parar)", flush=True)
    mapper = MapRunner(not args.no_map)
    t0 = time.monotonic()
    last = t0

    def tick(frame, now):
        nonlocal last
        dets = detect_cubes(frame, vparams) if frame is not None else []
        square = detect_square(frame, vparams) if frame is not None else None
        black = line.black()
        scoop = scoop_content(frame, vparams)[0] if frame is not None else ""
        peer = detect_peer(frame, vparams) if frame is not None else None
        refs = detect_refs(frame, vparams) if frame is not None else None
        cubes = [Cube(d.color, d.cx_norm, d.cy_norm, d.area_px) for d in dets]
        where = mapper.inputs(now, black, square, refs, cubes, fsm)  # also while driving by hand
        auto = fsm.step(BehaviorInput(
            **where, now=now, cubes=cubes, line_front=black,
            scoop_color=scoop, square_visible=square is not None and square.visible,
            square_cx=square.cx_norm if square is not None else 0.0,
            square_cy=square.cy_norm if square is not None else 0.0,
            square_area=square.area_px if square is not None else 0.0,
            refs_visible=refs is not None and refs.visible, peer_seen=peer,
            refs_cx=refs.center_cx if refs is not None else 0.0,
            refs_skew=refs.skew if refs is not None else 0.0))
        change = dash.take_change()
        if change is not None:
            fsm.set_mission(*change)  # already validated by the dashboard
        manual = dash.manual
        cmd, reason = gate.filter(SafetyInput(
            cmd=teleop.current() if manual else auto, line_front=black, own_pose=None, peer_pose=None, now=now,
            heartbeat_age=None, carrying=fsm.carrying and not manual, peer_seen=peer, estop=teleop.estop,
            delivering=fsm.state is State.DELIVER and not manual))
        motors.drive(*cmd)
        mapper.sent(cmd, now)
        t = time.monotonic()
        dash.update_status(fsm=fsm.state.value, delivered=fsm.delivered, forbidden=fsm.forbidden_touches,
                           est=mapper.text(),
                           reason=reason, line=bool(black), scoop=scoop or "",
                             peer=peer[1] if peer else None, cubes=[d.color for d in dets],
                             square=square is not None and square.visible, fps=1.0 / max(t - last, 1e-3))
        last = t
        if frame is not None:
            frames.put(annotate(frame, dets, vparams, [f"{'manual' if manual else fsm.state.value} linea={int(black)} pala={scoop or '-'}",
                                                       f"cmd vx={cmd[0]:+.2f} vy={cmd[1]:+.2f} wz={cmd[2]:+.2f} ({reason})"]))
    try:
        run_loop(cam.read, tick, motors.stop)
    finally:
        motors.stop()
        motors.gpio.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("mode", choices=["motors", "threshold", "drive", "line", "camera", "chase", "dashboard"])
    parser.add_argument("--robot", default="phineas", choices=["phineas", "ferb"])
    parser.add_argument("--dry-run", action="store_true", help="no GPIO access (laptop check)")
    parser.add_argument("--chip", type=int, default=None, help="gpiochip number (default: autodetect)")
    parser.add_argument("--camera", default="auto")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--max-speed", type=float, default=0.05, help="m/s, lab cap on translation")
    parser.add_argument("--battery", type=float, default=None,
                        help="measured battery voltage (V): sets the PWM cap that keeps the TT motors <= 6 V")
    parser.add_argument("--duty", type=float, default=0.5, help="motors mode: fraction of the duty cap")
    parser.add_argument("--seconds", type=float, default=1.0, help="motors mode: seconds per test")
    parser.add_argument("--speed", type=float, default=0.08, help="motors mode: m/s of the forward and strafe steps")
    parser.add_argument("--wheels", default=",".join(WHEELS), help="motors mode: order of the one-wheel steps")
    parser.add_argument("--only-wheels", action="store_true", help="motors mode: skip the whole-robot steps")
    parser.add_argument("--star", action="store_true",
                        help="motors mode: robot steps in the 8 translation directions and both spins, in return pairs")
    parser.add_argument("--only-robot", action="store_true", help="motors mode: skip the one-wheel steps")
    parser.add_argument("--delay", type=float, default=0.0, help="motors mode: seconds to wait before moving")
    parser.add_argument("--step", type=float, default=2.0, help="threshold mode: power step (percent)")
    parser.add_argument("--step-time", type=float, default=0.6, help="threshold mode: seconds per step")
    parser.add_argument("--start-power", type=float, default=8.0, help="threshold mode: first power step (percent)")
    parser.add_argument("--max-power", type=float, default=45.0,
                        help="threshold mode: highest power tried (the 21-28 %% of the star test already moves)"),
    parser.add_argument("--latch-timeout", type=float, default=3.0, help="drive mode: stop after N s with no key")
    parser.add_argument("--save-every", type=float, default=0.0,
                        help="camera mode: save the raw and the annotated frame every N seconds")
    parser.add_argument("--save-dir", default="capturas", help="camera mode: folder for --save-every")
    parser.add_argument("--save-max", type=int, default=200, help="camera mode: stop saving after N snapshots")
    parser.add_argument("--capture-cy", type=float, default=None, help="chase: override capture_cy")
    parser.add_argument("--target", default=None, help="chase: target colors, comma separated (the mission)")
    parser.add_argument("--avoid", default=None, help="chase: forbidden colors, comma separated")
    parser.add_argument("--max-turn", type=float, default=0.5, help="rad/s, lab cap on spins (teleop, chase, dashboard)")
    parser.add_argument("--no-map", action="store_true",
                        help="chase/dashboard: no particle filter or memory (the old bump-and-turn exploration)")
    parser.add_argument("--stop-at-capture", action="store_true",
                        help="chase: stop when a cube reaches the scoop instead of carrying it to the square")
    args = parser.parse_args()
    stop_on_signals()
    cfg = load_config(args.robot)
    if args.battery is not None:
        cfg["hat_motor_driver"]["battery_voltage"] = args.battery
    modes = {"motors": mode_motors, "threshold": mode_threshold, "drive": mode_drive, "line": mode_line, "camera": mode_camera,
             "chase": mode_chase, "dashboard": mode_dashboard}
    try:
        modes[args.mode](args, cfg)
    except KeyboardInterrupt:
        print("\nparado", flush=True)


if __name__ == "__main__":
    main()
