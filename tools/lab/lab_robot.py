#!/usr/bin/env python3
"""Lab tool for the real robot without ROS (Raspberry Pi 5 + Fusion HAT+ + IMX219 + line sensor).

    python3 tools/lab/lab_robot.py motors              # each port/wheel forward then backward (wiring check)
    python3 tools/lab/lab_robot.py threshold [--all]   # power at which each wheel (or the robot) starts moving
    python3 tools/lab/lab_robot.py drive               # keyboard teleop over SSH
    python3 tools/lab/lab_robot.py line                # line sensor level
    python3 tools/lab/lab_robot.py camera              # camera + cube detection, view at http://<ip>:8080/
    python3 tools/lab/lab_robot.py chase --target blue --avoid red   # autonomous, with a mission

Common options: --robot phineas|ferb, --dry-run (no GPIO), --chip N, --camera auto|picamera2|gstreamer|
v4l2|file:<img>, --port 8080, --max-speed 0.10. Ctrl+C always stops the motors. See tools/lab/README.md.
"""

import argparse
import math
import os
import select
import sys
import termios
import time
import tty

from hardware import LOG, Camera, LineSensor, Motors, WHEELS, load_config, open_gpio
from stream import FrameBuffer, serve

from mobile_base.core.behavior import BehaviorFSM, BehaviorInput, BehaviorParams, Cube, State  # noqa: E402
from mobile_base.core.colors import parse_colors, ranges_from_flat  # noqa: E402
from mobile_base.core.safety import SafetyGate, SafetyInput, SafetyParams  # noqa: E402
from mobile_base.vision import VisionParams, detect_cubes, detect_square, scoop_content  # noqa: E402

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


def mode_threshold(args, cfg):
    """Power at which each wheel starts turning (fusion_start_power): it rises by --step percent every
    --step-time seconds; press any key as soon as the wheel turns steadily (x: it never did). With --all
    the four wheels rise together forward: on the floor, the power at which the whole robot moves."""
    motors = Motors(cfg, open_motor_gpio(args, cfg), args.max_speed, args.dry_run)
    if motors.bank is None:
        print("sin la Fusion HAT+ no hay nada que medir", flush=True)
        return
    top = motors.cap * 100.0
    groups = [("todas", WHEELS)] if args.all else [(w, (w,)) for w in WHEELS]
    found = {}
    print(f"La potencia sube de a {args.step:.0f} % cada {args.step_time:.1f} s hasta {top:.0f} %. "
          "Presiona cualquier tecla apenas gire de forma continua (x: no giro).", flush=True)
    try:
        with RawKeys() as raw:
            for name, wheels in groups:
                hits = []
                for direction in (1.0, -1.0) if not args.all else (1.0,):
                    label = "adelante" if direction > 0 else "atras"
                    power, hit = 0.0, None
                    while power < top:
                        power = min(top, power + args.step)
                        for wheel in wheels:  # raw percent: no start power or trim on top of it
                            motors.bank.set_power(wheel, direction * power)
                        print(f"\r{name} {label}: {power:5.1f} %   ", end="", flush=True)
                        key = raw.get(args.step_time)
                        if key:
                            hit = None if key == "x" else power
                            break
                    motors.stop()
                    print(f"-> {'no giro' if hit is None else f'{hit:.0f} %'}", flush=True)
                    hits.append(hit)
                    time.sleep(0.8)
                found[name] = max((h for h in hits if h is not None), default=None)
    finally:
        motors.stop()
        motors.gpio.close()
    print("resultado:", ", ".join(f"{k} {'no giro' if v is None else f'{v:.0f} %'}" for k, v in found.items()),
          flush=True)
    if not args.all and all(found.get(w) is not None for w in WHEELS):
        print(f"fusion_start_power: [{', '.join(f'{found[w]:.1f}' for w in WHEELS)}]  "
              "(medido en el aire: en el piso suele hacer falta mas, comprobar con --all)", flush=True)


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


def mode_chase(args, cfg):
    gpio = open_gpio(args.chip, args.dry_run)
    motors = Motors(cfg, gpio, args.max_speed, args.dry_run)
    line = LineSensor(cfg, gpio)
    cam = Camera(args.camera)
    vparams = vision_params(cfg)
    bcfg = cfg["behavior"]
    fields = BehaviorParams.__dataclass_fields__
    values = {k: v for k, v in bcfg.items() if k in fields and k != "my_colors"}
    targets = parse_colors(args.target if args.target is not None else bcfg.get("my_colors", ""))
    avoid = parse_colors(args.avoid if args.avoid is not None else bcfg.get("avoid_colors", ""))
    if not targets:
        raise SystemExit("sin mision: pasar --target (por ejemplo --target blue --avoid red)")
    values.update(has_gate=False, explore_speed=min(values.get("explore_speed", 0.12), args.max_speed),
                  approach_speed=min(values.get("approach_speed", 0.10), args.max_speed))
    if args.capture_cy is not None:
        values["capture_cy"] = args.capture_cy
    fsm = BehaviorFSM(BehaviorParams(my_colors=targets, **values))
    fsm.set_mission(targets, avoid)
    gcfg = cfg["safety_gate"]
    gfields = SafetyParams.__dataclass_fields__
    gate = SafetyGate(SafetyParams(**{**{k: v for k, v in gcfg.items() if k in gfields},
                                      "require_pose": False, "require_heartbeat": False}))
    frames = FrameBuffer()
    serve(frames, args.port)
    print(f"objetivo {list(targets)}, prohibidos {list(avoid)}; video en http://<ip-del-robot>:{args.port}/ ; Ctrl+C para parar",
          flush=True)
    t0 = time.monotonic()
    previous = None
    try:
        while True:
            now = time.monotonic() - t0
            frame = cam.read()
            dets = detect_cubes(frame, vparams) if frame is not None else []
            cubes = [Cube(d.color, d.cx_norm, d.cy_norm, d.area_px) for d in dets]
            # The line sensor reads the blue square as white: the camera keeps the body out of it.
            square = detect_square(frame, vparams) if frame is not None else None
            black = line.black()
            scoop = scoop_content(frame, vparams)[0] if frame is not None else ""
            want = fsm.step(BehaviorInput(
                now=now, cubes=cubes, line_front=black, scoop_color=scoop,
                square_visible=square is not None and square.visible,
                square_cx=square.cx_norm if square is not None else 0.0,
                square_cy=square.cy_norm if square is not None else 0.0,
                square_area=square.area_px if square is not None else 0.0))
            cmd, reason = gate.filter(SafetyInput(cmd=want, line_front=black, own_pose=None, peer_pose=None, now=now,
                                                   heartbeat_age=None, carrying=fsm.carrying))
            if fsm.state is not previous:
                note = f" ({fsm.note})" if fsm.note else ""
                print(f"[{now:6.1f}] {previous.value if previous else '-'} -> {fsm.state.value}{note}", flush=True)
                previous = fsm.state
            if fsm.state is State.CAPTURE and args.stop_at_capture:
                motors.stop()
                print(f"[{now:6.1f}] cubo en la boca de la pala: parado (sin pala aun). Ctrl+C para salir.", flush=True)
                while True:
                    f = cam.read()
                    if f is not None:
                        frames.put(annotate(f, detect_cubes(f, vparams), vparams, ["CAPTURE: parado"]))
                    time.sleep(0.1)
            motors.drive(*cmd)
            if frame is not None:
                frames.put(annotate(frame, dets, vparams,
                                    [f"{fsm.state.value} linea={int(black)} pala={scoop or '-'} "
                                     f"prohibidos tocados={fsm.forbidden_touches}",
                                     f"cmd vx={cmd[0]:+.2f} vy={cmd[1]:+.2f} wz={cmd[2]:+.2f} ({reason})"]))
            time.sleep(max(0.0, 0.1 - (time.monotonic() - t0 - now)))
    finally:
        motors.stop()
        motors.gpio.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("mode", choices=["motors", "threshold", "drive", "line", "camera", "chase"])
    parser.add_argument("--robot", default="phineas", choices=["phineas", "ferb"])
    parser.add_argument("--dry-run", action="store_true", help="no GPIO access (laptop check)")
    parser.add_argument("--chip", type=int, default=None, help="gpiochip number (default: autodetect)")
    parser.add_argument("--camera", default="auto")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--max-speed", type=float, default=0.10, help="m/s, lab cap")
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
    parser.add_argument("--step-time", type=float, default=0.7, help="threshold mode: seconds per step")
    parser.add_argument("--all", action="store_true", help="threshold mode: all four wheels together (on the floor)")
    parser.add_argument("--latch-timeout", type=float, default=3.0, help="drive mode: stop after N s with no key")
    parser.add_argument("--save-every", type=float, default=0.0,
                        help="camera mode: save the raw and the annotated frame every N seconds")
    parser.add_argument("--save-dir", default="capturas", help="camera mode: folder for --save-every")
    parser.add_argument("--save-max", type=int, default=200, help="camera mode: stop saving after N snapshots")
    parser.add_argument("--capture-cy", type=float, default=None, help="chase: override capture_cy")
    parser.add_argument("--target", default=None, help="chase: target colors, comma separated (the mission)")
    parser.add_argument("--avoid", default=None, help="chase: forbidden colors, comma separated")
    parser.add_argument("--no-stop-at-capture", dest="stop_at_capture", action="store_false")
    args = parser.parse_args()
    cfg = load_config(args.robot)
    if args.battery is not None:
        cfg["hat_motor_driver"]["battery_voltage"] = args.battery
    modes = {"motors": mode_motors, "threshold": mode_threshold, "drive": mode_drive, "line": mode_line, "camera": mode_camera,
             "chase": mode_chase}
    try:
        modes[args.mode](args, cfg)
    except KeyboardInterrupt:
        print("\nparado", flush=True)


if __name__ == "__main__":
    main()
