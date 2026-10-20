#!/usr/bin/env python3
"""Lab tool for the real robot without ROS (Raspberry Pi 5 + Fusion HAT+ + IMX219 + line sensor).

    python3 tools/lab/lab_robot.py motors              # each port/wheel forward then backward (wiring check)
    python3 tools/lab/lab_robot.py drive               # keyboard teleop over SSH
    python3 tools/lab/lab_robot.py line                # line sensor level
    python3 tools/lab/lab_robot.py camera              # camera + cube detection, view at http://<ip>:8080/
    python3 tools/lab/lab_robot.py chase               # autonomous: explore and approach cubes of my color

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
from mobile_base.core.colors import ranges_from_flat  # noqa: E402
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


def mode_motors(args, cfg):
    motors = Motors(cfg, open_motor_gpio(args, cfg), args.max_speed, args.dry_run)
    try:
        for wheel in WHEELS:
            for direction in (1.0, -1.0):
                label = "adelante" if direction > 0 else "atras"
                print(f"rueda {wheel} ({motors.port(wheel)}): {label} {args.seconds:.1f} s "
                      f"(duty {args.duty:.2f} del tope)", flush=True)
                motors.wheel(wheel, direction * args.duty)
                time.sleep(args.seconds)
                motors.stop()
                time.sleep(0.5)
        for name, cmd in (("avanzar", (0.08, 0, 0)), ("lateral izquierda", (0, 0.08, 0)), ("girar antihorario", (0, 0, 0.8))):
            print(f"robot: {name} {args.seconds:.1f} s", flush=True)
            motors.drive(*cmd)
            time.sleep(args.seconds)
            motors.stop()
            time.sleep(0.5)
    finally:
        motors.stop()
        motors.gpio.close()


def mode_drive(args, cfg):
    motors = Motors(cfg, open_motor_gpio(args, cfg), args.max_speed, args.dry_run)
    # u/o: forward arcs (how the robot carries a cube; a spin in place can throw it out of the scoop).
    keys = {"w": (1, 0, 0), "s": (-1, 0, 0), "a": (0, 1, 0), "d": (0, -1, 0), "q": (0, 0, 1), "e": (0, 0, -1),
            "u": (0.5, 0, 0.6), "o": (0.5, 0, -0.6)}
    speed, turn = args.max_speed, 1.0
    print("w/s avanzar/retroceder, a/d lateral, q/e girar, u/o arco izq/der, espacio parar, +/- velocidad, x salir", flush=True)
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
    values.update(has_gate=False, explore_speed=min(values.get("explore_speed", 0.12), args.max_speed),
                  approach_speed=min(values.get("approach_speed", 0.10), args.max_speed))
    if args.capture_cy is not None:
        values["capture_cy"] = args.capture_cy
    fsm = BehaviorFSM(BehaviorParams(my_colors=tuple(bcfg["my_colors"]), **values))
    gcfg = cfg["safety_gate"]
    gfields = SafetyParams.__dataclass_fields__
    gate = SafetyGate(SafetyParams(**{**{k: v for k, v in gcfg.items() if k in gfields},
                                      "require_pose": False, "require_heartbeat": False}))
    frames = FrameBuffer()
    serve(frames, args.port)
    print(f"persiguiendo {bcfg['my_colors']}; video en http://<ip-del-robot>:{args.port}/ ; Ctrl+C para parar",
          flush=True)
    t0 = time.monotonic()
    previous = None
    try:
        while True:
            now = time.monotonic() - t0
            frame = cam.read()
            dets = detect_cubes(frame, vparams) if frame is not None else []
            cubes = [Cube(d.color, d.cx_norm, d.cy_norm, d.area_px) for d in dets]
            black = line.black()
            want = fsm.step(BehaviorInput(now=now, cubes=cubes, line_front=black))
            cmd, reason = gate.filter(SafetyInput(cmd=want, line_front=black, own_pose=None, peer_pose=None, now=now,
                                                   heartbeat_age=None, carrying=fsm.carrying))
            if fsm.state is not previous:
                print(f"[{now:6.1f}] {previous.value if previous else '-'} -> {fsm.state.value}", flush=True)
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
                                    [f"{fsm.state.value} linea={int(black)}",
                                     f"cmd vx={cmd[0]:+.2f} wz={cmd[2]:+.2f} ({reason})"]))
            time.sleep(max(0.0, 0.1 - (time.monotonic() - t0 - now)))
    finally:
        motors.stop()
        motors.gpio.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("mode", choices=["motors", "drive", "line", "camera", "chase"])
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
    parser.add_argument("--latch-timeout", type=float, default=3.0, help="drive mode: stop after N s with no key")
    parser.add_argument("--save-every", type=float, default=0.0,
                        help="camera mode: save the raw and the annotated frame every N seconds")
    parser.add_argument("--save-dir", default="capturas", help="camera mode: folder for --save-every")
    parser.add_argument("--save-max", type=int, default=200, help="camera mode: stop saving after N snapshots")
    parser.add_argument("--capture-cy", type=float, default=None, help="chase: override capture_cy")
    parser.add_argument("--no-stop-at-capture", dest="stop_at_capture", action="store_false")
    args = parser.parse_args()
    cfg = load_config(args.robot)
    if args.battery is not None:
        cfg["hat_motor_driver"]["battery_voltage"] = args.battery
    modes = {"motors": mode_motors, "drive": mode_drive, "line": mode_line, "camera": mode_camera,
             "chase": mode_chase}
    try:
        modes[args.mode](args, cfg)
    except KeyboardInterrupt:
        print("\nparado", flush=True)


if __name__ == "__main__":
    main()
