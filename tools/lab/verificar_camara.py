#!/usr/bin/env python3
"""
verificar_camara.py — Verifica el funcionamiento de la cámara en la Raspberry Pi 5
y transmite el video con detección de cubos en http://<ip-del-robot>:8080/

- Si la cámara CSI (IMX219) o USB (V4L2) está disponible, transmite el video en vivo
  con la detección de los 6 colores (igual que `lab_robot.py camera`).
- Si el sensor IMX219 aún no responde por I2C (-121 EREMOTEIO), mantiene activo el
  servidor web en http://<ip>:8080/ mostrando en vivo el diagnóstico de los puertos
  CAM0/CAM1 y la prueba del detector de visión, reintentando conectar la cámara
  automáticamente cada 3 segundos.

Uso:
    python3 tools/lab/verificar_camara.py
"""
import argparse
import os
import pathlib
import subprocess
import sys
import time

import cv2
import numpy as np

THIS_DIR = pathlib.Path(__file__).resolve().parent
REPO = THIS_DIR.parents[1]
sys.path.insert(0, str(THIS_DIR))
sys.path.insert(0, str(REPO / "src" / "mobile_base"))

from hardware import LOG, load_config  # noqa: E402
from lab_robot import annotate, vision_params  # noqa: E402
from stream import FrameBuffer, serve  # noqa: E402
from mobile_base.vision import detect_cubes, scoop_content  # noqa: E402


def try_open_real_camera(width=640, height=480, target_fps=12):
    """Intenta abrir Picamera2 (CSI) o V4L2 (USB) limitando el consumo de energia."""
    cv2.setNumThreads(1)
    errors = []
    # 1. Picamera2 (libcamera CSI en Pi 5)
    try:
        from picamera2 import Picamera2

        infos = Picamera2.global_camera_info()
        if not infos:
            errors.append("picamera2: 0 camaras detectadas por libcamera")
        else:
            cam = Picamera2()
            frame_us = int(1_000_000 / max(1, target_fps))
            cam.configure(
                cam.create_video_configuration(
                    main={"size": (width, height), "format": "RGB888"},
                    controls={"FrameDurationLimits": (frame_us, frame_us)},
                )
            )
            cam.start()
            time.sleep(0.4)
            return ("picamera2", cam.capture_array, cam.close, "")
    except Exception as exc:
        errors.append(f"picamera2: {exc}")

    # 2. V4L2 (webcam USB si existe /dev/video0..2)
    for idx in (0, 1):
        if not os.path.exists(f"/dev/video{idx}"):
            continue
        try:
            cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            if cap.isOpened():
                ok, frame = cap.read()
                if ok and frame is not None:
                    return (
                        f"v4l2:/dev/video{idx}",
                        lambda c=cap: c.read()[1],
                        cap.release,
                        "",
                    )
            cap.release()
        except Exception as exc:
            errors.append(f"v4l2({idx}): {exc}")

    return (None, None, None, " | ".join(errors))


def get_csi_diagnostics() -> list[str]:
    """Lee el estado de dmesg e I2C para CAM0 (10-0010) y CAM1 (11-0010)."""
    lines = []
    for bus, label in ((10, "CAM/DISP 0 (i2c-10)"), (11, "CAM/DISP 1 (i2c-11)")):
        dev_path = f"/sys/bus/i2c/devices/{bus}-0010"
        driver_link = f"{dev_path}/driver"
        if os.path.exists(driver_link):
            lines.append(f"{label}: DETECTADA (driver imx219 activo)")
        elif os.path.exists(dev_path):
            lines.append(f"{label}: Sin respuesta I2C (-121 EREMOTEIO)")
        else:
            lines.append(f"{label}: Overlay no cargado")

    try:
        r = subprocess.run(
            ["dmesg"],
            capture_output=True,
            text=True,
            timeout=2,
            errors="replace",
        )
        imx_lines = [
            ln.split("]", 1)[-1].strip()
            for ln in r.stdout.splitlines()
            if "imx219" in ln.lower()
        ]
        if imx_lines:
            lines.append(f"dmesg: {imx_lines[-1][:62]}")
    except Exception:
        pass
    return lines


def build_diagnostic_frame(
    fixture_img, step: int, diag_lines: list[str], err_summary: str
) -> np.ndarray:
    """Genera un frame 640x480 con imagen de prueba + cubos de los 6 colores + estado."""
    if fixture_img is not None:
        frame = cv2.resize(fixture_img, (640, 480))
    else:
        frame = np.full((480, 640, 3), (225, 225, 225), dtype=np.uint8)

    # Dibujar los 6 cubos de prueba en la parte inferior para verificar el detector de colores
    # BGR aproximados con alta saturación dentro de los rangos HSV de phineas.yaml
    test_cubes = [
        ("red", (20, 20, 230)),
        ("orange", (10, 115, 245)),
        ("yellow", (15, 225, 240)),
        ("green", (35, 190, 35)),
        ("blue", (230, 110, 20)),
        ("purple", (200, 35, 160)),
    ]
    offset = int(12 * np.sin(step * 0.15))
    for i, (_name, bgr) in enumerate(test_cubes):
        x = 65 + i * 92 + offset
        y = 370
        cv2.rectangle(frame, (x - 24, y - 24), (x + 24, y + 24), bgr, -1)

    # Panel superior informativo sobre el estado físico de la cámara
    overlay = frame.copy()
    cv2.rectangle(overlay, (10, 52), (630, 215), (20, 20, 20), -1)
    frame = cv2.addWeighted(overlay, 0.82, frame, 0.18, 0)

    cv2.putText(
        frame,
        "MODO DIAGNOSTICO (CAMARA FISICA NO DETECTADA AUN)",
        (22, 78),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 200, 255),
        2,
    )
    y_text = 104
    for dline in diag_lines[:3]:
        cv2.putText(
            frame,
            dline,
            (22, y_text),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.46,
            (220, 220, 220),
            1,
        )
        y_text += 22

    cv2.putText(
        frame,
        "Revisa el cable CSI de 22 pines (con la Pi apagada). Reintentando...",
        (22, y_text + 6),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.44,
        (140, 230, 140),
        1,
    )
    if err_summary:
        cv2.putText(
            frame,
            err_summary[:75],
            (22, y_text + 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.40,
            (180, 180, 255),
            1,
        )
    return frame


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", default="phineas", choices=("phineas", "ferb"))
    parser.add_argument("--port", type=int, default=8080)
    args = parser.parse_args()

    cfg = load_config(args.robot)
    params = vision_params(cfg)
    frames = FrameBuffer()
    serve(frames, args.port)

    fixture_path = REPO / "src" / "mobile_base" / "test" / "fixtures" / "webots_cube_at_mouth.jpg"
    fixture_img = cv2.imread(str(fixture_path)) if fixture_path.exists() else None

    print(f"[info] Servidor web activo en: http://10.68.17.64:{args.port}/", flush=True)

    backend, read_fn, close_fn, err_summary = try_open_real_camera()
    if backend:
        LOG.info(f"Camara real conectada: {backend}")
    else:
        LOG.warn(f"Camara fisica no detectada ({err_summary}). Mostrando diagnostico en el navegador.")

    count = 0
    t0 = time.monotonic()
    last_print = 0.0
    last_probe = time.monotonic()
    diag_lines = get_csi_diagnostics()

    try:
        while True:
            now = time.monotonic()
            if backend is None and (now - last_probe) >= 3.0:
                last_probe = now
                diag_lines = get_csi_diagnostics()
                backend, read_fn, close_fn, err_summary = try_open_real_camera()
                if backend:
                    LOG.info(f"Camara real conectada: {backend}")
                    count, t0 = 0, time.monotonic()

            if backend is not None:
                frame = read_fn()
                if frame is None:
                    LOG.warn("Frame vacio de la camara")
                    time.sleep(0.1)
                    continue
                if frame.ndim == 3 and frame.shape[2] == 4:
                    frame = frame[:, :, :3]
                mode_label = backend
            else:
                frame = build_diagnostic_frame(fixture_img, count, diag_lines, err_summary)
                mode_label = "diagnostico (sin camara CSI)"

            dets = detect_cubes(frame, params)
            scoop, fill = scoop_content(frame, params)
            count += 1
            elapsed = max(1e-3, time.monotonic() - t0)
            fps = count / elapsed

            annotated = annotate(
                frame,
                dets,
                params,
                [f"{mode_label} {fps:.1f} fps", f"pala: {scoop or '-'} {fill:.2f}"],
            )
            frames.put(annotated)

            if now - last_print >= 2.0:
                last_print = now
                found = ", ".join(
                    f"{d.color}(cx {d.cx_norm:+.2f}, cy {d.cy_norm:.2f}, {d.area_px:.0f}px)"
                    for d in dets
                )
                print(
                    f"[{mode_label}] {fps:4.1f} fps | pala={scoop or '-'} {fill:.2f} | {found or 'sin cubos'}",
                    flush=True,
                )
                # Keepalive UDP ligero hacia la laptop para mantener vivo el ARP en AP-Prototipos
                try:
                    import socket
                    s_ka = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    s_ka.sendto(b"ka", ("10.68.17.54", 9))
                    s_ka.close()
                except Exception:
                    pass

            if backend is None:
                time.sleep(0.08)
    except KeyboardInterrupt:
        pass
    finally:
        if close_fn:
            try:
                close_fn()
            except Exception:
                pass


if __name__ == "__main__":
    main()
