"""
Camera backends for the browser tool. Each one is a function that returns a
BGR frame (or None when no frame is ready).

Needs OpenCV and NumPy; picamera2 is imported lazily because it only exists
on the Pi. `camera.py` imports this module lazily, so the robot still
drives when OpenCV is missing.
"""

import time
from collections.abc import Callable

import cv2
import numpy as np

FrameSource = Callable[[], np.ndarray | None]

BACKEND_PICAMERA2 = "picamera2"

# --- Fake camera (laptop) -----------------------------------------------------
BGR_CHANNELS = 3
FAKE_BACKGROUND_BGR = (40, 40, 40)
FAKE_SQUARE_BGR = (0, 0, 255)
FAKE_SQUARE_WIDTH_PX = 40
FAKE_SQUARE_HALF_HEIGHT_PX = 20
FAKE_SQUARE_SPEED_PX_S = 120
FAKE_LABEL = "fake camera"
FAKE_LABEL_ORIGIN_PX = (10, 30)
FAKE_LABEL_FONT = cv2.FONT_HERSHEY_SIMPLEX
FAKE_LABEL_SCALE = 0.8
FAKE_LABEL_BGR = (255, 255, 255)
FAKE_LABEL_THICKNESS = 2

# --- Pi camera ----------------------------------------------------------------
# Part of the camera's id that identifies the CSI port the OV5647 is on.
DEFAULT_PICAMERA_ID_HINT = "i2c@80000"
# In Picamera2/DRM, "RGB888" gives [B, G, R] bytes (the BGR order OpenCV
# expects).
PICAMERA_FORMAT = "RGB888"
# Bit depth of the sensor modes the IMX219 offers at the sizes we use.
SENSOR_BIT_DEPTH = 10
DEFAULT_VFLIP = True
DEFAULT_HFLIP = False

# cv2.flip codes.
FLIP_VERTICAL = 0
FLIP_HORIZONTAL = 1
FLIP_BOTH = -1


def open_frame_source(cfg: dict, fake: bool) -> FrameSource | None:
    """
    The frame source selected by the config, or None if the camera cannot be
    opened.
    """
    width_px, height_px = cfg["width_px"], cfg["height_px"]
    if fake:
        return _fake_source(width_px, height_px)
    if cfg["backend"] == BACKEND_PICAMERA2:
        return _picamera2_source(cfg, width_px, height_px)
    return _opencv_source(cfg, width_px, height_px)


def encode_jpeg(frame: np.ndarray, quality: int) -> bytes | None:
    """
    JPEG-encode a BGR frame; None if OpenCV could not encode it.
    """
    ok, buffer = cv2.imencode(
        ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality]
    )
    return buffer.tobytes() if ok else None


def _fake_source(width_px: int, height_px: int) -> FrameSource:
    """
    A grey frame with a red square sliding across it.
    """
    start_s = time.monotonic()

    def read_frame():
        """
        A frame of the fake camera.
        """
        frame = np.zeros((height_px, width_px, BGR_CHANNELS), np.uint8)
        frame[:] = FAKE_BACKGROUND_BGR
        elapsed_s = time.monotonic() - start_s
        x = int(elapsed_s * FAKE_SQUARE_SPEED_PX_S) % width_px
        mid_y = height_px // 2
        cv2.rectangle(
            frame,
            (x, mid_y - FAKE_SQUARE_HALF_HEIGHT_PX),
            (x + FAKE_SQUARE_WIDTH_PX, mid_y + FAKE_SQUARE_HALF_HEIGHT_PX),
            FAKE_SQUARE_BGR,
            -1,
        )
        cv2.putText(
            frame,
            FAKE_LABEL,
            FAKE_LABEL_ORIGIN_PX,
            FAKE_LABEL_FONT,
            FAKE_LABEL_SCALE,
            FAKE_LABEL_BGR,
            FAKE_LABEL_THICKNESS,
        )
        return frame

    return read_frame


def _picamera2_source(cfg: dict, width_px: int, height_px: int) -> FrameSource:
    """
    The Pi camera module through picamera2, flipped as configured.
    """
    # Only available on the Pi.
    from picamera2 import Picamera2

    id_hint = cfg.get("picamera_id_hint", DEFAULT_PICAMERA_ID_HINT)
    camera_number = next(
        (
            c["Num"]
            for c in Picamera2.global_camera_info()
            if id_hint in c.get("Id", "")
        ),
        cfg["device_index"],
    )
    camera = Picamera2(camera_number)
    camera.configure(
        camera.create_video_configuration(
            main={"size": (width_px, height_px), "format": PICAMERA_FORMAT},
            sensor=_sensor_mode(cfg.get("sensor_size")),
        )
    )
    camera.start()
    flip_code = _flip_code(
        vflip=bool(cfg.get("vflip", DEFAULT_VFLIP)),
        hflip=bool(cfg.get("hflip", DEFAULT_HFLIP)),
    )

    def read_frame():
        """
        Next frame of the Pi camera, flipped as configured.
        """
        frame = camera.capture_array()
        return frame if flip_code is None else cv2.flip(frame, flip_code)

    return read_frame


def _sensor_mode(sensor_size) -> dict | None:
    """
    The sensor mode to request, or None to let libcamera pick one. Asking for
    the full-frame binned size (1640x1232) keeps the whole field of view; the
    default 640x480 mode is a small crop of the centre (about 28 degrees).
    """
    if not sensor_size:
        return None
    return {
        "output_size": tuple(sensor_size),
        "bit_depth": SENSOR_BIT_DEPTH,
    }


def _flip_code(vflip: bool, hflip: bool) -> int | None:
    """
    The cv2.flip code for the requested flips, or None for no flip.
    """
    if vflip and hflip:
        return FLIP_BOTH
    if vflip:
        return FLIP_VERTICAL
    if hflip:
        return FLIP_HORIZONTAL
    return None


def _opencv_source(
    cfg: dict, width_px: int, height_px: int
) -> FrameSource | None:
    """
    A USB webcam through OpenCV, or None if it cannot be opened.
    """
    capture = cv2.VideoCapture(cfg["device_index"])
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, width_px)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height_px)
    if not capture.isOpened():
        return None

    def read_frame():
        """
        Next frame of the webcam, or None if none is ready.
        """
        ok, frame = capture.read()
        return frame if ok else None

    return read_frame
