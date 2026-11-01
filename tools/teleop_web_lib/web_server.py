"""
HTTP layer of the browser tool: the control page, the MJPEG video stream and
the /cmd endpoint the page polls. No hardware or ROS: it only talks to the
`Drive` and `Camera` objects it is given.
"""

import json
import logging
import time
from collections.abc import Callable
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler
from pathlib import Path

log = logging.getLogger("teleop_web")

PAGE_PATH = Path(__file__).with_name("web") / "index.html"

PATH_INDEX = "/"
PATH_STREAM = "/stream.mjpg"
PATH_CMD = "/cmd"

CONTENT_HTML = "text/html; charset=utf-8"
CONTENT_JSON = "application/json"
MJPEG_BOUNDARY = "frame"
MJPEG_CONTENT_TYPE = f"multipart/x-mixed-replace; boundary={MJPEG_BOUNDARY}"
# Sequence number older than any real frame: asks for the current frame.
ANY_FRAME_SEQ = -1
# While there is no camera, the stream checks again after this long.
NO_CAMERA_RETRY_S = 1.0
# /cmd bodies are tiny JSON objects; anything bigger is rejected.
MAX_BODY_BYTES = 4096


class RequestTooLargeError(ValueError):
    """
    The POST body is larger than MAX_BODY_BYTES.
    """


def read_page() -> bytes:
    """
    The control page, read once at startup.
    """
    return PAGE_PATH.read_bytes()


class ControlRequestHandler(BaseHTTPRequestHandler):
    """
    Request handler; `make_handler` binds it to a drive, a camera and the
    page.
    """

    drive: object
    camera: object
    motors_label: str
    page: bytes
    is_running: Callable[[], bool]

    def log_message(self, fmt, *args):
        """
        Send the per-request line to the debug log.
        """
        log.debug(fmt, *args)

    # ---------- GET ----------
    def do_GET(self):
        """
        Serve the page or the video stream.
        """
        if self.path == PATH_INDEX:
            self._send(HTTPStatus.OK, self.page, CONTENT_HTML)
        elif self.path == PATH_STREAM:
            self._stream()
        else:
            self.send_error(HTTPStatus.NOT_FOUND)

    def _stream(self):
        """
        Stream frames as multipart MJPEG until the client leaves.
        """
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", MJPEG_CONTENT_TYPE)
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        seq = ANY_FRAME_SEQ
        try:
            while self.is_running():
                seq, jpeg = self.camera.wait_frame(seq)
                if jpeg is not None:
                    self._write_mjpeg_part(jpeg)
                elif not self.camera.ok:
                    time.sleep(NO_CAMERA_RETRY_S)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _write_mjpeg_part(self, jpeg: bytes):
        """
        Write one frame of the multipart stream.
        """
        header = (
            f"--{MJPEG_BOUNDARY}\r\n"
            "Content-Type: image/jpeg\r\n"
            f"Content-Length: {len(jpeg)}\r\n\r\n"
        )
        self.wfile.write(header.encode() + jpeg + b"\r\n")

    # ---------- POST ----------
    def do_POST(self):
        """
        Apply the page's command and answer with the robot status.
        """
        if self.path != PATH_CMD:
            self._discard_body()
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        try:
            data = self._read_json_object()
            self._apply(data)
        except RequestTooLargeError:
            self.send_error(HTTPStatus.REQUEST_ENTITY_TOO_LARGE)
            return
        except (ValueError, TypeError):
            self.send_error(HTTPStatus.BAD_REQUEST)
            return
        status = {
            "motors": self.motors_label,
            "camera": self.camera.ok,
            **self.drive.status(),
        }
        self._send(HTTPStatus.OK, json.dumps(status).encode(), CONTENT_JSON)

    def _discard_body(self):
        """
        Read and drop a small request body, so closing the socket right
        after the answer does not reset the client's connection.
        """
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            return
        self.rfile.read(max(0, min(length, MAX_BODY_BYTES)))

    def _read_json_object(self) -> dict:
        """
        Read the request body as a JSON object.
        """
        length = int(self.headers.get("Content-Length", 0))
        if length > MAX_BODY_BYTES:
            raise RequestTooLargeError(
                f"body larger than {MAX_BODY_BYTES} bytes"
            )
        data = json.loads(self.rfile.read(length) or b"{}")
        if not isinstance(data, dict):
            raise ValueError("expected a JSON object")
        return data

    def _apply(self, data: dict):
        """
        Forward the Auto switch and the manual command to the drive.
        """
        if "auto" in data or "reset_auto" in data:
            self.drive.set_auto(
                data.get("auto"),
                bool(data.get("reset_auto", False)),
            )
        self.drive.command(
            float(data.get("vx", 0)),
            float(data.get("vy", 0)),
            float(data.get("wz", 0)),
            float(data.get("speed", 0)),
        )

    def _send(self, code: int, body: bytes, content_type: str):
        """
        Send a complete response with the given body.
        """
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def make_handler(
    drive,
    camera,
    motors_label: str,
    page: bytes,
    is_running: Callable[[], bool] = lambda: True,
) -> type[ControlRequestHandler]:
    """
    Handler class bound to the drive, the camera, the label of the motor
    driver shown in the page, the page itself and a callable that tells
    whether the application is still running.
    """
    return type(
        "BoundControlRequestHandler",
        (ControlRequestHandler,),
        {
            "drive": drive,
            "camera": camera,
            "motors_label": motors_label,
            "page": page,
            "is_running": staticmethod(is_running),
        },
    )
