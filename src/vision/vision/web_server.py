"""
HTTP layer of the web viewer. No rclpy: the ROS node is reached through the
small `ViewerBackend` interface, so the routing can be tested on the laptop.

Routes:
  GET  /              page with the video, detected cubes and behaviour state
  GET  /stream.mjpg   MJPEG video
  GET  /snapshot.jpg  latest frame as a single JPEG
  GET  /api/state     JSON with the cubes and the state
  POST /api/params    JSON: changes behavior_fsm parameters
  POST /api/teleop    JSON {vx, vy, wz, speed}: WASD manual control
"""

import json
from collections.abc import Callable
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler
from typing import Protocol
from urllib.parse import urlsplit

from vision.frame_buffer import FrameBuffer

PATHS_INDEX = ("/", "/index.html")
PATH_STATE = "/api/state"
PATH_STREAM = "/stream.mjpg"
PATH_SNAPSHOT = "/snapshot.jpg"
PATH_PARAMS = "/api/params"
PATH_TELEOP = "/api/teleop"

CONTENT_HTML = "text/html; charset=utf-8"
CONTENT_JSON = "application/json"
CONTENT_JPEG = "image/jpeg"
CONTENT_TEXT = "text/plain"

MJPEG_BOUNDARY = "frame"
MJPEG_CONTENT_TYPE = f"multipart/x-mixed-replace; boundary={MJPEG_BOUNDARY}"
# How long a stream waits for the next frame before checking again.
STREAM_WAIT_S = 2.0
# Sequence number older than any real frame: asks for the current frame.
ANY_FRAME_SEQ = -1
# POST bodies are tiny JSON objects; anything bigger is rejected.
MAX_BODY_BYTES = 4096


class ViewerBackend(Protocol):
    """
    What the HTTP layer needs from the ROS node.
    """

    frames: FrameBuffer

    def snapshot(self) -> dict:
        """
        State shown by the page: cubes, behaviour, fps and colour names.
        """

    def teleop(self, data: dict) -> None:
        """
        Publish a manual-control command from the page's raw JSON.
        """

    def set_behavior_params(self, values: dict) -> tuple[bool, str]:
        """
        Change behavior_fsm parameters; returns (ok, message).
        """


class RequestTooLargeError(ValueError):
    """
    The POST body is larger than MAX_BODY_BYTES.
    """


class ViewerRequestHandler(BaseHTTPRequestHandler):
    """
    Request handler; `make_handler` binds it to a backend.
    """

    backend: ViewerBackend
    index_html: bytes
    is_running: Callable[[], bool]

    def log_message(self, fmt, *args):
        """
        Silence the per-request log line.
        """

    # ---------- GET ----------
    def do_GET(self):
        """
        Serve the page, the API state, the video stream or a snapshot.
        """
        path = urlsplit(self.path).path
        if path in PATHS_INDEX:
            self._send(HTTPStatus.OK, self.index_html, CONTENT_HTML)
        elif path == PATH_STATE:
            self._json(HTTPStatus.OK, self.backend.snapshot())
        elif path == PATH_STREAM:
            self._stream()
        elif path == PATH_SNAPSHOT:
            self._snapshot()
        else:
            self._not_found()

    def _snapshot(self):
        """
        Send the latest frame, or 503 if no frame has arrived yet.
        """
        _, jpeg = self.backend.frames.wait_next(ANY_FRAME_SEQ, 0.0)
        if jpeg is None:
            self._send(
                HTTPStatus.SERVICE_UNAVAILABLE, b"no image", CONTENT_TEXT
            )
        else:
            self._send(HTTPStatus.OK, jpeg, CONTENT_JPEG)

    def _stream(self):
        """
        Stream frames as multipart MJPEG until the client leaves.
        """
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", MJPEG_CONTENT_TYPE)
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        seq = ANY_FRAME_SEQ
        try:
            while self.is_running():
                seq, jpeg = self.backend.frames.wait_next(seq, STREAM_WAIT_S)
                if jpeg is not None:
                    self._write_mjpeg_part(jpeg)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _write_mjpeg_part(self, jpeg: bytes):
        """
        Write one frame of the multipart stream.
        """
        header = (
            f"--{MJPEG_BOUNDARY}\r\n"
            f"Content-Type: {CONTENT_JPEG}\r\n"
            f"Content-Length: {len(jpeg)}\r\n\r\n"
        )
        self.wfile.write(header.encode())
        self.wfile.write(jpeg + b"\r\n")

    # ---------- POST ----------
    def do_POST(self):
        """
        Handle the teleop and parameter-change endpoints.
        """
        path = urlsplit(self.path).path
        if path not in (PATH_TELEOP, PATH_PARAMS):
            self._discard_body()
            self._not_found()
            return
        try:
            values = self._read_json_object()
        except RequestTooLargeError as e:
            self._json_error(HTTPStatus.REQUEST_ENTITY_TOO_LARGE, e)
            return
        except ValueError as e:
            self._json_error(HTTPStatus.BAD_REQUEST, e)
            return
        if path == PATH_TELEOP:
            self._handle_teleop(values)
        else:
            self._handle_params(values)

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
        length = int(self.headers.get("Content-Length", "0"))
        if length > MAX_BODY_BYTES:
            raise RequestTooLargeError(
                f"body larger than {MAX_BODY_BYTES} bytes"
            )
        values = json.loads(self.rfile.read(length) or b"{}")
        if not isinstance(values, dict):
            raise ValueError("expected a JSON object")
        return values

    def _handle_teleop(self, values: dict):
        """
        Forward a manual-control command to the node.
        """
        try:
            self.backend.teleop(values)
        except (ValueError, TypeError) as e:
            self._json_error(HTTPStatus.BAD_REQUEST, e)
            return
        self._json(HTTPStatus.OK, {"ok": True, "message": "ok"})

    def _handle_params(self, values: dict):
        """
        Forward a parameter change to behavior_fsm.
        """
        ok, message = self.backend.set_behavior_params(values)
        status = HTTPStatus.OK if ok else HTTPStatus.CONFLICT
        self._json(status, {"ok": ok, "message": message})

    # ---------- responses ----------
    def _send(self, code: int, body: bytes, content_type: str):
        """
        Send a complete response with the given body.
        """
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, code: int, data: dict):
        """
        Send `data` as a JSON response.
        """
        self._send(code, json.dumps(data).encode(), CONTENT_JSON)

    def _json_error(self, code: int, error: Exception):
        """
        Send an error as {"ok": false, "message": ...}.
        """
        self._json(code, {"ok": False, "message": str(error)})

    def _not_found(self):
        """
        Send a 404 response.
        """
        self._send(HTTPStatus.NOT_FOUND, b"not found", CONTENT_TEXT)


def make_handler(
    backend: ViewerBackend,
    index_html: bytes,
    is_running: Callable[[], bool],
) -> type[ViewerRequestHandler]:
    """
    Handler class bound to a backend, the page to serve and a callable that
    tells whether the application is still running.
    """
    return type(
        "BoundViewerRequestHandler",
        (ViewerRequestHandler,),
        {
            "backend": backend,
            "index_html": index_html,
            "is_running": staticmethod(is_running),
        },
    )
