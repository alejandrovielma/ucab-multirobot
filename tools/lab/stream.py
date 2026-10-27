"""Tiny MJPEG HTTP server: open http://<robot-ip>:<port>/ in a browser on the same network.

Standard library only (http.server + threading) so nothing has to be installed on the Pi.
"""

import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

PAGE = b"""<!doctype html><html><head><meta charset="utf-8"><title>Camara del robot</title>
<style>body{margin:0;background:#111;color:#ddd;font:14px sans-serif;text-align:center}
img{max-width:100%;height:auto}</style></head>
<body><p>Camara del robot (se actualiza sola)</p><img src="/stream.mjpg"></body></html>"""


class FrameBuffer:
    def __init__(self):
        self._jpeg = None
        self._cond = threading.Condition()

    def put(self, bgr) -> None:
        import cv2

        ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok:
            with self._cond:
                self._jpeg = buf.tobytes()
                self._cond.notify_all()

    def wait(self, timeout: float = 2.0):
        with self._cond:
            self._cond.wait(timeout)
            return self._jpeg


def serve(frames: FrameBuffer, port: int, page: bytes = PAGE, api=None, token: str = "") -> ThreadingHTTPServer:
    """Serve `page` at / and the camera at /stream.mjpg. api(method, path, body) -> (status, json bytes) or
    None answers the other routes (the teleop page). With a token every route but / needs ?t=<token>
    (or the X-Token header): the server listens on every interface, the university Wi-Fi included."""
    from urllib.parse import parse_qs, urlsplit

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_args):  # keep the console for the robot output
            pass

        def _route(self) -> str | None:
            url = urlsplit(self.path)
            given = parse_qs(url.query).get("t", [self.headers.get("X-Token", "")])[0]
            if token and url.path != "/" and given != token:
                self.send_error(403, "token")
                return None
            return url.path

        def _reply(self, status: int, body: bytes, content_type: str = "application/json") -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self):
            path = self._route()
            if path is None:
                return
            length = min(int(self.headers.get("Content-Length", 0) or 0), 4096)  # tiny JSON only
            answer = api("POST", path, self.rfile.read(length)) if api else None
            if answer is None:
                self.send_error(404)
                return
            self._reply(*answer)

        def do_GET(self):
            path = self._route()
            if path is None:
                return
            if path == "/":
                self._reply(200, page, "text/html; charset=utf-8")
                return
            if path != "/stream.mjpg":
                answer = api("GET", path, b"") if api else None
                if answer is None:
                    self.send_error(404)
                    return
                self._reply(*answer)
                return
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    jpeg = frames.wait()
                    if jpeg is None:
                        continue
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass  # browser closed

    server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server
