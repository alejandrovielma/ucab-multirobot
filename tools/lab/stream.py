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


def serve(frames: FrameBuffer, port: int) -> ThreadingHTTPServer:
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_args):  # keep the console for the robot output
            pass

        def do_GET(self):
            if self.path == "/":
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.end_headers()
                self.wfile.write(PAGE)
                return
            if self.path != "/stream.mjpg":
                self.send_error(404)
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
