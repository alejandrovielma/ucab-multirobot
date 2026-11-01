"""
Tests: HTTP routing of the web viewer against a fake backend (no ROS).
"""

import http.client
import json
import threading
from http.server import ThreadingHTTPServer

import pytest

from vision.frame_buffer import FrameBuffer
from vision.web_server import MAX_BODY_BYTES, make_handler

LOCALHOST = "127.0.0.1"
INDEX_HTML = b"<html>viewer</html>"
JPEG = b"\xff\xd8fake-jpeg\xff\xd9"


class FakeBackend:
    """
    Records what the HTTP layer forwards; params succeed when `params_ok`.
    """

    def __init__(self):
        self.frames = FrameBuffer()
        self.teleop_calls = []
        self.param_calls = []
        self.params_ok = True

    def snapshot(self):
        return {"cubes": [], "fps": 0.0}

    def teleop(self, data):
        if "vx" in data and not isinstance(data["vx"], int | float):
            raise ValueError("bad vx")
        self.teleop_calls.append(data)

    def set_behavior_params(self, values):
        self.param_calls.append(values)
        return self.params_ok, "ok" if self.params_ok else "refused"


@pytest.fixture
def server():
    backend = FakeBackend()
    handler = make_handler(backend, INDEX_HTML, lambda: False)
    httpd = ThreadingHTTPServer((LOCALHOST, 0), handler)
    httpd.daemon_threads = True
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    yield backend, httpd.server_address[1]
    httpd.shutdown()
    httpd.server_close()


def request(port, method, path, body=None):
    conn = http.client.HTTPConnection(LOCALHOST, port, timeout=5)
    conn.request(method, path, body=body)
    response = conn.getresponse()
    data = response.read()
    conn.close()
    return response.status, data


def test_index_and_state(server):
    _, port = server
    assert request(port, "GET", "/") == (200, INDEX_HTML)
    status, data = request(port, "GET", "/api/state?x=1")
    assert status == 200 and json.loads(data) == {"cubes": [], "fps": 0.0}


def test_unknown_paths_are_404(server):
    _, port = server
    assert request(port, "GET", "/nope")[0] == 404
    assert request(port, "POST", "/nope", b"{}")[0] == 404


def test_snapshot_is_503_until_a_frame_arrives(server):
    backend, port = server
    assert request(port, "GET", "/snapshot.jpg")[0] == 503
    backend.frames.put(JPEG)
    assert request(port, "GET", "/snapshot.jpg") == (200, JPEG)


def test_teleop_is_forwarded(server):
    backend, port = server
    body = json.dumps({"vx": 1, "speed": 0.5}).encode()
    assert request(port, "POST", "/api/teleop", body)[0] == 200
    assert backend.teleop_calls == [{"vx": 1, "speed": 0.5}]


def test_teleop_value_errors_are_400(server):
    _, port = server
    body = json.dumps({"vx": "fast"}).encode()
    assert request(port, "POST", "/api/teleop", body)[0] == 400


def test_non_object_and_bad_json_are_400(server):
    _, port = server
    assert request(port, "POST", "/api/teleop", b"[1, 2]")[0] == 400
    assert request(port, "POST", "/api/params", b"not json")[0] == 400


def test_params_status_follows_the_backend(server):
    backend, port = server
    body = json.dumps({"enabled": True}).encode()
    assert request(port, "POST", "/api/params", body)[0] == 200
    backend.params_ok = False
    assert request(port, "POST", "/api/params", body)[0] == 409


def test_oversized_body_is_413(server):
    backend, port = server
    # Only the header declares the size: the server must answer without
    # reading (or waiting for) the body, so nothing is sent after it.
    conn = http.client.HTTPConnection(LOCALHOST, port, timeout=5)
    conn.putrequest("POST", "/api/params")
    conn.putheader("Content-Length", str(MAX_BODY_BYTES + 1))
    conn.endheaders()
    response = conn.getresponse()
    status = response.status
    conn.close()
    assert status == 413
    assert backend.param_calls == []
