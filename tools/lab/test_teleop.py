import json
import os
import sys
import urllib.error
import urllib.request

import pytest

sys.path.insert(0, os.path.dirname(__file__))
from stream import FrameBuffer, serve  # noqa: E402
from teleop import Teleop  # noqa: E402


class Clock:
    t = 0.0

    def __call__(self):
        return self.t


def post(tp, path, obj):
    return tp.handle("POST", path, json.dumps(obj).encode())


@pytest.fixture
def tp():
    return Teleop(0.2, 1.0, clock=Clock())


@pytest.mark.parametrize("body", [b"nope", b"[1]", b"{}", b'{"vx":"1","vy":0,"wz":0}', b'{"vx":true,"vy":0,"wz":0}',
                                  b'{"vx":NaN,"vy":0,"wz":0}', b'{"vx":Infinity,"vy":0,"wz":0}'])
def test_bad_command_is_400_and_ignored(tp, body):
    assert tp.handle("POST", "/cmd", body)[0] == 400
    assert tp.current() == (0.0, 0.0, 0.0)


def test_limits_and_arcs_pass_through(tp):
    post(tp, "/cmd", {"vx": 5, "vy": -5, "wz": 0})
    assert tp.current() == (0.2, -0.2, 0.0)
    post(tp, "/cmd", {"vx": 0.1, "vy": 0.1, "wz": 9})
    assert tp.current() == (0.1, 0.1, 1.0)  # the safety gate caps a combined move


def test_watchdog(tp):
    post(tp, "/cmd", {"vx": 0.1, "vy": 0, "wz": 0})
    tp._clock.t = 0.39
    assert tp.current() == (0.1, 0.0, 0.0)
    tp._clock.t = 0.41
    assert tp.current() == (0.0, 0.0, 0.0)


def test_estop_latches_until_cleared(tp):
    assert post(tp, "/estop", {"on": True})[0] == 200
    post(tp, "/cmd", {"vx": 0.1, "vy": 0, "wz": 0})
    assert tp.estop and tp.current() == (0.0, 0.0, 0.0)
    post(tp, "/estop", {"on": False})
    assert not tp.estop and tp.current() == (0.0, 0.0, 0.0)  # the old command was dropped
    assert post(tp, "/estop", {"on": "yes"})[0] == 400


def test_status_keys_and_unknown_route(tp):
    tp.update_status(line=True, peer=0.5)
    status = json.loads(tp.handle("GET", "/status", b"")[1])
    assert set(status) == {"estop", "cmd", "reason", "line", "scoop", "peer", "cubes", "square", "fps"}
    assert status["line"] is True and status["peer"] == 0.5
    assert tp.handle("GET", "/nope", b"") is None


def test_token_over_http():
    tp = Teleop(0.2, 1.0)
    server = serve(FrameBuffer(), 0, page=b"x", api=tp.handle, token="abc")
    base = f"http://127.0.0.1:{server.server_address[1]}"
    try:
        assert urllib.request.urlopen(base + "/").status == 200
        with pytest.raises(urllib.error.HTTPError) as err:
            urllib.request.urlopen(base + "/status")
        assert err.value.code == 403
        assert urllib.request.urlopen(base + "/status?t=abc").status == 200
    finally:
        server.shutdown()
