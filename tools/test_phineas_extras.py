"""
Phineas-only extras on top of Ferb's tool: never give up searching, a lighter
video stream and the overhead (DroidCam) relay.
"""

import cv2
import numpy as np
from test_autopilot import make_pilot

from teleop_web_lib import overhead
from teleop_web_lib.camera import Camera
from teleop_web_lib.config import AutoConfig
from teleop_web_lib.frame_source import shrink_frame
from teleop_web_lib.overhead import OverheadCamera, jpeg_width, split_jpegs


def test_no_cubes_stop_zero_never_gives_up():
    pilot, _ = make_pilot()
    pilot.cfg = AutoConfig(no_cubes_stop_s=0.0)
    pilot.configure(now=0.0, enabled=True, reset=True)
    pilot.step(0.0)
    assert pilot.step(100000.0).speed != 0.0
    assert pilot.state != "NO CUBES (stopped)"


def test_no_cubes_stop_nonzero_still_gives_up():
    pilot, _ = make_pilot()
    pilot.configure(now=0.0, enabled=True, reset=True)
    pilot.step(0.0)
    assert pilot.cfg.no_cubes_stop_s > 0
    pilot.step(pilot.cfg.no_cubes_stop_s + 1.0)
    assert pilot.state == "NO CUBES (stopped)"


def test_shrink_frame_sets_the_width_and_keeps_the_aspect_ratio():
    frame = np.zeros((480, 640, 3), np.uint8)
    small = shrink_frame(frame, 400)
    assert small.shape == (300, 400, 3)


def _stream_width_of(camera_cfg):
    camera = Camera(camera_cfg, fake=True, detection_overrides=None)
    try:
        seq, jpeg = camera.wait_frame(-1)
        while jpeg is None:
            seq, jpeg = camera.wait_frame(seq, timeout_s=5.0)
        return cv2.imdecode(np.frombuffer(jpeg, np.uint8), 1).shape[1]
    finally:
        camera.close()


def test_the_stream_copy_is_smaller_only_when_configured():
    base = {
        "width_px": 640,
        "height_px": 480,
        "fps": 30,
        "jpeg_quality": 70,
        "backend": "opencv",
        "device_index": 0,
    }
    assert _stream_width_of(base) == 640
    assert _stream_width_of({**base, "stream_width_px": 400}) == 400


JPEG_HEAD = b"\xff\xd8"
JPEG_TAIL = b"\xff\xd9"


def test_split_jpegs_returns_complete_frames_and_the_rest():
    first = JPEG_HEAD + b"aa" + JPEG_TAIL
    second = JPEG_HEAD + b"b" + JPEG_TAIL
    frames, rest = split_jpegs(b"junk" + first + second + JPEG_HEAD + b"cc")
    assert frames == [first, second]
    assert rest == JPEG_HEAD + b"cc"
    assert split_jpegs(b"no image here") == ([], b"")


def test_jpeg_width_reads_the_start_of_frame_header():
    ok, buffer = cv2.imencode(".jpg", np.zeros((30, 50, 3), np.uint8))
    assert ok and jpeg_width(buffer.tobytes()) == 50
    assert jpeg_width(b"not a jpeg") is None


class FakeResponse:
    """
    Stands in for the DroidCam HTTP response: sends one frame, then ends.
    """

    def __init__(self, data):
        self._chunks = [data, b""]

    def read1(self, _size):
        return self._chunks.pop(0)

    def __enter__(self):  # noqa: D105
        return self

    def __exit__(self, *_exc):  # noqa: D105
        return False


def test_the_overhead_relay_shrinks_the_frame_without_any_network(monkeypatch):
    ok, buffer = cv2.imencode(".jpg", np.zeros((480, 640, 3), np.uint8))
    data = b"--frame\r\n" + buffer.tobytes()
    monkeypatch.setattr(
        overhead.urllib.request, "urlopen", lambda *_a, **_k: FakeResponse(data)
    )
    camera = OverheadCamera("http://fake.invalid/video", width_px=320)
    try:
        seq, jpeg = camera.wait_frame(0, timeout_s=5.0)
        assert jpeg is not None and jpeg_width(jpeg) == 320
    finally:
        camera.close()


def test_the_overhead_relay_survives_an_unreachable_phone(monkeypatch):
    def refuse(*_args, **_kwargs):
        raise OSError("unreachable")

    monkeypatch.setattr(overhead.urllib.request, "urlopen", refuse)
    camera = OverheadCamera("http://fake.invalid/video")
    try:
        assert camera.wait_frame(0, timeout_s=0.2)[1] is None
        assert not camera.ok
    finally:
        camera.close()
