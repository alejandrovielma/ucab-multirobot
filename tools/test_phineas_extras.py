"""
Phineas-only extras on top of Ferb's tool: never give up searching, a lighter
video stream and the overhead (DroidCam) relay.
"""

import cv2
import numpy as np
from test_autopilot import make_pilot

from teleop_web_lib.camera import Camera
from teleop_web_lib.config import AutoConfig
from teleop_web_lib.frame_source import shrink_frame


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
