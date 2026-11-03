"""
Capture thread of the browser tool: grabs frames, runs the detection
pipeline on them and keeps the latest one as a JPEG for the page.

OpenCV is imported lazily (inside the thread): the camera is optional, and
the robot keeps driving if it cannot start.
"""

import logging
import threading
import time

log = logging.getLogger("teleop_web")

# How long a stream waits for the next frame before checking again.
FRAME_WAIT_S = 2.0
# Sequence number older than any real frame: asks for the current frame.
ANY_FRAME_SEQ = -1


class Camera:
    """
    Captures in a thread and keeps the latest frame as JPEG.

    It also answers the autopilot's questions (`latest_detection`,
    `latest_zone`) through the detection pipeline, once that exists.
    """

    def __init__(self, cfg: dict, fake: bool, detection_overrides=None):
        """
        Start the capture thread.
        """
        self._cfg = cfg
        self._fake = fake
        self._detection_overrides = detection_overrides
        self._cond = threading.Condition()
        self._jpeg: bytes | None = None
        self._seq = 0
        self._pipeline = None
        self.ok = False
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    # ---------- what the autopilot asks ----------
    def latest_detection(self, now: float, timeout_s: float):
        """
        The best cube seen less than `timeout_s` ago, or None.
        """
        pipeline = self._pipeline
        return (
            None
            if pipeline is None
            else pipeline.latest_detection(now, timeout_s)
        )

    def latest_zone(self, now: float, timeout_s: float):
        """
        The delivery gate seen less than `timeout_s` ago, or None.
        """
        pipeline = self._pipeline
        return (
            None if pipeline is None else pipeline.latest_zone(now, timeout_s)
        )

    # ---------- what the web server asks ----------
    def wait_frame(
        self,
        last_seq: int,
        timeout_s: float = FRAME_WAIT_S,
    ) -> tuple[int, bytes | None]:
        """
        Wait for a frame newer than `last_seq`; returns (seq, jpeg).
        """
        with self._cond:
            self._cond.wait_for(
                lambda: self._seq != last_seq,
                timeout=timeout_s,
            )
            return self._seq, self._jpeg

    def close(self) -> None:
        """
        Ask the capture thread to stop.
        """
        self._running = False

    # ---------- capture thread ----------
    def _loop(self) -> None:
        """
        Grab, annotate and encode frames until closed.
        """
        try:
            from teleop_web_lib.detection import build_pipeline
            from teleop_web_lib.frame_source import (
                encode_jpeg,
                open_frame_source,
            )

            grab = open_frame_source(self._cfg, self._fake)
        except Exception as exc:  # camera is optional: control keeps working
            log.warning("no camera: %s", exc)
            return
        if grab is None:
            log.warning("could not open camera %s", self._cfg["device_index"])
            return
        self._pipeline = build_pipeline(self._cfg, self._detection_overrides)
        self.ok = True
        period_s = 1.0 / self._cfg["fps"]
        while self._running:
            started_s = time.monotonic()
            frame = grab()
            if frame is not None:
                if self._pipeline is not None:
                    frame = self._pipeline.process(frame)
                jpeg = encode_jpeg(frame, self._cfg["jpeg_quality"])
                if jpeg is not None:
                    self._store(jpeg)
            time.sleep(max(0.0, period_s - (time.monotonic() - started_s)))

    def _store(self, jpeg: bytes) -> None:
        """
        Keep a new frame and wake up the streams waiting for it.
        """
        with self._cond:
            self._jpeg = jpeg
            self._seq += 1
            self._cond.notify_all()
