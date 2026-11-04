"""
Overhead view for the browser tool: relays an MJPEG stream (DroidCam on a
phone above the track) to the page.

DroidCam serves one client at a time, so the relay keeps the only connection
and every viewer of the page reads the latest frame from here.
"""

import logging
import threading
import time
import urllib.request

log = logging.getLogger("teleop_web")

JPEG_START = b"\xff\xd8"
JPEG_END = b"\xff\xd9"
# Start-of-frame markers (baseline and progressive): the size follows them.
JPEG_SOF0 = b"\xff\xc0"
JPEG_SOF2 = b"\xff\xc2"
CONNECT_TIMEOUT_S = 5.0
READ_CHUNK_BYTES = 65536
# Anything longer without a complete JPEG is not an image stream (DroidCam
# answers a HTML page while another client holds the connection).
MAX_BUFFER_BYTES = 4_000_000
DEFAULT_WIDTH_PX = 640
DEFAULT_QUALITY = 60
DEFAULT_FPS = 15.0
RETRY_S = 2.0
FRAME_WAIT_S = 2.0


def jpeg_width(jpeg: bytes) -> int | None:
    """
    Width in pixels from the JPEG's start-of-frame header, or None.
    """
    for marker in (JPEG_SOF0, JPEG_SOF2):
        index = jpeg.find(marker)
        if index >= 0 and index + 9 <= len(jpeg):
            return int.from_bytes(jpeg[index + 7 : index + 9], "big")
    return None


def split_jpegs(buffer: bytes) -> tuple[list[bytes], bytes]:
    """
    The complete JPEGs in `buffer` and the rest, which may start a new one.
    """
    frames = []
    while True:
        start = buffer.find(JPEG_START)
        end = buffer.find(JPEG_END, start + len(JPEG_START)) if start >= 0 else -1
        if end < 0:
            return frames, buffer[max(start, 0) :] if start >= 0 else b""
        end += len(JPEG_END)
        frames.append(buffer[start:end])
        buffer = buffer[end:]


def _no_signal_jpeg() -> bytes:
    """
    A dark frame that says the stream dropped, so the page does not keep
    showing the last real frame as if it were live.
    """
    import cv2
    import numpy as np

    frame = np.zeros((360, 640, 3), np.uint8)
    cv2.putText(
        frame, "DroidCam: sin senal", (120, 190),
        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 2,
    )
    return cv2.imencode(".jpg", frame)[1].tobytes()


class OverheadCamera:
    """
    Reads the upstream stream in a thread and keeps the latest frame.

    Same interface the page's stream needs from `Camera`: `ok` and
    `wait_frame`.
    """

    def __init__(
        self,
        url: str,
        width_px: int = DEFAULT_WIDTH_PX,
        jpeg_quality: int = DEFAULT_QUALITY,
        fps: float = DEFAULT_FPS,
        arena_px=None,
        zone_columns=None,
    ):
        """
        Start the reader thread. Frames are shrunk to `width_px` and sent at
        most `fps` times a second: the phone's full-size stream is ~20 Mbit/s,
        which clogs the Wi-Fi and makes every viewer lag behind.
        """
        self._url = url
        self._arena_px = arena_px
        self._zone_columns = zone_columns
        self._width_px = width_px
        self._quality = jpeg_quality
        self._period_s = 1.0 / fps
        self._next_s = 0.0
        self._cond = threading.Condition()
        self._jpeg: bytes | None = None
        self._seq = 0
        self.ok = False
        # Where the robot and the area are in the last frame (pixels of the shrunk frame).
        self.pose = None
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()

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
        Ask the reader thread to stop.
        """
        self._running = False

    def _loop(self) -> None:
        """
        Connect, relay frames, and reconnect when the stream ends.
        """
        while self._running:
            try:
                self._relay()
            except OSError as exc:
                log.warning("overhead stream %s: %s", self._url, exc)
            if self.ok:
                self._store(_no_signal_jpeg())
            self.ok = False
            threading.Event().wait(RETRY_S)

    def _relay(self) -> None:
        """
        Read one connection until it fails or stops sending images.
        """
        buffer = b""
        with urllib.request.urlopen(self._url, timeout=CONNECT_TIMEOUT_S) as response:
            while self._running:
                # read1: whatever has arrived, not a wait to fill the chunk size.
                chunk = response.read1(READ_CHUNK_BYTES)
                if not chunk:
                    return
                frames, buffer = split_jpegs(buffer + chunk)
                if len(buffer) > MAX_BUFFER_BYTES:
                    return
                if frames:
                    self.ok = True
                    now = time.monotonic()
                    if now >= self._next_s:
                        self._next_s = now + self._period_s
                        self._store(self._shrink(frames[-1]))

    def _shrink(self, jpeg: bytes) -> bytes:
        """
        The frame scaled to the configured width and re-encoded; the original
        when it cannot be decoded.
        """
        import cv2
        import numpy as np

        # Decoding at 1/2 size is several times cheaper than full size, and
        # the Pi's CPU is shared with the control loop.
        source_width = jpeg_width(jpeg)
        flag = (
            cv2.IMREAD_REDUCED_COLOR_2
            if source_width and source_width >= 2 * self._width_px
            else cv2.IMREAD_COLOR
        )
        frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), flag)
        if frame is None:
            return jpeg
        height, width = frame.shape[:2]
        if width > self._width_px:
            frame = cv2.resize(
                frame,
                (self._width_px, round(height * self._width_px / width)),
                interpolation=cv2.INTER_AREA,
            )
        from teleop_web_lib.overhead_pose import (
            DEFAULT_ARENA_PX,
            DEFAULT_ZONE_COLUMNS,
            annotate,
            locate,
        )

        self.pose = locate(
            frame,
            self._arena_px or DEFAULT_ARENA_PX,
            self._zone_columns or DEFAULT_ZONE_COLUMNS,
        )
        frame = annotate(frame, self.pose)
        ok, buffer = cv2.imencode(
            ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self._quality]
        )
        return buffer.tobytes() if ok else jpeg

    def _store(self, jpeg: bytes) -> None:
        """
        Keep a new frame and wake up the streams waiting for it.
        """
        with self._cond:
            self._jpeg = jpeg
            self._seq += 1
            self._cond.notify_all()
