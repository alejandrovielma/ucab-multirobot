"""
Latest-frame buffer shared between the ROS thread and the HTTP clients.
Pure logic, no rclpy.
"""

import threading


class FrameBuffer:
    """
    Latest JPEG frame, shared between the ROS thread and the HTTP clients.
    """

    def __init__(self):
        """
        Start with no frame.
        """
        self._cond = threading.Condition()
        self._jpeg: bytes | None = None
        self._seq = 0

    def put(self, jpeg: bytes) -> None:
        """
        Store a new frame and wake up everyone waiting for one.
        """
        with self._cond:
            self._jpeg = jpeg
            self._seq += 1
            self._cond.notify_all()

    def wait_next(
        self,
        last_seq: int,
        timeout_s: float,
    ) -> tuple[int, bytes | None]:
        """
        Wait up to `timeout_s` for a frame newer than `last_seq`.

        Returns (sequence number, jpeg). The jpeg is None until the first
        frame arrives.
        """
        with self._cond:
            self._cond.wait_for(
                lambda: self._seq != last_seq,
                timeout=timeout_s,
            )
            return self._seq, self._jpeg
