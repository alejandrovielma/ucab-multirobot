"""Fail-safe supervised loop for the lab modes: a failing tick or a dead camera never leaves the motors on.

Python only: it cannot help against SIGKILL, a dead Pi or a brownout."""

import time
import traceback
from collections import deque


def run_loop(read_frame, tick, stop, *, period=0.1, clock=time.monotonic, sleep=time.sleep,
             max_consecutive_errors=5, max_errors_per_minute=20,
             frame_timeout=1.0, frame_abort_time=30.0):
    """Run `tick(frame, now)` forever at `period`. `read_frame()` may return None (a missed frame is fine).

    Ordinary exceptions stop the motors for that tick and the loop goes on; too many abort with SystemExit.
    No frame for `frame_timeout` s stops the motors until one returns; `frame_abort_time` s aborts.
    KeyboardInterrupt and SystemExit are never swallowed. Callers keep their own `finally: stop()`."""
    t0 = clock()
    last_frame = t0
    waiting = False  # True while stopped for lack of frames
    consecutive, recent, last_log = 0, deque(), {}

    def fail(exc):
        nonlocal consecutive
        consecutive += 1
        now = clock()
        recent.append(now)
        while recent and now - recent[0] > 60.0:
            recent.popleft()
        try:
            stop()
        except Exception as stop_exc:  # a failing stop must not hide the original error
            print(f"supervisor: stop() failed too: {stop_exc!r}", flush=True)
        kind = type(exc).__name__
        if kind not in last_log or now - last_log[kind] >= 10.0:
            last_log[kind] = now
            tb = "".join(traceback.format_exception(exc, limit=-3)).rstrip()
            print(f"supervisor: tick failed, motors stopped, going on:\n{tb}", flush=True)
        if consecutive >= max_consecutive_errors or len(recent) > max_errors_per_minute:
            raise SystemExit(f"supervisor: ABORT, {consecutive} consecutive / {len(recent)} in the last minute "
                             f"failing ticks (last: {kind}); motors stopped")

    while True:
        now = clock() - t0
        try:
            frame = read_frame()
            if frame is not None:
                last_frame = clock()
                if waiting:
                    waiting = False
                    print("supervisor: frames are back, resuming", flush=True)
            elif clock() - last_frame > frame_timeout:
                if not waiting:
                    waiting = True
                    print(f"supervisor: no frames for {frame_timeout:g} s, motors stopped until they return",
                          flush=True)
                stop()
                if clock() - last_frame > frame_abort_time:
                    raise SystemExit(f"supervisor: ABORT, no frames for {frame_abort_time:g} s; motors stopped")
            if not waiting:
                tick(frame, now)
            consecutive = 0
        except Exception as exc:
            fail(exc)
        sleep(max(0.0, period - (clock() - t0 - now)))
