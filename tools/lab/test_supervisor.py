"""run_loop with a fake clock and fake motors: no hardware, no real sleeping."""

import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from supervisor import run_loop  # noqa: E402


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def sleep(self, dt):
        self.t += max(dt, 1e-9)


class Motors:
    def __init__(self):
        self.stops = 0

    def stop(self):
        self.stops += 1


def run(ticks, frames=lambda n, t: "frame", max_calls=1000, **kw):
    """Run until `ticks(n)` raises something that ends the loop; returns (motors, calls)."""
    clock, motors, calls = Clock(), Motors(), []

    def tick(frame, now):
        calls.append((now, motors.stops))
        if len(calls) >= max_calls:
            raise KeyboardInterrupt
        ticks(len(calls))

    n = [0]

    def read():
        n[0] += 1
        return frames(n[0], clock())

    try:
        run_loop(read, tick, motors.stop, clock=clock, sleep=clock.sleep, **kw)
    except BaseException as exc:
        return motors, calls, exc
    raise AssertionError("run_loop returned")


def test_a_failing_tick_stops_the_motors_and_the_next_ticks_run():
    def ticks(n):
        if n == 3:
            raise ValueError("boom")

    motors, calls, exc = run(ticks, max_calls=6)
    assert isinstance(exc, KeyboardInterrupt) and len(calls) == 6
    assert motors.stops == 1


def test_five_consecutive_failures_abort_with_a_nonzero_exit():
    def ticks(n):
        raise OSError("sensor")

    motors, calls, exc = run(ticks)
    assert isinstance(exc, SystemExit) and exc.code
    assert len(calls) == 5 and motors.stops == 5


def test_failures_spaced_by_a_success_do_not_abort():
    def ticks(n):
        if n % 2:
            raise OSError("flaky")

    motors, calls, exc = run(ticks, max_calls=60, max_errors_per_minute=1000)
    assert isinstance(exc, KeyboardInterrupt)


def test_too_many_errors_in_a_minute_abort_even_if_spaced():
    def ticks(n):
        if n % 2:
            raise OSError("flaky")

    motors, calls, exc = run(ticks, max_errors_per_minute=5)
    assert isinstance(exc, SystemExit) and exc.code


@pytest.mark.parametrize("stopper", [KeyboardInterrupt, lambda: SystemExit(0)])
def test_keyboard_interrupt_and_system_exit_propagate(stopper):
    def ticks(n):
        raise stopper()

    motors, calls, exc = run(ticks)
    assert isinstance(exc, (KeyboardInterrupt, SystemExit)) and len(calls) == 1


def test_a_missed_frame_below_the_timeout_does_not_stop_and_a_long_gap_does():
    # frames only on read 1, then none for ~0.5 s (no stop), then none for longer (stop), then back
    def frames(n, t):
        return None if 2 <= n <= 5 else ("frame" if t < 10 else None)

    motors, calls, exc = run(lambda n: None, frames=frames, max_calls=12, frame_timeout=1.0)
    assert calls[1][1] == 0 and len(calls) > 2  # the missed frames ran ticks without stopping

    def gap(n, t):
        return None if 1.0 <= t < 3.0 else "frame"

    motors, calls, exc = run(lambda n: None, frames=gap, max_calls=40, frame_timeout=1.0)
    assert motors.stops > 0  # stopped during the gap
    gap_ticks = [c for c in calls if 2.1 < c[0] < 2.9]
    assert not gap_ticks  # no tick while stopped
    assert calls[-1][0] > 3.0  # resumed afterwards


def test_no_frames_for_the_abort_time_aborts_safely():
    motors, calls, exc = run(lambda n: None, frames=lambda n, t: "frame" if t < 1 else None,
                             frame_timeout=1.0, frame_abort_time=30.0)
    assert isinstance(exc, SystemExit) and exc.code
    assert motors.stops > 0
