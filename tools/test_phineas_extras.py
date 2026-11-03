"""
Phineas-only extras on top of Ferb's tool: never give up searching, a lighter
video stream and the overhead (DroidCam) relay.
"""

from test_autopilot import make_pilot

from teleop_web_lib.config import AutoConfig


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
