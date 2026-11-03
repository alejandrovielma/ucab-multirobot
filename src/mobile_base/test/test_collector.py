"""
Tests: the collector state machine (catch, carry, wait for the zone turn,
deliver, release, count, black-line escape).
"""

from mobile_base.approach import ApproachController, ApproachParams, Cube
from mobile_base.collector import CollectorFsm, CollectorParams, Inputs, Zone
from mobile_base.states import State

CUBE_AT_SCOOP = Cube("yellow", 0.0, 0.16)
FAR_GATE = Zone("gate", 0.0, 1.0, 200)
NEAR_GATE = Zone("gate", 0.0, 0.60, 300)
AREA_AT_SCOOP = Zone("floor", 0.0, 0.17, 470)


def fsm(**kw):
    approach = ApproachController(ApproachParams(target_colors=["yellow"]))
    return CollectorFsm(approach, CollectorParams(**kw))


def run(f, start, end, inp, dt=0.05):
    """
    Step the machine from `start` to `end` with the same inputs; returns
    the last output and the time reached.
    """
    now, out = start, None
    while now < end:
        out = f.step(now, inp)
        now += dt
    return out, now


def caught(f):
    """
    Drive `f` through a catch; returns the time after it.
    """
    f.step(0.0, Inputs(cubes=[CUBE_AT_SCOOP]))
    assert f.state == State.CATCH
    _, now = run(f, 0.05, 1.0, Inputs())
    assert f.state == State.CARRY
    return now


def test_catches_an_aligned_cube_at_the_scoop():
    f = fsm()
    out = f.step(0.0, Inputs(cubes=[CUBE_AT_SCOOP]))
    assert f.state == State.CATCH
    assert out.cmd.vx > 0 and out.scoop_lower
    run(f, 0.05, 1.0, Inputs())
    assert f.state == State.CARRY
    assert f.carrying_color == "yellow"


def test_turns_in_place_when_close_but_not_centred():
    f = fsm()
    out = f.step(0.0, Inputs(cubes=[Cube("yellow", 0.3, 0.16)]))
    assert f.state == State.ARRIVED
    assert out.cmd.vx == 0.0 and out.cmd.wz > 0


def test_carry_turns_to_look_for_the_zone_then_drives_to_it():
    f = fsm()
    now = caught(f)
    out = f.step(now, Inputs())
    assert out.cmd.vx == 0.0 and out.cmd.wz != 0.0
    out = f.step(now + 0.05, Inputs(zone=FAR_GATE))
    assert f.state == State.CARRY
    assert out.cmd.vx > 0 and not out.zone_want


def test_waits_outside_the_zone_until_granted():
    f = fsm()
    now = caught(f)
    out = f.step(now, Inputs(zone=NEAR_GATE))
    assert f.state == State.WAIT_ZONE
    assert out.zone_want and out.cmd.vx == 0.0
    out = f.step(now + 0.05, Inputs(zone=NEAR_GATE, zone_granted=True))
    assert f.state == State.CARRY and out.cmd.vx > 0


def test_full_delivery_counts_once_and_frees_the_zone():
    f = fsm(require_zone_grant=False)
    now = caught(f)
    out = f.step(now, Inputs(zone=AREA_AT_SCOOP))
    assert f.state == State.DELIVER and out.cmd.vx > 0
    delivered = []
    end = now + 10.0
    while now < end and f.state != State.SEARCH:
        now += 0.05
        out = f.step(now, Inputs())
        if out.delivered_color:
            delivered.append(out.delivered_color)
        if f.state == State.RELEASE:
            assert not out.scoop_lower
    assert delivered == ["yellow"]
    assert f.count == 1
    assert f.state == State.SEARCH
    assert not out.zone_want and out.scoop_lower
    assert f.carrying_color is None


def test_black_line_backs_up_and_resumes_carrying():
    f = fsm()
    now = caught(f)
    out = f.step(now, Inputs(zone=FAR_GATE, line_black=True))
    assert f.state == State.LINE_ESCAPE and out.cmd.vx < 0
    out, _ = run(f, now + 0.05, now + 3.0, Inputs())
    assert f.state == State.CARRY


def test_rejects_a_cube_of_another_colour():
    f = fsm(confirm_color=True)
    f.step(0.0, Inputs(cubes=[CUBE_AT_SCOOP]))
    run(f, 0.05, 0.8, Inputs(scoop_color="red"))
    assert f.state == State.RELEASE
    assert f.carrying_color is None
    run(f, 0.8, 6.0, Inputs())
    assert f.state == State.SEARCH and f.count == 0


def test_scoop_colour_overrides_the_camera():
    f = fsm(confirm_color=True)
    f.approach.params.target_colors = ["yellow", "orange"]
    f.step(0.0, Inputs(cubes=[CUBE_AT_SCOOP]))
    run(f, 0.05, 0.8, Inputs(scoop_color="orange"))
    assert f.carrying_color == "orange"


def test_missed_catch_searches_again_when_presence_is_required():
    f = fsm(require_has_cube=True)
    f.step(0.0, Inputs(cubes=[CUBE_AT_SCOOP]))
    run(f, 0.05, 3.0, Inputs(has_cube=False))
    assert f.state == State.SEARCH and f.carrying_color is None


def test_stays_still_without_vision_while_carrying():
    f = fsm()
    now = caught(f)
    out = f.step(now, Inputs(zone=FAR_GATE, vision_ok=False))
    assert out.cmd.vx == 0.0 and out.cmd.wz == 0.0
    assert f.needs_vision


def test_paused_keeps_the_cube_and_resumes_carrying():
    f = fsm()
    now = caught(f)
    f.approach.enabled = False
    f.step(now, Inputs())
    assert f.state == State.PAUSED
    f.approach.enabled = True
    f.step(now + 0.05, Inputs(zone=FAR_GATE))
    assert f.state == State.CARRY
