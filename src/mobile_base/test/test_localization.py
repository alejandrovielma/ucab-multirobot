"""Particle filter and cartographer: the robot's own estimate of where it is, and its memory."""

import math

import pytest

from mobile_base.core.behavior import BehaviorFSM, BehaviorInput, BehaviorParams, Cube, State
from mobile_base.core.cartographer import Cartographer
from mobile_base.core.localization import LANDMARKS, CameraModel, Localizer, TapeMap

TAPE = TapeMap()
CAM = CameraModel()


def sight(pose, name):
    """(cx, cy) a landmark has from a robot pose, through the camera model (inverse of CameraModel)."""
    lx, ly = LANDMARKS[name]
    x, y, th = pose
    b = math.remainder(math.atan2(ly - y, lx - x) - th, 2.0 * math.pi)
    d = math.hypot(lx - x, ly - y)
    return -b / (CAM.hfov / 2.0), 0.66 * math.sqrt(0.2 / d)  # the table's model


def test_tape_map_matches_the_measured_track():
    assert not TAPE.black(0.0, 0.0)
    assert TAPE.black(0.594, 0.0) and TAPE.black(0.0, -0.361)  # the rectangle's tape
    assert not TAPE.black(-0.594, 0.0)  # the square's open side: no tape
    assert not TAPE.black(-0.70, 0.0)  # the blue interior reads white
    assert TAPE.black(-0.797, 0.0)  # the square's far tape
    assert TAPE.distance_ahead(0.0, 0.0, 0.0) == pytest.approx(0.585, abs=0.011)


def test_odometry_follows_the_command_from_a_known_start():
    loc = Localizer(seed=1, start=(0.0, 0.0, 0.0))
    for _ in range(10):
        loc.predict((0.1, 0.0, 0.0), 0.1)
    est = loc.estimate()
    assert 0.05 < est.x < 0.13 and abs(est.y) < 0.03  # 0.1 m times an unknown wheel efficiency in [0.6, 1.2]


def test_landmarks_find_a_lost_robot():
    """Standing still, the three landmarks (almost in a line, range known to 25 %) leave some error along
    the line of sight: the median over seeds is what is checked. Moving and touching the tape refine it."""
    truth = (-0.10, 0.05, math.pi)  # facing the square from the track
    errors, headings = [], []
    for seed in range(2, 10):
        loc = Localizer(seed=seed)  # no idea where it is
        for _ in range(20):
            for name in ("square", "green", "red"):
                loc.update_landmark(name, *sight(truth, name))
            loc.predict((0.0, 0.0, 0.0), 0.1)
        est = loc.estimate()
        errors.append(math.hypot(est.x - truth[0], est.y - truth[1]))
        headings.append(abs(math.remainder(est.theta - truth[2], 2.0 * math.pi)))
    errors.sort()
    assert errors[len(errors) // 2] < 0.08 and errors[-1] < 0.25
    assert max(headings) < 0.3


def test_an_unexplained_black_reading_reseeds_on_the_tape():
    loc = Localizer(seed=3, start=(0.0, 0.0, 0.0))  # sure it is at the center
    loc.update_line(True)  # but the sensor sees the tape
    on = TAPE.black(loc.p[:, 0], loc.p[:, 1])
    assert on.mean() > 0.3  # a good share of the particles now sits on the tape
    assert not loc.estimate().localized


def test_the_wheel_efficiency_is_learned_from_the_tape():
    """The filter is told nothing about the real efficiency (0.75): the tape hits teach it."""
    loc = Localizer(seed=4, start=(0.0, 0.0, 0.0))
    x = 0.0
    for _ in range(200):
        loc.predict((0.1, 0.0, 0.0), 0.1)
        x += 0.1 * 0.1 * 0.75
        loc.update_line(bool(TAPE.black(x, 0.0)))
        if TAPE.black(x, 0.0):
            break
    assert loc.estimate().scale == pytest.approx(0.75, abs=0.12)


def test_cartographer_remembers_a_target_and_forgets_it_when_its_place_is_empty():
    carto = Cartographer(Localizer(seed=5, start=(0.0, 0.0, 0.0)))
    carto.est = carto.loc.estimate()
    assert carto.est.localized
    seen = Cube("red", 0.0, 0.5, 1000.0)  # ahead, about 0.35 m
    carto._look(1.0, carto.est, [seen], ("red",), carrying=False)
    assert len(carto.cubes) == 1 and carto.cubes[0].x == pytest.approx(CAM.distance(0.5), abs=0.02)
    assert carto.goal_for(2.0, ("red",)) == pytest.approx((carto.cubes[0].x, carto.cubes[0].y))
    carto._look(3.0, carto.est, [], ("red",), carrying=False)  # looked at the same place: gone
    assert carto.cubes == []


def test_cartographer_frontier_stays_away_from_the_tape_and_prefers_unseen_cells():
    carto = Cartographer(Localizer(seed=6, start=(0.0, 0.0, 0.0)))
    carto.est = carto.loc.estimate()
    carto._look(1.0, carto.est, [], (), carrying=False)  # it has looked east only
    gx, gy = carto.goal_for(1.5, ())
    ix, iy = TAPE.inner
    assert abs(gx) <= ix - carto.params.tape_margin + 1e-9 and abs(gy) <= iy - carto.params.tape_margin + 1e-9
    assert not (gx > 0.1 and abs(gy) < 0.2)  # not where it just looked


def explorer():
    fsm = BehaviorFSM(BehaviorParams(my_colors=("red",)))
    fsm.set_mission(("red",), ())
    fsm.step(BehaviorInput(now=0.0, cubes=[], line_front=False))
    assert fsm.state is State.EXPLORE
    return fsm


def test_guided_exploration_spins_to_face_the_goal_then_goes_straight():
    fsm = explorer()
    p = fsm.params
    back = fsm.step(BehaviorInput(now=0.1, cubes=[], line_front=False, est_pose=(0.0, 0.0, 0.0),
                                  explore_goal=(-0.3, 0.0), tape_ahead=0.5))
    assert back[0] == 0.0 and abs(back[2]) == p.turn_speed  # goal behind: spin in place
    ahead = fsm.step(BehaviorInput(now=0.2, cubes=[], line_front=False, est_pose=(0.0, 0.0, math.pi),
                                   explore_goal=(-0.3, 0.0), tape_ahead=0.5))
    assert ahead == (p.explore_speed, 0.0, 0.0)


def test_guided_exploration_turns_before_the_estimated_tape():
    fsm = explorer()
    cmd = fsm.step(BehaviorInput(now=0.1, cubes=[], line_front=False, est_pose=(0.45, 0.0, 0.0),
                                 explore_goal=(0.50, 0.0), tape_ahead=0.12))
    assert cmd[0] == 0.0 and cmd[2] != 0.0  # never drives on into the tape


def test_without_an_estimate_exploration_is_the_old_one():
    fsm = explorer()
    assert fsm.step(BehaviorInput(now=0.1, cubes=[], line_front=False)) == (fsm.params.explore_speed, 0.0, 0.0)
