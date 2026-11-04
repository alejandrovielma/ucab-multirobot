"""
Tests for the Auto mode state machine and its settings (run with:
pytest tools). The autopilot is pure logic, so no thread or hardware is
involved.
"""

import math
from pathlib import Path

import pytest

from teleop_web_lib.autopilot import AutoPilot, DriveCommand
from teleop_web_lib.config import (
    HSV_CONFIG_PATH,
    HSV_NODE_NAME,
    VISION_PACKAGE_DIR,
    AutoConfig,
    load_config,
    read_node_params,
)

TOOL_YAML_PATH = Path(__file__).with_name("teleop_web.yaml")

FAST_PHASES = {
    "enabled": True,
    "scoop_push_s": 0.5,
    "floor_deliver_backup_s": 0.5,
    "deliver_turn_s": 0.5,
    "line_backup_s": 0.5,
    "line_turn_s": 0.5,
}


class StubVision:
    """
    Returns whatever cube and zone it is given.
    """

    def __init__(self):
        self.det = None
        self.foreign = None
        self.zone = None

    def latest_detection(self, now, timeout_s):
        return self.det

    def latest_foreign(self, now, timeout_s):
        return self.foreign

    def latest_zone(self, now, timeout_s):
        return self.zone


class Seen:
    """
    A detection or region: colour, distance, bearing, bottom row and the
    turn that faces the area's entry line (0 = already square).
    """

    def __init__(
        self,
        color,
        distance_m,
        bearing_rad=0.0,
        v=300,
        entry_angle_rad=0.0,
        far_distance_m=None,
    ):
        self.color = color
        self.distance_m = distance_m
        self.bearing_rad = bearing_rad
        self.v = v
        self.entry_angle_rad = entry_angle_rad
        self.far_distance_m = far_distance_m


def blind_push_end_s(cfg, edge_m, start_s, blind_s):
    """
    End of a delivery push whose edge was last seen at `edge_m` when the
    push started at `start_s` and went blind at `blind_s` (too short to
    measure the speed: the fallback forward_speed_mps is used).
    """
    speed = cfg.forward_speed_mps
    edge_m -= speed * (blind_s - start_s)
    return blind_s + (edge_m + cfg.deliver_insert_m - cfg.cube_ahead_m) / speed


def make_pilot():
    vision = StubVision()
    pilot = AutoPilot(AutoConfig(**FAST_PHASES), vision)
    pilot.state = "SEARCH"
    return pilot, vision


def test_pick_carry_deliver_and_count():
    pilot, vision = make_pilot()
    vision.det = Seen("red", pilot.cfg.stop_distance_m - 0.01)
    assert pilot.step(0.0).vy == 1.0 and pilot.state == "CATCH"
    vision.det = None
    pilot.step(1.0)
    assert pilot.carrying and pilot.state == "CARRY"
    pilot.step(1.1)
    assert pilot.state.startswith("CARRY")  # no zone: looks for it
    vision.zone = Seen("green", 0.6, bearing_rad=0.5)
    pilot.step(1.2)
    assert pilot.state == "CARRY (align)"
    vision.zone = Seen("floor", 0.11)
    pilot.step(1.3)
    assert pilot.state == "DROP"
    vision.zone = None  # the edge went under the scoop
    pilot.step(1.35)
    assert pilot.state == "DROP (blind)"
    end_s = blind_push_end_s(pilot.cfg, 0.11, 1.3, 1.35)
    assert pilot.step(end_s + 0.1).vy == -1.0  # backs up leaving the cube
    assert pilot.delivered_count == 1 and not pilot.carrying
    pilot.step(end_s + 0.6)
    assert pilot.state == "DROP (turn)"
    pilot.step(end_s + 1.2)
    assert pilot.state.startswith("EXPLORE")


def test_a_line_hit_during_the_delivery_push_counts_the_cube_and_stops():
    pilot, vision = make_pilot()
    pilot.carrying = True
    pilot.state = "CARRY"
    vision.zone = Seen("floor", 0.18)
    pilot.step(0.0)
    assert pilot.state == "DROP"
    # The area's back tape: the cube is already inside.
    assert pilot.step(0.2, line_black=True).speed == 0.0
    assert pilot.state.startswith("LINE")
    assert pilot.delivered_count == 1 and not pilot.carrying


def test_carrying_never_crosses_the_black_line_even_next_to_the_paper():
    pilot, vision = make_pilot()
    pilot.carrying = True
    pilot.state = "CARRY"
    vision.zone = Seen("floor", 0.3)
    pilot.step(0.0, line_black=True)
    assert pilot.state.startswith("LINE") and not pilot.carrying


def test_line_hit_while_carrying_leaves_the_cube_and_searches_again():
    pilot, _ = make_pilot()
    pilot.carrying = True
    pilot.state = "CARRY"
    pilot.step(0.0, line_black=True)
    assert not pilot.carrying and pilot.delivered_count == 0
    for step in range(1, 100):
        pilot.step(step * 0.1)
    assert not pilot.state.startswith("CARRY")


def test_gives_up_carrying_when_the_gate_is_never_found():
    pilot, _ = make_pilot()
    pilot.configure(now=0.0, enabled=True, reset=True)  # search clock at 0
    pilot.carrying = True
    pilot.state = "CARRY"
    timeout_s = pilot.cfg.carry_zone_timeout_s
    pilot.step(timeout_s - 1.0)
    assert pilot.carrying
    assert pilot.step(timeout_s + 0.1).speed == 0.0
    assert not pilot.carrying and pilot.state == "SEARCH"


def test_a_line_hit_while_going_for_a_cube_ignores_cubes_for_a_while():
    pilot, vision = make_pilot()
    vision.det = Seen("red", 0.5)
    pilot.step(0.0)
    assert pilot.state == "APPROACH"
    pilot.step(0.1, line_black=True)  # reached the border line
    ban_s = pilot.cfg.line_cube_ban_s
    for step in range(3, 30):
        pilot.step(step * 0.1)
    assert pilot.state.startswith("EXPLORE")  # the cube is still in view
    pilot.step(0.1 + ban_s + 1.0)
    assert pilot.state in ("APPROACH", "ALIGN")  # ban over: tries again


def test_close_to_the_cube_a_small_bearing_error_does_not_realign():
    pilot, vision = make_pilot()
    tolerance_rad = pilot.cfg.align_tolerance_rad
    # Past the normal tolerance but within the widened one.
    bearing_rad = tolerance_rad * 1.2
    vision.det = Seen("red", 0.8, bearing_rad=bearing_rad)
    pilot.step(0.0)
    assert pilot.state == "ALIGN"  # far: turns to face it
    fresh, fresh_vision = make_pilot()  # not turning yet
    fresh_vision.det = Seen("red", 0.25, bearing_rad=bearing_rad)
    assert fresh.step(0.0).vy == 1.0 and fresh.state == "APPROACH"


def test_a_far_cube_that_flickers_out_keeps_the_heading_without_backing_up():
    pilot, vision = make_pilot()
    vision.det = Seen("red", 0.8, bearing_rad=0.5)  # far, to the left
    pilot.step(0.0)
    vision.det = None
    assert pilot.step(0.1).speed == 0.0  # waits briefly first
    command = pilot.step(0.7)
    assert pilot.state == "EXPLORE (reacquire)"
    assert command.vy == 0.0 and command.wz == 1.0  # turns toward it
    after_s = pilot.cfg.reacquire_s + 0.1
    pilot.step(after_s)
    assert pilot.state != "EXPLORE (reacquire)"  # memory expired: roams


def test_a_lost_cube_straight_ahead_is_approached_not_forgotten():
    pilot, vision = make_pilot()
    vision.det = Seen("red", 0.8, bearing_rad=0.0)
    pilot.step(0.0)
    vision.det = None
    pilot.step(0.1)
    command = pilot.step(0.7)
    assert pilot.state == "EXPLORE (reacquire)" and command.vy == 1.0


def test_backs_up_once_to_uncover_a_cube_that_vanished_from_view():
    pilot, vision = make_pilot()
    vision.det = Seen("red", 0.25)  # close: may be under the scoop
    pilot.step(0.0)
    vision.det = None
    assert pilot.step(0.1).speed == 0.0  # waits briefly first
    command = pilot.step(0.7)
    assert command.vy == -1.0 and pilot.state == "EXPLORE (back up)"
    later_s = 0.7 + pilot.cfg.reveal_backup_s + 0.1
    command = pilot.step(later_s)
    assert command.vy != -1.0 and pilot.state != "EXPLORE (back up)"


def test_never_roams_forward_toward_a_close_gate_without_a_cube():
    pilot, vision = make_pilot()
    near_m = pilot.cfg.gate_avoid_distance_m - 0.1
    vision.zone = Seen("green", near_m, bearing_rad=0.3)  # to the left
    command = pilot.step(0.0)
    assert pilot.state == "EXPLORE (avoid gate)"
    assert command.vy == 0.0 and command.wz == -1.0  # turns right, away


def test_the_area_paper_is_avoided_like_the_gate_when_no_cube_is_in_view():
    pilot, vision = make_pilot()
    pilot.configure(now=0.0, enabled=True, reset=True)  # search clock at 0
    pilot.step(0.0)  # the first move is the back-up that uncovers cubes
    near_m = pilot.cfg.gate_avoid_distance_m - 0.1
    vision.zone = Seen("floor", near_m, bearing_rad=0.3)  # to the left
    command = pilot.step(pilot.cfg.reveal_backup_s + 0.1)
    assert pilot.state == "EXPLORE (avoid gate)"
    assert command.vy == 0.0 and command.wz == -1.0  # turns right, away


def test_the_avoid_turn_keeps_its_side_when_the_bearing_noise_flips_sign():
    pilot, vision = make_pilot()
    near_m = pilot.cfg.gate_avoid_distance_m - 0.1
    vision.zone = Seen("floor", near_m, bearing_rad=0.02)
    first = pilot.step(0.0)
    vision.zone = Seen("floor", near_m, bearing_rad=-0.02)
    second = pilot.step(0.1)
    assert first.wz == second.wz != 0.0


def test_after_avoiding_the_area_the_scan_keeps_turning_away_from_it():
    pilot, vision = make_pilot()
    pilot.configure(now=0.0, enabled=True, reset=True)  # roam clock at 0
    near_m = pilot.cfg.gate_avoid_distance_m - 0.1
    vision.zone = Seen("floor", near_m, bearing_rad=0.3)  # area on the left
    away = pilot.step(0.0).wz
    assert pilot.state == "EXPLORE (avoid gate)" and away < 0
    vision.zone = None  # turned away: the area is out of view
    assert pilot.step(0.1).wz == away


def test_a_turn_to_align_is_finished_before_driving_again():
    pilot, vision = make_pilot()
    tol_rad = pilot.cfg.align_tolerance_rad
    vision.det = Seen("red", 0.8, bearing_rad=tol_rad * 1.5)
    pilot.step(0.0)
    assert pilot.state == "ALIGN"
    vision.det = Seen("red", 0.8, bearing_rad=tol_rad * 0.8)  # inside it
    assert pilot.step(0.1).vy == 0.0 and pilot.state == "ALIGN"
    vision.det = Seen("red", 0.8, bearing_rad=tol_rad * 0.3)
    assert pilot.step(0.2).vy == 1.0 and pilot.state == "APPROACH"


def test_delivery_squares_up_to_the_pink_line_slides_in_front_then_pushes():
    pilot, vision = make_pilot()
    pilot.carrying = True
    pilot.state = "CARRY"
    cfg = pilot.cfg
    # Arrived at an angle (the pink line turned to the left): turn left.
    vision.zone = Seen("floor", 0.18, bearing_rad=0.3, entry_angle_rad=0.4)
    command = pilot.step(0.0)
    assert pilot.state == "CARRY (square)"
    assert command.wz > 0 and command.vx == command.vy == 0.0
    # Square, but the paper's centre is to the left: slide left, no turn.
    vision.zone = Seen("floor", 0.18, bearing_rad=0.3)
    command = pilot.step(0.05)
    assert pilot.state == "CARRY (center)"
    assert command.vx < 0 and command.vy == command.wz == 0.0
    # Square and in front: push straight in, far enough to bring the cube
    # there.
    vision.zone = Seen("floor", 0.18)
    assert pilot.step(0.1).vy == 1.0 and pilot.state == "DROP"


def test_the_delivery_push_follows_the_edge_then_ends_blind_at_its_speed():
    pilot, vision = make_pilot()
    pilot.carrying = True
    pilot.state = "CARRY"
    cfg = pilot.cfg
    vision.zone = Seen("floor", 0.18)
    assert pilot.step(0.1).vy == 1.0 and pilot.state == "DROP"
    # The edge comes closer at 0.1 m/s: keep pushing while it is in view,
    # however long a fixed-time push would have lasted.
    vision.zone = Seen("floor", 0.14)
    assert pilot.step(0.5).vy == 1.0 and pilot.state == "DROP"
    # Close enough that the scoop hides it: the rest is blind, timed with
    # the measured 0.1 m/s.
    vision.zone = Seen("floor", cfg.deliver_blind_edge_m - 0.01)
    pilot.step(0.8)
    assert pilot.state == "DROP (blind)"
    speed = (0.18 - (cfg.deliver_blind_edge_m - 0.01)) / 0.7
    blind_m = cfg.deliver_blind_edge_m - 0.01 + cfg.deliver_insert_m - cfg.cube_ahead_m
    end_s = 0.8 + blind_m / speed
    assert pilot.step(end_s - 0.02).vy == 1.0
    assert pilot.step(end_s + 0.02).vy == -1.0


def test_without_the_pink_line_in_view_it_waits_instead_of_pushing():
    pilot, vision = make_pilot()
    pilot.carrying = True
    pilot.state = "CARRY"
    vision.zone = Seen("floor", 0.18, entry_angle_rad=None)
    assert pilot.step(0.0).speed == 0.0
    assert pilot.state == "CARRY (no entry line)" and pilot.carrying


def test_too_close_to_see_the_pink_line_it_centres_and_pushes_anyway():
    pilot, vision = make_pilot()
    pilot.carrying = True
    pilot.state = "CARRY"
    close_m = pilot.cfg.entry_hidden_distance_m - 0.05
    vision.zone = Seen("floor", close_m, bearing_rad=0.3, entry_angle_rad=None)
    command = pilot.step(0.0)
    assert pilot.state == "CARRY (center)" and command.vx < 0  # slides left
    vision.zone = Seen("floor", close_m, entry_angle_rad=None)
    assert pilot.step(0.1).vy == 1.0 and pilot.state == "DROP"


def test_squaring_up_finishes_the_turn_before_sliding():
    pilot, vision = make_pilot()
    pilot.carrying = True
    pilot.state = "CARRY"
    tol_rad = pilot.cfg.entry_square_tolerance_rad
    vision.zone = Seen("floor", 0.18, entry_angle_rad=-tol_rad * 1.5)
    assert pilot.step(0.0).wz < 0 and pilot.state == "CARRY (square)"
    # Inside the tolerance but not past the exit fraction: keep turning.
    vision.zone = Seen("floor", 0.18, entry_angle_rad=-tol_rad * 0.8)
    assert pilot.step(0.1).wz < 0 and pilot.state == "CARRY (square)"
    vision.zone = Seen("floor", 0.18, entry_angle_rad=-tol_rad * 0.3)
    assert pilot.step(0.2).vy == 1.0 and pilot.state == "DROP"


def test_arriving_diagonally_it_slides_toward_the_paper_before_turning():
    pilot, vision = make_pilot()
    pilot.carrying = True
    pilot.state = "CARRY"
    side_rad = pilot.cfg.square_max_bearing_rad

    def crooked_paper(bearing_rad):
        return Seen("floor", 0.18, bearing_rad=bearing_rad, entry_angle_rad=0.5)

    # Paper off to the right and the strip crooked: turning would lose it.
    vision.zone = crooked_paper(-side_rad * 1.5)
    command = pilot.step(0.0)
    assert pilot.state == "CARRY (center)"
    assert command.vx > 0 and command.wz == 0
    # Past the tolerance but not the exit fraction: keep sliding.
    vision.zone = crooked_paper(-side_rad * 0.8)
    assert pilot.step(0.1).vx > 0 and pilot.state == "CARRY (center)"
    # Centred enough: now it turns to face the strip.
    vision.zone = crooked_paper(-side_rad * 0.3)
    assert pilot.step(0.2).wz > 0 and pilot.state == "CARRY (square)"


def test_without_the_pink_line_and_the_paper_to_a_side_it_slides_toward_it():
    pilot, vision = make_pilot()
    pilot.carrying = True
    pilot.state = "CARRY"
    bearing_rad = pilot.cfg.square_max_bearing_rad * 1.5
    vision.zone = Seen(
        "floor", 0.18, bearing_rad=bearing_rad, entry_angle_rad=None
    )
    assert pilot.step(0.0).vx < 0 and pilot.state == "CARRY (center)"


def test_paper_lost_while_squaring_it_slides_back_instead_of_the_landmarks():
    pilot, vision = make_pilot()
    pilot.carrying = True
    pilot.state = "CARRY"
    cfg = pilot.cfg
    # Turning to square up: the paper last seen on the right.
    vision.zone = Seen("floor", 0.18, bearing_rad=-0.2, entry_angle_rad=0.5)
    pilot.step(0.0)
    assert pilot.state == "CARRY (square)"
    vision.zone = Seen("gate", 0.25, bearing_rad=0.4)  # only the green lego
    assert pilot.step(0.5).speed == 0.0  # the hold
    command = pilot.step(cfg.carry_paper_hold_s + 0.1)
    assert pilot.state == "CARRY (center)" and command.vx > 0  # slides right
    # Not back after the slide either: head for the area by the landmarks.
    pilot.step(cfg.carry_paper_hold_s + cfg.carry_paper_slide_s + 0.1)
    assert pilot.state == "CARRY (align)"


def test_a_cube_still_in_front_after_backing_up_is_not_counted():
    pilot, vision = make_pilot()
    pilot.carrying = True
    pilot.state = "CARRY"
    vision.zone = Seen("floor", 0.11)
    pilot.step(0.0)
    assert pilot.state == "DROP"
    vision.zone = None
    pilot.step(0.05)
    end_s = blind_push_end_s(pilot.cfg, 0.11, 0.0, 0.05)
    pilot.step(end_s + 0.1)  # backing up: counted for now
    assert pilot.delivered_count == 1
    vision.det = Seen("yellow", pilot.cfg.drop_verify_distance_m - 0.05)
    pilot.step(end_s + 0.6)  # backed up, and the cube is right there
    assert pilot.delivered_count == 0 and not pilot.carrying
    assert pilot.state == "SEARCH" or pilot.state.startswith("APPROACH")


def test_an_edge_never_lost_still_ends_the_push_after_the_limit():
    config = AutoConfig(**{**FAST_PHASES, "floor_deliver_backup_s": 0.3})
    pilot = AutoPilot(config, StubVision())
    pilot.vision.zone = Seen("floor", 0.18)  # the edge reads 18 cm forever
    pilot.carrying = True
    pilot.state = "CARRY"
    assert pilot.step(0.0).vy == 1.0 and pilot.state == "DROP"
    limit_s = config.deliver_push_max_s
    assert pilot.step(limit_s - 0.1).vy == 1.0 and pilot.state == "DROP"
    pilot.step(limit_s + 0.01)
    assert pilot.state == "DROP (blind)"  # then the blind rest, and back up
    assert pilot.step(limit_s + 5.0).vy != 1.0


def test_without_cubes_it_spins_then_moves_to_another_spot_and_keeps_going():
    pilot, vision = make_pilot()
    pilot.configure(now=0.0, enabled=True, reset=True)
    cfg = pilot.cfg
    vision.zone = Seen("gate", 1.0)  # the area far away: never goes there
    turn_s = cfg.roam_period_s - cfg.roam_forward_s
    command = pilot.step(0.1)
    assert pilot.state == "EXPLORE (turn)" and command.wz != 0.0
    command = pilot.step(turn_s + 0.1)
    assert pilot.state == "EXPLORE (forward)" and command.vy == 1.0
    # A second period: still searching, not stopped.
    command = pilot.step(cfg.roam_period_s + 0.1)
    assert pilot.state == "EXPLORE (turn)" and command.wz != 0.0


def test_the_robot_goes_straight_for_a_cube_the_moment_it_sees_one():
    pilot, vision = make_pilot()
    pilot.configure(now=0.0, enabled=True, reset=True)
    assert pilot.step(0.0).wz != 0.0  # scanning
    vision.det = Seen("red", 0.6, bearing_rad=0.0)
    command = pilot.step(0.1)
    assert command.vy == 1.0 and pilot.state == "APPROACH"


def test_auto_starting_does_not_back_up_blindly():
    pilot, _ = make_pilot()
    pilot.configure(now=0.0, enabled=True, reset=True)
    for step in range(0, 20):
        command = pilot.step(step * 0.05)
        assert command.vy >= 0.0  # roams or turns, never reverses


def test_black_line_moves_away_opposite_to_the_last_motion():
    cases = [
        (
            DriveCommand(0.0, 1.0, 0.0, 0.4),
            0.0,
            -1.0,
        ),  # forward (W) -> back (S)
        (
            DriveCommand(0.0, -1.0, 0.0, 0.4),
            0.0,
            1.0,
        ),  # reverse (S) -> forward (W)
        (DriveCommand(-1.0, 0.0, 0.0, 0.4), 1.0, 0.0),  # left (A) -> right (D)
        (DriveCommand(1.0, 0.0, 0.0, 0.4), -1.0, 0.0),  # right (D) -> left (A)
    ]
    for moved, expect_vx, expect_vy in cases:
        pilot, _ = make_pilot()
        pilot.note_motion(0.0, moved)
        pilot.step(0.1, line_black=True)  # brake
        command = pilot.step(0.4, line_black=True)
        assert (command.vx, command.vy, command.wz) == (
            expect_vx,
            expect_vy,
            0.0,
        )


def test_black_line_backs_up_when_there_is_no_recent_motion():
    pilot, _ = make_pilot()
    pilot.note_motion(0.0, DriveCommand(0.0, -1.0, 0.0, 0.4))
    late = pilot.cfg.line_motion_memory_s + 1.0
    pilot.step(late, line_black=True)
    assert pilot.step(late + 0.3, line_black=True).vy == -1.0


def test_recovery_does_not_overwrite_the_direction_it_escapes_from():
    pilot, _ = make_pilot()
    pilot.note_motion(0.0, DriveCommand(0.0, -1.0, 0.0, 0.4))
    pilot.step(0.1, line_black=True)
    pilot.note_motion(0.2, DriveCommand(0.0, 1.0, 0.0, 0.4))  # escape move
    assert pilot.step(0.4, line_black=True).vy == 1.0


def test_black_line_stops_backs_up_then_searches_without_the_lost_cube():
    pilot, _ = make_pilot()
    pilot.carrying = True
    pilot.state = "CARRY"
    command = pilot.step(0.0, line_black=True)
    assert command.speed == 0.0 and pilot.state == "LINE (stop)"
    command = pilot.step(0.3, line_black=True)
    assert command.vy == -1.0 and pilot.state == "LINE (back up)"
    pilot.step(1.0)  # white again: backs up the margin, then turns
    pilot.step(1.2)
    assert pilot.state == "LINE (turn)"
    pilot.step(2.0)
    # The cube stayed on the border: nothing left to carry.
    assert not pilot.carrying and pilot.state.startswith("EXPLORE")


def test_black_line_keeps_backing_up_while_still_on_black():
    pilot, _ = make_pilot()
    pilot.step(0.0, line_black=True)
    for t in (0.3, 0.6, 0.9):  # still black: never reaches the turn yet
        command = pilot.step(t, line_black=True)
        assert command.vy == -1.0 and pilot.state == "LINE (back up)"
    # Gives up backing after line_backup_max_s even if the sensor is stuck.
    states = set()
    for step in range(10, 30):
        pilot.step(step * 0.1, line_black=True)
        states.add(pilot.state)
    assert "LINE (turn)" in states


def test_reversing_onto_black_line_escapes_forward_without_turning_back():
    pilot, _ = make_pilot()
    pilot.note_motion(0.0, DriveCommand(0.0, -1.0, 0.0, 0.4))
    assert pilot.step(0.1, line_black=True).speed == 0.0
    assert pilot.step(0.4, line_black=True).vy == 1.0  # drives forward
    pilot.step(1.0, line_black=False)  # resumes without spinning 180 deg back
    assert not pilot.state.startswith("LINE")


def test_approach_aligns_before_driving():
    pilot, vision = make_pilot()
    vision.det = Seen("red", 0.8, bearing_rad=0.5)
    command = pilot.step(0.0)
    assert pilot.state == "ALIGN" and command.vy == 0.0 and command.wz != 0.0
    vision.det = Seen("red", 0.8, bearing_rad=0.0)
    command = pilot.step(0.1)
    assert pilot.state == "APPROACH" and command.vy == 1.0


def test_gives_up_when_no_cube_is_ever_seen():
    pilot, _ = make_pilot()
    pilot.configure(now=0.0, enabled=True, reset=True)  # search clock at 0
    pilot.step(0.0)  # starts roaming
    assert pilot.step(pilot.cfg.no_cubes_stop_s + 1.0).speed == 0.0
    assert pilot.state == "NO CUBES (stopped)"
    # Stays stopped until Auto is restarted.
    assert pilot.step(pilot.cfg.no_cubes_stop_s + 2.0).speed == 0.0


def test_reset_clears_counters_and_restarts_search():
    pilot, _ = make_pilot()
    pilot.carrying = True
    pilot.delivered_count = 3
    pilot.collected_count = 4
    pilot.configure(now=5.0, enabled=True, reset=True)
    assert pilot.state == "SEARCH"
    assert (pilot.carrying, pilot.delivered_count, pilot.collected_count) == (
        False,
        0,
        0,
    )


def test_disabling_pauses_and_enabling_resumes_carrying():
    pilot, _ = make_pilot()
    pilot.carrying = True
    pilot.configure(now=0.0, enabled=False)
    assert pilot.state == "PAUSED"
    pilot.configure(now=1.0, enabled=True)
    assert pilot.state == "CARRY"


def test_auto_config_defaults_and_overrides():
    assert AutoConfig.from_dict(None) == AutoConfig()
    config = AutoConfig.from_dict({"forward_speed": 1, "enabled": True})
    assert config.forward_speed == 1.0 and isinstance(
        config.forward_speed, float
    )
    assert config.enabled is True


def test_auto_config_rejects_unknown_keys():
    with pytest.raises(ValueError, match="forwrad_speed"):
        AutoConfig.from_dict({"forwrad_speed": 0.4})


def test_reads_the_shared_hsv_file_with_or_without_namespace_wildcard(tmp_path):
    body = "{node}:\n  ros__parameters:\n    a: 1\n"
    plain = tmp_path / "plain.yaml"
    plain.write_text(body.format(node="cube_detector"))
    wild = tmp_path / "wild.yaml"
    wild.write_text(body.format(node="/**/cube_detector"))
    assert read_node_params(plain, "cube_detector") == {"a": 1}
    assert read_node_params(wild, "cube_detector") == {"a": 1}
    with pytest.raises(KeyError):
        read_node_params(plain, "web_viewer")


def test_the_real_vision_config_has_what_the_tool_needs():
    params = read_node_params(HSV_CONFIG_PATH, HSV_NODE_NAME)
    assert {"hsv", "camera", "filter"} <= set(params)
    assert {"hfov_rad", "height_m", "tilt_rad"} <= set(params["camera"])


def test_every_area_colour_the_tool_looks_for_has_a_range():
    camera = load_config(TOOL_YAML_PATH)["camera"]
    shared = read_node_params(HSV_CONFIG_PATH, HSV_NODE_NAME)["hsv"]
    known = set(shared) | set(camera.get("area_hsv", {}))
    assert set(camera["zone"]["floor_colors"]) <= known
    for ranges in camera["area_hsv"].values():
        assert len(ranges) % 6 == 0  # [h, s, v] min and max per range


def test_only_the_area_paper_starts_the_delivery_never_the_landmarks():
    pilot, vision = make_pilot()
    pilot.carrying = True
    pilot.state = "CARRY"
    # Right at the landmarks, even at the image bottom: creep on, no push.
    vision.zone = Seen("gate", 0.05, v=479)
    command = pilot.step(0.0)
    assert pilot.state == "CARRY" and 0.0 < command.speed
    assert command.speed < pilot.cfg.forward_speed
    # The paper's centre farther than the commit distance: keep driving.
    commit_m = pilot.cfg.deliver_commit_distance_m
    vision.zone = Seen("floor", commit_m + 0.2)
    pilot.step(0.1)
    assert pilot.state == "CARRY"
    vision.zone = Seen("floor", commit_m)
    pilot.step(0.2)
    assert pilot.state == "DROP"


def test_a_cube_in_front_of_the_gate_is_not_taken_for_delivered(monkeypatch):
    pytest.importorskip("cv2")  # the vision package needs OpenCV
    monkeypatch.syspath_prepend(str(VISION_PACKAGE_DIR))
    from vision.color_detection import Detection, Region
    from vision.zone import ZoneParams, without_cubes_in_zone

    zone_cfg = load_config(TOOL_YAML_PATH)["camera"]["zone"]
    margin_px = ZoneParams(**zone_cfg).ignore_margin_px
    gate = Region("gate", 316, 224, 1000, (238, 182, 200, 42), 0.0, 0.97)
    in_front = Detection("yellow", 309, 266, 25, 25, (296, 241, 25, 25))
    behind = Detection("yellow", 316, 220, 12, 12, (310, 208, 12, 12))
    kept = without_cubes_in_zone([in_front, behind], gate, margin_px)
    assert kept == [in_front]  # the one behind the legos is inside the area


def test_a_cube_still_in_front_after_the_catch_push_is_not_collected():
    pilot, vision = make_pilot()
    cfg = pilot.cfg
    vision.det = Seen("yellow", cfg.stop_distance_m - 0.01)
    pilot.step(0.0)
    assert pilot.state == "CATCH"
    # The push fell short: the cube is still right in front.
    vision.det = Seen("yellow", 0.08)
    assert pilot.step(cfg.scoop_push_s + 0.05).speed == 0.0
    assert pilot.state == "CATCH (check)"
    pilot.step(cfg.scoop_push_s + cfg.catch_check_s + 0.05)
    assert not pilot.carrying and pilot.collected_count == 0
    assert not pilot.state.startswith("CARRY")


def test_another_cube_off_to_a_side_does_not_undo_a_delivery():
    pilot, vision = make_pilot()
    pilot.carrying = True
    pilot.state = "CARRY"
    cfg = pilot.cfg
    vision.zone = Seen("floor", 0.11)
    pilot.step(0.0)
    assert pilot.state == "DROP"
    vision.zone = None
    pilot.step(0.05)
    end_s = blind_push_end_s(cfg, 0.11, 0.0, 0.05)
    pilot.step(end_s + 0.1)  # backing up
    # Another cube on the table, close but to a side: not the delivered one.
    vision.det = Seen(
        "yellow",
        cfg.drop_verify_distance_m - 0.05,
        bearing_rad=cfg.drop_verify_bearing_rad + 0.1,
    )
    pilot.step(end_s + cfg.floor_deliver_backup_s + 0.05)
    assert pilot.state == "DROP (turn)" and pilot.delivered_count == 1
    pilot.step(end_s + cfg.floor_deliver_backup_s + 0.2)  # still turning
    assert pilot.delivered_count == 1


def test_the_push_stops_when_the_far_edge_is_close_never_past_the_area():
    pilot, vision = make_pilot()
    pilot.carrying = True
    pilot.state = "CARRY"
    stop_m = pilot.cfg.deliver_far_stop_m
    vision.zone = Seen("floor", 0.18, far_distance_m=0.35)
    assert pilot.step(0.0).vy == 1.0 and pilot.state == "DROP"
    # The near edge goes under the scoop, the far one is still in view:
    # keep pushing, no blind guess.
    vision.zone = Seen("floor", 0.09, far_distance_m=0.25)
    assert pilot.step(0.4).vy == 1.0 and pilot.state == "DROP"
    vision.zone = Seen("floor", 0.05, far_distance_m=stop_m - 0.005)
    assert pilot.step(0.8).vy != 1.0  # there: backs up, cube delivered
    assert pilot.delivered_count == 1


def test_a_blind_rest_never_goes_past_the_far_edge_last_seen():
    pilot, vision = make_pilot()
    pilot.carrying = True
    pilot.state = "CARRY"
    cfg = pilot.cfg
    vision.zone = Seen("floor", 0.20, far_distance_m=cfg.deliver_far_stop_m + 0.02)
    pilot.step(0.0)
    vision.zone = None  # area lost: blind rest
    pilot.step(0.05)
    assert pilot.state == "DROP (blind)"
    # 2 cm left to the stop point, however far the near edge said.
    assert pilot._phase_until - 0.05 <= 0.02 / cfg.forward_speed_mps + 1e-6


def test_squaring_up_stays_committed_when_the_paper_jitters_or_drops_out():
    pilot, vision = make_pilot()
    pilot.carrying = True
    pilot.state = "CARRY"
    cfg = pilot.cfg
    commit_m = cfg.deliver_commit_distance_m
    vision.zone = Seen("floor", commit_m - 0.02, entry_angle_rad=0.4)
    pilot.step(0.0)
    assert pilot.state == "CARRY (square)"
    # The paper reads a bit farther (cubes on it): keep squaring up.
    vision.zone = Seen("floor", commit_m + 0.02, entry_angle_rad=0.4)
    pilot.step(0.25)
    assert pilot.state == "CARRY (square)"
    # Only the landmarks for a moment: wait still, do not turn to them.
    vision.zone = Seen("gate", 0.30, bearing_rad=0.5)
    assert pilot.step(0.5).speed == 0.0
    assert pilot.state == "CARRY (square)"
    # Still no paper after the hold: back to heading for the area.
    pilot.step(0.25 + cfg.carry_paper_hold_s + 0.1)
    assert pilot.state.startswith("CARRY") and pilot.state != "CARRY (square)"


def _catch_then_check(assume_m, cube_m):
    vision = StubVision()
    cfg = AutoConfig(**FAST_PHASES, assume_in_scoop_distance_m=assume_m)
    pilot = AutoPilot(cfg, vision)
    pilot.state = "SEARCH"
    vision.det = Seen("blue", cfg.stop_distance_m - 0.01)
    assert pilot.step(0.0).vy == 1.0 and pilot.state == "CATCH"
    vision.det = Seen("blue", cube_m)  # still seen after the push
    pilot.step(cfg.scoop_push_s + 0.1)  # checking
    pilot.step(cfg.scoop_push_s + cfg.catch_check_s + 0.2)
    return pilot


def test_a_cube_still_seen_very_close_after_the_push_is_in_the_scoop():
    pilot = _catch_then_check(0.12, 0.09)
    assert pilot.carrying and pilot.state == "CARRY"
    assert pilot.collected_count == 1


def test_a_cube_seen_farther_after_the_push_is_chased_again():
    pilot = _catch_then_check(0.12, 0.17)
    assert not pilot.carrying and pilot.state == "SEARCH"


def test_a_far_edge_too_close_behind_the_near_one_does_not_stop_the_push():
    vision = StubVision()
    pilot = AutoPilot(AutoConfig(**FAST_PHASES, deliver_far_min_span_m=0.08), vision)
    pilot.carrying = True
    pilot.state = "CARRY"
    # Near 7.8 cm, far 12.1 cm: the real read that made the push 0 cm.
    vision.zone = Seen("floor", 0.078, far_distance_m=0.121)
    pilot.step(0.0)
    assert pilot.state == "DROP"
    assert pilot.step(0.05).vy == 1.0  # keeps pushing on the near edge
    assert pilot._drop_far_last is None


def test_a_cube_to_avoid_in_the_way_is_slid_around_while_carrying():
    pilot, vision = make_pilot()
    pilot.carrying = True
    pilot.state = "CARRY"
    vision.zone = Seen("green", 0.60)  # straight ahead: would drive on
    vision.foreign = Seen("yellow", 0.25, bearing_rad=0.1)  # a bit to the left
    command = pilot.step(0.0)
    assert command.vx > 0 and command.vy == 0 and command.speed > 0  # right
    assert pilot.carrying and pilot.state.startswith("CARRY")
    assert "avoiding yellow" in pilot.target_info


def test_a_cube_to_avoid_off_the_path_or_far_is_ignored():
    pilot, vision = make_pilot()
    pilot.carrying = True
    pilot.state = "CARRY"
    vision.zone = Seen("green", 0.60)
    vision.foreign = Seen("yellow", 0.25, bearing_rad=0.8)  # far to the side
    assert pilot.step(0.0).vy == 1.0
    vision.foreign = Seen("yellow", 0.60, bearing_rad=0.0)  # still far ahead
    assert pilot.step(0.1).vy == 1.0


def test_the_slide_around_a_cube_to_avoid_keeps_its_side_until_clear():
    pilot, vision = make_pilot()
    cfg = pilot.cfg
    pilot.carrying = True
    pilot.state = "CARRY"
    vision.zone = Seen("green", 0.60)
    vision.foreign = Seen("yellow", 0.25, bearing_rad=-0.1)  # to the right
    assert pilot.step(0.0).vx < 0  # slides left
    # Just past the clearance but inside the exit margin: same side.
    lateral_m = cfg.foreign_clearance_m + cfg.foreign_exit_margin_m / 2
    vision.foreign = Seen("yellow", 0.25, -math.asin(lateral_m / 0.25))
    assert pilot.step(0.1).vx < 0
    vision.foreign = Seen("yellow", 0.25, bearing_rad=-0.8)  # clear: drive on
    assert pilot.step(0.2).vy == 1.0


def test_a_cube_to_avoid_is_also_avoided_when_searching():
    pilot, vision = make_pilot()
    vision.det = Seen("blue", 0.40)  # ahead: approach drives straight
    vision.foreign = Seen("yellow", 0.22, bearing_rad=-0.05)
    command = pilot.step(0.0)
    assert command.vx < 0 and command.vy == 0


def test_the_blind_catch_push_is_not_interrupted_by_a_cube_to_avoid():
    pilot, vision = make_pilot()
    vision.det = Seen("blue", pilot.cfg.stop_distance_m - 0.01)
    assert pilot.step(0.0).vy == 1.0 and pilot.state == "CATCH"
    vision.foreign = Seen("yellow", 0.15, bearing_rad=0.05)
    assert pilot.step(0.1).vy == 1.0 and pilot.state == "CATCH"


def test_catch_does_not_start_with_a_cube_to_avoid_in_the_way():
    pilot, vision = make_pilot()
    vision.det = Seen("blue", pilot.cfg.stop_distance_m - 0.01)
    vision.foreign = Seen("yellow", 0.11, bearing_rad=0.05)
    command = pilot.step(0.0)
    assert command.vx > 0 and command.vy == 0 and pilot.state == "APPROACH"


def test_drop_does_not_start_with_a_cube_to_avoid_in_front_of_the_zone():
    pilot, vision = make_pilot()
    pilot.carrying = True
    pilot.state = "CARRY"
    vision.zone = Seen("floor", 0.18, bearing_rad=0.0, entry_angle_rad=0.0)
    vision.foreign = Seen("yellow", 0.15, bearing_rad=0.05)
    command = pilot.step(0.0)
    assert command.vx > 0 and command.vy == 0
    assert pilot.state.startswith("CARRY")


def test_the_area_flickering_out_while_carrying_only_stops_the_robot():
    vision = StubVision()
    pilot = AutoPilot(AutoConfig(**FAST_PHASES, carry_zone_hold_s=0.3), vision)
    pilot.carrying = True
    pilot.state = "CARRY"
    vision.zone = Seen("green", 0.60)
    assert pilot.step(0.0).vy == 1.0
    vision.zone = None
    assert pilot.step(0.1) == DriveCommand()  # just lost: wait for it
    assert pilot.step(0.4).wz != 0  # gone for good: look for it

