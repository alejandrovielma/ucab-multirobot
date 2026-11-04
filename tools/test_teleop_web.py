"""
Tests for the kinematics, timeout brake and control thread of the browser
tool (run with: pytest tools).
"""

import time

import pytest

from teleop_web_lib.autopilot import STATE_MANUAL, STATE_PAUSED
from teleop_web_lib.drive import Drive
from teleop_web_lib.kinematics import mecanum_wheel_ratios

STOPPED = {"fl": 0, "fr": 0, "rl": 0, "rr": 0}


def test_forward_all_wheels_same():
    assert mecanum_wheel_ratios(1, 0, 0) == {"fl": 1, "fr": 1, "rl": 1, "rr": 1}


def test_strafe_left():
    r = mecanum_wheel_ratios(0, 1, 0)
    assert r == {"fl": -1, "fr": 1, "rl": 1, "rr": -1}


def test_turn_ccw():
    r = mecanum_wheel_ratios(0, 0, 1)
    assert r == {"fl": -1, "fr": 1, "rl": -1, "rr": 1}


def test_combined_is_normalized():
    r = mecanum_wheel_ratios(1, 1, 1)
    assert max(abs(v) for v in r.values()) == 1


class RecordingMotors:
    """
    Motors that remember the last ratios they were given.
    """

    def __init__(self):
        self.last = None

    def set(self, ratios):
        self.last = ratios

    def close(self):
        pass


def test_brakes_without_commands():
    motors = RecordingMotors()
    drive = Drive(motors, max_duty_ratio=0.5, cmd_timeout_s=0.1)
    drive.command(1, 0, 0, 1)
    time.sleep(0.05)
    assert motors.last["fl"] == 0.5
    time.sleep(0.2)
    assert motors.last == STOPPED
    drive.close()


def test_manual_command_is_reported_as_manual_then_paused():
    drive = Drive(RecordingMotors(), max_duty_ratio=0.5, cmd_timeout_s=0.1)
    drive.command(1, 0, 0, 1)
    time.sleep(0.05)
    assert drive.status()["state"] == STATE_MANUAL
    time.sleep(0.2)
    assert drive.status()["state"] == STATE_PAUSED
    drive.close()


def test_status_has_the_keys_the_page_reads():
    drive = Drive(RecordingMotors(), max_duty_ratio=0.5, cmd_timeout_s=0.1)
    assert set(drive.status()) == {
        "auto",
        "state",
        "target",
        "collected",
        "delivered",
        "line",
        "line_black",
        "line_raw",
        "line_counts",
        "line_mv",
    }
    drive.close()


def test_set_auto_switches_the_mode():
    drive = Drive(RecordingMotors(), max_duty_ratio=0.5, cmd_timeout_s=0.1)
    assert drive.status()["auto"] is False
    drive.set_auto(True)
    assert drive.status()["auto"] is True
    drive.set_auto(False)
    assert drive.status()["auto"] is False
    drive.close()


def test_adc_word_arrives_low_byte_first_and_is_swapped():
    from teleop_web_lib.line_sensor import word_to_counts

    # 0x0A05 = 2565 counts: SMBus returns the low byte (0x05) first.
    assert word_to_counts(0x050A) == 0x0A05
    assert word_to_counts(0x0000) == 0
    assert word_to_counts(0xFF0F) == 0x0FFF  # 4095, full scale


def test_adc_counts_convert_to_millivolts_of_a_3v3_reference():
    from teleop_web_lib.line_sensor import counts_to_millivolts

    assert counts_to_millivolts(0) == 0
    assert counts_to_millivolts(2048) == 1650  # half scale
    assert counts_to_millivolts(4095) == 3299


def test_a_dark_analog_read_counts_as_black_even_if_do_says_white():
    from teleop_web_lib.line_sensor import reads_black

    assert reads_black(1, 1, None, None)  # DO alone still works
    assert not reads_black(0, 1, 1610, None)  # no analog threshold set
    assert reads_black(0, 1, 1610, 900)  # tape under the DO threshold
    assert not reads_black(0, 1, 180, 900)  # white table
    assert not reads_black(0, 1, None, 900)  # AO read failed


def test_line_sensor_ignores_a_single_black_read():
    from teleop_web_lib.line_sensor import BlackConfirm

    confirm = BlackConfirm(confirm_reads=2)
    reads = [True, False, True, True, True, False]
    assert [confirm.update(r) for r in reads] == [
        False,
        False,
        False,
        True,
        True,
        False,
    ]


class FakeLine:
    """
    A line sensor that reads black for the first `black_reads` reads.
    """

    def __init__(self, black_reads, counts):
        self._left = black_reads
        self._counts = counts

    def raw_value(self):
        return int(self._left > 0)

    def is_black(self):
        black = self._left > 0
        self._left -= 1
        return black

    def analog_counts(self):
        return self._counts


def test_each_black_edge_of_the_line_sensor_is_logged_with_its_analog_value(
    caplog,
):
    caplog.set_level("INFO", logger="teleop_web")
    drive = Drive(
        RecordingMotors(),
        max_duty_ratio=0.5,
        cmd_timeout_s=0.1,
        line_sensor=FakeLine(black_reads=3, counts=3400),
    )
    time.sleep(0.3)
    drive.close()
    edges = [
        r.getMessage()
        for r in caplog.records
        if "line sensor" in r.getMessage()
    ]
    assert len(edges) == 1 and "ao=3400" in edges[0]


def test_fusion_hat_motors_use_the_library(monkeypatch):
    import sys
    import types

    from teleop_web_lib.motors import FusionHatMotors

    class FakeMotor:
        made = {}

        def __init__(self, port):
            self.reverse, self.powers = None, []
            FakeMotor.made[port] = self

        def set_is_reverse(self, flag):
            self.reverse = flag

        def power(self, value):
            self.powers.append(value)

    module = types.ModuleType("fusion_hat.motor")
    module.Motor = FakeMotor
    monkeypatch.setitem(sys.modules, "fusion_hat", types.ModuleType("fusion_hat"))
    monkeypatch.setitem(sys.modules, "fusion_hat.motor", module)
    cfg = {
        "fusion_channels": {"fl": [4, 5], "fr": [11, 10], "rl": [6, 7], "rr": [9, 8]},
        "fl_inverted": False, "fr_inverted": True, "rl_inverted": False, "rr_inverted": True,
    }
    motors = FusionHatMotors(cfg)
    motors.set({"fl": 0.5, "fr": 0.5, "rl": 0.0, "rr": -0.25})
    assert FakeMotor.made["M3"].powers[-1] == 50
    assert FakeMotor.made["M0"].reverse is True
    assert FakeMotor.made["M1"].powers[-1] == -25


def test_overhead_pose_finds_the_robot_and_the_area():
    import numpy as np

    from teleop_web_lib.overhead_pose import locate

    frame = np.full((360, 640, 3), 230, np.uint8)  # white track
    frame[210:240, 118:158] = (150, 140, 20)  # teal paper (BGR), left of the arena
    frame[170:215, 280:330] = (25, 25, 25)  # dark chassis inside the arena
    pose = locate(frame)
    assert abs(pose.robot_px[0] - 304.5) < 3 and abs(pose.robot_px[1] - 192) < 3
    assert abs(pose.zone_px[0] - 137.5) < 3 and abs(pose.zone_px[1] - 224.5) < 3
    assert locate(np.full((360, 640, 3), 230, np.uint8)).robot_px is None


def test_overhead_pose_lists_cubes_with_their_colours():
    import numpy as np

    from teleop_web_lib.overhead_pose import locate

    frame = np.full((360, 640, 3), 230, np.uint8)
    frame[200:208, 300:308] = (200, 60, 20)  # blue cube, BGR
    frame[250:258, 400:408] = (20, 230, 240)  # yellow cube
    frame[170:215, 420:470] = (25, 25, 25)  # robot
    frame[190:198, 430:438] = (200, 60, 20)  # blue part on the robot: ignored
    cubes = sorted((c, round(x), round(y)) for c, x, y in locate(frame).cubes)
    assert cubes == [("blue", 304, 204), ("yellow", 404, 254)]


@pytest.mark.parametrize(
    "power,start,expected",
    [(10.0, 20.0, 20.0), (-10.0, 20.0, -20.0), (50.0, 20.0, 50.0), (1.0, 20.0, 0.0), (10.0, 0.0, 10.0)],
)
def test_with_start_power(power, start, expected):
    from teleop_web_lib.motors import with_start_power

    assert with_start_power(power, start) == expected


def test_pipeline_switches_target_colour():
    from types import SimpleNamespace

    from teleop_web_lib.detection import DetectionPipeline

    detector = SimpleNamespace(ranges={"blue": 1})
    pipeline = DetectionPipeline(
        detector, None, cube_ranges={"blue": 1, "yellow": 2, "red": 3}
    )
    assert pipeline.cube_colors == ["blue", "yellow", "red"]
    pipeline.set_target_colors(["yellow"])
    assert pipeline.target_colors == ["yellow"]
    with pytest.raises(ValueError):
        pipeline.set_target_colors(["pink"])
    assert pipeline.target_colors == ["yellow"]
