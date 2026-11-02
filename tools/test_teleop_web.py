"""
Tests for the kinematics, timeout brake and control thread of the browser
tool (run with: pytest tools).
"""

import time

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
