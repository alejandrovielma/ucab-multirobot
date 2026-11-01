"""Pure tests of the analog line-sensor helpers (no GPIO, no I2C)."""

import pytest

from analog_line import AnalogReader, BlackConfirm, counts_to_millivolts, word_to_counts


def test_word_to_counts_swaps_bytes():
    # SMBus gives the low byte first; the MCU sends the high byte first: 3383 = 0x0D37.
    assert word_to_counts(0x370D) == 3383
    assert word_to_counts(0x0000) == 0


def test_counts_to_millivolts_uses_12_bit_3v3():
    assert counts_to_millivolts(0) == 0
    assert counts_to_millivolts(4095) == 3299
    assert counts_to_millivolts(3383) == 2726


def test_black_confirm_needs_n_reads_in_a_row():
    confirm = BlackConfirm(2)
    assert [confirm.update(x) for x in (True, False, True, True, True, False)] == [False, False, False, True, True, False]


def test_black_confirm_one_means_no_filter_and_clamps():
    assert BlackConfirm(0).update(True) is True
    assert BlackConfirm(1).update(False) is False


class FakeBus:
    def __init__(self, word=0x370D, fail=False):
        self.word, self.fail, self.calls = word, fail, []

    def read_word_data(self, address, register):
        self.calls.append((address, register))
        if self.fail:
            raise OSError("i2c")
        return self.word


def test_analog_reader_reads_register_of_channel():
    bus = FakeBus()
    assert AnalogReader(bus, 2).counts() == 3383
    assert bus.calls == [(0x17, 0x14)]  # 0x10 + 2 * 2


def test_analog_reader_returns_none_on_i2c_error():
    assert AnalogReader(FakeBus(fail=True), 0).counts() is None


def test_analog_reader_rejects_bad_channel():
    with pytest.raises(ValueError):
        AnalogReader(FakeBus(), 4)
