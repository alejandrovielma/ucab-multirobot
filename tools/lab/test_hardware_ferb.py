"""LineSensor of tools/lab/hardware.py with Ferb's confirm filter and analog read (fake GPIO and bus)."""

from hardware import LineSensor


class FakeGpio:
    def __init__(self, levels):
        self.levels = iter(levels)

    def claim_input(self, pin):
        pass

    def read(self, pin):
        return next(self.levels)


class FakeBus:
    def read_word_data(self, address, register):
        return 0x370D


def cfg(**extra):
    return {"line_sensor": {"pin": 5, "active_level": 1, **extra}}


def test_default_behaviour_is_unchanged():
    line = LineSensor(cfg(), FakeGpio([1, 0]))
    assert line.black() is True and line.black() is False
    assert line.counts() is None


def test_confirm_reads_filters_single_black_read():
    line = LineSensor(cfg(confirm_reads=2), FakeGpio([1, 0, 1, 1]))
    assert [line.black() for _ in range(4)] == [False, False, False, True]


def test_raw_is_not_filtered():
    line = LineSensor(cfg(confirm_reads=3), FakeGpio([1]))
    assert line.raw() == 1


def test_counts_and_millivolts_with_bus():
    line = LineSensor(cfg(adc_channel=0), FakeGpio([]), bus=FakeBus())
    assert line.counts() == 3383 and line.millivolts() == 2726
