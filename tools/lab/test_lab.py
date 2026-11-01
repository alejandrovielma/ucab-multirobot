"""Checks of the lab tool without hardware: python3 -m pytest tools/lab"""

import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from hardware import WHEELS, Camera, LineSensor, Motors, load_config, open_gpio  # noqa: E402
from lab_robot import vision_params  # noqa: E402

from mobile_base.vision import detect_cubes  # noqa: E402

FIXTURE = pathlib.Path(__file__).resolve().parents[2] / "src/mobile_base/test/fixtures/webots_cube_at_mouth.jpg"


@pytest.mark.parametrize("robot", ["phineas", "ferb"])
def test_config_has_every_section_the_tool_reads(robot):
    cfg = load_config(robot)
    for section in ("hat_motor_driver", "line_sensor", "cube_detector", "behavior", "safety_gate"):
        assert section in cfg
    hat = cfg["hat_motor_driver"]
    assert hat["motor_backend"] == "fusion_hat"
    assert hat["battery_voltage"] == 8.4
    assert len(hat["fusion_ports"]) == len(hat["fusion_reversed"]) == len(WHEELS)
    for wheel in WHEELS:
        assert len(hat[f"pins_{wheel}"]) == 2


def test_motor_duties_never_exceed_the_cap_and_respect_the_speed_cap():
    motors = Motors(load_config("phineas"), open_gpio(None, dry_run=True), max_speed=0.08, dry_run=True)
    duties = motors.drive(1.0, 0.0, 0.0)  # asks 1 m/s: limited to 0.08 m/s
    assert all(abs(d) <= motors.cap + 1e-9 for d in duties)
    slow = motors.drive(0.08, 0.0, 0.0)
    assert duties == pytest.approx(slow)


def test_line_sensor_in_dry_run_reads_white():
    line = LineSensor(load_config("phineas"), open_gpio(None, dry_run=True))
    assert line.raw() == 0 and not line.black()


def test_file_camera_and_detection_with_the_yaml_hue_ranges():
    cam = Camera(f"file:{FIXTURE}")
    dets = detect_cubes(cam.read(), vision_params(load_config("ferb")))
    assert "green" in {d.color for d in dets}


class FakeMotor:
    def __init__(self, port):
        self.port, self.power_log, self.reverse = port, [], None

    def power(self, value):
        self.power_log.append(value)

    def set_is_reverse(self, flag):
        self.reverse = flag

    def stop(self):
        self.power_log.append(0.0)


def test_fusion_backend_drives_the_ports_from_the_yaml(monkeypatch):
    import hardware

    monkeypatch.setattr(hardware, "load_motor_class", lambda log: FakeMotor)
    cfg = load_config("ferb")
    cfg["hat_motor_driver"]["fusion_ports"] = ["M3", "M2", "M1", "M0"]
    cfg["hat_motor_driver"]["fusion_reversed"] = [True, False, False, False]
    motors = Motors(cfg, open_gpio(None, dry_run=True), max_speed=0.10)
    assert not motors.dry_run
    assert motors.port("fl") == "puerto M3 invertido"
    assert motors.bank.motors["fl"].port == "M3" and motors.bank.motors["fl"].reverse is True
    motors.wheel("fr", 1.0)  # full duty asked: the battery cap still applies
    assert motors.bank.motors["fr"].power_log[-1] == pytest.approx(motors.cap * 100.0)
    assert motors.bank.motors["fl"].power_log[-1] == 0.0
    motors.drive(1.0, 0.0, 3.0)
    assert all(abs(m.power_log[-1]) <= motors.cap * 100.0 + 1e-9 for m in motors.bank.motors.values())


def test_fusion_backend_applies_the_start_power_from_the_yaml(monkeypatch):
    import hardware

    monkeypatch.setattr(hardware, "load_motor_class", lambda log: FakeMotor)
    cfg = load_config("phineas")
    cfg["hat_motor_driver"]["fusion_start_power"] = [30.0, 30.0, 30.0, 30.0]
    motors = Motors(cfg, open_gpio(None, dry_run=True), max_speed=0.10)
    motors.drive(0.02, 0.0, 0.0)  # slow: without the start power this is ~4 % and the wheels stall
    assert all(m.power_log[-1] > 30.0 for m in motors.bank.motors.values())


def test_fusion_backend_dry_run_never_loads_the_library(monkeypatch):
    import hardware

    def boom(log):
        raise AssertionError("dry run must not import fusion_hat")

    monkeypatch.setattr(hardware, "load_motor_class", boom)
    motors = Motors(load_config("phineas"), open_gpio(None, dry_run=True), max_speed=0.10, dry_run=True)
    assert motors.dry_run and motors.bank is None
    assert motors.port("rr") == "puerto M1 invertido"  # Phineas map measured 2026-10-27
    motors.drive(0.1, 0.0, 0.0)  # no hardware, no error


def test_battery_override_sets_the_cap():
    cfg = load_config("phineas")
    cfg["hat_motor_driver"]["battery_voltage"] = 7.4
    motors = Motors(cfg, open_gpio(None, dry_run=True), max_speed=0.10, dry_run=True)
    assert motors.cap == pytest.approx(6.0 / 7.4)


def test_cokoino_backend_still_available():
    cfg = load_config("phineas")
    cfg["hat_motor_driver"]["motor_backend"] = "cokoino_drv8833"
    motors = Motors(cfg, open_gpio(None, dry_run=True), max_speed=0.10)
    assert motors.dry_run
    assert motors.port("fl") == "pines IN [17, 27]"
    motors.drive(0.1, 0.0, 0.0)


def test_camera_status_line_reports_the_blue_square():
    import numpy as np

    from lab_robot import camera_status

    frame = np.full((480, 640, 3), 255, np.uint8)
    frame[300:470, 100:540] = (180, 60, 20)  # a big blue region: the delivery square
    text, square = camera_status(frame, vision_params(load_config("phineas")))
    assert square.visible
    assert "cuadro azul" in text


def test_camera_holders_finds_another_process_with_the_camera_open(tmp_path):
    from hardware import camera_holders

    def proc(pid, cmdline, *targets):
        fd = tmp_path / str(pid) / "fd"
        fd.mkdir(parents=True)
        (tmp_path / str(pid) / "cmdline").write_bytes(cmdline.encode().replace(b" ", b"\0"))
        for i, target in enumerate(targets):
            (fd / str(i)).symlink_to(target)

    proc(100, "python3 tools/lab/lab_robot.py camera", "/dev/media1", "/dev/null")
    proc(200, "bash", "/dev/pts/0")
    proc(300, "python3 me", "/dev/video0")  # this process: never reported
    assert camera_holders(proc_root=tmp_path, me=300) == [(100, "python3 tools/lab/lab_robot.py camera")]


def test_camera_holders_without_proc_reports_nothing(tmp_path):
    from hardware import camera_holders

    assert camera_holders(proc_root=tmp_path / "missing") == []


def test_busy_camera_fails_at_once_naming_the_process(monkeypatch):
    import hardware

    monkeypatch.setattr(hardware, "camera_holders", lambda: [(1448, "python3 tools/lab/lab_robot.py camera")])
    with pytest.raises(RuntimeError, match=r"camera in use by PID 1448"):
        Camera("auto")


def test_star_steps_are_pure_and_come_back_to_the_start():
    import math

    from lab_robot import robot_steps

    steps = robot_steps(0.1, 2.0, star=True)
    assert all(wz == 0.0 or (vx == 0.0 and vy == 0.0) for _, (vx, vy, wz) in steps)
    assert sum(vx for _, (vx, _, _) in steps) == pytest.approx(0.0)
    assert sum(vy for _, (_, vy, _) in steps) == pytest.approx(0.0)
    assert sum(wz for _, (_, _, wz) in steps) == pytest.approx(0.0)
    assert max(math.hypot(vx, vy) for _, (vx, vy, _) in steps) == pytest.approx(0.1)


def test_threshold_motions_drive_every_wheel_like_a_differential_for_spins():
    from lab_robot import THRESHOLD_MOTIONS, motion_signs

    patterns = {name: motion_signs(*cmd) for name, cmd in THRESHOLD_MOTIONS}
    assert patterns["girar antihorario"] == (-1.0, 1.0, -1.0, 1.0)  # left side back, right side forward
    assert patterns["adelante"] == (1.0, 1.0, 1.0, 1.0)
    assert patterns["izquierda"] == (-1.0, 1.0, 1.0, -1.0)  # mecanum strafe
    assert all(0.0 not in signs for signs in patterns.values())  # all four wheels, always


def test_motion_is_detected_after_two_steps_well_above_the_still_noise():
    from lab_robot import moving_since

    assert moving_since([0.8, 0.9, 1.0, 3.5, 4.0, 6.0], still_mean=0.8, still_std=0.1) == 3
    assert moving_since([0.8, 3.5, 0.9, 1.0], still_mean=0.8, still_std=0.1) is None  # one spike is noise
    assert moving_since([], still_mean=0.8, still_std=0.1) is None


def test_mission_colors_come_from_the_arguments_and_a_bad_name_exits_with_a_message():
    from types import SimpleNamespace

    from lab_robot import mission_colors

    assert mission_colors(SimpleNamespace(target="yellow", avoid=None), {}) == (("yellow",), ())
    assert mission_colors(SimpleNamespace(target="blue", avoid="red"), {}) == (("blue",), ("red",))
    for bad in ("yellow,blue", "red"):  # one color per robot, and only yellow or blue
        with pytest.raises(SystemExit, match="mision invalida"):
            mission_colors(SimpleNamespace(target=bad, avoid=None), {})
    assert mission_colors(SimpleNamespace(target=None, avoid=None), {"my_colors": "", "avoid_colors": ""}) == ((), ())
    with pytest.raises(SystemExit, match="mision invalida.*rosa"):
        mission_colors(SimpleNamespace(target="rosa", avoid=None), {})


def test_lab_speed_caps_limit_the_spins_too_not_only_the_translation():
    from types import SimpleNamespace

    from lab_robot import lab_speed_caps

    behavior = {"explore_speed": 0.12, "approach_speed": 0.10, "turn_speed": 1.2, "approach_turn_min": 0.4,
                "k_turn": 1.5, "capture_speed": 0.08, "carry_speed": 0.08}
    slow = lab_speed_caps(dict(behavior), SimpleNamespace(max_speed=0.05, max_turn=0.5))
    assert (slow["explore_speed"], slow["approach_speed"], slow["turn_speed"]) == (0.05, 0.05, 0.5)
    # centering a cube spins at k_turn * |cx| (up to k_turn): it needs the spin cap too; the push is capped too
    assert (slow["k_turn"], slow["capture_speed"], slow["carry_speed"]) == (0.5, 0.05, 0.05)
    assert slow["approach_turn_min"] == 0.4 and slow["has_gate"] is False
    assert lab_speed_caps(dict(behavior), SimpleNamespace(max_speed=0.05, max_turn=0.3))["approach_turn_min"] == 0.3
    fast = lab_speed_caps(dict(behavior), SimpleNamespace(max_speed=1.0, max_turn=5.0))
    assert (fast["explore_speed"], fast["turn_speed"]) == (0.12, 1.2)  # a cap never speeds the robot up
    assert (fast["k_turn"], fast["capture_speed"]) == (1.5, 0.08)


def test_sigterm_and_sighup_unwind_the_modes_so_the_motors_stop():
    import signal

    from lab_robot import stop_on_signals

    signals = [s for s in (signal.SIGTERM, getattr(signal, "SIGHUP", None)) if s is not None]
    old = {s: signal.getsignal(s) for s in signals}
    try:
        stop_on_signals()
        for s in signals:
            with pytest.raises(SystemExit):
                signal.getsignal(s)(s, None)
    finally:
        for s, handler in old.items():
            signal.signal(s, handler)


def test_park_is_sticky_once_the_robot_parks_at_the_capture():
    from lab_robot import Park

    park = Park(True)
    assert not park.check(False)  # not parked before the first capture
    assert park.check(True)
    # A later tick (for example after an exception inside the parked tick) can see the FSM past CAPTURE:
    # the robot must stay parked, never drive again.
    assert park.check(False)
    assert not Park(False).check(True)  # without --stop-at-capture nothing parks


class FakeBank:
    """Stands in for FusionMotorBank: records calls, can be told to fail."""

    def __init__(self, apply_ok=True, stop_results=(True,)):
        self.apply_ok, self.stop_results, self.applied, self.stops = apply_ok, list(stop_results), [], 0

    def apply(self, duties, cap):
        self.applied.append(duties)
        return self.apply_ok

    def stop(self):
        self.stops += 1
        return self.stop_results.pop(0) if len(self.stop_results) > 1 else self.stop_results[0]


def banked_motors(bank, **kw):
    motors = Motors(load_config("phineas"), open_gpio(None, dry_run=True), max_speed=0.10, dry_run=True, **kw)
    motors.bank = bank
    return motors


def test_drive_raises_when_the_hat_rejects_the_command_so_the_supervisor_counts_a_failing_tick():
    motors = banked_motors(FakeBank(apply_ok=False))
    with pytest.raises(RuntimeError, match="motor command failed"):
        motors.drive(0.05, 0.0, 0.0)


def test_stop_retries_once_and_returns_whether_it_worked():
    bank = FakeBank(stop_results=(False, True))
    assert banked_motors(bank).stop() is True and bank.stops == 2
    bank = FakeBank(stop_results=(False,))
    assert banked_motors(bank).stop() is False and bank.stops == 2  # never raises


def test_stop_does_not_raise_when_the_bank_does():
    class Boom(FakeBank):
        def apply(self, duties, cap):
            raise OSError("i2c")

    assert banked_motors(Boom()).stop() is False


def test_a_diagonal_is_capped_by_its_length_not_by_its_larger_component():
    motors = Motors(load_config("phineas"), open_gpio(None, dry_run=True), max_speed=0.10, dry_run=True)
    diagonal, straight = motors.drive(0.10, 0.10, 0.0), motors.drive(0.10 / 2 ** 0.5, 0.10 / 2 ** 0.5, 0.0)
    assert diagonal == pytest.approx(straight)


def test_max_turn_caps_the_spin_and_is_off_by_default():
    free = Motors(load_config("phineas"), open_gpio(None, dry_run=True), max_speed=0.10, dry_run=True)
    capped = Motors(load_config("phineas"), open_gpio(None, dry_run=True), max_speed=0.10, dry_run=True, max_turn=0.5)
    assert capped.drive(0.0, 0.0, 5.0) == pytest.approx(free.drive(0.0, 0.0, 0.5))
    assert capped.drive(0.0, 0.0, -5.0) == pytest.approx(free.drive(0.0, 0.0, -0.5))
    assert free.drive(0.0, 0.0, 1.0) != pytest.approx(free.drive(0.0, 0.0, 0.5))


@pytest.mark.parametrize("bad", [0.0, -0.1, float("nan"), float("inf")])
def test_motors_refuse_a_speed_cap_that_does_not_cap(bad):
    with pytest.raises(ValueError):
        Motors(load_config("phineas"), open_gpio(None, dry_run=True), max_speed=bad, dry_run=True)


def test_non_finite_commands_drive_nothing():
    motors = Motors(load_config("phineas"), open_gpio(None, dry_run=True), max_speed=0.10, dry_run=True, max_turn=0.5)
    for cmd in ((float("nan"), 0.0, 0.0), (0.0, float("inf"), 0.0), (0.0, 0.0, float("nan"))):
        assert motors.drive(*cmd) == (0.0, 0.0, 0.0, 0.0)


@pytest.mark.parametrize("battery", [5.0, float("nan"), 12.0])
def test_a_battery_voltage_outside_6_to_9_v_is_refused(battery):
    cfg = load_config("phineas")
    cfg["hat_motor_driver"]["battery_voltage"] = battery
    with pytest.raises(ValueError):
        Motors(cfg, open_gpio(None, dry_run=True), max_speed=0.10, dry_run=True)


# ---- argument checks, HAT and wiring guards, mapper (audit 2026-10-31)

LAB = pathlib.Path(__file__).resolve().parent


def lab_args(*argv):
    """Parse like main() does; returns the namespace or raises SystemExit (argparse error, code 2)."""
    from lab_robot import build_parser

    return build_parser().parse_args(list(argv))


@pytest.mark.parametrize("flag", ["--max-speed", "--max-turn", "--speed", "--seconds", "--step", "--duty",
                                  "--latch-timeout"])
@pytest.mark.parametrize("bad", ["0", "-1", "nan", "inf", "abc"])
def test_numeric_arguments_must_be_finite_and_positive(flag, bad):
    with pytest.raises(SystemExit):
        lab_args("chase", flag, bad)


def test_duty_is_a_fraction_and_battery_must_be_finite():
    assert lab_args("motors", "--duty", "1").duty == 1.0
    for argv in (("--duty", "1.5"), ("--battery", "nan"), ("--battery", "inf")):
        with pytest.raises(SystemExit):
            lab_args("motors", *argv)
    assert lab_args("motors", "--battery", "7.4").battery == 7.4


def test_a_bad_battery_ends_as_a_clear_message_not_a_traceback():
    from types import SimpleNamespace

    from lab_robot import make_motors

    cfg = load_config("phineas")
    cfg["hat_motor_driver"]["battery_voltage"] = 5.0
    args = SimpleNamespace(dry_run=True)
    with pytest.raises(SystemExit, match="bateria"):
        make_motors(args, cfg, open_gpio(None, dry_run=True), 0.05)


def test_a_hat_that_does_not_answer_is_not_a_working_robot_unless_dry_run(monkeypatch):
    import hardware
    from types import SimpleNamespace

    from lab_robot import make_motors

    monkeypatch.setattr(hardware, "load_motor_class", lambda log: None)
    monkeypatch.setattr(hardware, "open_bank", lambda *a, **k: None)  # the bank could not be created
    cfg, gpio = load_config("phineas"), open_gpio(None, dry_run=True)
    with pytest.raises(SystemExit, match="Fusion HAT.*no responde"):
        make_motors(SimpleNamespace(dry_run=False), cfg, gpio, 0.05)
    assert make_motors(SimpleNamespace(dry_run=True), cfg, gpio, 0.05).dry_run


def test_only_a_robot_with_verified_wiring_runs_the_autonomous_modes_for_real():
    from types import SimpleNamespace

    from hardware import VERIFIED_WIRING
    from lab_robot import require_verified_wiring

    assert "phineas" in VERIFIED_WIRING and "ferb" not in VERIFIED_WIRING
    require_verified_wiring(SimpleNamespace(robot="phineas", dry_run=False))
    require_verified_wiring(SimpleNamespace(robot="ferb", dry_run=True))
    with pytest.raises(SystemExit, match="ferb.*only-wheels.*VERIFIED_WIRING"):
        require_verified_wiring(SimpleNamespace(robot="ferb", dry_run=False))


def test_the_star_spin_is_refused_when_a_short_seconds_makes_it_too_fast():
    from lab_robot import robot_steps

    with pytest.raises(ValueError, match="seconds"):
        robot_steps(0.08, 0.1, star=True)
    assert robot_steps(0.08, 1.0, star=True)[-2][1][2] == pytest.approx(1.5707963)


def test_the_dashboard_token_has_256_bits():
    assert "token_urlsafe(32)" in (LAB / "lab_robot.py").read_text()


class FakeCarto:
    def __init__(self):
        self.moved_calls = []

    def moved(self, cmd, dt):
        self.moved_calls.append((cmd, dt))

    def observe(self, *a, **k):
        from types import SimpleNamespace

        return SimpleNamespace(localized=False)

    def goal_for(self, *a):
        return None


def mapper_inputs(mapper, now):
    from types import SimpleNamespace

    return mapper.inputs(now, False, None, None, [], SimpleNamespace(targets=(), carrying=False))


def test_the_mapper_never_integrates_a_command_for_longer_than_a_tick_or_after_a_stop(monkeypatch):
    import lab_robot

    monkeypatch.setattr(lab_robot, "landmark_sightings", lambda *a: [])
    mapper = lab_robot.MapRunner(True)
    mapper.carto = FakeCarto()
    mapper.sent((0.05, 0.0, 0.0), 0.0)
    mapper_inputs(mapper, 20.0)  # a 20 s outage: the integration step is clamped
    assert mapper.carto.moved_calls == [((0.05, 0.0, 0.0), 0.3)]
    mapper.sent((0.05, 0.0, 0.0), 20.0)
    mapper.sent(None, 20.1)  # the supervisor stopped the motors
    mapper_inputs(mapper, 40.0)
    assert len(mapper.carto.moved_calls) == 1  # nothing moved: nothing integrated


def test_lab_speed_caps_also_cap_the_spin_that_carries_a_cube():
    from types import SimpleNamespace

    from lab_robot import lab_speed_caps

    args = SimpleNamespace(max_speed=0.05, max_turn=0.3)
    assert lab_speed_caps({"carry_max_turn": 0.4}, args)["carry_max_turn"] == 0.3
    assert lab_speed_caps({}, SimpleNamespace(max_speed=0.05, max_turn=0.5))["carry_max_turn"] == 0.4
