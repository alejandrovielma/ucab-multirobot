# Guide: mobile base and hardware

**Owner:** role 1 · **Files:** `src/mobile_base/mobile_base/{motor_driver,line_sensors,ultrasonic,scoop_driver}.py`, `scripts/setup_pi.sh`, `scripts/test_hat.py`, `tools/teleop_web_lib/`, `docs/hardware.md`

## What you need to know

- Hiwonder mecanum chassis, 65 mm wheels, TT 1:120 motors **without encoders**: speed is open loop.
- Cokoino HAT with 2× DRV8833: each motor uses two pins (IN1/IN2) with PWM. Maximum 10.8 V → 2S battery.
- The Pi 5 needs 5 V / 5 A; the regulator is separate from the HAT.
- On the Pi 5 use `lgpio` or `gpiozero`; RPi.GPIO does not work. The HAT pins must be measured (it was designed for the Pi 4).
- The HC-SR04 echo outputs 5 V: it needs a divider to 3.3 V.
- REP 103 sign convention: `linear.x` > 0 forward, `linear.y` > 0 left, `angular.z` > 0 counter-clockwise.
- Wheel and sensor order throughout the project: fl, fr, rl, rr.

## Testing

- First with the wheels in the air (`scripts/test_hat.py`).
- Direct HAT control, without ROS (while `motor_driver` does not exist): `python3 tools/teleop_web.py` and open http://localhost:8081 (`--fake` on the laptop). Pins, limits and Auto settings in `tools/teleop_web.yaml`; the code is in `tools/teleop_web_lib/` (run its tests with `pytest tools`). **It does not go through `obstacle_guard`**: wheels in the air. From another machine: `ssh -L 8081:localhost:8081 <user>@<pi-ip>`.
- With ROS, manual WASD control is in the web viewer (`web_viewer`, port 8080).
- If a wheel spins backwards, invert that motor in the YAML, not in the wiring.
