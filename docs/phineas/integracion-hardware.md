# Hardware integration (T7): Ferb's path in tools/lab

## Adopted
- **AO analog line reading and confirm filter** -> `tools/lab/analog_line.py` (vendored from `tools/teleop_web_lib/line_sensor.py`: ADC at I2C 0x17, register `0x10 + 2*N`, 12-bit, 3.3 V reference, `BlackConfirm`). `hardware.LineSensor` keeps its API (`raw`, `black`) and gains `counts()`, `millivolts()`. Both features are opt-in via `line_sensor.adc_channel` and `line_sensor.confirm_reads`; without them behaviour is identical to before. `lab_robot.py line` prints AO counts and mV when `adc_channel` is set.
  Why: our lab only read DO. Ferb measured AO (white 285-299, black 3383 counts, `docs/hardware.md`) and the filter fixed false line reads from the sensor lifting off the table.
- No analog threshold was added: Ferb's code has none (it decides with DO and only logs AO), so inventing one would be guessing.

## Skipped
- **Motor path** (`motor_hardware.py` / `teleop_web_lib/motors.py`). Ours (`tools/lab/hardware.py` + `mobile_base.core.fusion_hat`) already supports both Fusion HAT+ and Cokoino DRV8833, has a dry-run backend, per-wheel start power and trim, duty cap from battery voltage, validated arguments and a stop that retries and reports failure. Ferb's writes raw I2C registers, has no per-wheel calibration, and its `cmd_timeout_s` brake lives in the node, not the driver; our lab already stops motors on Ctrl+C, camera stall and hang watchdog. Not better, and porting it would change the wiring path verified on Phineas.

## Discrepancies (not overwritten)
| Quantity | Ours | Ferb | File |
|---|---|---|---|
| Wheel radius | 0.0325 m | 65 mm wheels (same) | phineas.yaml / docs/hardware.md: equal |
| Max speed | `max_wheel_speed_m_s: 0.24` (rim speed at full duty, TODO measure) | forward 0.31 m/s at command 0.42, backward 0.23 m/s at 0.35 | collector.yaml vs docs/hardware.md: different quantity (measured path speed at a command, browser tool), not replaced |
| Lab speed cap | `--max-speed 0.05` m/s | n/a | lab_robot.py: a safety cap, not a measurement |
| Wheel wiring | phineas.yaml `fusion_ports [M3,M0,M2,M1]`, `fusion_reversed [F,T,F,T]` (verified on the floor) | teleop_web.yaml channels M0=FL, M1=FR, M2=RR, M3=RL, "TODO confirm"; ferb.yaml ports `[M0..M3]`, unreversed | different robots or unverified: kept ours |
| Line sensor pin / level | pin 5, active_level 1 | pin 5, black_value 1 | equal |
| Line confirm reads | none (1) | 2 | teleop_web.yaml: available now as `confirm_reads`, not enabled in phineas.yaml |
| lx, ly | 0.0605, 0.0655 | chassis 186 x 161 mm (hardware.md) | not comparable (wheel half-spacing vs body size) |

No config value was changed.

## To test on the real Pi
- `lab_robot.py line` with `adc_channel: 0` in `line_sensor` of the robot YAML: AO should read about 285-299 on white and about 3383 on black. Confirm the AO wire is on the HAT's A0 (flag: depends on wiring I cannot confirm).
- `confirm_reads: 2` in the loop of `chase`/`drive`: check black runs are not missed on the 19 mm tape.
- Run `tools/lab/test_lab.py` on Linux (imports termios).
