# Hardware and connections

Values marked **measure** come from photos or datasheets and must be confirmed with the robot in hand.

## Collector

| Part | Model | Notes |
|---|---|---|
| Chassis | LewanSoul / Hiwonder mecanum | 186 × 161 × 91 mm, 65 mm wheels |
| Motors | 4× TT 1:120, 3–6 V | no encoders; duty cap `0.71` (`6.0 V / 8.4 V` full 2S battery) |
| Computer | Raspberry Pi 5, 8 GB | needs 5 V / 5 A |
| Primary driver | SunFounder Fusion HAT+ (MCU I2C `0x17`) | 4 motor ports `M0..M3` via PWM channels `P4..P11`; 12-bit ADC `A0..A3` |
| Backup driver | Cokoino 4WD HAT (2× DRV8833) | **max 10.8 V → 2S battery**; BCM GPIO pins below |
| Pi regulator | Fusion HAT+ onboard / XL4015 at 5.1 V | 5 V / 5 A for Pi 5 |
| Battery | 2S Li-ion 18650 (7.4 V nominal, 8.4 V full) | with fuse and main switch |
| Scoop | 3D printed (fixed U scoop) | one cube at a time |
| Scoop servo | SG90 (optional) | 0° down, ≈40° up (**measure**) |
| Camera | Sony IMX219-120 CSI Pi camera (or USB webcam) | 640 × 480 (1640 × 1232 2×2 binned sensor mode, `vflip: true`, `hflip: true`) |
| Line sensor | 1× TCRT5000 | underneath, between the front wheels |
| Ultrasonic | HC-SR04 | front; the 5 V echo needs a divider to 3.3 V |
| Scoop presence | FC-51 (IR) | 1 = cube present |
| Scoop colour | TCS34725 (I2C) | confirms the cube's colour |
| IMU (optional) | MPU6050 (I2C) | approximate turns |

## Pins and channels (Phineas)

Configured in `src/mobile_base/config/collector.yaml` and `tools/teleop_web.yaml`. A pin left at `-1` means "not wired": that node runs as a fake and warns.

### SunFounder Fusion HAT+ (I2C `0x17`, verified on Phineas 2026-10-27)

| Wheel | Port | PWM channels `(a, b)` | Inverted | Status |
|---|---|---|---|---|
| Motor FL | `M3` | `4 / 5` | `false` | verified on floor |
| Motor FR | `M0` | `11 / 10` | `true` | verified on floor |
| Motor RL | `M2` | `6 / 7` | `false` | verified on floor |
| Motor RR | `M1` | `9 / 8` | `true` | verified on floor |

### GPIO (BCM) — Cokoino HAT & Sensors (Phineas)

| Signal | GPIO (BCM) | Status |
|---|---|---|
| DRV8833 #1 / #2 nSLEEP | 12 / 13 | confirmed (CKK0018 schematic) |
| Motor FL IN1 / IN2 | 17 / 27 | confirmed on Phineas (DRV8833 #1 A) |
| Motor FR IN1 / IN2 | 24 / 25 | confirmed on Phineas (DRV8833 #2 A) |
| Motor RL IN1 / IN2 | 22 / 23 | confirmed on Phineas (DRV8833 #1 B) |
| Motor RR IN1 / IN2 | 26 / 16 | confirmed on Phineas (DRV8833 #2 B) |
| Scoop servo | ? (`-1`) | measure (fixed scoop) |
| Bottom TCRT5000 (DO, at 3.3 V) | 5 | wired on Phineas (pin 29, `black_value: 1`); in `tools/teleop_web.yaml` → `line_sensor.pin` and `collector.yaml` → `line_sensors.pin` |
| HC-SR04 TRIG / ECHO | ? / ? (`-1 / -1`) | measure |
| FC-51 | ? (`-1`) | measure |
| I2C SDA / SCL | 2 / 3 | fixed on the Pi (`i2c_bus: 1`) |

## Track

| Measurement | Value | Status |
|---|---|---|
| Board | ≈ 1.93 × 1.22 m (from the photo) | measure |
| Tape width | ≈ 19 mm | measure |
| Delivery area | blue + green paper in a gap of the line, ≈ 24 cm per side; green landmark on the left, red on the right | measure |
| Arena (inside the black line) | 116.5 cm long × 70 cm wide (measured) | measured |
| Temporary delivery area | blue sheet 16.5 cm wide × 13.6 cm long (measured); pink line where it starts; black tape closes it at the back | measured; check which side is the depth |
| Landmarks (legos) | 8.4 cm high (measured); ≈ 34.5 cm apart, it changes | measured; 8.4 cm with the base; the coloured section the camera sees is 6.4 cm. The code uses an effective 13.25 cm: the configured hfov (0.94 rad) is about twice the real ~0.48 rad (cropped 640x480 sensor mode), so distances by size need it |
| Camera (browser tool) | lens centre 5.8 cm above the table; sensor mode 1640x1232 (hfov ~1.18 rad, ~68 deg); tilt 0.061 rad (3.5 deg); fitted to cubes at 20/30/40/60 cm within 2.1 px (`camera.model` in `tools/teleop_web.yaml`) | measured |
| Drive speeds (browser tool) | forward at 0.42 ~0.31 m/s; backward at 0.35 ~0.23 m/s (measured from the legos' size in the camera, 4 and 2 pushes) | measured |
| Camera vs scoop | scoop starts 4.8 cm below the lens centre; the scoop front is ~0.7 cm ahead of the lens (a cube touching it is seen from 5.2 cm of separation, when its top edge reaches row 479) | measured |
| Image bottom row (479) | sees a whole cube down to 10 cm from the lens (~9 cm from the scoop); only its top edge from 5.7 cm; a cube touching the scoop is blind | derived from the fit |
| Line sensor over blue, pink, white, black | AO 285 / 290 / 299 / 3383 counts (blue and pink read as white) | measured: cannot detect them |
| Line sensor over the area paper | must read white on blue and green | measure |
| Cubes | 2.5 cm, ≈ 9.4 g | measure |
| Colours | red, orange, yellow, green, blue, purple/maroon | confirm maroon |
