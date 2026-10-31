# Hardware and connections

Values marked **measure** come from photos or datasheets and must be confirmed with the robot in hand.

## Collector

| Part | Model | Notes |
|---|---|---|
| Chassis | LewanSoul / Hiwonder mecanum | 186 × 161 × 91 mm, 65 mm wheels |
| Motors | 4× TT 1:120, 3–6 V | no encoders |
| Computer | Raspberry Pi 5, 8 GB | needs 5 V / 5 A |
| Driver | Cokoino 4WD HAT (2× DRV8833) | **max 10.8 V → 2S battery**; sold for Pi 4, confirm pins on the Pi 5 |
| Pi regulator | XL4015 at 5.1 V | separate from the HAT |
| Battery | 2S Li-ion 18650 (7.4 V) | with fuse and main switch |
| Scoop | 3D printed (see plan, section 6) | one cube at a time |
| Scoop servo | SG90 | 0° down, ≈40° up (**measure**) |
| Camera | Pi camera module or USB webcam | 640 × 480 |
| Line sensor | 1× TCRT5000 | underneath, between the front wheels |
| Ultrasonic | HC-SR04 | front; the 5 V echo needs a divider to 3.3 V |
| Scoop presence | FC-51 (IR) | 1 = cube present |
| Scoop colour | TCS34725 (I2C) | confirms the cube's colour |
| IMU (optional) | MPU6050 (I2C) | approximate turns |

## Pins (to be filled in)

Fill in with what is measured on the HAT and copy the same numbers into `src/mobile_base/config/collector.yaml`. A pin left at `-1` there means "not wired": that node runs as a fake and warns.

| Signal | GPIO (BCM) | Status |
|---|---|---|
| DRV8833 #1 / #2 nSLEEP | 12 / 13 | confirmed (CKK0018 schematic) |
| Motor FL (M3) IN1 / IN2 | 24 / 25 | confirmed (DRV8833 #2 A) |
| Motor FR (M4) IN1 / IN2 | 26 / 16 | confirmed (DRV8833 #2 B) |
| Motor RL (M1) IN1 / IN2 | 17 / 27 | confirmed (DRV8833 #1 A) |
| Motor RR (M2) IN1 / IN2 | 22 / 23 | confirmed (DRV8833 #1 B) |
| Scoop servo | ? | measure |
| Bottom TCRT5000 (DO, at 3.3 V) | 5 | wired (pin 29); in `tools/teleop_web.yaml` → `line_sensor.pin` and `collector.yaml` → `line_sensors.pin` |
| HC-SR04 TRIG / ECHO | ? / ? | measure |
| FC-51 | ? | measure |
| I2C SDA / SCL | 2 / 3 | fixed on the Pi |

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
