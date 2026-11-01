# Guide: simulation, integration and quality

**Owner:** role 4 · **Files:** `simulation/`, `src/bringup/`, `scripts/setup_dev.sh`, `.github/`

## What you need to know

- Webots R2025a. The nodes connect through our own `webots_bridge` bridge (`<extern>` controller), not through `webots_ros2`. Environment: Docker (`compose.yaml`, `docker/README.md`); on Windows, native Webots + `docker compose up sim-external`.
- `pista_ucab_ros.wbt` is generated from `pista_ucab.wbt` with `tools/generate_ros_world.py`.
- The `pista_ucab.wbt` world has the track rectified from the photo, 14 cubes and two collectors (the second one plays the other team's robot).
- PROTO device names (Spanish, kept as is): `motor_fl/fr/rl/rr`, `servo_pala` (scoop servo), `camara` (camera), `linea_fl/fr/rl/rr` (line sensors), `ultrasonido` (ultrasonic), `ir_pala` (scoop IR), `color_pala` (scoop colour sensor), `acelerometro` (accelerometer), `giroscopio` (gyroscope).
- Simulated line sensors: ≈240 on white, 0–30 on black.
- The wheel materials (fl/rr `Exterior`, fr/rl `Interior`) make the mecanum friction work; do not change them.
- When the real track is measured, update the world and `docs/hardware.md`.

## To do

- `webots_bridge` only publishes camera and ultrasonic: `line`, `scoop/has_cube`, `scoop/color` and moving the scoop are missing.
- `scripts/setup_dev.sh`: for now the recommended environment is Docker (`docker/README.md`).
- Enable `main` protection and CI on GitHub.
