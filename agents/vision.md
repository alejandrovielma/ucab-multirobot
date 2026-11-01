# Guide: vision and colour

**Owner:** role 2 · **Files:** `src/vision/`, `src/mobile_base/mobile_base/color_sensor.py`, `tools/hsv_tuner.py`

## What you need to know

- OpenCV uses H from 0 to 179. Red wraps around (near 0 and near 179).
- Cube colours: red, orange, yellow, green, blue and purple/burgundy. Known risk: burgundy vs red.
- The lab lighting changes: calibrate under the real light and save the ranges in `config/hsv_ranges.yaml`.
- The camera is on the scoop, close to the floor and tilted down; the cube's row in the image gives an approximate distance.
- Pure logic is split by concern: `hsv.py` (ranges), `camera_model.py` (geometry), `color_detection.py` (detector), `drawing.py`. The first two need no OpenCV, so their tests run anywhere.
- The TCS34725 on the scoop confirms the colour of the captured cube.
- If reserved colours are agreed with the other collector, they are set by parameter (see `docs/external-interface.md`).
