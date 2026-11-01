#!/usr/bin/env python3
"""
Tests the 4 HAT motors one by one, without ROS, with the wheels off the
ground.

Each wheel spins forward, stops, spins backward and stops, in the order
fl, fr, rl, rr. Watch that the right wheel moves and that "forward" pushes
the robot forward; if not, fix the channels / pins or the `*_inverted`
flags in tools/teleop_web.yaml (and copy them to collector.yaml).

Uses the browser tool's motor drivers and its `motors:` config:

    python3 scripts/test_hat.py                 # real HAT
    python3 scripts/test_hat.py --fake          # laptop, only logs
    python3 scripts/test_hat.py --ratio 0.3 --seconds 1.5
"""

import argparse
import logging
import sys
import time
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"
sys.path.insert(0, str(TOOLS_DIR))

from teleop_web_lib.config import SECTION_MOTORS, load_config  # noqa: E402
from teleop_web_lib.kinematics import WHEELS  # noqa: E402
from teleop_web_lib.motors import build_motors  # noqa: E402

DEFAULT_CONFIG = TOOLS_DIR / "teleop_web.yaml"
# Gentle enough for wheels in the air.
DEFAULT_RATIO = 0.4
DEFAULT_SECONDS = 1.0
PAUSE_S = 0.5
FORWARD = 1.0
BACKWARD = -1.0
LOG_FORMAT = "%(levelname)s %(message)s"


def parse_args() -> argparse.Namespace:
    """
    Command-line options.
    """
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--ratio", type=float, default=DEFAULT_RATIO)
    parser.add_argument("--seconds", type=float, default=DEFAULT_SECONDS)
    parser.add_argument("--fake", action="store_true")
    return parser.parse_args()


def spin(motors, wheel: str, ratio: float, seconds: float) -> None:
    """
    Spin one wheel at `ratio` for `seconds`, then stop them all.
    """
    ratios = dict.fromkeys(WHEELS, 0.0)
    ratios[wheel] = ratio
    motors.set(ratios)
    time.sleep(seconds)
    motors.set(dict.fromkeys(WHEELS, 0.0))
    time.sleep(PAUSE_S)


def main() -> None:
    """
    Run the wheel-by-wheel test.
    """
    args = parse_args()
    logging.basicConfig(level=logging.INFO, format=LOG_FORMAT)
    cfg = load_config(args.config)[SECTION_MOTORS]
    motors, label = build_motors(cfg, args.fake, args.config)
    print(f"driver: {label}. Wheels off the ground! Ctrl+C stops.")
    try:
        for wheel in WHEELS:
            print(f"{wheel}: forward")
            spin(motors, wheel, FORWARD * args.ratio, args.seconds)
            print(f"{wheel}: backward")
            spin(motors, wheel, BACKWARD * args.ratio, args.seconds)
    except KeyboardInterrupt:
        print("stopped")
    finally:
        motors.close()


if __name__ == "__main__":
    main()
