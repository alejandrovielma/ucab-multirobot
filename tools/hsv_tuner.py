#!/usr/bin/env python3
"""
HSV range tuner with sliders (OpenCV), for photos or the live camera.

Move the six sliders until only the wanted colour is white in the mask,
then press `p` to print the range as a line for
src/vision/config/hsv_ranges.yaml (`q` or Esc quits and prints it too).
Red wraps around H = 0/179: tune each end and join the two ranges.

    python3 tools/hsv_tuner.py --image photo.jpg --color yellow
    python3 tools/hsv_tuner.py --color blue      # camera from teleop_web.yaml

It needs a display (run it on the laptop with a photo, or over `ssh -X`).
"""

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

from teleop_web_lib.config import (
    HSV_CONFIG_PATH,
    HSV_NODE_NAME,
    SECTION_CAMERA,
    VISION_PACKAGE_DIR,
    load_config,
    read_node_params,
)
from teleop_web_lib.frame_source import open_frame_source

sys.path.insert(0, str(VISION_PACKAGE_DIR))
from vision.hsv import parse_hsv_ranges  # noqa: E402

DEFAULT_CONFIG = Path(__file__).with_name("teleop_web.yaml")
WINDOW = "hsv tuner"
# Slider name and maximum, in [h_min, s_min, v_min, h_max, s_max, v_max]
# order (OpenCV: H 0-179, S and V 0-255).
SLIDERS = (
    ("H min", 179),
    ("S min", 255),
    ("V min", 255),
    ("H max", 179),
    ("S max", 255),
    ("V max", 255),
)
# Starting range when no colour is given: everything.
FULL_RANGE = [0, 0, 0, 179, 255, 255]
KEY_PRINT = ord("p")
KEY_QUIT = ord("q")
KEY_ESC = 27
KEY_WAIT_MS = 30
# waitKey returns more than the key code on some platforms.
KEY_MASK = 0xFF
# Width of each half of the side-by-side view.
VIEW_WIDTH_PX = 480


def parse_args() -> argparse.Namespace:
    """
    Command-line options.
    """
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument("--image", type=Path, help="photo instead of camera")
    parser.add_argument("--color", help="start from this colour's range")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    return parser.parse_args()


def initial_range(color: str | None) -> list[int]:
    """
    The first range of `color` in hsv_ranges.yaml, or the full range.
    """
    if not color:
        return list(FULL_RANGE)
    hsv = read_node_params(HSV_CONFIG_PATH, HSV_NODE_NAME)["hsv"]
    first = parse_hsv_ranges(hsv[color])[0]
    return [*first.lower, *first.upper]


def make_window(values: list[int]) -> None:
    """
    Create the window with one slider per range bound.
    """
    cv2.namedWindow(WINDOW)
    for (name, maximum), value in zip(SLIDERS, values, strict=True):
        cv2.createTrackbar(name, WINDOW, value, maximum, lambda _v: None)


def read_sliders() -> list[int]:
    """
    Current [h_min, s_min, v_min, h_max, s_max, v_max].
    """
    return [cv2.getTrackbarPos(name, WINDOW) for name, _ in SLIDERS]


def masked_view(bgr: np.ndarray, values: list[int]) -> np.ndarray:
    """
    The frame and its mask side by side.
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(values[:3]), np.array(values[3:]))
    scale = VIEW_WIDTH_PX / bgr.shape[1]
    size = (VIEW_WIDTH_PX, round(bgr.shape[0] * scale))
    frame = cv2.resize(bgr, size)
    mask_bgr = cv2.cvtColor(cv2.resize(mask, size), cv2.COLOR_GRAY2BGR)
    return np.hstack([frame, mask_bgr])


def frames(args: argparse.Namespace):
    """
    The photo forever, or the camera configured in teleop_web.yaml.
    """
    if args.image is not None:
        bgr = cv2.imread(str(args.image))
        if bgr is None:
            sys.exit(f"cannot read {args.image}")
        while True:
            yield bgr
    grab = open_frame_source(load_config(args.config)[SECTION_CAMERA], False)
    if grab is None:
        sys.exit("cannot open the camera")
    while True:
        frame = grab()
        if frame is not None:
            yield frame


def print_range(color: str | None, values: list[int]) -> None:
    """
    Print the range as a YAML line for hsv_ranges.yaml.
    """
    numbers = ", ".join(str(v) for v in values)
    print(f"      {color or 'colour'}: [{numbers}]", flush=True)


def main() -> None:
    """
    Show the sliders until the user quits.
    """
    args = parse_args()
    make_window(initial_range(args.color))
    for bgr in frames(args):
        values = read_sliders()
        cv2.imshow(WINDOW, masked_view(bgr, values))
        key = cv2.waitKey(KEY_WAIT_MS) & KEY_MASK
        if key == KEY_PRINT:
            print_range(args.color, values)
        elif key in (KEY_QUIT, KEY_ESC):
            print_range(args.color, values)
            break
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
