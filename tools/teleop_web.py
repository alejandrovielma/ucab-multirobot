#!/usr/bin/env python3
"""
Browser control for the collector: WASD driving, camera video and an Auto
mode.

Auto mode: find a target-colour cube, push it into the scoop, carry it to
the delivery gate (the paper between the green and red landmarks), leave it
there and count it. The bottom line sensor stops the robot at the black
border (it backs up and turns inward).

Interim tool without ROS: it talks straight to the HAT, so it does NOT go
through obstacle_guard. Use it with the wheels off the ground or a hand near
the switch.

Usage:
  python3 tools/teleop_web.py            # on the Pi, with the robot
  python3 tools/teleop_web.py --fake     # on the laptop: fake motors and camera
Then open http://localhost:8081 (8080 is the ROS web_viewer)

Keys: W/S forward/back, A/D turn, Q/E strafe, P auto, R reset, space brake,
Shift = faster.

The code lives in tools/teleop_web_lib/; this file only wires it together.
"""

import argparse
import logging
import signal
from http.server import ThreadingHTTPServer
from pathlib import Path

from teleop_web_lib.camera import Camera
from teleop_web_lib.config import (
    SECTION_AUTO,
    SECTION_CAMERA,
    SECTION_DETECTION,
    SECTION_LINE_SENSOR,
    SECTION_MOTORS,
    SECTION_SERVER,
    AutoConfig,
    load_config,
)
from teleop_web_lib.drive import Drive
from teleop_web_lib.line_sensor import build_line_sensor
from teleop_web_lib.motors import build_motors
from teleop_web_lib.web_server import make_handler, read_page

log = logging.getLogger("teleop_web")

DEFAULT_CONFIG = Path(__file__).with_name("teleop_web.yaml")
LOG_FORMAT = "%(asctime)s %(levelname)s %(message)s"
LOCAL_HOST = "127.0.0.1"
LOCAL_HOST_NAME = "localhost"


def parse_args() -> argparse.Namespace:
    """
    Command-line options.
    """
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument(
        "--fake",
        action="store_true",
        help="fake motors and test camera",
    )
    parser.add_argument(
        "--host",
        help="0.0.0.0 to open it from another machine on the network",
    )
    parser.add_argument("--port", type=int)
    return parser.parse_args()


def main() -> None:
    """
    Build the hardware, the control thread and the web server, and run until
    Ctrl+C.
    """
    args = parse_args()
    logging.basicConfig(level=logging.INFO, format=LOG_FORMAT)
    # SIGTERM (kill, systemd) also stops the motors before exiting.
    signal.signal(signal.SIGTERM, signal.default_int_handler)

    cfg = load_config(args.config)
    motor_cfg, camera_cfg, server_cfg = (
        cfg[SECTION_MOTORS],
        cfg[SECTION_CAMERA],
        cfg[SECTION_SERVER],
    )
    motors, motors_label = build_motors(motor_cfg, args.fake, args.config)
    line = build_line_sensor(
        cfg.get(SECTION_LINE_SENSOR), args.fake, args.config
    )

    drive = Drive(
        motors,
        motor_cfg["max_duty_ratio"],
        motor_cfg["cmd_timeout_s"],
        AutoConfig.from_dict(cfg.get(SECTION_AUTO)),
        line_sensor=line,
    )
    camera = Camera(
        camera_cfg,
        fake=args.fake,
        detection_overrides=cfg.get(SECTION_DETECTION),
    )
    drive.set_vision(camera)

    host = args.host or server_cfg["host"]
    port = args.port or server_cfg["port"]
    handler = make_handler(drive, camera, motors_label, read_page())
    server = ThreadingHTTPServer((host, port), handler)
    server.daemon_threads = True
    shown = LOCAL_HOST_NAME if host == LOCAL_HOST else host
    log.info("open http://%s:%d (Ctrl+C to quit)", shown, port)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        log.info("stopping and closing")
        server.server_close()
        camera.close()
        drive.close()
        if line is not None:
            line.close()


if __name__ == "__main__":
    main()
