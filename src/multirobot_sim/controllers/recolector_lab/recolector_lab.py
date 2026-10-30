"""Phineas in Webots WITHOUT ROS: the logic of the real robot's lab tools (tools/lab/lab_robot.py chase)
on Webots' rendered camera, line sensor and mecanum wheels.

Same pipeline and the same YAML: vision.py on the camera image, BehaviorFSM (mission), SafetyGate,
and the cartographer (particle filter + memory). Nothing here reads the simulator's ground truth: the
judge supervisor (controllers/juez) does that, for the metrics only.

Started as an extern controller by tools/sim/webots_run.py (with the project's Python, numpy, OpenCV).
"""

import argparse
import dataclasses
import json
import math
import sys
from pathlib import Path

import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO / "src" / "mobile_base"))

from controller import Robot  # noqa: E402

from mobile_base.core.behavior import BehaviorFSM, BehaviorInput, BehaviorParams, Cube, State  # noqa: E402
from mobile_base.core.cartographer import Cartographer  # noqa: E402
from mobile_base.core.colors import parse_colors, ranges_from_flat  # noqa: E402
from mobile_base.core.kinematics import MecanumGeometry, normalize, wheel_speeds  # noqa: E402
from mobile_base.core.localization import CameraModel, Localizer  # noqa: E402
from mobile_base.core.safety import SafetyGate, SafetyInput, SafetyParams  # noqa: E402
from mobile_base.vision import (VisionParams, detect_cubes, detect_refs, detect_square,  # noqa: E402
                                landmark_sightings, scoop_content)

WHEELS = ("fl", "fr", "rl", "rr")
MAX_WHEEL = 10.5 * 0.7  # rad/s: TT motor at ~6 V, the 2S battery cap of the real robot
LINE_THRESHOLD = 120.0  # simulated TCRT5000: ~240 on white, 0-30 on the tape
PERIOD_MS = 96  # control period: the camera's sampling period (6 basic steps of 16 ms)


def load_config(robot: str) -> dict:
    data = yaml.safe_load((REPO / "src" / "mobile_base" / "config" / f"{robot}.yaml").read_text())
    return {key.split("/")[-1]: value["ros__parameters"] for key, value in data.items()}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", default="phineas")
    ap.add_argument("--targets", default="red,orange")
    ap.add_argument("--avoid", default="yellow")
    ap.add_argument("--max-speed", type=float, default=0.10)
    ap.add_argument("--no-map", action="store_true")
    ap.add_argument("--log", required=True)
    ap.add_argument("--frames", default="", help="folder: save the camera view with the detections every 2 s")
    args = ap.parse_args()

    cfg = load_config(args.robot)
    det = dict(cfg["cube_detector"])
    vparams = VisionParams(hue_ranges=ranges_from_flat(det.pop("hue_ranges")),
                           **{k: v for k, v in det.items() if k in VisionParams.__dataclass_fields__})
    # The scoop of RecolectorUCAB.proto seen by its camera: the floor is visible from 0.134 m ahead of
    # the center, so a cube against the middle plate (center 0.14 m) only shows its top face at the
    # bottom of the image, cy 0.82-0.95 (at the wings' tips it is at ~0.69). The open scoop holds the
    # cube anywhere along its 6 cm middle plate: +-3 cm at 6.5 cm from the camera is x_half 0.46.
    # The YAML window is the real robot's (to be measured with its camera).
    vparams = dataclasses.replace(vparams, scoop_y_min=0.82, scoop_y_max=0.95, scoop_x_half=0.46)
    values = {k: v for k, v in cfg["behavior"].items() if k in BehaviorParams.__dataclass_fields__ and k != "my_colors"}
    values.update(assume_capture=True, has_gate=False, explore_speed=min(values.get("explore_speed", 0.12), args.max_speed),
                  approach_speed=min(values.get("approach_speed", 0.10), args.max_speed))
    targets, avoid = parse_colors(args.targets), parse_colors(args.avoid)
    fsm = BehaviorFSM(BehaviorParams(my_colors=targets, **values))
    fsm.set_mission(targets, avoid)
    gate = SafetyGate(SafetyParams(**{**{k: v for k, v in cfg["safety_gate"].items()
                                         if k in SafetyParams.__dataclass_fields__},
                                      "require_pose": False, "require_heartbeat": False}))
    # RecolectorUCAB.proto: camera 0.075 m ahead of the center, 0.11 m up, tilted 0.42 rad down, 640 x 480.
    camera = CameraModel.from_mount(1.60, 640, 480, 0.11, 0.42, 0.075)
    carto = None if args.no_map else Cartographer(Localizer(camera=camera))
    geom = MecanumGeometry()

    robot = Robot()
    step = int(robot.getBasicTimeStep())
    motors = [robot.getDevice(f"motor_{n}") for n in WHEELS]
    for m in motors:
        m.setPosition(math.inf)
        m.setVelocity(0.0)
    robot.getDevice("servo_pala").setPosition(0.0)
    cam = robot.getDevice("camara")
    cam.enable(PERIOD_MS)
    line = robot.getDevice("linea_frente")
    line.enable(step)
    width, height = cam.getWidth(), cam.getHeight()

    sent, sent_at, next_tick, next_frame = (0.0, 0.0, 0.0), 0.0, 0.0, 0.0
    if args.frames:
        Path(args.frames).mkdir(parents=True, exist_ok=True)
    with open(args.log, "w", encoding="utf-8") as log:
        while robot.step(step) != -1:
            now = robot.getTime()
            if now + 1e-9 < next_tick:
                continue
            next_tick = now + PERIOD_MS / 1000.0
            try:
                raw = cam.getImage()
            except ValueError:  # no frame yet
                raw = None
            if not raw:
                continue
            frame = np.frombuffer(raw, np.uint8).reshape(height, width, 4)[:, :, :3].copy()  # BGRA -> BGR
            dets = detect_cubes(frame, vparams)
            cubes = [Cube(d.color, d.cx_norm, d.cy_norm, d.area_px) for d in dets]
            square = detect_square(frame, vparams)
            refs = detect_refs(frame, vparams)
            scoop = scoop_content(frame, vparams)[0]
            black = line.getValue() < LINE_THRESHOLD
            where, est = {}, None
            if carto is not None:
                carto.moved(sent, now - sent_at)
                est = carto.observe(now, black, landmark_sightings(square, refs), cubes, fsm.targets,
                                    carrying=fsm.carrying)
                goal = carto.goal_for(now, fsm.targets)
                if est.localized:
                    where = dict(est_pose=(est.x, est.y, est.theta), explore_goal=goal, tape_ahead=carto.tape_ahead())
            want = fsm.step(BehaviorInput(
                **where, now=now, cubes=cubes, line_front=black, scoop_color=scoop,
                square_visible=square.visible, square_cx=square.cx_norm, square_cy=square.cy_norm,
                square_area=square.area_px, refs_visible=refs.visible, refs_cx=refs.center_cx, refs_skew=refs.skew))
            cmd, reason = gate.filter(SafetyInput(cmd=want, line_front=black, own_pose=None, peer_pose=None, now=now,
                                                   heartbeat_age=None, carrying=fsm.carrying,
                                                   delivering=fsm.state is State.DELIVER))
            for motor, speed in zip(motors, normalize(wheel_speeds(*cmd, geom), MAX_WHEEL)):
                motor.setVelocity(speed)
            sent, sent_at = cmd, now
            row = dict(t=round(now, 3), state=fsm.state.value, note=fsm.note, cmd=[round(v, 3) for v in cmd],
                       reason=reason, black=bool(black), scoop=scoop, delivered=fsm.delivered,
                       forbidden=fsm.forbidden_touches, cubes=[d.color for d in dets], square=square.visible,
                       refs=[n for n, _, _ in refs.seen])
            if est is not None:
                row["est"] = [round(est.x, 3), round(est.y, 3), round(est.theta, 3), round(est.spread, 3),
                              est.localized, round(est.scale, 3)]
            log.write(json.dumps(row) + "\n")
            if args.frames and now >= next_frame:
                import cv2

                next_frame = now + 2.0
                view = frame.copy()
                for d in dets:
                    u, v = int((d.cx_norm + 1.0) / 2.0 * width), int(d.cy_norm * height)
                    cv2.circle(view, (u, v), 6, (255, 255, 255), 2)
                    cv2.putText(view, f"{d.color} {d.area_px:.0f}", (u + 8, v), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                                (0, 0, 0), 1)
                cv2.putText(view, f"t={now:.1f} {fsm.state.value} {reason} refs={[n for n, _, _ in refs.seen]}",
                            (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
                cv2.imwrite(str(Path(args.frames) / f"f{now:07.1f}.jpg"), view)


if __name__ == "__main__":
    main()
