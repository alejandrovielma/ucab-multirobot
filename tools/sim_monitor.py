#!/usr/bin/env python3
"""Compact live monitor of both robots for debugging runs in Webots or on the track.

Usage (after sourcing ROS 2 and the workspace):
    python3 tools/sim_monitor.py --duration 120 --out /tmp/monitor [--frames-every 5]

Writes one line every --period seconds (wall time) to stdout and <out>/monitor.log:
sim time, per robot state, pose, commanded velocity, front line flag and the closest cube
of its own colors (cx_norm, cy_norm), cubes delivered, plus the distance between the robots and
the holder of the delivery-square lease. Optionally saves
the camera frames of --frames-robots (default phineas) as PNG every --frames-every seconds.
"""

import argparse
import math
import os
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from multirobot_interfaces.msg import CubeDetectionArray, RobotPose, RobotState

ROBOTS = {"phineas": {"red", "orange", "yellow"}, "ferb": {"green", "blue", "purple"}}


class SimMonitor(Node):
    def __init__(self, out_dir: str, frames_every: float, frames_robots: list[str]):
        super().__init__("sim_monitor")
        self.out_dir = out_dir
        self.frames_every = frames_every
        self.sim_time = 0.0
        self.data = {r: {"state": "-", "pose": None, "cmd": None, "line": False, "cube": None, "delivered": 0, "scoop": ""} for r in ROBOTS}
        self.last_frame = {robot: 0.0 for robot in frames_robots}
        self.occupant = ""
        self.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.create_subscription(String, "/zone/occupant", self._on_occupant, 10)
        for robot in ROBOTS:
            self.create_subscription(RobotState, f"/{robot}/state", self._state_cb(robot), 10)
            self.create_subscription(RobotPose, f"/{robot}/pose", self._setter(robot, "pose", lambda m: (m.x, m.y, m.theta)), 10)
            self.create_subscription(Twist, f"/{robot}/cmd_vel", self._setter(robot, "cmd", lambda m: (m.linear.x, m.linear.y, m.angular.z)), 10)
            self.create_subscription(Bool, f"/{robot}/line_front", self._setter(robot, "line", lambda m: m.data), 10)
            self.create_subscription(CubeDetectionArray, f"/{robot}/cubes", self._cubes_cb(robot), 10)
        if frames_every > 0:
            for robot in frames_robots:
                self.create_subscription(Image, f"/{robot}/camera/image_raw", self._image_cb(robot), 1)

    def _on_clock(self, msg: Clock):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _on_occupant(self, msg: String):
        self.occupant = msg.data

    def _state_cb(self, robot):
        def cb(msg: RobotState):
            self.data[robot]["state"] = msg.state
            self.data[robot]["delivered"] = msg.delivered
        return cb

    def _setter(self, robot, key, extract):
        def cb(msg):
            self.data[robot][key] = extract(msg)
        return cb

    def _cubes_cb(self, robot):
        def cb(msg: CubeDetectionArray):
            mine = [c for c in msg.cubes if c.color in ROBOTS[robot]]
            best = max(mine, key=lambda c: c.area_px, default=None)
            self.data[robot]["cube"] = None if best is None else (best.color, best.cx_norm, best.cy_norm)
            self.data[robot]["scoop"] = msg.scoop_color
        return cb

    def _image_cb(self, robot):
        def cb(msg: Image):
            self._on_image(robot, msg)
        return cb

    def _on_image(self, robot: str, msg: Image):
        now = time.monotonic()
        if now - self.last_frame[robot] < self.frames_every:
            return
        self.last_frame[robot] = now
        import cv2
        import numpy as np

        channels = 4 if msg.encoding == "bgra8" else 3
        img = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.width, channels)
        cv2.imwrite(os.path.join(self.out_dir, f"{robot}_cam_{self.sim_time:07.1f}.png"), img[:, :, :3])

    def line(self) -> str:
        parts = [f"t={self.sim_time:6.1f}"]
        for robot, d in self.data.items():
            pose = "pose=----" if d["pose"] is None else f"pose=({d['pose'][0]:+.2f},{d['pose'][1]:+.2f},{math.degrees(d['pose'][2]):+4.0f})"
            cmd = "cmd=----" if d["cmd"] is None else f"cmd=({d['cmd'][0]:+.2f},{d['cmd'][1]:+.2f},{d['cmd'][2]:+.2f})"
            cube = "cube=-" if d["cube"] is None else f"cube={d['cube'][0]}@({d['cube'][1]:+.2f},{d['cube'][2]:.2f})"
            parts.append(f"| {robot[0].upper()} {d['state']:<8} {pose} {cmd} line={int(d['line'])} {cube} scoop={d['scoop'] or '-'} n={d['delivered']}")
        p, f = self.data["phineas"]["pose"], self.data["ferb"]["pose"]
        if p and f:
            parts.append(f"| dist={math.hypot(p[0] - f[0], p[1] - f[1]):.2f}")
        parts.append(f"| zone={self.occupant or '-'}")
        return " ".join(parts)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--period", type=float, default=0.5)
    parser.add_argument("--out", default="/tmp/monitor")
    parser.add_argument("--frames-every", type=float, default=0.0)
    parser.add_argument("--frames-robots", default="phineas", help="comma separated, e.g. phineas,ferb")
    args = parser.parse_args()
    os.makedirs(args.out, exist_ok=True)

    rclpy.init()
    node = SimMonitor(args.out, args.frames_every, args.frames_robots.split(","))
    end = time.monotonic() + args.duration
    next_line = time.monotonic()
    with open(os.path.join(args.out, "monitor.log"), "w") as log:
        while rclpy.ok() and time.monotonic() < end:
            # spin_once runs a single callback; drain the queue so fast topics do not starve slow ones.
            rclpy.spin_once(node, timeout_sec=0.02)
            for _ in range(30):
                rclpy.spin_once(node, timeout_sec=0.0)
            if time.monotonic() >= next_line:
                next_line += args.period
                text = node.line()
                print(text, flush=True)
                log.write(text + "\n")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
