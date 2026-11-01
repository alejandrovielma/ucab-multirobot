"""Judge: a supervisor that only WATCHES. It logs the robot's true pose and every cube's position every
100 ms (for the metrics of tools/sim/webots_run.py) and ends the simulation after --seconds. Nothing it
reads reaches the robot's logic."""

import argparse
import json
import math

from controller import Supervisor


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=240.0)
    ap.add_argument("--log", required=True)
    ap.add_argument("--robot-def", default="PHINEAS")
    ap.add_argument("--movie", default="")
    args = ap.parse_args()

    sup = Supervisor()
    step = int(sup.getBasicTimeStep())
    robot = sup.getFromDef(args.robot_def)
    children = sup.getRoot().getField("children")
    cubes = []
    for i in range(children.getCount()):
        node = children.getMFNode(i)
        if node.getTypeName() == "CuboMadera":
            cubes.append((node.getField("name").getSFString(), node))
    if args.movie:
        sup.movieStartRecording(args.movie, 1280, 720, 0, 90, 4, False)  # 4x speed
    next_log = 0.0
    with open(args.log, "w", encoding="utf-8") as log:
        while sup.step(step) != -1:
            now = sup.getTime()
            if now + 1e-9 >= next_log:
                next_log = now + 0.1
                x, y, _ = robot.getPosition()
                r = robot.getOrientation()
                row = dict(t=round(now, 3), robot=[round(x, 4), round(y, 4), round(math.atan2(r[3], r[0]), 4)],
                           cubes=[[name, round(n.getPosition()[0], 4), round(n.getPosition()[1], 4)] for name, n in cubes])
                log.write(json.dumps(row) + "\n")
            if now >= args.seconds:
                if args.movie:
                    sup.movieStopRecording()
                    while not sup.movieIsReady():
                        sup.step(step)
                sup.simulationQuit(0)
                break


if __name__ == "__main__":
    main()
