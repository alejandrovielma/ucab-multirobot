# Guide: behaviour

**Owner:** role 3 · **Files:** `src/mobile_base/mobile_base/{behavior_fsm,obstacle_guard,fleet_link,approach,fleet,peer_avoidance,speed_limits,states,topics}.py`, `src/recolector_interfaces/`, `docs/architecture.md`, `docs/external-interface.md`

## What you need to know

- The proposed nodes, topics and states are in `docs/architecture.md`. It is a starting point, not an obligation.
- Velocity always goes through `obstacle_guard` before the motor.
- Ultrasonic level idea (from the plan): >30 cm normal, 15–30 cm half speed, <15 cm stopped.
- The delivery area is blue and green paper filling a gap in the black line, between two upright traffic-light landmarks (green on the left, red on the right, seen from the arena). Black is always the border: if the line sensor sees black, the robot backs up, also while delivering (it hit the end of the line beside the gap, so it was misaligned and tries again).
- The scoop carries one cube at a time.
- The other robots belong to other teams; whatever is agreed with them goes in `docs/external-interface.md`.
- It is best to write the state machine as pure logic without `rclpy`, so it can be tested with `pytest`.
- A new parameter is a dataclass field plus a line in `config/collector.yaml` (see `recolector_common/params.py`); a new topic name goes in `topics.py` and a new state in `states.py`.
