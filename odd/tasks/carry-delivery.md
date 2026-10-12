# Feature: carry-delivery

## Objective
Close the autonomous loop of each collector (EXPLORE -> APPROACH -> CAPTURE -> CARRY -> deliver -> EXPLORE)
in Webots, and bring the APA 7 technical document up to date with the decisions validated in simulation.

## Problem
CARRY returned ZERO: robots stopped forever after capturing a cube. Two safety rules made delivery impossible:
the gate removes every negative vx (forward-only) and blocks forward motion over any black line, but the
delivery square is outlined in black tape and the fixed scoop only releases the cube by reversing.
A carrying robot that reached the border was frozen forever (no forward, no in-place turn).

## Decisions (with rationale)
- Delivery exit by **bounded reverse under the zone lease** (option 1 of docs/team-todo.md). Rejected: spinning
  in place to eject the cube (Webots: ejection happens at ~45 deg, direction not controllable, robot
  diagonal ~0.25 m vs 0.22 m square interior) and forward-arc exit (drags the cube with the U).
- The gate does not trust the behavior: the exception is granted from independent inputs only
  (own overhead pose inside the delivery corridor AND /zone/occupant == this robot).
- Square geometry measured from the Webots texture: interior x -0.239..-0.020, y -0.388..-0.180,
  north tape y -0.180..-0.164. Cube center sits ~0.155 m ahead of the robot center inside the U.
- Lease state comes from /zone/occupant (already subscribed); the FSM stays pure and only emits
  "request"/"release" actions that the node turns into service calls.
- Carrying robot with the line ahead: forced slow arc (carry_min_forward, carry_max_turn) instead of a
  permanent freeze. Tradeoff: the front may overrun the tape by a few cm.

## Scope
src/mobile_base (core delivery, behavior, safety, nodes, config, tests), src/bringup tests if needed,
docs (README, architecture, team-todo), technical document (.docx/.pdf at repo root).
Out of scope: overhead camera on real hardware, real camera node, HSV calibration, Webots 120 deg FOV.

## Tasks
- [ ] T1 Technical document v3: 3 cm cubes, 40x48 mm gray fixed U scoop, single front line sensor,
      forward-only avoidance, arc-only turns when carrying, delivery with bounded reverse under lease,
      real inventory. Route: delegated writer (docx generator in scratchpad, 2+ files).
- [ ] T2 Pure delivery logic in BehaviorFSM CARRY (goto entry, lease wait, enter, drop by reverse, release) + tests.
- [ ] T3 SafetyGate: delivery corridor exception (line + bounded reverse under lease) and carry-at-line arc + tests.
- [ ] T4 Nodes and config: behavior_node service calls and delivered count, gate holds_zone, YAML params.
- [ ] T5 Webots run: at least one full delivery observed with sim_monitor.
- [ ] T6 Docs: README status, architecture safety rules, team-todo task 1 closed.

## Acceptance criteria
- vx never negative while carrying (existing test keeps passing).
- Reverse only inside the corridor while holding the lease; never more than max_reverse.
- All Docker tests green.
- In Webots, a robot delivers a cube inside the square and the arbiter count increases.

## Checks
powershell -File scripts/test_in_docker.ps1 ; Webots run with tools/sim_monitor.py.

## Progress / evidence
(pending)
