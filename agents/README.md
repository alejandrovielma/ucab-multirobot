# agents/

Short guides so an AI assistant (or someone new) can understand an area without reading the whole repo. `AGENTS.md` at the root has the general rules and `docs/code-style.md` the code style (constants, multi-line layout, docstrings, modularity).

| File | Area | Owner |
|---|---|---|
| `mobile-base.md` | motors, HAT, line, ultrasonic, scoop | role 1 |
| `vision.md` | camera, HSV, colour sensor | role 2 |
| `behavior.md` | state machine, safety, messages | role 3 |
| `simulation-integration.md` | Webots, launch files, CI | role 4 |
| `reviewer.md` | checklist for reviewing a PR | everyone |

Usage:
- **Claude Code** reads `CLAUDE.md` → `AGENTS.md` on its own. For a task: "read `agents/vision.md` and help me with …".
- **Copilot, Cursor or Codex:** open the guide in the editor or paste it at the start of the chat.
- **Reviewing a PR:** "review this diff with `agents/reviewer.md`".

Each owner maintains their guide: whatever is learned the hard way (a pin, a threshold, a failure) goes here.
