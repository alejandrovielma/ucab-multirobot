# How to contribute

## Workflow

1. Take or create an issue with the "Task" template.
2. Create a branch from `main`: `feature/<area>-<topic>`, `fix/<area>-<topic>` or `docs/<topic>`.
3. Small commits with the format `area: what`, in English:
   - `mobile_base: brake if cmd_vel stops arriving`
   - `vision: HSV range for purple`

   Areas: `interfaces`, `mobile_base`, `vision`, `bringup`, `simulation`, `tools`, `scripts`, `docs`, `ci`.
4. Before the PR: `ruff check` and `ruff format --check` (commands in `docs/code-style.md`), `colcon build`, `colcon test` and `python3 -m pytest tools`.
5. Open the PR against `main` with the template filled in and request review from the area owner (`docs/team.md`).
6. It is squash-merged when CI is green and someone other than the author approved it.

## Rules

- `main` is protected.
- One PR, one change. If it goes over ~400 lines, split it.
- If you change a topic or message, update `docs/architecture.md` in the same PR.
- New parameters go in YAML, with the unit in the name, and are declared as a dataclass field (no hardcoded values: see `docs/code-style.md`).
- Code style (constants, 80 columns and multi-line layout, docstring format, modularity): `docs/code-style.md`. `ruff` checks most of it in CI.
- Everything technical in English: code, comments, logs and documentation.
- If you used an AI assistant, review the code yourself: the PR author answers for it.
