# PR review checklist

1. Does it do what the description says?
2. **Safety:** does anything publish to `cmd_vel` bypassing `obstacle_guard`? Does it still brake without commands? Was a speed raised without measuring?
3. **Topics:** if a topic or message changed, was `docs/architecture.md` updated?
4. **Parameters:** are there magic numbers that should go in the YAML, with the unit in the name?
5. **Tests:** does the new logic have a test? Do `colcon build` and `colcon test` pass?
6. **Laptop:** does the node run without the real hardware?
7. **Style:** everything technical in English, `ruff` clean, commits `area: what`. Check it against `docs/code-style.md`: docstrings in the repo format, no magic numbers or hardcoded names, 80 columns with the vertical layout, thin nodes, one responsibility per module and function.
8. **Extra files:** no `build/`, `install/`, `log/`, heavy photos or credentials.
