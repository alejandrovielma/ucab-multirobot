"""Structural checks of the launch files (they are executed for real in scripts/test_in_docker.sh)."""

import ast
import pathlib

LAUNCH_DIR = pathlib.Path(__file__).resolve().parents[1] / "launch"
EXPECTED = {"robot.launch.py", "robot_logic.launch.py", "sim.launch.py", "perry.launch.py"}


def test_expected_launch_files_exist():
    present = {p.name for p in LAUNCH_DIR.glob("*.launch.py")}
    assert EXPECTED <= present


def test_every_launch_file_defines_generate_launch_description():
    for path in LAUNCH_DIR.glob("*.launch.py"):
        tree = ast.parse(path.read_text())
        names = {n.name for n in tree.body if isinstance(n, ast.FunctionDef)}
        assert "generate_launch_description" in names, path.name
