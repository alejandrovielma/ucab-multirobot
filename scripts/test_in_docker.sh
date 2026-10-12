#!/usr/bin/env bash
# Build and test the whole workspace inside a ROS 2 Jazzy container.
# Usage (inside the container, repo mounted at /ws): ./scripts/test_in_docker.sh
# The sources are copied to a container-local workspace so build/, install/ and log/
# never land on the (possibly Windows) bind mount and --symlink-install works.
set -euo pipefail

REPO_DIR="${REPO_DIR:-/ws}"
WORK_DIR="${WORK_DIR:-/tmp/phineas_ws}"

# Webots-only modules cannot be imported without webots_ros2; they are byte-compiled instead.
COMPILE_ONLY_PACKAGES=("multirobot_sim")

if ! python3 -c "import cv2" 2>/dev/null; then
  echo "== OpenCV missing: installing test dependencies with apt"
  apt-get update -qq
  apt-get install -y -qq --no-install-recommends python3-opencv python3-numpy python3-pytest >/dev/null
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u

rm -rf "${WORK_DIR}"
mkdir -p "${WORK_DIR}"
cp -r "${REPO_DIR}/src" "${WORK_DIR}/src"
cd "${WORK_DIR}"

echo "== colcon build"
colcon build --symlink-install --event-handlers console_direct-

echo "== colcon test"
colcon test --event-handlers console_direct- --return-code-on-test-failure || true
colcon test-result --verbose

set +u
# shellcheck disable=SC1091
source install/setup.bash
set -u

echo "== import smoke test"
python3 - "${COMPILE_ONLY_PACKAGES[@]}" <<'PY'
import importlib
import pathlib
import py_compile
import sys

compile_only = set(sys.argv[1:])
failures = []
checked = 0
for setup_py in sorted(pathlib.Path("src").glob("*/setup.py")):
    pkg_dir = setup_py.parent
    pkg = pkg_dir.name
    module_root = pkg_dir / pkg
    if not module_root.is_dir():
        continue
    for path in sorted(module_root.rglob("*.py")):
        rel = path.relative_to(pkg_dir).with_suffix("")
        name = ".".join(rel.parts)
        if name.endswith(".__init__"):
            name = name[: -len(".__init__")]
        try:
            if pkg in compile_only:
                py_compile.compile(str(path), doraise=True)
            else:
                importlib.import_module(name)
            checked += 1
            print(f"ok   {name}")
        except Exception as exc:  # noqa: BLE001 - report every failure
            failures.append(f"{name}: {exc!r}")
            print(f"FAIL {name}: {exc!r}")

for launch_file in sorted(pathlib.Path("src").glob("*/launch/*.py")):
    try:
        py_compile.compile(str(launch_file), doraise=True)
        checked += 1
        print(f"ok   {launch_file}")
    except Exception as exc:  # noqa: BLE001
        failures.append(f"{launch_file}: {exc!r}")
        print(f"FAIL {launch_file}: {exc!r}")

print(f"smoke test: {checked} ok, {len(failures)} failed")
sys.exit(1 if failures else 0)
PY

echo "== all checks passed"
