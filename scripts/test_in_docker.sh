#!/usr/bin/env bash
# Build and test the whole workspace inside a ROS 2 Lyrical container (ROS_DISTRO selects the install).
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
source "/opt/ros/${ROS_DISTRO:-lyrical}/setup.bash"
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
import importlib.util
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
        if "webots_ros2_driver" not in launch_file.read_text():
            spec = importlib.util.spec_from_file_location(launch_file.stem.replace(".", "_"), launch_file)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            module.generate_launch_description()
        checked += 1
        print(f"ok   {launch_file}")
    except Exception as exc:  # noqa: BLE001
        failures.append(f"{launch_file}: {exc!r}")
        print(f"FAIL {launch_file}: {exc!r}")

print(f"smoke test: {checked} ok, {len(failures)} failed")
sys.exit(1 if failures else 0)
PY

# Start each real-robot stack for a few seconds without hardware (lgpio absent -> GPIO dry run,
# no camera -> logged error). Any node crash fails the check.
for robot in phineas ferb; do
  echo "== launch smoke test: robot.launch.py robot:=${robot}"
  log_file="/tmp/launch_${robot}.log"
  set +e
  timeout --foreground -s INT 6 ros2 launch bringup robot.launch.py "robot:=${robot}" >"${log_file}" 2>&1
  set -e
  # Only the window before the Ctrl-C is judged: a crash there is a real startup failure.
  if sed '/user interrupted/q' "${log_file}" | grep -qE "process has died|Traceback"; then
    tail -n 40 "${log_file}"
    echo "FAIL launch ${robot}"
    exit 1
  fi
  started=$(grep -c "process started" "${log_file}" || true)
  echo "ok   ${started} processes started and stayed alive"
done

echo "== all checks passed"
