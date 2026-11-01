#!/bin/bash
source "/opt/ros/${ROS_DISTRO:-lyrical}/setup.bash"; source ~/ws/install/setup.bash
export WEBOTS_HOME="/mnt/c/Program Files/Webots"
S=${OUT:-/tmp/scoop_shapes}; T=$(dirname "$(readlink -f "$0")")
W=$(ros2 pkg prefix multirobot_sim)/share/multirobot_sim/worlds
mkdir -p $S/shapes
for spec in "A 0.040 0.040 0.048" "B 0.034 0.040 0.048" "C 0.032 0.040 0.048" "D 0.032 0.044 0.048" "E 0.034 0.040 0.060" "F 0.032 0.036 0.060" "G 0.034 0.034 0.060"; do
  set -- $spec
  python3 - "$W" "$1" "$2" "$3" "$4" <<'PY'
import sys
w, label, boca, fondo, prof = sys.argv[1:]
s = open(f"{w}/calibracion_pala.wbt").read()
extra = f'  palaBoca {boca}\n  palaFondoAncho {fondo}\n  palaProfundidad {prof}\n  compuerta FALSE\n'
s = s.replace('  name "Phineas"\n', '  name "Phineas"\n' + extra, 1).replace('  name "Ferb"\n', '  name "Ferb"\n  compuerta FALSE\n', 1)
open(f"{w}/forma_{label}.wbt", "w").write(s)
PY
  pkill -INT -f "ros2 launch bringup sim.launch.py"; sleep 4; pkill -f webots_ros2_driver; pkill -f ros2_supervisor
  /mnt/c/Windows/System32/taskkill.exe /IM webots-bin.exe /F > /dev/null 2>&1
  for i in $(seq 1 20); do /mnt/c/Windows/System32/tasklist.exe 2>/dev/null | grep -qi webots-bin || break; sleep 1; done
  sleep 3
  ros2 launch bringup sim.launch.py world:=forma_$1.wbt logic:=false > $S/shapes/launch_$1.log 2>&1 &
  for i in $(seq 1 40); do ros2 topic list 2>/dev/null | grep -q /phineas/camera/image_raw && break; sleep 3; done
  sleep 4
  python3 $T/scoop_shape_test.py "$1_boca${2}_fondo${3}_prof${4}" $S/shapes 2>&1 | grep RESULT
done
pkill -INT -f "ros2 launch bringup sim.launch.py"
