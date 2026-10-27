#!/usr/bin/env bash
# Autonomous run with a mission, without the SSH cable: explore, chase the target colors, avoid the
# rest (star motion: translate or spin in place). Run it on the Pi after a manual `ssh`, unplug the
# cable during the countdown; the run keeps going after the SSH session drops. The camera view with
# the state is at http://<robot-ip>:8080/ when the robot is on the network.
#
#   bash tools/lab/prueba_mision.sh <target colors> [forbidden colors] [seconds] [delay s] [max speed m/s]
#   bash tools/lab/prueba_mision.sh blue red 60 15 0.08
# Colors are comma separated: red, orange, yellow, green, blue, purple. Without the overhead camera
# there is no pose: the robot stops at the scoop mouth (CAPTURE), it does not carry the cube yet.
set -euo pipefail

cd "$(dirname "$0")/../.."
target=${1:?"falta el color objetivo, por ejemplo: bash tools/lab/prueba_mision.sh blue red"}
avoid=${2:-}
duration=${3:-60}
delay=${4:-15}
speed=${5:-0.08}

if pgrep -f "tools/lab/lab_robot.py" > /dev/null; then
    echo "ya hay un lab_robot.py corriendo (PID $(pgrep -f tools/lab/lab_robot.py | tr '\n' ' ')): terminarlo con kill <PID>" >&2
    exit 1
fi
# Bad color names fail here, before the countdown, instead of in the background.
python3 -c "import sys; sys.path.insert(0, 'src/mobile_base'); from mobile_base.core.colors import parse_colors; parse_colors('$target'); parse_colors('$avoid')"

log="prueba-mision-$(date +%Y%m%d-%H%M%S)-${target//,/_}.log"
setsid nohup bash -c "sleep $delay; exec timeout --signal=INT $duration python3 tools/lab/lab_robot.py chase \
    --target '$target' --avoid '$avoid' --max-speed $speed" > "$log" 2>&1 < /dev/null &

cat <<EOF
Mision: buscar [$target], evitar [${avoid:-ninguno}], a $speed m/s durante $duration s; empieza en $delay s (PID $!).
Poner cubos del color objetivo y de los prohibidos en la pista. Esta prueba si mira la linea negra.
Anotar: si gira en el sitio para centrar el cubo y luego avanza recto, si esquiva los prohibidos,
si alguna vez avanza y gira a la vez (no deberia).
Ahora se puede desconectar el cable. Al volver a conectarlo:
  cat ~/ucab-multirobot/$log
Para detenerlo antes de tiempo: pkill -INT -f tools/lab/lab_robot.py
EOF
