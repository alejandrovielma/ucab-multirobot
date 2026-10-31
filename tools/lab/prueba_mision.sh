#!/usr/bin/env bash
# Autonomous run with a mission, without the SSH cable: explore, chase the target colors, avoid the
# rest (star motion: translate or spin in place). Run it on the Pi after a manual `ssh`, unplug the
# cable during the countdown; the run keeps going after the SSH session drops. The camera view with
# the state is at http://<robot-ip>:8080/ when the robot is on the network.
#
#   bash tools/lab/prueba_mision.sh <target colors> [forbidden colors] [seconds] [delay s] [max speed m/s]
#   bash tools/lab/prueba_mision.sh blue red 60 15 0.05
# Colors are comma separated: red, orange, yellow, green, blue, purple. Without the overhead camera
# there is no pose: the robot stops at the scoop mouth (CAPTURE), it does not carry the cube yet.
set -euo pipefail

cd "$(dirname "$0")/../.."
target=${1:?"falta el color objetivo, por ejemplo: bash tools/lab/prueba_mision.sh blue red"}
avoid=${2:-}
duration=${3:-60}
delay=${4:-15}
speed=${5:-0.05}
for value in "$speed" "$delay" "$duration"; do
    [[ $value =~ ^[0-9]+([.][0-9]+)?$ ]] || { echo "numero invalido: '$value'" >&2; exit 1; }
done

# Anchored: only a python3 process running the tool, not a shell whose command line mentions it.
running=$(pgrep -f "^python3 tools/lab/lab_robot.py" | tr '\n' ' ' || true)
if [ -n "$running" ]; then
    echo "ya hay un lab_robot.py corriendo (PID $running): terminarlo con kill <PID>" >&2
    exit 1
fi
# Bad color names fail here, before the countdown, instead of in the background. The colors travel as
# arguments, never pasted into code (a quote in them would inject it).
python3 -c 'import sys; sys.path.insert(0, "src/mobile_base"); from mobile_base.core.colors import parse_colors
for text in sys.argv[1:]: parse_colors(text)' "$target" "$avoid"

log="prueba-mision-$(date +%Y%m%d-%H%M%S)-${target//,/_}.log"
setsid nohup bash -c 'sleep "$1"; exec timeout --signal=INT "$2" python3 tools/lab/lab_robot.py chase \
    --target "$3" --avoid "$4" --max-speed "$5"' _ "$delay" "$duration" "$target" "$avoid" "$speed" \
    > "$log" 2>&1 < /dev/null &

cat <<EOF
Mision: buscar [$target], evitar [${avoid:-ninguno}], a $speed m/s durante $duration s; empieza en $delay s (PID $!).
Poner cubos del color objetivo y de los prohibidos en la pista. Esta prueba si mira la linea negra.
Anotar: si gira en el sitio para centrar el cubo y luego avanza recto, si esquiva los prohibidos,
si alguna vez avanza y gira a la vez (no deberia).
Ahora se puede desconectar el cable. Al volver a conectarlo:
  cat ~/ucab-multirobot/$log
Para detenerlo antes de tiempo: pkill -INT -f tools/lab/lab_robot.py
EOF
