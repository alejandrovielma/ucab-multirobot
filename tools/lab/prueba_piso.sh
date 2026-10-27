#!/usr/bin/env bash
# Floor test of the robot motion without the SSH cable dragging it (a pulled cable makes it drift).
# Run it on the Pi after a manual `ssh`, unplug the cable during the countdown, plug it back in once
# the robot stops and read the log it names. The robot keeps going if the SSH session drops.
#
#   bash tools/lab/prueba_piso.sh [speed m/s] [delay s] [seconds per step] [--star]
#   bash tools/lab/prueba_piso.sh 0.2 15 2           # forward, strafe left, turn left
#   bash tools/lab/prueba_piso.sh 0.1 15 1.5 --star  # 8 directions and both spins, in return pairs
set -euo pipefail

cd "$(dirname "$0")/../.."
speed=${1:-0.2}
delay=${2:-15}
seconds=${3:-2}
shift $(($# < 3 ? $# : 3))

# Two programs driving the HAT at once fight over the motors.
# Anchored: only a python3 process running the tool, not a shell whose command line mentions it.
running=$(pgrep -f "^python3 tools/lab/lab_robot.py" | tr '\n' ' ' || true)
if [ -n "$running" ]; then
    echo "ya hay un lab_robot.py corriendo (PID $running): terminarlo con kill <PID>" >&2
    exit 1
fi

star=false
for arg in "$@"; do [ "$arg" = "--star" ] && star=true; done
log="prueba-piso-$(date +%Y%m%d-%H%M%S)-v${speed}$($star && echo -estrella || true).log"
distance=$(python3 -c "print(f'{$speed * $seconds * 100:.0f}')")

setsid nohup python3 tools/lab/lab_robot.py motors --only-robot --speed "$speed" --delay "$delay" \
    --seconds "$seconds" "$@" > "$log" 2>&1 < /dev/null &

echo "Prueba en el piso a $speed m/s: empieza en $delay s (PID $!)."
if $star; then
    cat <<EOF
  Adelante y atras, izquierda y derecha, las cuatro diagonales (ida y vuelta), ~$distance cm cada tramo;
  despues 90 grados antihorario y 90 grados horario. Debe terminar donde empezo y con el mismo rumbo:
  anotar cuanto se desplazo y cuanto giro de mas o de menos.
Dejar libres unos $((distance + 10)) cm alrededor: esta prueba no mira la linea negra.
EOF
else
    turn=$(python3 -c "import math; print(f'{math.degrees(10 * $speed * $seconds):.0f}')")
    cat <<EOF
  1. Avanzar           ~$distance cm
  2. Lateral izquierda ~$distance cm
  3. Girar antihorario ~$turn grados
Dejar libres unos $((distance + 10)) cm al frente y a la izquierda: esta prueba no mira la linea negra.
EOF
fi
cat <<EOF
Ahora se puede desconectar el cable. Al volver a conectarlo:
  cat ~/ucab-multirobot/$log
Para detenerlo antes de tiempo: kill $!
EOF
