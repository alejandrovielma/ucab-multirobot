#!/usr/bin/env bash
# Floor test of the robot motion without the SSH cable dragging it (a pulled cable makes it drift).
# Run it on the Pi after a manual `ssh`, unplug the cable during the countdown, plug it back in once
# the robot stops and read the log it names. The robot keeps going if the SSH session drops.
#
#   bash tools/lab/prueba_piso.sh [speed m/s] [delay s] [seconds per step]
#   bash tools/lab/prueba_piso.sh 0.2 15 2     # default: forward, strafe left, turn left
set -euo pipefail

cd "$(dirname "$0")/../.."
speed=${1:-0.2}
delay=${2:-15}
seconds=${3:-2}

# Two programs driving the HAT at once fight over the motors.
if pgrep -f "tools/lab/lab_robot.py" > /dev/null; then
    echo "ya hay un lab_robot.py corriendo (PID $(pgrep -f tools/lab/lab_robot.py | tr '\n' ' ')): terminarlo con kill <PID>" >&2
    exit 1
fi

log="prueba-piso-$(date +%Y%m%d-%H%M%S)-v${speed}.log"
distance=$(python3 -c "print(f'{$speed * $seconds * 100:.0f}')")
turn=$(python3 -c "import math; print(f'{math.degrees(10 * $speed * $seconds):.0f}')")

setsid nohup python3 tools/lab/lab_robot.py motors --only-robot --speed "$speed" --delay "$delay" \
    --seconds "$seconds" > "$log" 2>&1 < /dev/null &

cat <<EOF
Prueba en el piso a $speed m/s: empieza en $delay s (PID $!).
  1. Avanzar           ~$distance cm
  2. Lateral izquierda ~$distance cm
  3. Girar antihorario ~$turn grados
Dejar libres unos $((distance + 10)) cm al frente y a la izquierda: esta prueba no mira la linea negra.
Ahora se puede desconectar el cable. Al volver a conectarlo:
  cat ~/ucab-multirobot/$log
Para detenerlo antes de tiempo: kill $!
EOF
