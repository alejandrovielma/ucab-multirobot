#!/usr/bin/env bash
# Start power of the robot (fusion_start_power), measured with the cable unplugged: the four wheels
# ramp up together for each whole-robot motion (spins like a differential, forward, back, sideways)
# and the camera says when the robot starts to move. Run it on the Pi after a manual `ssh`, unplug
# the cable during the countdown, plug it back in once the robot is still for good, read the log.
#
#   bash tools/lab/prueba_umbral.sh [delay s] [max power %]
#   bash tools/lab/prueba_umbral.sh 15 45
set -euo pipefail

cd "$(dirname "$0")/../.."
delay=${1:-15}
max_power=${2:-45}
for value in "$delay" "$max_power"; do
    [[ $value =~ ^[0-9]+([.][0-9]+)?$ ]] || { echo "numero invalido: '$value'" >&2; exit 1; }
done

# Anchored: only a python3 process running the tool, not a shell whose command line mentions it.
running=$(pgrep -f "^python3 tools/lab/lab_robot.py" | tr '\n' ' ' || true)
if [ -n "$running" ]; then
    echo "ya hay un lab_robot.py corriendo (PID $running): terminarlo con kill <PID>" >&2
    exit 1
fi

log="prueba-umbral-$(date +%Y%m%d-%H%M%S).log"
setsid nohup python3 tools/lab/lab_robot.py threshold --delay "$delay" --max-power "$max_power" \
    > "$log" 2>&1 < /dev/null &

cat <<EOF2
Umbral de arranque: empieza en $delay s (PID $!).
Poner el robot en el centro de la pista, mirando hacia algo con detalle (la cinta, cubos, el borde de la
mesa): la camara decide cuando se mueve. Por cada movimiento (girar a cada lado, adelante, atras,
izquierda, derecha) la potencia sube de a 2 % hasta $max_power % y se detiene apenas se mueve o ve la linea.
Se desplaza pocos centimetros; tarda unos 2 minutos.
Ahora se puede desconectar el cable. Al volver a conectarlo:
  cat ~/ucab-multirobot/$log
Para detenerlo antes de tiempo: kill $!
EOF2
