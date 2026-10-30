#!/usr/bin/env bash
# Mission buttons, camera view and manual WASD in the browser, without the SSH cable. Run it on the Pi after a
# manual `ssh`, unplug the cable during the countdown; it keeps running after the SSH session drops.
# The URL (with a random token) is written to the log; open it from a phone or laptop on the same network.
#
#   bash tools/lab/prueba_dashboard.sh [seconds] [delay s] [max speed m/s]
#   bash tools/lab/prueba_dashboard.sh 600 15 0.10
set -euo pipefail

cd "$(dirname "$0")/../.."
duration=${1:-600}
delay=${2:-15}
speed=${3:-0.10}
for value in "$speed" "$delay" "$duration"; do
    [[ $value =~ ^[0-9]+([.][0-9]+)?$ ]] || { echo "numero invalido: '$value'" >&2; exit 1; }
done

running=$(pgrep -f "^python3 tools/lab/lab_robot.py" | tr '\n' ' ' || true)
if [ -n "$running" ]; then
    echo "ya hay un lab_robot.py corriendo (PID $running): terminarlo con kill <PID>" >&2
    exit 1
fi

log="prueba-dashboard-$(date +%Y%m%d-%H%M%S).log"
setsid nohup bash -c 'sleep "$1"; exec timeout --signal=INT "$2" python3 -u tools/lab/lab_robot.py dashboard \
    --max-speed "$3"' _ "$delay" "$duration" "$speed" > "$log" 2>&1 < /dev/null &

cat <<MSG
Dashboard a $speed m/s durante $duration s; empieza en $delay s (PID $!).
Ahora se puede desconectar el cable. Despues de $delay s, la direccion aparece con:
  cat ~/ucab-multirobot/$log
Abrirla completa (con ?t=...) en el navegador: elegir colores (objetivo/prohibido) e INICIAR MISION, o
MODO MANUAL con WASD y Q/E (giran en el sitio). Esc = parada; sin mision el robot queda quieto.
Para detenerlo antes de tiempo: pkill -INT -f tools/lab/lab_robot.py
MSG
