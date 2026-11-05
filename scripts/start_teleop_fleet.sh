#!/bin/bash
# Start the teleop tool on the Pi with the fleet link: ROS 2 sourced,
# fleet_link next to it under /$ROBOT. Auto only uses the turns with
# fleet.enabled: true in tools/teleop_web.yaml.
# Usage: [ROBOT=phineas] [TELEOP_CMD="..."] scripts/start_teleop_fleet.sh
#   ROBOT       namespace and robot_name of fleet_link (default phineas)
#   TELEOP_CMD  how the tool starts (default python3 tools/teleop_web.py)
#   ROS_DOMAIN_ID defaults to 0
REPO=$(cd "$(dirname "$0")/.." && pwd)
ROBOT=${ROBOT:-phineas}
TELEOP_CMD=${TELEOP_CMD:-python3 tools/teleop_web.py}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
source /opt/ros/lyrical/setup.bash
source "$REPO/install/setup.bash"

# [x] keeps the pattern from matching this shell's own command line.
pkill -f 'python3.*[t]eleop_web.py'
pkill -f 'python3.*[r]un_teleop_(trace|mirror).py'
pkill -f '[f]leet_link'
sleep 1

prefix=$(ros2 pkg prefix mobile_base) || {
    echo "mobile_base not found: build and source install/setup.bash" >&2
    exit 1
}
cfg=$prefix/share/mobile_base/config/collector.yaml
[ -f "$cfg" ] || { echo "missing $cfg" >&2; exit 1; }
# --params-file BEFORE -p: collector.yaml sets robot_name: ferb and, after
# the -p, it would win.
nohup ros2 run mobile_base fleet_link --ros-args -r __ns:=/"$ROBOT" \
    --params-file "$cfg" -p robot_name:="$ROBOT" -p want_timeout_s:=3.0 \
    > /tmp/fleet_link.log 2>&1 < /dev/null &
disown

cd "$REPO"
nohup $TELEOP_CMD > /tmp/teleop_web.log 2>&1 < /dev/null &
disown
