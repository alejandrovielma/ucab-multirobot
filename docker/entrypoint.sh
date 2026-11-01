#!/usr/bin/env bash
# Sources ROS 2, builds the workspace mounted at /ws (incremental) and runs the command.
set -e
source /opt/ros/lyrical/setup.bash

if [ -d /ws/src ] && [ "${SKIP_BUILD:-0}" != "1" ]; then
  echo "[entrypoint] colcon build..."
  (cd /ws && colcon build --symlink-install --event-handlers console_direct- >/tmp/colcon_build.log 2>&1) \
    || { cat /tmp/colcon_build.log; echo "[entrypoint] colcon build failed"; exit 1; }
fi
if [ -f /ws/install/setup.bash ]; then
  source /ws/install/setup.bash
fi

exec "$@"
