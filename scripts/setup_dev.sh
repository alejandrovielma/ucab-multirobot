#!/usr/bin/env bash
# Sets up a laptop with Ubuntu 26.04 or WSL2 for development (ROS 2 Lyrical, colcon, ruff, Webots).
#
# Run from the repo root: bash scripts/setup_dev.sh
# Pass --no-webots to skip the simulator (amd64 only, ~1 GB).
set -euo pipefail

repo_root=$(cd "$(dirname "$0")/.." && pwd)
# Same version as docker/Dockerfile.
WEBOTS_VERSION=R2025a
WEBOTS_DEB=webots_2025a_amd64.deb

bash "$repo_root/scripts/install_ros.sh"

# ruff, same as CI.
sudo apt-get install -y pipx
pipx install ruff || pipx upgrade ruff
pipx ensurepath

if [ "${1:-}" != "--no-webots" ] && ! command -v webots >/dev/null; then
  curl -fL -o "/tmp/$WEBOTS_DEB" \
    "https://github.com/cyberbotics/webots/releases/download/${WEBOTS_VERSION}/${WEBOTS_DEB}"
  sudo apt-get install -y "/tmp/$WEBOTS_DEB" xvfb
  rm -f "/tmp/$WEBOTS_DEB"
  line="export WEBOTS_HOME=/usr/local/webots"
  grep -qxF "$line" ~/.bashrc || echo "$line" >> ~/.bashrc
fi

cd "$repo_root"
# ROS setup scripts read unset variables.
set +u
source /opt/ros/lyrical/setup.bash
set -u
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install

echo "Done. Open a new terminal, then: source install/setup.bash"
