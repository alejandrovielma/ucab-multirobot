#!/usr/bin/env bash
# Installs ROS 2 Lyrical (ros-base) and the build tools on Ubuntu 26.04.
# Official steps, as in docs/ssh-deployment.md (phase 1). Used by setup_pi.sh and setup_dev.sh.
set -euo pipefail

ROS_DISTRO=lyrical

sudo apt-get update
sudo apt-get install -y software-properties-common curl git
sudo add-apt-repository -y universe

if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
  version=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | grep -F '"tag_name"' | awk -F'"' '{print $4}')
  codename=$(. /etc/os-release && echo "$VERSION_CODENAME")
  curl -fL -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${version}/ros2-apt-source_${version}.${codename}_all.deb"
  sudo dpkg -i /tmp/ros2-apt-source.deb
  rm -f /tmp/ros2-apt-source.deb
fi

sudo apt-get update
sudo apt-get install -y \
  "ros-${ROS_DISTRO}-ros-base" \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-opencv \
  python3-numpy \
  python3-yaml \
  python3-pytest

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update

line="source /opt/ros/${ROS_DISTRO}/setup.bash"
grep -qxF "$line" ~/.bashrc || echo "$line" >> ~/.bashrc
