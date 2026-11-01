#!/usr/bin/env bash
# Sets up the Raspberry Pi 5 (Ubuntu 26.04, ROS 2 Lyrical, lgpio, I2C, camera).
#
# Run from the repo root on the Pi: bash scripts/setup_pi.sh
# Then log out and in again (new groups) and reboot (I2C).
set -euo pipefail

repo_root=$(cd "$(dirname "$0")/.." && pwd)
config_txt=/boot/firmware/config.txt

bash "$repo_root/scripts/install_ros.sh"

# GPIO on the Pi 5 goes through lgpio (RPi.GPIO does not work); I2C for the
# Fusion HAT+ and the TCS34725; v4l-utils to check the camera.
sudo apt-get install -y \
  python3-gpiozero \
  python3-lgpio \
  python3-smbus2 \
  i2c-tools \
  v4l-utils \
  tmux
# Pi camera module (CSI). Not packaged on every Ubuntu release: optional.
sudo apt-get install -y python3-picamera2 \
  || echo "python3-picamera2 not available: use a USB webcam or install it by hand"

# Enable the I2C bus (needs a reboot).
if [ -f "$config_txt" ] && ! grep -qx "dtparam=i2c_arm=on" "$config_txt"; then
  echo "dtparam=i2c_arm=on" | sudo tee -a "$config_txt" >/dev/null
fi

# Access to GPIO, I2C and the camera without sudo.
for group in gpio i2c video dialout; do
  if getent group "$group" >/dev/null; then
    sudo usermod -aG "$group" "$USER"
  fi
done

cd "$repo_root"
# ROS setup scripts read unset variables.
set +u
source /opt/ros/lyrical/setup.bash
set -u
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install

echo "Done. Log out and in again, reboot, then check the HAT with scripts/test_hat.py."
