# Getting the code onto the robot over SSH

Plan for taking what currently runs in simulation (vision + approaching a cube) to the
robot's Raspberry Pi 5, working from the laptop over SSH. It goes in phases: each phase is
validated before moving to the next (team rule: nothing is tested on the robot without first
going through the simulation).

In the examples the Pi is called `phineas` and the user is `ucab`; replace them with yours.

## Phase 0. Pi ready for SSH access

1. With **Raspberry Pi Imager**, flash **Ubuntu Server 26.04 LTS (64-bit)** to the microSD.
   In the options (⚙):
   - hostname: `phineas`
   - username and password: `ucab` / (whatever you like)
   - Wi‑Fi: the team hotspot (see `docs/external-interface.md`)
   - enable SSH
2. Power on the Pi and find it from the laptop (same network):

   ```bash
   ping phineas.local           # if it does not answer, look up the IP in the hotspot's list
   ssh ucab@phineas.local
   ```

3. Log in with a key instead of a password (on the laptop, once):

   ```bash
   # Linux / WSL / macOS
   ssh-keygen -t ed25519
   ssh-copy-id ucab@phineas.local
   ```

   ```powershell
   # Windows PowerShell (ships with OpenSSH)
   ssh-keygen -t ed25519
   type $env:USERPROFILE\.ssh\id_ed25519.pub | ssh ucab@phineas.local "mkdir -p ~/.ssh && cat >> ~/.ssh/authorized_keys"
   ```

4. Shortcut in the laptop's `~/.ssh/config`:

   ```
   Host phineas
       HostName phineas.local      # or the fixed IP
       User ucab
   ```

   From now on `ssh phineas` is enough.

**Done when:** `ssh phineas` logs in without asking for a password.

## Phase 1. Install ROS 2 Lyrical on the Pi (native)

On the Pi it is installed natively (no Docker): less memory and direct access to the camera and GPIO.
These are the official ROS 2 Lyrical steps for Ubuntu 26.04 (arm64), base version without desktop:

```bash
ssh phineas
sudo apt update && sudo apt install -y software-properties-common curl git tmux
sudo add-apt-repository -y universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update && sudo apt install -y ros-lyrical-ros-base python3-colcon-common-extensions \
    python3-rosdep python3-opencv python3-numpy v4l-utils
sudo rosdep init && rosdep update
echo "source /opt/ros/lyrical/setup.bash" >> ~/.bashrc
```

> If the official steps change, the ROS 2 Lyrical guide takes precedence
> (docs.ros.org → Installation → Ubuntu deb packages). Once role 1 fills in
> `scripts/setup_pi.sh`, these commands move into that script.

**Done when:** `ros2 doctor --report | head` runs without errors.

## Phase 2. Getting the code onto the Pi

There are two ways; use **A** for what is already on GitHub and **B** to try changes that
are not committed yet.

**A. Clone on the Pi** (the normal way):

```bash
ssh phineas
git clone https://github.com/notoriussss/Phineas-Recolector.git ~/recolector
cd ~/recolector && git checkout develop      # or the branch you want to test
# to update later:  git pull
```

**B. Copy from the laptop with rsync** (uncommitted local changes):

```bash
# from the repo root on the laptop (Linux or WSL)
rsync -avz --delete --exclude build --exclude install --exclude log --exclude .git \
    ./ phineas:~/recolector/
```

Build on the Pi (the `recolector_sim` package builds as well, but it is only used on the laptop):

```bash
ssh phineas
cd ~/recolector
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
echo "source ~/recolector/install/setup.bash" >> ~/.bashrc
```

**Done when:** `ros2 pkg executables vision` lists `camera_node`, `cube_detector` and
`web_viewer`.

## Phase 3. Camera and viewer (the robot does not move)

1. Connect the camera and check that the system sees it:

   ```bash
   v4l2-ctl --list-devices        # a USB webcam shows up as /dev/video0
   ```

   - **USB webcam:** works directly with `device:=0`.
   - **Pi camera (OV5647, CSI):** on the Pi 5 it goes through libcamera, and OpenCV cannot open it
     as `/dev/videoN`. `camera_node` accepts a GStreamer pipeline in `device`
     (for example with `libcamerasrc`), but **it is untested**: that is a role 2 task.
     To start, use a USB webcam.

2. Launch only the vision inside `tmux` (so it keeps running if SSH drops):

   ```bash
   ssh phineas
   tmux new -s robot
   ros2 launch bringup vision.launch.py device:=0
   # leave tmux without stopping it: Ctrl+B then D.  Come back: tmux attach -t robot
   ```

3. Watch the camera from the laptop. Two options:
   - same network: open `http://phineas.local:8080`
   - through an SSH tunnel (works even if the port is closed):

     ```bash
     ssh -N -L 8080:localhost:8080 phineas
     ```

     and open `http://localhost:8080`.

4. Put cubes in front of the robot and check that the boxes come out with the right colour.

**Done when:** the viewer marks all 6 colours correctly under the lab lighting.

## Phase 4. Calibrate with the real robot

The HSV ranges and camera geometry in the YAML come from the **simulation**.

1. **Colours.** Change a range live and watch the viewer update instantly:

   ```bash
   ros2 param set /cube_detector hsv.red "[0, 110, 100, 6, 255, 255, 173, 110, 100, 179, 255, 255]"
   ```

   Once it looks right, copy the numbers into `src/vision/config/hsv_ranges.yaml` (on the laptop,
   with a commit). The known risk is purple/maroon versus red.
2. **Camera.** Measure the camera's height above the floor, its tilt and its field of
   view, and put them in `camera.*` in the same YAML. Check with a ruler: place a cube at
   20 cm and make sure the viewer says ~20 cm.
3. **Stopping distance.** Tune `stop_distance_m` in `src/mobile_base/config/collector.yaml`
   so the cube ends up right in front of the scoop.

## Phase 5. Make it drive to the cube

This needs what does not exist on the real robot yet:

- `motor_driver` (role 1): `cmd_vel` → PWM for the 4 motors, braking if no commands arrive.
- `ultrasonic` (role 1): without `range`, `obstacle_guard` **does not allow driving forward** (only turning).

Once they exist, they are added to `bringup/launch/collector.launch.py` and tested like this:

1. Wheels in the air (robot on top of a box).
2. `ros2 launch bringup collector.launch.py` → starts **paused**.
3. In the web viewer, choose the colours and press **Start**. Check that the wheels turn towards
   the cube; **Pause** stops everything.
4. On the floor, with a hand ready on the battery switch.

## Network notes

- All the robots use the same hotspot: set your own `ROS_DOMAIN_ID` on the Pi
  (`echo "export ROS_DOMAIN_ID=<n>" >> ~/.bashrc`) so topics do not mix with the other
  teams'. The number is agreed in `docs/external-interface.md`.
- Viewing the camera does not need ROS on the laptop: a browser is enough (the viewer is HTTP).
- If the hotspot's multicast fails and you only need the viewer, on the Pi you can use
  `export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` (all nodes run on the Pi).

## Later

- A `systemd` service so `collector.launch.py` starts by itself when the Pi powers on.
- A `scripts/deploy.sh` script with the `rsync` + remote `colcon build` from phase 2.
