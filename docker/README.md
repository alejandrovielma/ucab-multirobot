# Docker environment

A single image with everything needed to develop without installing ROS on your machine:
Ubuntu 26.04, ROS 2 Lyrical, OpenCV, ruff and Webots R2025a (headless).

| File | What it does |
|---|---|
| `docker/Dockerfile` | `base` stage (ROS + OpenCV, also works on arm64) and `sim` stage (+ Webots) |
| `docker/entrypoint.sh` | sources ROS, builds the workspace with `colcon build` and runs the command |
| `compose.yaml` (root) | `sim`, `sim-external` and `dev` services |

The repo is mounted at `/ws`: whatever you edit in your folder is used immediately (each start
rebuilds what changed). `build/`, `install/` and `log/` live in Docker volumes.

## Requirements

- **Windows:** Docker Desktop with the WSL2 backend.
- **Linux:** Docker Engine with the `compose` plugin.
- ~4 GB free for the image. The first build takes a few minutes (it downloads Webots).

```bash
docker compose build          # once, or whenever docker/Dockerfile changes
```

## 1. Everything inside Docker (easiest)

```bash
docker compose up sim
```

Starts headless Webots with `pista_ucab_ros.wbt`, the ROS bridge and the nodes.
Open **http://localhost:8080**: you see the robot's camera with the cube boxes, choose
the target colours and the stop distance, and pause or start it. You can also
drive it by hand with **W/A/S/D** (Q/E to rotate, Shift for faster): while you hold a key
down the state is `MANUAL`, even if it is paused.

The robot rotates searching, approaches the nearest cube of the chosen colours and stops
at `stop_distance_m` (0.17 m from the camera by default, about 5 cm in front of the scoop).

> Webots without a GPU renders in software: the camera runs at ~6–10 frames per second and
> sometimes stalls for a moment (the state becomes `NO_CAMERA` and the robot stays still, on purpose).

## 2. Webots on your machine, nodes in Docker (to watch the robot move)

1. Install Webots R2025a on Windows or Linux (https://cyberbotics.com).
2. Open `simulation/webots/worlds/pista_ucab_ros.wbt` and press ▶. The console will say
   `Waiting for local or remote connection on port 1234 targeting robot named 'Ferb'`.
3. In another terminal:

   ```bash
   docker compose up sim-external
   ```

   The bridge connects to `tcp://host.docker.internal:1234/Ferb`. If Webots is on another
   machine: `WEBOTS_HOST=192.168.x.y docker compose up sim-external`.
4. Open http://localhost:8080.

If the bridge says `Cannot connect to Webots instance`, check that Webots is running with
the simulation playing and that the Windows firewall lets port 1234 through.

## 3. Console to build, test and lint

```bash
docker compose run --rm dev
# inside:
colcon test && colcon test-result --verbose
ruff check src tools && ruff format --check src tools
ros2 topic list
```

To open another console in a container that is already running: `docker compose exec sim bash`
(and inside, `source /ws/install/setup.bash`).

## Useful commands

```bash
docker compose exec sim bash -c "source /ws/install/setup.bash && ros2 topic echo /cubes --once"
docker compose exec sim bash -c "source /ws/install/setup.bash && \
  ros2 param set /behavior_fsm target_colors '[blue, green]'"
docker compose down            # shuts everything down
docker compose down -v         # also deletes the build/install/log volumes
```

## Details

- The base image can be changed with `--build-arg BASE_IMAGE=...` (for example, a Docker Hub
  mirror if you get `429 Too Many Requests`: `mirror.gcr.io/library/ros:lyrical-ros-base`).
- The `sim` stage only exists for amd64 (Webots does not publish an arm64 package). On the Pi the
  native installation is used: see [`docs/ssh-deployment.md`](../docs/ssh-deployment.md).
