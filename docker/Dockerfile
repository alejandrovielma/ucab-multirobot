# Test image: ROS 2 Jazzy plus the Python libraries used by the nodes.
# Webots and webots_ros2 are intentionally NOT installed: the simulation runs on the host.
FROM ros:jazzy-ros-base

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-opencv \
        python3-numpy \
        python3-pytest \
        python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
