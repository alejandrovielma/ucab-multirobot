# Collector development image: ROS 2 Lyrical + OpenCV (+ Webots R2025a in the "sim" stage).
#
#   base -> the minimum to build and run the nodes (also works on the Pi 5, arm64)
#   sim  -> base + headless Webots R2025a (amd64 only)
#
# Used from compose.yaml (repo root). See docker/README.md.

ARG BASE_IMAGE=ros:lyrical-ros-base
FROM ${BASE_IMAGE} AS base

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      python3-opencv \
      python3-numpy \
      python3-pytest \
      python3-pip \
      curl \
 && rm -rf /var/lib/apt/lists/*

# ruff for linting (same as CI)
RUN pip3 install --no-cache-dir --break-system-packages ruff

# Webots and ROS complain if there is no user
ENV USER=ros

WORKDIR /ws
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]


FROM base AS sim

ARG WEBOTS_VERSION=R2025a
RUN apt-get update \
 && curl -fsSL -o /tmp/webots.deb \
      "https://github.com/cyberbotics/webots/releases/download/${WEBOTS_VERSION}/webots_2025a_amd64.deb" \
 && apt-get install -y --no-install-recommends /tmp/webots.deb xvfb \
 && rm -f /tmp/webots.deb \
 && rm -rf /var/lib/apt/lists/*

ENV WEBOTS_HOME=/usr/local/webots
# Webots without GPU: software rendering
ENV LIBGL_ALWAYS_SOFTWARE=1
