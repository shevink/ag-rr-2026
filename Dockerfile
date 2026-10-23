# ============================================================
# docker/Dockerfile
# Single multi-stage Dockerfile for all runtime targets.
# ============================================================

# ---- ARGs (available to FROM) ----
ARG ROS_DISTRO=jazzy
ARG BASE_IMAGE=ros:${ROS_DISTRO}-ros-base-noble
ARG ENABLE_NVIDIA=false

# ============================================================
# Stage 1: Common base (shared by ALL targets)
# Default packages only: ROS 2 base image + rviz + utilities.
# ============================================================
FROM ${BASE_IMAGE} AS base

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-pip \
    python3-pynput \
    python3-rosdep \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-nav2-costmap-2d \
    ros-${ROS_DISTRO}-nav2-lifecycle-manager \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-nav2-msgs \
    wget \
    ca-certificates \
    libegl1 \
    && rm -rf /var/lib/apt/lists/*

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

RUN if [ "$ENABLE_NVIDIA" = "true" ]; then \
      apt-get update && apt-get install -y --no-install-recommends \
        nvidia-cuda-toolkit \
        && \
      rm -rf /var/lib/apt/lists/*; \
    fi

# Ensure rosdep is initialized (idempotent).
RUN rosdep init 2>/dev/null || true
RUN rosdep update --rosdistro ${ROS_DISTRO}

ENV QT_X11_NO_MITSHM=1
WORKDIR /opt/ros_ws

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo 'if [ -f /opt/ros_ws/install/setup.bash ]; then source /opt/ros_ws/install/setup.bash; fi' >> ~/.bashrc

# ============================================================
# Stage 2: Dependency installation via rosdep
# Do not hardcode ROS dependencies in this Dockerfile.
# Declare dependencies in package.xml and resolve here.
# ============================================================
FROM base AS deps

COPY . /opt/ros_ws

RUN set -e; \
    mkdir -p /tmp/src_manifests; \
    if [ -d /opt/ros_ws/src ]; then \
      cp -a /opt/ros_ws/src/. /tmp/src_manifests/; \
    fi; \
    if [ -d /opt/ros_ws/tasks ]; then \
      mkdir -p /tmp/src_manifests/tasks; \
      cp -a /opt/ros_ws/tasks/. /tmp/src_manifests/tasks/; \
    fi; \
    if [ -d /opt/ros_ws/robots ]; then \
      mkdir -p /tmp/src_manifests/robots; \
      cp -a /opt/ros_ws/robots/. /tmp/src_manifests/robots/; \
    fi; \
    find /tmp/src_manifests -not -name "package.xml" -not -type d -delete; \
    find /tmp/src_manifests -mindepth 1 -empty -type d -delete; \
    if find /tmp/src_manifests -name "package.xml" -print -quit | grep -q .; then \
      apt-get update && \
      rosdep install \
        --from-paths /tmp/src_manifests \
        --ignore-src \
        --rosdistro ${ROS_DISTRO} \
        -y; \
    else \
      echo "No ROS package manifests found under /opt/ros_ws/src, /opt/ros_ws/tasks, or /opt/ros_ws/robots; skipping rosdep install."; \
    fi; \
    rm -rf /var/lib/apt/lists/* /tmp/src_manifests

# ============================================================
# Stage 3a: Linux runtime target
# ============================================================
FROM deps AS linux

ENV DISPLAY=:0
CMD ["bash"]

# ============================================================
# Stage 3b: Windows runtime target
# For use with WSLg (Docker Desktop WSL2 backend) or
# an external Windows X server (e.g. VcXsrv/X410).
# Override DISPLAY at runtime for external X servers:
#   DISPLAY=host.docker.internal:0.0
# ============================================================
FROM deps AS windows

ENV DISPLAY=:0
ENV WAYLAND_DISPLAY=wayland-0
ENV XDG_RUNTIME_DIR=/run/user/1000
CMD ["bash"]

# ============================================================
# Stage 3c: VNC fallback runtime target
# ============================================================
FROM deps AS vnc

RUN apt-get update && apt-get install -y --no-install-recommends \
    xvfb \
    x11vnc \
    fluxbox \
    novnc \
    websockify \
    && rm -rf /var/lib/apt/lists/*

EXPOSE 5900 6080
CMD ["bash", "-lc", "Xvfb :1 -screen 0 1280x720x24 & fluxbox & x11vnc -display :1 -forever -nopw -listen 0.0.0.0 -xkb & websockify --web=/usr/share/novnc 6080 localhost:5900"]