# ── Stage 1: dependency layer (cached separately from source code) ─────────────
FROM osrf/ros:humble-desktop AS deps

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-rviz2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-visualization-msgs \
    ros-humble-rosbridge-suite \
    git \
    # Virtual framebuffer + VNC + noVNC (browser-based display — no XQuartz needed)
    xvfb \
    x11vnc \
    novnc \
    websockify \
    # Mesa software rendering
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Install Python deps as a cached layer — only rebuilds if requirements.txt changes
COPY multi_robot_coverage/requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# ── Stage 2: workspace ─────────────────────────────────────────────────────────
FROM deps AS workspace

WORKDIR /coverage_ws

# Copy source into the image so it can be built.
# When developing locally we override this with a volume mount (see docker-compose).
COPY . /coverage_ws/src/

# Pre-build so `docker run` starts immediately.
RUN /bin/bash -c "\
    source /opt/ros/humble/setup.bash && \
    cd /coverage_ws && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
"

# Generate the PGM map files
RUN /bin/bash -c "\
    source /coverage_ws/install/setup.bash && \
    ros2 run multi_robot_coverage generate_maps \
"

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# noVNC runs on port 6080 — open this in your browser to see RViz2
EXPOSE 6080

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
