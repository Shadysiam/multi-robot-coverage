# ── Stage 1: dependency layer (cached separately from source code) ─────────────
FROM osrf/ros:humble-desktop AS deps

ENV DEBIAN_FRONTEND=noninteractive

# We do NOT install xvfb / x11vnc / novnc — visualisation is handled entirely
# by the React dashboard (see web_dashboard/) over rosbridge.  RViz is also
# unused now, so we keep only the ROS message types and rosbridge runtime.
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-visualization-msgs \
    ros-humble-rosbridge-suite \
    git \
    && rm -rf /var/lib/apt/lists/*

# Install Python deps as a cached layer — only rebuilds if requirements.txt changes
COPY multi_robot_coverage/requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# ── Stage 2: workspace ─────────────────────────────────────────────────────────
FROM deps AS workspace

WORKDIR /coverage_ws

COPY . /coverage_ws/src/

RUN /bin/bash -c "\
    source /opt/ros/humble/setup.bash && \
    cd /coverage_ws && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
"

RUN /bin/bash -c "\
    source /coverage_ws/install/setup.bash && \
    ros2 run multi_robot_coverage generate_maps \
"

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Only rosbridge is exposed (port 9090) — dashboard connects from the browser.
EXPOSE 9090

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
