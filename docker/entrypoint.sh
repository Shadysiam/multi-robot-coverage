#!/bin/bash
# Container entrypoint — sets up virtual display, VNC, noVNC, then launches ROS2.
set -e

source /opt/ros/humble/setup.bash

# ── Build workspace if needed ──────────────────────────────────────────────────
if [ ! -f "/coverage_ws/install/setup.bash" ]; then
    echo "[entrypoint] Workspace not built — running colcon build..."
    cd /coverage_ws
    colcon build --symlink-install
    echo "[entrypoint] Build complete."
fi

source /coverage_ws/install/setup.bash

# Generate maps if missing
if [ ! -f "/coverage_ws/src/multi_robot_coverage/maps/simple_room.pgm" ]; then
    echo "[entrypoint] Generating maps..."
    ros2 run multi_robot_coverage generate_maps
fi

# ── Virtual display setup ──────────────────────────────────────────────────────
export DISPLAY=:99
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
export MESA_GL_VERSION_OVERRIDE=3.3

echo "[entrypoint] Starting Xvfb virtual display on :99 ..."
Xvfb :99 -screen 0 1280x900x24 -ac +extension GLX +render -noreset &
sleep 2

echo "[entrypoint] Starting x11vnc ..."
x11vnc -display :99 -nopw -listen 0.0.0.0 -xkb -forever -shared -bg -quiet

echo "[entrypoint] Starting noVNC on port 6080 ..."
websockify --web /usr/share/novnc/ --wrap-mode=ignore 6080 localhost:5900 &
sleep 1

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║  RViz2 is available in your browser:                 ║"
echo "║  → http://localhost:6080/vnc.html                    ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""

exec "$@"
