#!/bin/bash
# Container entrypoint — sources the ROS2 workspace and launches the sim.
# No virtual display / VNC needed: visualisation lives in the React dashboard,
# which connects to rosbridge_websocket on port 9090.

source /opt/ros/humble/setup.bash

# Build workspace if needed
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

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║  Multi-Robot Coverage — sim ready                    ║"
echo "║  rosbridge: ws://localhost:9090                      ║"
echo "║  Dashboard: http://localhost:5173                    ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""

exec "$@"
