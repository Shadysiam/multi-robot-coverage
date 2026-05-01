#!/bin/bash
# Container entrypoint — sources ROS2 and the workspace on every docker exec/run.
set -e

source /opt/ros/humble/setup.bash

# If the workspace hasn't been built yet (e.g. first run with a fresh volume),
# build it now before handing control to the caller.
if [ ! -f "/coverage_ws/install/setup.bash" ]; then
    echo "[entrypoint] Workspace not built — running colcon build..."
    cd /coverage_ws
    colcon build --symlink-install
    echo "[entrypoint] Build complete."
fi

source /coverage_ws/install/setup.bash

# Generate maps if they don't exist yet
if [ ! -f "/coverage_ws/src/multi_robot_coverage/maps/simple_room.pgm" ]; then
    echo "[entrypoint] Generating maps..."
    ros2 run multi_robot_coverage generate_maps
fi

exec "$@"
