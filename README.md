# Navigation Simulation Project

ROS2 Humble workspace implementing multi-robot coverage path planning.

## Packages

| Package | Type | Description |
|---|---|---|
| [`multi_robot_coverage`](multi_robot_coverage/) | `ament_python` | Algorithms, nodes, maps, launch files |
| [`multi_robot_coverage_msgs`](multi_robot_coverage_msgs/) | `ament_cmake` | Custom message definitions |

See [`multi_robot_coverage/README.md`](multi_robot_coverage/README.md) for full documentation.

## Quick start

```bash
# 1. Install Python deps
pip3 install -r multi_robot_coverage/requirements.txt

# 2. Generate map files (writes .pgm to multi_robot_coverage/maps/)
cd multi_robot_coverage && python3 -m multi_robot_coverage.map_generator && cd ..

# 3. Build
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# 4. Run
ros2 launch multi_robot_coverage coverage_demo.launch.py \
    num_robots:=3 algorithm:=boustrophedon map:=obstacle_room
```

## Unit tests (no ROS2 required)

```bash
cd multi_robot_coverage
pip install pytest numpy scipy
pytest test/ -v
```
