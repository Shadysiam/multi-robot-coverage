# 🤖 Multi-Robot Coverage Path Planning · ROS2 Humble

[![CI](https://github.com/Shadysiam/multi-robot-coverage/actions/workflows/ci.yml/badge.svg)](https://github.com/Shadysiam/multi-robot-coverage/actions/workflows/ci.yml)
![ROS2](https://img.shields.io/badge/ROS2-Humble-22314E?logo=ros&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.10+-3776AB?logo=python&logoColor=white)
![License](https://img.shields.io/badge/License-MIT-22c55e)

> A from-scratch multi-robot coverage simulator in ROS2 — comparing a
> **Boustrophedon Cellular Decomposition** planner against **frontier-based
> exploration**, with live RViz2 visualisation and mid-mission fault recovery.

---

## Demo

<!-- Replace the rows below with your actual GIFs/screenshots after running the sim -->

| Boustrophedon · 3 robots · obstacle room | Failure injection & reallocation |
|:---:|:---:|
| *(screenshot coming)* | *(screenshot coming)* |

| Frontier-based · 3 robots | Algorithm comparison view |
|:---:|:---:|
| *(screenshot coming)* | *(screenshot coming)* |

---

## What This Project Demonstrates

This isn't a tutorial project — every component was designed and implemented from scratch:

- **Coverage path planning** — full Boustrophedon Cellular Decomposition: sweep-line map decomposition into convex cells, lawnmower path generation per cell, greedy area-balanced robot assignment
- **Reactive exploration** — frontier-based algorithm with BFS cluster detection and a coordination layer that prevents multiple robots targeting the same goal
- **A\* planner** — 8-connected grid search with Euclidean heuristic, corner-clipping prevention, and configurable obstacle inflation for robot footprint
- **Fault tolerance** — simulate a robot dying mid-mission; remaining tasks propagate to active neighbours using the strategy from [Gong et al. 2024](https://doi.org/10.3390/s24237482)
- **ROS2 systems design** — custom message packages, transient-local QoS, TF2 broadcasting, OpaqueFunction launch files, fully parameterised with zero hardcoded values
- **Testing** — 47 unit tests on pure-Python algorithm modules (no ROS2 needed), GitHub Actions CI across Python 3.10 and 3.11

---

## How It Works

```
map_server  ──/map──►  coverage_coordinator  ──/robot_N/waypoints──►  robot_agent × N
                              │ runs:                                        │
                              │  · BoustrophedonDecomposer                  │ publishes
                              │  · AStar                          /robot_N/pose, /status
                              │  · FrontierExplorer                         │
                              └──────────────────────────────────►  visualizer ──► RViz2
```

The planner lives in **pure Python with no ROS2 dependency** — it can be unit-tested, benchmarked, or ported to C++ without touching the node layer. The coordinator subscribes to robot poses, tracks coverage in real time, and handles reallocation when a failure is detected.

### Boustrophedon Cellular Decomposition

A vertical sweep-line traverses the grid. Each time free-space connectivity changes — an obstacle appearing, disappearing, splitting, or merging a free interval — a **critical event** fires and a cell boundary is drawn. The resulting cells tile the free space exactly, with zero overlap. Each cell is then covered by a back-and-forth lawnmower sweep.

```
 Free space          BCD cells          Per-cell paths
 ┌─────────┐        ┌──┬────┬──┐        →→→→  ←←←←←
 │  ██ ██  │  ──►   │A │ B  │C │  ──►   ←←←  →→→→→→
 │         │        │  └────┘  │        →→→→  ←←←←←
 └─────────┘        └──────────┘
```

---

## Results

> Measured results and GIFs will be added after running the full simulation on ROS2 Humble.
> Numbers below reflect theoretical properties of each algorithm.

| Metric | Boustrophedon | Frontier-based |
|---|:---:|:---:|
| Guaranteed 100% coverage | ✅ | ❌ |
| Works without prior map | ❌ | ✅ |
| Path overlap | Low | High |
| Deterministic | ✅ | ❌ |
| Measured coverage % | — | — |
| Measured time to complete | — | — |

---

## Quick Start

### Option A — Docker (recommended, works on Mac/Windows/Linux)

**Prerequisites:** [Docker Desktop](https://www.docker.com/products/docker-desktop/) · [XQuartz](https://www.xquartz.org/) (Mac only, for RViz2 display)

```bash
git clone git@github.com:Shadysiam/multi-robot-coverage.git
cd multi-robot-coverage

make build   # build the image (~3 min, once only)
make sim     # launches full simulation + RViz2
```

Common scenarios:

```bash
# 4 robots in the warehouse
make sim-args ARGS="num_robots:=4 map:=warehouse algorithm:=boustrophedon"

# Kill robot 1 at t=20s — watch reallocation live
make sim-args ARGS="num_robots:=3 map:=warehouse enable_failure_sim:=true failure_time:=20.0"

# Frontier-based for comparison
make sim-args ARGS="num_robots:=3 map:=obstacle_room algorithm:=frontier"

# Run tests inside Docker
make test

# Open a shell inside the container
make shell
```

> **Mac setup (one-time):** Install XQuartz, open it, go to
> Preferences → Security → tick *Allow connections from network clients*, then reboot.

---

### Option B — Native ROS2 (Ubuntu 22.04)

```bash
mkdir -p ~/coverage_ws/src && cd ~/coverage_ws/src
git clone git@github.com:Shadysiam/multi-robot-coverage.git .
pip3 install -r multi_robot_coverage/requirements.txt
cd ~/coverage_ws && source /opt/ros/humble/setup.bash
colcon build --symlink-install && source install/setup.bash
ros2 launch multi_robot_coverage coverage_demo.launch.py
```

### Unit tests (no ROS2 or Docker needed)

```bash
cd multi_robot_coverage && pip install pytest numpy scipy
pytest test/ -v   # 47 tests
```

---

## Project Layout

```
multi_robot_coverage/
├── multi_robot_coverage/
│   ├── algorithms/          # Pure Python — ROS2-free, fully unit-tested
│   │   ├── astar.py         # 8-connected A* with obstacle inflation
│   │   ├── boustrophedon.py # BCD decomposition + lawnmower path gen
│   │   └── frontier_based.py
│   └── nodes/               # ROS2 layer — thin wrappers around algorithms
│       ├── map_server.py
│       ├── robot_agent.py
│       ├── coverage_coordinator.py
│       └── visualizer.py
├── maps/                    # 3 procedurally generated PGM maps
├── test/                    # 47 pytest unit tests
├── launch/coverage_demo.launch.py
└── config/params.yaml · coverage.rviz

multi_robot_coverage_msgs/   # Custom ROS2 message definitions
└── msg/CoverageStats.msg · AlgorithmComparison.msg
```

---

## References

Gong, X., et al. (2024). Multi-Robot Coverage Path Planning Based on Boustrophedon Cellular Decomposition with Propagation-Based Task Reallocation. *Sensors*, 24(23), 7482. https://doi.org/10.3390/s24237482

---

## License

MIT © 2024 Shady Siam
