# Project Context — Multi-Robot Coverage Planner

> **Purpose of this file:** Hand-off document so an AI assistant (or future you) can pick up exactly where the work was left. Contains the full architecture, every bug fixed and why, and the current state of every feature.

---

## 1. What this project is

A **multi-robot coverage path planning** simulation. Three robots cooperatively cover a 2D obstacle map using **Boustrophedon Cellular Decomposition (BCD)** with A\* inter-cell routing. Built end-to-end with:

- **ROS2 Humble** (Python) — algorithms, robot agents, coordinator
- **Docker + docker-compose** — fully containerised, runs anywhere
- **rosbridge** — exposes ROS topics over WebSocket
- **React 18 + Vite + Tailwind** — live web dashboard at `localhost:5173`

It is *not* a Gazebo simulation — robots are mathematical agents (moving coordinates), not 3D physics objects. Algorithms are real and identical to what would run on physical hardware. This is closer to how real multi-robot coordination is prototyped at companies like Amazon Robotics.

---

## 2. Architecture (one-paragraph summary)

```
map_generator.py  →  obstacle_room.pgm
                          ↓
                    map_server  →  /map (OccupancyGrid)
                          ↓
              coverage_coordinator
              (BCD + A* + densification + Bresenham validation)
                          ↓
              /robot_N/waypoints  →  robot_agent_N (×3)
                                          ↓
                                    /robot_N/pose
                                    /robot_N/path
                                    /robot_N/status
                                          ↓
                                  rosbridge (port 9090)
                                          ↓
                              React Dashboard (port 5173)
```

Two Docker containers: `coverage_sim` (the brain) and `dashboard` (Vite dev server).

---

## 3. Algorithms (4 implemented, all selectable live)

| Algorithm | File | What it does | Expected coverage |
|-----------|------|--------------|-------------------|
| `boustrophedon` *(default, ours)* | `algorithms/boustrophedon.py` | Column-sweep cellular decomposition → lawnmower per cell → greedy area-balanced robot assignment → A\* inter-cell + intra-strip gap bridging | **95%+** |
| `frontier` | `algorithms/frontier_based.py` | Reactive frontier-based exploration (BFS clustering, claimed-set assignment) | ~88% |
| `simple_boustrophedon` | `algorithms/simple_boustrophedon.py` | Naive map-wide lawnmower split into N horizontal bands, no obstacle awareness | ~75% (gets stuck at obstacles) |
| `random_walk` | `algorithms/random_walk.py` | Biased random walk baseline (continues straight until blocked) | ~55% (chaotic, slow) |

Plus failure recovery via **propagation-based reallocation** (Gong et al. 2024, Sensors 24:7482) — when a robot fails, surviving robots take over its remaining cells, sorted by proximity to the failed robot's last position.

---

## 4. The headline bug fixes (chronological — keep these in mind)

These were all real bugs I hit and fixed. Understanding them is critical context.

### Bug 1: Y-axis flip mismatch (caused robots to "cross obstacles")
- **Symptom:** Robots driving through walls; coverage cells appearing in mirror-Y positions relative to the actual robot.
- **Root cause:** `map_server` correctly publishes `/map` with `data[0]` = bottom-left cell (standard ROS convention). But `coverage_coordinator` loaded it via `arr.reshape(rows, cols)` with **no flip** — so `arr[0]` = bottom row. Meanwhile, `_world_to_grid` treats row 0 as the **top** of the world. Net result: the entire grid was used upside-down — A\* routed through walls; coverage painted in the wrong Y position.
- **Fix:** `np.flipud(arr)` on map ingest (so internal storage matches `_world_to_grid`'s row-0=top convention), and `np.flipud` again when publishing `/coverage_map` (to restore ROS row-0=bottom for the dashboard).

### Bug 2: Lawnmower waypoints linearly interpolating through obstacles
- **Symptom:** Even after Y-flip fix, robots still occasionally crossed obstacles within a strip.
- **Root cause:** BCD's `generate_path` collects all free columns in a strip row, e.g. `[5, 6, 7, 8, 9, 11, 12]` — column 10 is an obstacle. The robot linearly interpolates from `(row, 9)` → `(row, 11)` and **passes through** `(row, 10)` (the obstacle).
- **Fix:** Added `_densify_path()` in coordinator. Any pair of waypoints > √2 cells apart is bridged with A\*.

### Bug 3: `_densify_path` chained-skip bug
- **Symptom:** Still occasional obstacle crossings even with densification.
- **Root cause:** When A\* failed (start/goal inside the inflated zone), code dropped the destination waypoint. But the next iteration used `grid_path[i-1]` (the just-skipped point) as `prev` for the distance check. So the dense list could jump from a "good" point to one across an obstacle without triggering the gap detector.
- **Fix:** Distance check now uses `dense[-1]` (last accepted point). Dropping a waypoint can never silently create a longer cross-obstacle jump.

### Bug 4: A\* inflation blocking start/goal cells
- **Symptom:** A\* returned `None` for waypoints adjacent to obstacles, fallback was direct-jump-through-obstacle.
- **Root cause:** Lawnmower waypoints by definition often sit one cell from a wall. Inflating obstacles by 4 cells made the start cell itself appear blocked. A\* legitimately found no path.
- **Fix:** Two-tier retry — first try with full inflation (good clearance), then with `inflation_radius=0` (centre-line guarantee), only then drop the waypoint.

### Bug 5: Planning grid not respecting robot footprint
- **Symptom:** Algorithm-level gaps where a robot was technically "free" by grid math but physically couldn't fit.
- **Root cause:** BCD/A\*/lawnmower were all running on the raw obstacle grid. Robot radius wasn't baked into the geometry.
- **Definitive fix:** `_cb_map` now runs `binary_dilation` on obstacles by `robot_radius / resolution = 4 cells` before storing as `self._grid`. **All algorithms now plan in the safety-padded grid by construction.** A\* uses `inflation_radius=0` everywhere because the grid is already inflated.

### Bug 6: Bresenham final-pass safety net
- Added after all the above: every consecutive segment of the dense path is line-checked via Bresenham. If any segment grazes an obstacle, it's replaced with an A\* detour. This is the last defence — guarantees no segment crosses an obstacle even with diagonal moves.

---

## 5. Other significant fixes

| Issue | Fix |
|-------|-----|
| `roslib` CJS / Vite ESM mismatch | Added `optimizeDeps: { include: ['roslib'] }` to `vite.config.js` and `ROSLIB.default ?? ROSLIB` import dance |
| React StrictMode double-firing effects → `ros.close()` killing WebSocket | Removed `<StrictMode>` from `main.jsx` |
| Dashboard going blank with no error | Added `<ErrorBoundary>` wrapping `<App>` — shows red error card with stack trace instead of white screen |
| `OccupancyGrid` width/height destructured wrong (they're under `.info`) | Fixed in `MapCanvas.jsx` |
| `createImageData` getting NaN | Added guard `if (!width || !height || width <= 0) return` |
| Docker `coverage_sim` crashing on restart (x11vnc race with Xvfb) | Wait loop with `xdpyinfo` before x11vnc, retry on failure, removed `set -e` |
| Robots spawned all in the bottom row, crowding | `_default_start_positions` now spreads across X with BFS-snap to nearest free cell |

---

## 6. Live control (no Docker restart needed)

Dashboard publishes to ROS topics; coordinator/agents listen:

| Topic | Type | What it does |
|-------|------|--------------|
| `/set_algorithm` | `std_msgs/String` | Coordinator resets coverage state, replans with new algorithm. Robots receive empty path → reset to idle, then receive new waypoints. |
| `/set_speed` | `std_msgs/Float64` | All robot agents update `self._speed` live mid-mission |
| `/inject_failure` | `std_msgs/String` | Coordinator picks a random active robot, publishes `True` to `/robot_N/fail_trigger`, surviving robots reallocate cells |

The only action that **does** require a Docker restart is changing the **map** (the launch file picks the map file at startup). The dropdown has a `(restart sim)` hint for it.

---

## 7. Dashboard layout (current)

- **Header:** Geometric grid logo (no emoji), title "Multi-Robot Coverage Planner", live agent count, robot colour badges, connection status pill.
- **Left panel:** Live coverage map canvas (560×560) with chassis-style robot rendering (rotated rounded-rect, sensor dome, direction arrow, ID badge, optional FOV ring + path + trail + grid overlays). Below: `ControlBar` with overlay toggles, speed (0.5×/1×/2×/5×), algorithm dropdown (4 options), map preset dropdown, **⚠ Inject Failure** button. Legend below that.
- **Right panel:** `StatsPanel` — circular coverage ring (turns green on completion), 6-cell metric grid (Algorithm/Speed/Elapsed/ETA/Active/Distance), per-robot status with proportional progress bars. Below that: `CoverageChart` — live SVG line chart of coverage % over time, colour-coded per algorithm.

---

## 8. Files map (where everything lives)

```
multi_robot_coverage/multi_robot_coverage/
├── algorithms/
│   ├── astar.py                  # 8-connected A* with optional binary_dilation inflation
│   ├── boustrophedon.py          # BCD: column sweep, IN/OUT/SPLIT/MERGE events, lawnmower
│   ├── frontier_based.py         # Frontier exploration (BFS clustering, claimed-set assignment)
│   ├── random_walk.py            # NEW — baseline biased walk
│   └── simple_boustrophedon.py   # NEW — naive lawnmower (no decomposition)
├── nodes/
│   ├── coverage_coordinator.py   # Heart of the system — state machine, planning, replanning,
│   │                             #   Y-flip, inflation, densify_path, segment validation,
│   │                             #   /set_algorithm + /inject_failure subscribers
│   ├── robot_agent.py            # Per-robot waypoint follower, /set_speed listener, fail trigger
│   ├── map_server.py             # Reads PGM, publishes /map (with row-flip to ROS convention)
│   └── visualizer.py             # MarkerArray for RViz (legacy — RViz disabled)
└── map_generator.py              # Generates simple_room / obstacle_room / warehouse PGMs

web_dashboard/src/
├── App.jsx                       # Top-level composition, ROS subscriptions, state, handlers
├── main.jsx                      # No StrictMode, wraps App in <ErrorBoundary>
├── index.css                     # Tailwind + .stat-card / .badge utility classes
├── hooks/
│   └── useRos.js                 # ROSLIB connection, subscribe(), publish() — handles CJS quirk
├── components/
│   ├── MapCanvas.jsx             # Canvas renderer — chassis robots, paths, FOV, trails, grid
│   ├── StatsPanel.jsx            # Coverage ring + metric grid + per-robot bars
│   ├── ControlBar.jsx            # Overlay toggles, speed, algo dropdown, map dropdown, fail button
│   ├── CoverageChart.jsx         # Live SVG line chart of coverage over time
│   └── ErrorBoundary.jsx         # Catches React crashes, shows red card with stack
└── utils/
    └── colors.js                 # Robot palette + cellColor() with translucent blend

docker/
├── entrypoint.sh                 # Xvfb wait loop + x11vnc retry + noVNC + ros2 launch

.github/workflows/ci.yml          # pytest matrix (3.10, 3.11) + flake8
docker-compose.yml                # coverage_sim + dashboard services, port mappings
Dockerfile                        # Multi-stage: deps → workspace
Makefile                          # build / sim / web / test / shell / clean targets
```

---

## 9. How to run

```bash
# Build the ROS2 image (~5 min first time)
make build

# Run sim + dashboard (open http://localhost:5173)
make web

# Sim only (open noVNC at http://localhost:6080/vnc.html — RViz disabled by default)
make sim

# Run a specific algorithm (alternative to dashboard's live switching)
make sim-args ARGS="algorithm:=random_walk"
make sim-args ARGS="algorithm:=simple_boustrophedon"
make sim-args ARGS="algorithm:=frontier"

# Different map (requires Docker restart)
make sim-args ARGS="map_name:=warehouse"
make sim-args ARGS="map_name:=simple_room"

# Run all 47 unit tests
make test

# Open a shell inside the ROS2 container
make shell

# Tear it all down
make clean
```

---

## 10. Status checklist

### ✅ Done
- 4 algorithms (BCD, frontier, simple lawnmower, random walk)
- Live algorithm switching from dashboard (`/set_algorithm`)
- Live speed control from dashboard (`/set_speed`)
- Failure injection button (`/inject_failure`) + propagation reallocation
- Multi-layer obstacle-crossing fix (inflated grid + densify + Bresenham)
- Spread-out robot start positions (BFS-snap)
- Polished dashboard: chassis robots, refined coverage colours, vignette, ETA, distance per robot, coverage chart, overlay toggles
- Docker containerisation (sim + dashboard)
- 47 unit tests + GitHub Actions CI
- Y-flip bug fixed everywhere (load + publish)

### ⏭ To do (next session)
1. **Run each algorithm and record real numbers** — coverage %, time, distance for the README comparison table
2. **Generate demo GIFs** — one for each algorithm, plus a failure-injection demo
3. **Write the comprehensive recruiter-friendly README** — overview, architecture diagram, algorithms explained, comparison table with real numbers, demo GIFs, setup instructions, future work
4. **Optional: JSON metrics export** — coordinator dumps per-run metrics to `/tmp/metrics.json` for plotting
5. **Optional: redundancy metric** — count cells covered by multiple robots (lower is better → BCD wins again)
6. **Push to a clean public GitHub repo** — currently on origin/master, last commit `da50fb8`

### 🐛 Known cosmetic issues
- noVNC tab shows a black screen (RViz disabled). Expected — dashboard is the primary UI. We could remove the noVNC port mapping entirely if it's confusing.
- Dashboard's "Map" dropdown doesn't actually trigger restart — just a visual indicator. Could add a service/script to call `docker compose restart coverage_sim` with new env var, but probably overkill.

---

## 11. Quick context for the next session

If picking this up cold, the next concrete task is:

> "Run each of the 4 algorithms on the obstacle_room map for 60s, record coverage %, total distance, completion time. Put the numbers into a README comparison table. Then generate a demo GIF showing BCD running with the failure-injection button being clicked at ~30s, demonstrating reallocation. Then write the full recruiter-friendly README using the structure suggested in `chatgpt_nav_suggestions.md` (Overview / Problem / Architecture / Challenges / Solutions / Results / Algorithm comparison / Demo / Setup / Future work)."

The core engineering is solid. What's left is **packaging the result** for recruiters.
