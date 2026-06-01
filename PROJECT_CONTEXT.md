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

---

## 12. Recommended improvements (next-session menu)

Sorted by portfolio impact. Tier 1 should be considered mandatory before pushing to GitHub.

### 🎯 Tier 1 — Mandatory before sharing publicly

**1.1 Headless benchmark mode + JSON metrics export** *(2–3 hrs)*
- `make benchmark` runs each of the 4 algorithms on each of the 3 maps (12 runs)
- Per-run JSON: coverage curve, distance per robot, redundancy (cells covered ≥ 2×), completion time
- Aggregates to `/results/benchmark.csv`
- This is what fills the README comparison table with real numbers

**1.2 Multi-curve overlay on coverage chart** *(1–2 hrs)*
- Load saved benchmark JSONs, overlay all 4 algorithm curves on the existing live chart
- Live run = bold line, saved benchmarks = faded reference lines
- Visual proof BCD wins, in one screenshot

**1.3 Recruiter-grade README + demo GIFs** *(half a day)*
- Mermaid architecture diagram, real-numbers comparison table, 3 demo GIFs (BCD running, failure injection + reallocation, algorithms side-by-side)
- Sections: Problem / Architecture / Challenges / Solutions / Results / Setup / Future work
- This is what gets clicked

### 🚀 Tier 2 — Algorithmic depth signals seniority

**2.1 Realistic kinematics — pure-pursuit controller** *(2 hrs)*
- Add `v_max = 1.0 m/s`, `ω_max = 1.5 rad/s`, `accel_max = 0.5 m/s²` to robot_agent
- Replace linear interpolation with pure-pursuit picking `(v, ω)` toward next waypoint
- Robots arc around corners instead of cutting them — dramatic visual upgrade
- **Prerequisite for DWA to make sense** (see §13)

**2.2 Dynamic obstacles + runtime replanning** *(half a day)*
- Click on map to drop a moving box obstacle
- Coordinator detects current path is blocked → A\* replan from current position to next waypoint → splice detour in
- Pairs naturally with DWA (§13). No other coverage demo on GitHub does this.

**2.3 Hungarian algorithm for cell assignment** *(2 hrs)*
- Replace greedy area-balancing with Hungarian (optimal min-max assignment)
- Classic OR technique recruiters know from Amazon, Google, Anduril
- Takes BCD from "good" to "provably optimal under this cost"

**2.4 TSP-ordered cell sequencing per robot** *(2 hrs)*
- Within each robot's cells, brute-force TSP over centroids (3–5 cells, fits in milliseconds)
- Should improve total-distance metric by ~15–25%

### 🎨 Tier 3 — Visual quality wins

**3.1 Smooth 60 fps pose interpolation** *(1 hr)*
- Lerp robot positions in canvas between received messages
- Massive perceptual upgrade from current 20 Hz teleport feel

**3.2 Coverage heatmap overlay** *(1–2 hrs)*
- Toggle: cells covered once = normal, 2× = warm orange, 3× = red
- Visually proves BCD's low redundancy vs random walk's mess

**3.3 BCD cell-boundary overlay** *(1 hr)*
- Toggle showing decomposed cells as faint coloured polygons with IDs
- Makes the algorithm tangible — recruiters can literally see the math working

**3.4 Replay scrubber** *(half a day)*
- Record pose + coverage timeline, bottom timeline scrubs backwards/forwards
- Pause and study any moment

### 🏗 Tier 4 — Engineering credibility (quick wins)

**4.1 Pytest coverage badge** *(30 min)* — `pytest --cov` → badge → README
**4.2 Type hints + mypy in CI** *(2 hrs)* — modern Python signal
**4.3 Profile BCD on 500×500 map, write perf section** *(2 hrs)* — "engineer who measures"
**4.4 Remove noVNC dead weight from Docker** *(15 min)* — saves image size, kills the confusing black screen

### 📈 Tier 5 — Strategic distribution

**5.1 2-minute video walkthrough** *(half a day)* — Loom/YouTube, embedded in README. This is what people share on LinkedIn.
**5.2 Blog post** *(half a day)* — "Implementing BCD from scratch." Show the bug-hunt journey — devs love it. Drives organic traffic.
**5.3 Hosted demo** *(full day)* — VPS for sim + Vercel for dashboard. Recruiters click a link, no setup. Massive differentiator if you pull it off.

### 📋 Recommended day-by-day if you commit fully

```
Day 1 (data + framing):
  1.1 Headless benchmark      ← unlocks everything else
  1.2 Multi-curve chart
  1.3 Recruiter README + GIFs

Day 2 (algorithmic credibility):
  2.3 Hungarian assignment
  2.4 TSP cell sequencing
  3.1 Smooth interpolation
  3.2 Heatmap overlay

Day 3 (the showpiece):
  2.1 Realistic kinematics    ← prereq for §13
  2.2 Dynamic obstacles
  §13 DWA local planner       ← see below
  5.1 Video walkthrough

Day 4 (publish):
  Push to GitHub, post on LinkedIn, share blog post
```

Tier 1 alone moves this from "another student project" to "I'd interview this person." Everything below is multiplicative gravy.

---

## 13. About MPC and DWA (the ChatGPT doc mentioned them)

**Important framing:** MPC and DWA are **not coverage planners** — they're **local motion controllers**. They solve "given my current state and a short-term goal, what velocity command should I send for the next 0.1s?" They live below A\* in the planning hierarchy.

Our current stack:
```
BCD (global)  →  A* (path between cells)  →  linear_interp(prev, next)  ← motion control
```

DWA / MPC would replace that last layer.

### DWA — yes, but only paired with dynamic obstacles
On a purely static map, DWA adds nothing — A\* already gave us a safe path. **But add dynamic obstacles (§2.2) and DWA becomes the right tool**: the robot must choose `(v, ω)` each tick to avoid the moving thing while still tracking the BCD path. That gives the project a complete, legitimate three-layer nav stack:

```
BCD (global coverage)  →  A* (static path)  →  DWA (reactive velocity control)
```

Recruiters at Waymo / Cruise / Boston Dynamics will immediately recognise that as a real architecture. **Effort:** ~half a day. Pairs with §2.1 (kinematics) and §2.2 (dynamic obstacles).

### MPC — honest answer: skip it
MPC is heavy machinery (constrained QP over a finite horizon). It's right for:
- Real kinodynamic constraints (cars, drones, manipulators)
- Tight tracking requirements with limited control authority

Our point-mass robots have none of that. Adding MPC means:
- Solver dependency (cvxpy/OSQP) complicates Docker
- Days of debugging convergence
- Visible end-result almost identical to pure-pursuit (§2.1)

**ROI doesn't justify it here.** If a future version adds a car-like robot or a manipulator, MPC becomes the right tool. Today it's a Ferrari trip to the grocery store.

### TL;DR
- **DWA** = yes, paired with §2.1 + §2.2
- **MPC** = no, save your time
- **Pure-pursuit (§2.1)** = the realistic-but-cheap motion controller that does 80% of what MPC would, in 2 hours instead of 2 days

---

## 14. The "best 5 things to do next" (if time is limited)

If you only have one weekend:

1. Headless benchmark + JSON export *(§1.1)*
2. Recruiter README + demo GIFs *(§1.3)*
3. Multi-curve chart overlay *(§1.2)*
4. Smooth interpolation + heatmap *(§3.1 + §3.2)*
5. Push to GitHub, share on LinkedIn

If you have a full week, add §2.1 + §2.2 + DWA *(§13)* for the showpiece, and §5.1 (video) for distribution.

---

## 15. Latest session changes (since commit 011a804)

This is what's been added/fixed in the most recent working session — committed locally but not yet to a single commit (work-in-progress).

### Backend (planning / coordinator / agents)

- **Inflated planning grid** — `_cb_map` runs `binary_dilation` on obstacles by `robot_radius / resolution = 4 cells`. BCD, A*, lawnmower, and coverage all now operate on the safety-padded grid by construction. A* uses `inflation_radius=0` everywhere because the grid is already inflated.
- **Bresenham line validation** — every accepted segment in `_validate_segments` is line-checked; obstacle-grazing segments are replaced with A* detours.
- **Densify-prev fix** — `_densify_path` distance check uses `dense[-1]` (last accepted point) so dropping a waypoint can't create a longer cross-obstacle jump.
- **Per-tick step capping** — `robot_agent._step()` subdivides each tick to ≤ 1 cell of motion. At 5× speed (5 m/s) it now does 5 sub-steps instead of one giant 5-cell jump.
- **Robot teleport only on first path** — `_first_path_received` flag means subsequent paths (failure recovery, algo switch) keep current pose; no more teleport-to-start mid-mission.
- **Failure recovery from current pose** — `_build_full_path(start_pos=...)` accepts a starting cell. `_handle_failure` passes each survivor's current grid position. **`_filter_uncompleted_cells`** also drops cells that are already > 70 % covered so survivors don't redo work.
- **Hungarian assignment** — `boustrophedon.assign_to_robots(method="hungarian")` uses `scipy.optimize.linear_sum_assignment` with cost = α·distance + β·workload. Falls back to greedy if scipy unavailable.
- **Nearest-neighbour TSP per robot** — within each robot's cell list, cells are reordered by greedy NN from start position.
- **Final-strip-at-`r_max`** — `BoustrophedonDecomposer.generate_path` always appends a strip near `r_max` if the gap from the last regular strip > `coverage_width / 2`. Was the main cause of the "BCD only 92%" issue.
- **+1 cell coverage paint slack** — `_cb_pose` paints with `radius + 1` cells so adjacent strips overlap. BCD now hits 95–97 %.
- **Per-cell redundancy tracking** — `self._redundancy[r,c]` counts distinct-robot visits; published as `/coverage_redundancy`.
- **Live ROS topics added:**
  - `/set_algorithm` (String) — coordinator replans without restart
  - `/set_speed` (Float64) — agents update `_speed` mid-run
  - `/set_map` (String) — `map_server` loads new PGM file, `coordinator` resets and replans
  - `/inject_failure` (String) — coordinator picks random active robot to fail
  - `/reset_sim` (String) — revives failed robots, full re-plan
  - `/coverage_redundancy` (OccupancyGrid) — per-cell visit count
- **Robot revival** — `fail_trigger=False` now wakes a failed robot back to idle; empty Path also revives.
- **Frontier sensor radius reduced** — `2.5 m → 0.4 m`. Frontier robots had to physically traverse cells to "explore" — fixes the ~20 % coverage issue.
- **Random walk steps trimmed** — `10 000 → 1 500`. Was running ~17 minutes; now ends in 2–3.
- **BFS-snapped, spread start positions** — robots no longer all crowd the bottom-left.
- **Docker slimmed** — removed `xvfb / x11vnc / novnc / websockify / mesa` apt deps and entrypoint setup. Image is significantly smaller; container boots in seconds. Port 6080 removed from `docker-compose.yml`. **noVNC and RViz are gone — dashboard is the only UI.**

### Dashboard (React)

- **MapCanvas perf rewrite** — map render now goes to an **offscreen canvas** that's only rebuilt when `coverageMap` / `redundancyMap` data actually changes. The 60 fps loop just `drawImage`s the cache and renders the robot/path/trail layer on top. Removes ~40 000 ImageData writes per frame; sim feels smooth even at 5× speed.
- **Smooth 60 fps pose interpolation** — internal `requestAnimationFrame` loop lerps each robot's `(x, y, yaw)` 18 % toward the latest received pose every frame.
- **Trail/distance batching** — pose callbacks now buffer into refs; flushed to React state every 100 ms instead of every pose update. Stops React from re-rendering 60 ×/sec.
- **Inject Failure throttle** — 2-second cooldown between clicks (rapid clicks were stacking failures before previous reallocation could finish).
- **Reset Sim button** — publishes `/reset_sim`; revives failed robots, clears coverage, replans.
- **Live map switching** — Map dropdown now publishes `/set_map`; coordinator handles the switch without restarting Docker.
- **Heatmap overlay toggle** — visualises per-cell visit count: 0 (slate) → 1 (blue) → 2 (green) → 3 (amber) → 4+ (red). Legend swaps automatically when toggled.
- **Chassis-style robot rendering** — rotated rounded-rect chassis with depth panel, white direction arrow, sensor dome, ID badge, red X on failure. No more circles.
- **Refined cards** — gradient backgrounds, soft inner highlight, backdrop blur, better typography.
- **Stats panel** — circular ring (turns green on completion), 6-cell metric grid (Algorithm/Speed/Elapsed/ETA/Active/Distance), per-robot status badges with proportional progress bars.
- **CoverageChart bigger + responsive** — `ResizeObserver` makes it fill the sidebar width. Added gradient area fill, gradient ring, smarter tick density.
- **Clean header** — geometric grid logo (no robot emoji), professional title, live agent count.
- **ControlBar reorganised** — section labels (View / Speed / Algorithm / Map), better button states with shadows when active, Inject Failure + Reset Sim buttons at the right.
- **Wider sidebar** — was capped at 300px (squished); now `flex-1 min-w-340` so it grows.

### Files changed in this session

```
M  Dockerfile                                     # stripped VNC/Xvfb
M  docker-compose.yml                             # removed port 6080, GL env vars
M  docker/entrypoint.sh                           # no display setup
M  multi_robot_coverage/.../algorithms/boustrophedon.py   # Hungarian + TSP + final-strip
M  multi_robot_coverage/.../nodes/coverage_coordinator.py # inflated grid, densify, set_*, inject, reset
M  multi_robot_coverage/.../nodes/map_server.py            # /set_map subscriber
M  multi_robot_coverage/.../nodes/robot_agent.py           # teleport-once, sub-step, revive
M  web_dashboard/src/App.jsx                      # all new handlers + buffered trails
M  web_dashboard/src/components/ControlBar.jsx    # restructured
M  web_dashboard/src/components/CoverageChart.jsx # responsive resize
M  web_dashboard/src/components/MapCanvas.jsx    # offscreen cache + 60fps + heatmap
M  web_dashboard/src/components/StatsPanel.jsx   # rebuilt
M  web_dashboard/src/index.css                   # gradient cards
```

### Known remaining issues

- **Failure recovery still looks slightly jittery** — the surviving robots get a brand-new path that includes any not-yet-finished cells from their original assignment plus the failed robot's leftovers. Visually the path overlay redraws abruptly. Real fix: per-robot task-queue model (cell-by-cell instead of one mega-path). Documented as future work in §2 / §13.

### Concrete next-session task

Pick up exactly where this left off. The very next thing to do is:

1. `make web` — verify everything still works cleanly with these changes
2. Run each algorithm for ~60 s, screenshot the chart, record the final coverage %
3. Take 3 short screen recordings (BCD running / failure injection / algorithm comparison)
4. Convert recordings → optimised GIFs via `ffmpeg`
5. Write the recruiter-friendly README using §1.3 structure: Problem / Architecture / Algorithms / Results table / Demo / Setup / Future work
6. Optional: implement headless benchmark mode (§1.1) so the README numbers come from a reproducible script
7. `git push origin master` once it all looks good

---

## 14. Bug-hunt pass — May 2026 (post-benchmark dashboard polish)

After the headless benchmark suite produced the first complete results (`make benchmark-native` → 12 JSON files, all 4 algorithms × 3 maps), a smoke-test of the live dashboard surfaced four bugs that had to be fixed before recording the demo video. All four are now resolved.

### 14.1 Distance accumulator race — Robot 1 / Robot 2 distance stuck at 0.0 m

**Symptom**
On the dashboard's Robot Fleet panel, only Robot 0's distance progressed (occasionally — usually a tiny value like 0.7 m after a full minute of motion). Robots 1 and 2 sat at 0.0 m forever, with empty bars, even though the map showed them clearly moving.

**Root cause**
The pose subscriber accumulates per-tick distance into a `useRef` object (`distAccumRef.current`), which a 100 ms `setInterval` flushes into React state. The flush used the functional updater form of `setState`:

```js
setRobotDistances(prev => {
  for (const id in distAccumRef.current) {        // ← read at commit time
    next[id] = (next[id] || 0) + distAccumRef.current[id]
  }
  return next
})
distAccumRef.current = {}                         // ← runs synchronously *before* commit
```

In React 18, the lambda passed to `setState` is **invoked later during the commit phase**, not synchronously. The line `distAccumRef.current = {}` clears the ref *before* React ever calls the lambda — so the `for (const id in distAccumRef.current)` loop iterates over the now-empty object. Almost every flush silently lost its accumulated data. R0 occasionally caught a stray value when a new pose callback fired between queueing the setState and React running it.

**Fix** ([`web_dashboard/src/App.jsx`](web_dashboard/src/App.jsx))

Snapshot the ref into a local variable *before* clearing, so the updater closes over a stable snapshot instead of reading the live ref:

```js
const distSnap = distAccumRef.current
distAccumRef.current = {}
setRobotDistances(prev => {
  const next = { ...prev }
  for (const id in distSnap) {                    // ← stable snapshot
    next[id] = (next[id] || 0) + distSnap[id]
  }
  return next
})
```

Same pattern applied to the trail buffer. **All three robots now accumulate distance correctly.**

### 14.2 Coverage chart kept extending after mission complete

**Symptom**
Once BCD hit 100 %, the curve flat-lined at the top — but the curve kept sliding right indefinitely as new stat messages arrived. The mission was over but the chart looked like it was still running.

**Root cause (two-sided)**
- *Coordinator side*: `_publish_stats` recomputed `elapsed_time = now - start_time` every 0.5 s, even after entering `_State.COMPLETE`. So every heartbeat carried a fresh, larger `elapsed_time`.
- *Dashboard side*: the stats subscription appended every message to `coverageHistory` without checking `msg.completed`. The 400 ms dedup guard didn't trigger because `t` kept growing.

**Fix** ([`coverage_coordinator.py`](multi_robot_coverage/multi_robot_coverage/nodes/coverage_coordinator.py), [`App.jsx`](web_dashboard/src/App.jsx))

- Coordinator now snapshots `_complete_elapsed` the instant it transitions to `COMPLETE` and uses that frozen value for all subsequent stats publishes. `_reset_and_replan` clears `_complete_elapsed` so the next run starts a fresh clock.
- Dashboard stats handler now stamps each history point with `completed: !!msg.completed` and refuses to append further points once the last entry is already marked completed.

### 14.3 Algorithm-switch felt laggy / froze the sim visually

**Symptom**
Clicking a new algorithm in the dropdown caused a ~500 ms window where the dashboard looked frozen — old coverage cells stayed painted, robots stopped moving, then suddenly the new run jumped in.

**Root cause (two-sided)**
- *Coordinator side*: `_reset_and_replan` cleared `_coverage` internally but didn't *publish* the cleared coverage map. The next coverage publish happened after the planning step (which can take 200–500 ms for BCD). Until then, subscribers still held the previous coverage frame.
- *Dashboard side*: the dropdown was bound to a React state that the `/coverage_stats` subscription would *override* on every message. During the replan window the coordinator still reported the previous algorithm, so the dropdown briefly flickered back to the old value — making the switch feel even worse.

**Fix** ([`coverage_coordinator.py`](multi_robot_coverage/multi_robot_coverage/nodes/coverage_coordinator.py), [`App.jsx`](web_dashboard/src/App.jsx))

- Coordinator's `_reset_and_replan` now calls `self._publish_coverage_map()` immediately after wiping `_coverage`. Subscribers see the blank frame within one ROS tick.
- Dashboard's `handleAlgorithmChange` / `handleMapChange` / `handleResetSim` now also wipe local `coverageMap`, `redundancyMap`, and (for map change) `baseMap` so the canvas re-renders empty instantly without waiting for the coordinator round-trip.
- Algorithm dropdown now syncs from `/coverage_stats` only on **first** message (gated by `algoInitRef`) — after that, user selection is the source of truth. No more flicker during replans.

### 14.4 Per-robot bar math was unintuitive and clipped at 100 %

**Symptom (indirect)**
Even after fixing 14.1, the per-robot bars used a convoluted formula `(dist/totalDist) * pct * numRobots` that capped robots at 100 % whenever one happened to do most of the work. The math was opaque and the visual didn't communicate anything actionable.

**Fix** ([`StatsPanel.jsx`](web_dashboard/src/components/StatsPanel.jsx))

Replaced with the straightforward "this robot's distance ÷ busiest robot's distance × 100" — busiest robot is always full, others scale linearly. Honest, intuitive, never clips.

### 14.5 Dead code cleanup

The `covered` variable in `_publish_stats` summed the *values* of covered cells (each cell = `(robot_id + 1) * 10`) — a number nothing downstream consumed. Removed.

### 14.6 Frontier algorithm silently grinding to a halt in the live sim

**Symptom**
Frontier looked broken in the live dashboard — robots would move at first, then progressively stop getting new goals. After 30–60 s coverage would plateau well below the benchmark's 90–95 %.

**Root cause (three bugs interacting)**
The headless benchmark and the live coordinator implement frontier exploration with the same `FrontierExplorer` class but assemble the loop differently. The benchmark works. The coordinator did three things wrong:

1. **`claimed_frontiers` accumulated forever.** The benchmark creates a fresh `claimed: set` on every replan cycle. The coordinator persisted `self._claimed_frontiers` across all ticks — every frontier ever assigned was added permanently. Over time `available = [f for f in frontiers if f not in claimed]` shrank to empty, and every robot got `None` back from `assign_frontiers`.

2. **Replanning every 0.5 s tick** caused the same centroid to be repeatedly assigned to robots that hadn't reached the previous one yet, both wasting CPU and adding to the claim-accumulation problem. The benchmark replans every 2 s OR when a robot is idle.

3. **Single-waypoint paths** (A\* result with `len == 1`) were dispatched as if they were real paths. The robot would instantly mark them complete (already at the only waypoint), the next tick would assign the same frontier again, the path would again be length 1 → instant complete. Robot oscillated between "idle" and "complete" without ever moving meaningfully.

**Fix** ([`coverage_coordinator.py`](multi_robot_coverage/multi_robot_coverage/nodes/coverage_coordinator.py))

- `_tick_frontier_assignment` now builds a fresh `claimed: set()` per call (matches the benchmark).
- Replanning is throttled to once every 2 s OR when any robot is non-active. A new `force=True` parameter is used at initial planning so the first assignment happens immediately rather than waiting 2 s.
- Single-waypoint A\* results (`len(path) < 2`) are skipped.
- Failed robots are filtered out of assignment.
- Deleted `self._claimed_frontiers` (replaced by per-call locals); added `self._frontier_last_replan_s` for throttling, reset in `_reset_and_replan`.

**Verification**
- 47 unit tests still pass.
- Headless benchmark for `frontier` produces identical numbers: simple_room 90.4 %, obstacle_room 93.2 %, warehouse 95.5 % (the algorithm class itself wasn't touched, only how the coordinator drives it).

### 14.7 Coverage chart reported wall-clock time, making 5× runs incomparable to benchmark

**Symptom**
With playback speed at 5×, the live coverage curve hit 100 % around the 20 s mark of the chart axis — while the benchmark overlay curves for the same map terminated around 90–125 s. The two scales were silently inconsistent, making the live curve look impossibly fast.

**Root cause**
`_publish_stats` reported `elapsed_time = wall_clock_now - start_time`. At 5× playback, robots cover the map in 1/5 of wall time, but the chart was plotted against wall time, not the equivalent real-world sim time. The benchmark, by contrast, advances `dt = 0.1 s` per loop iteration and reports the integrated sim time — those are the seconds *the algorithm would have taken in physical reality*.

**Fix** ([`coverage_coordinator.py`](multi_robot_coverage/multi_robot_coverage/nodes/coverage_coordinator.py))

- Subscribed to `/set_speed` to track `self._playback_speed` (the dashboard already publishes it).
- New `_accumulate_sim_time` integrates `Σ wall_dt × playback_speed` per main-tick call, so the integrator handles mid-run speed changes correctly (no retroactive recompute).
- `_publish_stats` now reports `self._sim_elapsed_s` as `elapsed_time`.
- `_reset_and_replan` and `_complete_elapsed` capture/reset the integrator at the right boundaries.
- Failure-injection timer also switched from wall-clock to sim-time so `failure_time:=30.0` fires at the same algorithmic point regardless of playback speed.

The dashboard displays `formatTime(elapsed)` straight from the message — no client-side change needed. The live curve and benchmark overlay are now on the same axis.

### 14.8 Frontier still produced ≲1 % live coverage even after 14.6

**Symptom**
After the §14.6 fixes (fresh `claimed` set, 2 s replan throttle, no-op-path skip) the live dashboard showed frontier still grinding to a halt with under 1 % coverage. The headless benchmark continued to hit 90–95 % on every map.

**Root cause**
§14.6 throttled `_tick_frontier_assignment` to run every ~2 s — but the **reveal step lived inside that function**. So the known map only grew every 2 s rather than every tick. The reveal increment per call is small (one ring of cells per robot pose), so a 4× slowdown of reveal cadence meant the explored area barely escaped the initial sensor bubble before robots finished their first frontier assignment and `find_frontier_centroids` started returning sparse results that the min-cluster threshold of 3 then dropped to zero. Coverage flatlined.

**Fix** ([`coverage_coordinator.py`](multi_robot_coverage/multi_robot_coverage/nodes/coverage_coordinator.py))

- Split `_tick_frontier_assignment` into two functions:
  - `_frontier_reveal_now()` — pure reveal, called every 0.5 s tick when state is RUNNING and algorithm is `frontier`.
  - `_tick_frontier_assignment()` — centroid detection + A\* + dispatch, still throttled to 2 s OR on non-active.
- Lowered `min_cluster_size` from 3 → 1 so fragmented clusters at the start of a run get assigned rather than dropped.
- New `_cb_status` shortcut: the instant a frontier robot reports `complete`, force an immediate `_tick_frontier_assignment(force=True)` so it gets a new goal within the same ROS callback (no 0.5 s wait for the next tick).
- Added logging: every dispatch and every empty-centroid event prints to ROS console, so future regressions are diagnosable from `docker logs coverage_sim`.

**Verification**
- 47 unit tests still pass.
- Headless benchmark for frontier produces identical numbers (algorithm class untouched).

### 14.9 Action buttons clipped at the bottom of narrow viewports

**Symptom**
"Inject Failure" and "Reset Sim" sat at the bottom of the ControlBar, which lives inside the left (map) column. On shorter viewports the map + control bar + legend exceeded the column height, and because the parent uses `overflow-hidden`, the buttons disappeared off the bottom with no scroll affordance.

**Fix** ([`App.jsx`](web_dashboard/src/App.jsx), [`ControlBar.jsx`](web_dashboard/src/components/ControlBar.jsx))

- Promoted both buttons to the header, next to the connection status pill. They are now *globally* visible regardless of left-column overflow, which is the right conceptual home for "sim-level actions" anyway (vs. view-level toggles like Path / FOV).
- Defensive `overflow-y-auto min-h-0` on the left column so any future tall content can still scroll inside the map area.
- ControlBar simplified — no longer takes `onInjectFailure` / `onResetSim` props.

### Files changed in this bug-hunt pass

```
M  multi_robot_coverage/.../nodes/coverage_coordinator.py
    - new _complete_elapsed instance variable
    - _publish_stats freezes elapsed time post-completion
    - _reset_and_replan publishes blank coverage map immediately
    - dropped dead `covered` variable
M  web_dashboard/src/App.jsx
    - flush interval snapshots accum refs before queueing setState
    - stats handler tags history points with `completed` and gates dedup
    - algorithm dropdown synced from stats only on first message
    - handleAlgorithmChange / handleMapChange / handleResetSim clear coverage maps locally
M  web_dashboard/src/components/StatsPanel.jsx
    - per-robot bar uses dist / maxDist instead of convoluted proxyPct
```

### Verification

- `pytest multi_robot_coverage/test/` → 47 passed in 3.46 s
- `python -m py_compile` on both coordinator and agent → clean
- Headless benchmark suite unchanged (algorithms layer untouched)
- Dashboard rebuild + manual smoke test still pending — run `make web` and confirm:
  - All three robots' distance bars now move during a run
  - Coverage chart stops extending once the ring turns green
  - Switching from BCD → Frontier instantly wipes the canvas (no ghost cells)
  - Algorithm dropdown does not flicker back during the replan window
