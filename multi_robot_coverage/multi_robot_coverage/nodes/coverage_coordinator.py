"""Coverage coordinator node — central planner and task manager.

Lifecycle
---------
1. WAITING_FOR_MAP : subscribes to /map, waits for the first grid message.
2. PLANNING        : runs the selected algorithm; generates per-robot paths.
3. RUNNING         : publishes waypoints; monitors progress; updates coverage.
4. FAILURE         : redistributes remaining cells if a robot dies.
5. COMPLETE        : all free cells covered; publishes final stats.

ROS2 Topics
-----------
Subscribed
~~~~~~~~~~
  /map                       (nav_msgs/OccupancyGrid)  — occupancy grid
  /robot_{id}/pose           (geometry_msgs/PoseStamped) — robot position
  /robot_{id}/status         (std_msgs/String)           — idle|active|…

Published
~~~~~~~~~
  /robot_{id}/waypoints      (nav_msgs/Path)             — path assignment
  /robot_{id}/fail_trigger   (std_msgs/Bool)             — failure injection
  /coverage_map              (nav_msgs/OccupancyGrid)    — who covered what
  /coverage_stats            (multi_robot_coverage_msgs/CoverageStats)
  /algorithm_comparison      (multi_robot_coverage_msgs/AlgorithmComparison)

ROS2 Parameters
---------------
  num_robots         int    — number of robots (default 3)
  algorithm          str    — "boustrophedon" | "frontier" | "random_walk" | "simple_boustrophedon"
  map_name           str    — used only for display
  robot_speed        float  — forwarded to robot nodes
  enable_failure_sim bool   — randomly kill one robot mid-mission
  failure_time       float  — seconds after start to trigger failure
  failure_robot_id   int    — robot to kill (-1 = pick randomly)
  coverage_width_m   float  — lawnmower strip width in metres (default 0.4)
  robot_radius_m     float  — robot radius for A* inflation (default 0.2)
  resolution         float  — map resolution m/px, must match map YAML
"""

from __future__ import annotations

import math
import random
import time
from enum import Enum, auto
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Bool, String

from multi_robot_coverage_msgs.msg import AlgorithmComparison, CoverageStats

from multi_robot_coverage.algorithms.astar import AStar
from multi_robot_coverage.algorithms.boustrophedon import (
    BoustrophedonDecomposer,
    CoverageCell,
)
from multi_robot_coverage.algorithms.frontier_based import FrontierExplorer
from multi_robot_coverage.algorithms.random_walk import RandomWalkPlanner
from multi_robot_coverage.algorithms.simple_boustrophedon import SimpleBoustrophedonPlanner


class _State(Enum):
    WAITING_FOR_MAP = auto()
    PLANNING = auto()
    RUNNING = auto()
    FAILURE = auto()
    COMPLETE = auto()


# Colour encoding for coverage_map (value = robot_id * 10, 0 = uncovered)
_UNCOVERED = 0
_OBSTACLE = 100


class CoverageCoordinatorNode(Node):
    """Central planner that assigns coverage paths and tracks progress."""

    def __init__(self) -> None:
        super().__init__("coverage_coordinator")

        # ------------------------------------------------------------------
        # Declare parameters
        # ------------------------------------------------------------------
        self.declare_parameter("num_robots", 3)
        self.declare_parameter("algorithm", "boustrophedon")
        self.declare_parameter("map_name", "simple_room")
        self.declare_parameter("robot_speed", 1.0)
        self.declare_parameter("enable_failure_sim", False)
        self.declare_parameter("failure_time", 30.0)
        self.declare_parameter("failure_robot_id", -1)
        self.declare_parameter("coverage_width_m", 0.4)
        self.declare_parameter("robot_radius_m", 0.2)
        self.declare_parameter("resolution", 0.05)

        self._n: int = self.get_parameter("num_robots").value
        self._algorithm: str = self.get_parameter("algorithm").value
        self._speed: float = self.get_parameter("robot_speed").value
        self._fail_sim: bool = self.get_parameter("enable_failure_sim").value
        self._fail_time: float = self.get_parameter("failure_time").value
        self._fail_robot: int = self.get_parameter("failure_robot_id").value
        self._cov_width_m: float = self.get_parameter("coverage_width_m").value
        self._radius_m: float = self.get_parameter("robot_radius_m").value
        self._resolution: float = self.get_parameter("resolution").value

        # ------------------------------------------------------------------
        # QoS profiles
        # ------------------------------------------------------------------
        _tl = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ------------------------------------------------------------------
        # Subscriptions
        # ------------------------------------------------------------------
        self._sub_map = self.create_subscription(
            OccupancyGrid, "/map", self._cb_map, _tl
        )
        self._pose_subs: list = []
        self._status_subs: list = []
        for i in range(self._n):
            self._pose_subs.append(
                self.create_subscription(
                    PoseStamped,
                    f"/robot_{i}/pose",
                    lambda msg, rid=i: self._cb_pose(msg, rid),
                    10,
                )
            )
            self._status_subs.append(
                self.create_subscription(
                    String,
                    f"/robot_{i}/status",
                    lambda msg, rid=i: self._cb_status(msg, rid),
                    _tl,
                )
            )

        # Live control — dashboard can switch algorithm without restarting sim
        self._sub_set_algorithm = self.create_subscription(
            String, '/set_algorithm', self._cb_set_algorithm, 10
        )
        # Live failure injection from dashboard
        self._sub_inject_failure = self.create_subscription(
            String, '/inject_failure', self._cb_inject_failure, 10
        )
        # Full sim reset (revives failed robots + replans)
        self._sub_reset_sim = self.create_subscription(
            String, '/reset_sim', self._cb_reset_sim, 10
        )
        # Forwarded /set_map echo — tells us to expect a new map on /map
        self._sub_set_map_echo = self.create_subscription(
            String, '/set_map', self._cb_set_map_echo, 10
        )
        self._expecting_new_map = False

        # ------------------------------------------------------------------
        # Publishers
        # ------------------------------------------------------------------
        self._pub_waypoints: list = [
            self.create_publisher(Path, f"/robot_{i}/waypoints", _tl)
            for i in range(self._n)
        ]
        self._pub_fail: list = [
            self.create_publisher(Bool, f"/robot_{i}/fail_trigger", 10)
            for i in range(self._n)
        ]
        self._pub_cov_map = self.create_publisher(
            OccupancyGrid, "/coverage_map", _tl
        )
        self._pub_redundancy = self.create_publisher(
            OccupancyGrid, "/coverage_redundancy", _tl
        )
        self._pub_stats = self.create_publisher(
            CoverageStats, "/coverage_stats", 10
        )
        self._pub_comparison = self.create_publisher(
            AlgorithmComparison, "/algorithm_comparison", 10
        )

        # ------------------------------------------------------------------
        # State
        # ------------------------------------------------------------------
        self._state = _State.WAITING_FOR_MAP
        self._grid: Optional[np.ndarray] = None   # raw occ grid (0/100)
        self._grid_msg: Optional[OccupancyGrid] = None
        self._coverage: Optional[np.ndarray] = None  # 0=uncov, N*10=robot
        self._robot_poses: dict[int, tuple[float, float]] = {}
        self._robot_statuses: dict[int, str] = {i: "idle" for i in range(self._n)}
        self._robot_remaining_cells: dict[int, list[CoverageCell]] = {}
        self._failed_robots: set[int] = set()
        self._failure_triggered = False
        self._start_time: Optional[float] = None
        # Elapsed time frozen at the instant we entered COMPLETE — prevents
        # the dashboard chart from sliding right after the mission ended.
        self._complete_elapsed: Optional[float] = None
        self._total_free: int = 0

        # Frontier-specific state
        self._frontier_explorer: Optional[FrontierExplorer] = None
        # Last time we ran a frontier replan, used to throttle to ~2 s like
        # the benchmark does — replanning every 0.5 s tick accumulates churn
        # and starves the actual motion ticks of CPU on busy maps.
        self._frontier_last_replan_s: float = 0.0

        self._timer = self.create_timer(0.5, self._tick)
        self.get_logger().info(
            f"Coordinator ready — algorithm={self._algorithm}, "
            f"robots={self._n}, failure_sim={self._fail_sim}"
        )

    # ------------------------------------------------------------------
    # Map callback
    # ------------------------------------------------------------------

    def _cb_map(self, msg: OccupancyGrid) -> None:
        # Accept the map at startup, OR when /set_map was just received
        # (dashboard requested a switch).  Otherwise this is the 1Hz
        # republish keep-alive and we ignore it.
        is_initial = self._state == _State.WAITING_FOR_MAP
        if not (is_initial or self._expecting_new_map):
            return

        if self._expecting_new_map:
            self._expecting_new_map = False
            self.get_logger().info("New map received — full reset and replan")
            # Revive any failed robots so they participate in the new run
            for rid in list(self._failed_robots):
                b = Bool()
                b.data = False
                self._pub_fail[rid].publish(b)
            self._failed_robots.clear()

        self._grid_msg = msg
        rows = msg.info.height
        cols = msg.info.width
        arr = np.array(msg.data, dtype=np.int8).reshape(rows, cols)
        # OccupancyGrid stores row 0 at the bottom of the world.
        # Internally we use row 0 = top (to match _world_to_grid convention),
        # so flip Y now and flip back only when re-publishing.
        arr = np.flipud(arr)
        # Convert to simple 0/100 grid.
        raw_grid = np.where(arr > 50, 100, np.where(arr < 0, 0, arr)).astype(
            np.uint8
        )
        self._resolution = msg.info.resolution

        # Inflate obstacles by robot radius so all planning (BCD, A*, lawnmower)
        # automatically respects the robot footprint.  This is what makes paths
        # truly collision-free regardless of which algorithm generates them.
        inflation_cells = max(1, int(self._radius_m / self._resolution))
        self._grid = self._inflate_obstacles(raw_grid, inflation_cells)
        self._raw_grid = raw_grid   # kept for visualisation only

        self._coverage = np.zeros((rows, cols), dtype=np.uint8)
        # Per-cell count of distinct robots that have visited — used for
        # the redundancy heatmap.  0 = unvisited, 1 = single-cover, 2+ = redundant.
        self._redundancy = np.zeros((rows, cols), dtype=np.uint8)
        # Mark inflated obstacles in coverage map (the safety buffer is
        # intentionally rendered as obstacle — it tells the user where the
        # robot literally cannot fit).
        self._coverage[self._grid >= 50] = _OBSTACLE
        self._total_free = int(np.sum(self._grid < 50))
        self._state = _State.PLANNING
        self.get_logger().info(
            f"Map received ({cols}×{rows}).  Starting {self._algorithm} planning…"
        )

    # ------------------------------------------------------------------
    # Robot callbacks
    # ------------------------------------------------------------------

    def _cb_pose(self, msg: PoseStamped, robot_id: int) -> None:
        x = msg.pose.position.x
        y = msg.pose.position.y
        self._robot_poses[robot_id] = (x, y)
        if self._coverage is None or self._grid is None:
            return
        # Mark coverage: paint cells within robot radius (+1 cell slack) as
        # covered.  The +1 ensures adjacent lawnmower strips overlap by 1 cell
        # so no thin gaps survive between them.
        r, c = self._world_to_grid(x, y)
        rad = max(1, int(self._radius_m / self._resolution) + 1)
        rows, cols = self._grid.shape
        r_min = max(0, r - rad)
        r_max = min(rows - 1, r + rad)
        c_min = max(0, c - rad)
        c_max = min(cols - 1, c + rad)
        stamp = (robot_id + 1) * 10  # 10, 20, 30, …
        for rr in range(r_min, r_max + 1):
            for cc in range(c_min, c_max + 1):
                if not (math.hypot(rr - r, cc - c) <= rad):
                    continue
                if self._grid[rr, cc] >= 50:
                    continue
                cur = self._coverage[rr, cc]
                if cur == _UNCOVERED:
                    self._coverage[rr, cc] = stamp
                    self._redundancy[rr, cc] = 1
                elif cur != stamp and cur != _OBSTACLE:
                    # Covered before by a *different* robot — increment.
                    if self._redundancy[rr, cc] < 250:
                        self._redundancy[rr, cc] += 1

    def _cb_status(self, msg: String, robot_id: int) -> None:
        self._robot_statuses[robot_id] = msg.data
        if msg.data == "failed" and robot_id not in self._failed_robots:
            self._failed_robots.add(robot_id)
            self.get_logger().warn(
                f"Robot {robot_id} reported FAILED — initiating reallocation"
            )
            self._state = _State.FAILURE

    def _cb_set_map_echo(self, _msg: String) -> None:
        """Mark that we expect the next /map message to be a different map."""
        self._expecting_new_map = True

    def _cb_reset_sim(self, _msg: String) -> None:
        """Full reset — revive any failed robots and replan from scratch."""
        if self._grid is None:
            return
        self.get_logger().info("🔄 Full sim reset requested")
        # Revive every failed robot.  fail_trigger=False means "back to life".
        for rid in list(self._failed_robots):
            b = Bool()
            b.data = False
            self._pub_fail[rid].publish(b)
        self._failed_robots.clear()
        self._reset_and_replan()

    def _cb_inject_failure(self, msg: String) -> None:
        """Dashboard-triggered failure injection.

        Payload: empty string or "auto" picks a random active robot;
        otherwise the integer robot ID to fail.
        """
        active = [
            i for i in range(self._n)
            if self._robot_statuses.get(i) == "active"
            and i not in self._failed_robots
        ]
        if not active:
            self.get_logger().warn("inject_failure: no active robots to fail")
            return

        target: int
        payload = (msg.data or "").strip().lower()
        if payload in ("", "auto"):
            target = random.choice(active)
        else:
            try:
                target = int(payload)
            except ValueError:
                self.get_logger().warn(f"inject_failure: bad payload '{payload}'")
                return
            if target not in active:
                self.get_logger().warn(
                    f"inject_failure: robot {target} not active"
                )
                return

        self.get_logger().warn(f"💥 Injecting failure into robot {target}")
        self._failure_triggered = True
        b = Bool()
        b.data = True
        self._pub_fail[target].publish(b)

    def _cb_set_algorithm(self, msg: String) -> None:
        """Switch algorithm and replan without restarting the container."""
        new_algo = msg.data.strip()
        valid = {"boustrophedon", "frontier", "random_walk", "simple_boustrophedon"}
        if new_algo not in valid:
            self.get_logger().warn(f"Unknown algorithm '{new_algo}' — ignored")
            return
        if new_algo == self._algorithm and self._state not in (
            _State.COMPLETE, _State.WAITING_FOR_MAP
        ):
            return   # already running this algorithm, nothing to do
        self.get_logger().info(f"Switching algorithm: {self._algorithm} → {new_algo}")
        self._algorithm = new_algo
        self._reset_and_replan()

    def _reset_and_replan(self) -> None:
        """Reset coverage state and trigger a fresh planning cycle."""
        if self._grid is None or self._grid_msg is None:
            return
        rows, cols = self._grid.shape
        self._coverage = np.zeros((rows, cols), dtype=np.uint8)
        self._coverage[self._grid >= 50] = _OBSTACLE
        self._redundancy = np.zeros((rows, cols), dtype=np.uint8)
        self._robot_remaining_cells = {}
        self._failed_robots = set()
        self._failure_triggered = False
        self._start_time = None
        self._complete_elapsed = None
        self._robot_statuses = {i: "idle" for i in range(self._n)}
        self._frontier_explorer = None
        self._frontier_last_replan_s = 0.0
        # Send empty paths to reset all robots to idle
        for i in range(self._n):
            empty = Path()
            empty.header.stamp = self.get_clock().now().to_msg()
            empty.header.frame_id = "map"
            self._pub_waypoints[i].publish(empty)
        # IMMEDIATELY publish the cleared coverage map so subscribers see the
        # wipe right away — otherwise the dashboard sits on the old painted
        # cells until the next RUNNING-state publish, which makes algorithm
        # switches feel like they froze the sim.
        self._publish_coverage_map()
        self._state = _State.PLANNING

    # ------------------------------------------------------------------
    # Main tick
    # ------------------------------------------------------------------

    def _tick(self) -> None:
        if self._state == _State.PLANNING:
            self._run_planning()
        elif self._state == _State.RUNNING:
            self._tick_running()
        elif self._state == _State.FAILURE:
            self._handle_failure()
        elif self._state == _State.COMPLETE:
            self._publish_stats(final=True)
            return

        if self._state in (_State.RUNNING, _State.COMPLETE):
            self._publish_coverage_map()
            self._publish_stats()

    # ------------------------------------------------------------------
    # Planning
    # ------------------------------------------------------------------

    def _run_planning(self) -> None:
        if self._grid is None:
            return
        self._start_time = self.get_clock().now().nanoseconds * 1e-9

        if self._algorithm == "boustrophedon":
            self._plan_boustrophedon()
        elif self._algorithm == "random_walk":
            self._plan_random_walk()
        elif self._algorithm == "simple_boustrophedon":
            self._plan_simple_boustrophedon()
        else:
            self._plan_frontier()

        self._state = _State.RUNNING

    def _plan_boustrophedon(self) -> None:
        assert self._grid is not None
        cw = max(1, int(self._cov_width_m / self._resolution))
        # Grid is already inflated, so planner uses 0 here
        inflation = 0
        decomposer = BoustrophedonDecomposer(
            coverage_width=cw, occupied_threshold=50
        )
        planner = AStar()

        cells = decomposer.decompose(self._grid)
        self.get_logger().info(f"BCD produced {len(cells)} cells")

        # Hungarian assignment + nearest-neighbour TSP per robot
        # → minimises max workload AND inter-cell travel
        starts = self._default_start_positions()
        assignment = decomposer.assign_to_robots(
            cells, self._n,
            robot_starts=starts,
            method="hungarian",
        )
        self._robot_remaining_cells = {
            rid: list(clist) for rid, clist in assignment.items()
        }

        for robot_id, robot_cells in assignment.items():
            path_poses = self._build_full_path(
                robot_id, robot_cells, decomposer, planner, inflation
            )
            self._send_waypoints(robot_id, path_poses)

    def _plan_random_walk(self) -> None:
        """Baseline: each robot performs a biased random walk."""
        assert self._grid is not None
        starts = self._default_start_positions()
        # Random walk is a baseline — give it enough steps to be measurable
        # but not so many it runs forever.  ~1500 steps × 0.1 m = 150 m of travel
        # which is roughly 2-3 minutes at 1 m/s and converges to ~55-65% coverage.
        planner = RandomWalkPlanner(num_steps=1500, step_size=2, seed=42)
        paths = planner.assign_to_robots(self._grid, self._n, starts)
        for robot_id, grid_path in paths.items():
            world_path = [self._grid_to_world(*p) for p in grid_path]
            self._send_waypoints(robot_id, world_path)
        self.get_logger().info("Random-walk paths generated")

    def _plan_simple_boustrophedon(self) -> None:
        """Comparison: naive lawnmower with no cellular decomposition."""
        assert self._grid is not None
        cw = max(1, int(self._cov_width_m / self._resolution))
        inflation = 0   # grid is pre-inflated in _cb_map
        planner = SimpleBoustrophedonPlanner(coverage_width=cw)
        grid_paths = planner.generate_paths(self._grid, self._n)
        astar = AStar()
        for robot_id, grid_path in grid_paths.items():
            # Densify: add A* transitions where gaps would cross obstacles
            dense = self._densify_path(grid_path, astar, inflation_radius=0)
            world_path = [self._grid_to_world(*p) for p in dense]
            self._send_waypoints(robot_id, world_path)
        self.get_logger().info(
            f"Simple-boustrophedon paths generated ({self._n} robots)"
        )

    def _plan_frontier(self) -> None:
        assert self._grid is not None
        # Sensor radius governs how much area each robot "reveals" per step.
        # If we make it too large (e.g. 2.5 m), the robot doesn't need to
        # physically traverse cells to consider them explored — coverage
        # (cells *painted* by the robot footprint) ends up ~20%.  We use a
        # smaller radius (~0.4 m, just over the robot footprint) so frontier
        # exploration actually requires physical coverage.
        sensor_r = max(5, int(0.4 / self._resolution))  # ~0.4 m = 8 cells
        self._frontier_explorer = FrontierExplorer(
            self._grid, sensor_radius=sensor_r
        )
        # Reveal map around random start positions.
        starts = self._default_start_positions()
        self._frontier_explorer.reveal(starts)
        for i, (r, c) in enumerate(starts):
            x, y = self._grid_to_world(r, c)
            self._robot_poses[i] = (x, y)
            self.get_logger().info(
                f"Frontier: robot {i} starts at grid ({r},{c})"
            )
        # Force the first assignment so robots start moving immediately.
        self._frontier_last_replan_s = 0.0
        self._tick_frontier_assignment(force=True)

    # ------------------------------------------------------------------
    # Boustrophedon helpers
    # ------------------------------------------------------------------

    def _build_full_path(
        self,
        robot_id: int,
        cells: list[CoverageCell],
        decomposer: BoustrophedonDecomposer,
        planner: AStar,
        inflation: int,
        start_pos: Optional[tuple[int, int]] = None,
    ) -> list[tuple[float, float]]:
        """Concatenate lawnmower + A* inter-cell segments into world coords.

        Parameters
        ----------
        start_pos : (row, col) or None
            Grid cell to start planning from.  When ``None`` the default
            start position is used (initial planning).  During failure
            recovery this is set to the surviving robot's *current* grid
            position so the new path begins where the robot actually is —
            no teleport-to-start.
        """
        assert self._grid is not None
        world_path: list[tuple[float, float]] = []
        prev_grid: Optional[tuple[int, int]] = None

        if start_pos is not None:
            prev_grid = self._snap_to_free(*start_pos)
        else:
            starts = self._default_start_positions()
            if robot_id < len(starts):
                prev_grid = starts[robot_id]

        for cell in cells:
            sweep = decomposer.generate_path(cell, self._grid)
            if not sweep:
                continue

            # A* transit from last position to first sweep point.
            if prev_grid is not None:
                transit = planner.search(
                    self._grid, prev_grid, sweep[0], inflation_radius=inflation
                )
                if transit:
                    for gpt in transit[1:]:
                        world_path.append(self._grid_to_world(*gpt))

            # Densify lawnmower strip: fill gaps so the robot never
            # interpolates across an obstacle.  Use inflation=0 here because
            # the lawnmower waypoints are already confirmed free cells — full
            # inflation would block the start/goal cells themselves.
            dense_sweep = self._densify_path(sweep, planner, inflation_radius=0)
            for gpt in dense_sweep:
                world_path.append(self._grid_to_world(*gpt))
            prev_grid = sweep[-1] if sweep else prev_grid

        return world_path

    def _densify_path(
        self,
        grid_path: list[tuple[int, int]],
        planner: AStar,
        inflation_radius: int = 0,
    ) -> list[tuple[int, int]]:
        """Ensure no two consecutive waypoints linearly cross an obstacle.

        Two-stage guarantee:
          1. **Gap bridging** — any pair > √2 cells apart is bridged with A*.
             We always check distance from the *last accepted point* (not the
             original neighbour), so dropping a waypoint can never silently
             create a longer cross-obstacle jump on the next iteration.
          2. **Bresenham validation** — every accepted segment is line-checked
             against the obstacle grid; any segment that would graze an
             obstacle is replaced with an A* detour.

        Returns a path where every consecutive segment is provably free of
        obstacles even under linear interpolation.
        """
        if self._grid is None or len(grid_path) < 2:
            return list(grid_path)

        # Stage 1: gap bridging (uses dense[-1] as prev — fixes chained-skip bug)
        dense: list[tuple[int, int]] = [grid_path[0]]
        for i in range(1, len(grid_path)):
            curr = grid_path[i]
            prev = dense[-1]
            dist = math.hypot(curr[0] - prev[0], curr[1] - prev[1])

            if dist > 1.5:
                bridge = planner.search(
                    self._grid, prev, curr, inflation_radius=inflation_radius
                ) or planner.search(
                    self._grid, prev, curr, inflation_radius=0
                )
                if bridge and len(bridge) > 1:
                    dense.extend(bridge[1:])
                # else: skip curr — next iteration will check from same prev
            else:
                dense.append(curr)

        # Stage 2: line-of-sight validation
        return self._validate_segments(dense, planner)

    def _validate_segments(
        self,
        path: list[tuple[int, int]],
        planner: AStar,
    ) -> list[tuple[int, int]]:
        """Replace any obstacle-crossing segment with an A* detour."""
        if self._grid is None or len(path) < 2:
            return path

        validated: list[tuple[int, int]] = [path[0]]
        for i in range(1, len(path)):
            prev = validated[-1]
            curr = path[i]
            if self._segment_clear(prev, curr):
                validated.append(curr)
                continue
            # Segment crosses an obstacle — try to find a detour
            detour = planner.search(self._grid, prev, curr, inflation_radius=0)
            if detour and len(detour) > 1:
                validated.extend(detour[1:])
            # else: drop this waypoint silently (no safe path possible)
        return validated

    def _segment_clear(
        self,
        a: tuple[int, int],
        b: tuple[int, int],
    ) -> bool:
        """Bresenham line check — returns True iff every cell on the segment
        from *a* to *b* is non-obstacle in self._grid."""
        if self._grid is None:
            return True
        r0, c0 = a
        r1, c1 = b
        dr = abs(r1 - r0)
        dc = abs(c1 - c0)
        sr = 1 if r0 < r1 else -1
        sc = 1 if c0 < c1 else -1
        err = dr - dc
        rows, cols = self._grid.shape
        r, c = r0, c0
        while True:
            if not (0 <= r < rows and 0 <= c < cols):
                return False
            if self._grid[r, c] >= 50:
                return False
            if (r, c) == (r1, c1):
                return True
            e2 = 2 * err
            if e2 > -dc:
                err -= dc
                r += sr
            if e2 < dr:
                err += dr
                c += sc

    @staticmethod
    def _inflate_obstacles(grid: np.ndarray, radius: int) -> np.ndarray:
        """Dilate obstacles by *radius* cells using a square structuring element."""
        if radius <= 0:
            return grid.copy()
        try:
            from scipy.ndimage import binary_dilation

            mask = grid >= 50
            struct = np.ones((2 * radius + 1, 2 * radius + 1), dtype=bool)
            inflated = binary_dilation(mask, structure=struct)
            out = grid.copy()
            out[inflated] = 100
            return out
        except ImportError:
            return grid.copy()

    # ------------------------------------------------------------------
    # Frontier-specific tick (called every timer tick while running)
    # ------------------------------------------------------------------

    def _tick_frontier_assignment(self, force: bool = False) -> None:
        """Reveal around current robot positions and reassign frontiers.

        Replanning is throttled to once every ~2 s OR whenever any robot is
        non-active (idle/complete and ready for a new goal), matching the
        benchmark simulator's loop.  Replanning every 0.5 s tick was both
        wasteful and caused subtle churn because the same centroid kept
        getting reassigned to robots that hadn't reached it yet.
        """
        if self._frontier_explorer is None or self._grid is None:
            return

        now_s = self.get_clock().now().nanoseconds * 1e-9
        non_active_present = any(
            self._robot_statuses.get(i) not in ("active", "failed")
            for i in range(self._n)
            if i not in self._failed_robots
        )
        if not (
            force
            or non_active_present
            or now_s - self._frontier_last_replan_s >= 2.0
        ):
            return
        self._frontier_last_replan_s = now_s

        # Update sensor reveals from all current robot positions
        # (failed robots' stale poses are still valid — they don't move).
        grid_poses = [
            self._world_to_grid(*self._robot_poses.get(i, (0.0, 0.0)))
            for i in range(self._n)
        ]
        self._frontier_explorer.reveal(grid_poses)

        centroids = self._frontier_explorer.find_frontier_centroids()
        if not centroids:
            return  # nothing left to explore — coverage tick will catch up

        # CRITICAL: use a fresh `claimed` set per replan cycle.  Persisting
        # the set across ticks (the old behaviour) accumulated every frontier
        # ever assigned, until `available` was empty and robots could no
        # longer be assigned anything — frontier silently ground to a halt
        # after 30-60 s, which looked like the algorithm was broken.  The
        # headless benchmark gets this right, the coordinator didn't.
        claimed: set[tuple[int, int]] = set()
        assignments = FrontierExplorer.assign_frontiers(
            centroids, grid_poses, claimed
        )

        planner = AStar()
        for robot_id, frontier in assignments.items():
            if frontier is None:
                continue
            if robot_id in self._failed_robots:
                continue
            if self._robot_statuses.get(robot_id) == "active":
                continue  # robot still working on previous goal
            current_grid = self._world_to_grid(
                *self._robot_poses.get(robot_id, (0.0, 0.0))
            )
            path = planner.search(
                self._grid, current_grid, frontier, inflation_radius=0
            )
            # Skip trivial/no-op paths — a single-waypoint path makes the
            # robot complete instantly and starves the next replan window.
            if not path or len(path) < 2:
                continue
            world_poses = [self._grid_to_world(*p) for p in path]
            self._send_waypoints(robot_id, world_poses)

    # ------------------------------------------------------------------
    # Running tick
    # ------------------------------------------------------------------

    def _tick_running(self) -> None:
        # Check for failure simulation.
        if (
            self._fail_sim
            and not self._failure_triggered
            and self._start_time is not None
        ):
            elapsed = self.get_clock().now().nanoseconds * 1e-9 - self._start_time
            if elapsed >= self._fail_time:
                self._trigger_failure()

        # Frontier: reassign whenever a robot completes its current goal.
        if self._algorithm == "frontier":
            self._tick_frontier_assignment()

        # Check completion.
        if self._all_complete():
            self._state = _State.COMPLETE
            # Freeze elapsed time at the moment of completion
            if self._start_time is not None and self._complete_elapsed is None:
                self._complete_elapsed = (
                    self.get_clock().now().nanoseconds * 1e-9 - self._start_time
                )
            self.get_logger().info("Coverage COMPLETE!")

    def _tick_running_with_failure_check(self) -> None:
        self._tick_running()

    def _all_complete(self) -> bool:
        active = [
            i
            for i in range(self._n)
            if i not in self._failed_robots
        ]
        return all(
            self._robot_statuses.get(i, "idle") == "complete" for i in active
        )

    # ------------------------------------------------------------------
    # Failure handling
    # ------------------------------------------------------------------

    def _trigger_failure(self) -> None:
        self._failure_triggered = True
        if self._fail_robot < 0:
            active = [
                i
                for i in range(self._n)
                if self._robot_statuses.get(i) == "active"
            ]
            if not active:
                return
            target = random.choice(active)
        else:
            target = self._fail_robot

        self.get_logger().warn(f"Injecting failure into robot {target}")
        msg = Bool()
        msg.data = True
        self._pub_fail[target].publish(msg)

    def _handle_failure(self) -> None:
        if self._grid is None:
            return
        for failed_id in list(self._failed_robots):
            if failed_id not in self._robot_remaining_cells:
                continue
            remaining = self._robot_remaining_cells.pop(failed_id, [])
            if not remaining:
                continue

            grid_positions = {
                rid: self._world_to_grid(*self._robot_poses.get(rid, (0.0, 0.0)))
                for rid in self._robot_remaining_cells
            }
            grid_positions[failed_id] = self._world_to_grid(
                *self._robot_poses.get(failed_id, (0.0, 0.0))
            )

            decomposer = BoustrophedonDecomposer(
                coverage_width=max(1, int(self._cov_width_m / self._resolution))
            )
            self._robot_remaining_cells = decomposer.reallocate_failed_robot(
                failed_robot_id=failed_id,
                remaining_cells=remaining,
                active_assignments=self._robot_remaining_cells,
                robot_positions=grid_positions,
            )

            planner = AStar()
            inflation = 0   # grid is pre-inflated in _cb_map
            for robot_id, cells in self._robot_remaining_cells.items():
                if robot_id == failed_id:
                    continue
                # Filter out cells that are already > 70% covered — saves the
                # surviving robot from re-driving work it already finished.
                cells_to_do = self._filter_uncompleted_cells(cells)
                if not cells_to_do:
                    continue
                # Plan from the robot's CURRENT pose so it doesn't teleport.
                current_pos = grid_positions.get(robot_id)
                path_poses = self._build_full_path(
                    robot_id, cells_to_do, decomposer, planner, inflation,
                    start_pos=current_pos,
                )
                if path_poses:
                    self._send_waypoints(robot_id, path_poses)
                    self.get_logger().info(
                        f"Reallocated → robot {robot_id}: "
                        f"{len(cells_to_do)}/{len(cells)} cells still need work"
                    )

        self._state = _State.RUNNING

    def _filter_uncompleted_cells(
        self, cells: list[CoverageCell]
    ) -> list[CoverageCell]:
        """Return cells with ≥ 30% of their points still uncovered.

        Used during failure recovery so surviving robots skip re-doing
        cells they had already finished before the failure occurred.
        """
        if self._coverage is None:
            return list(cells)
        result: list[CoverageCell] = []
        for cell in cells:
            if not cell.points:
                continue
            uncovered = sum(
                1 for (r, c) in cell.points
                if self._coverage[r, c] == _UNCOVERED
            )
            if uncovered >= cell.area * 0.30:
                result.append(cell)
        return result

    # ------------------------------------------------------------------
    # Waypoint publishing
    # ------------------------------------------------------------------

    def _send_waypoints(
        self, robot_id: int, world_poses: list[tuple[float, float]]
    ) -> None:
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "map"
        for x, y in world_poses:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self._pub_waypoints[robot_id].publish(path)

    # ------------------------------------------------------------------
    # Coverage map publisher
    # ------------------------------------------------------------------

    def _publish_coverage_map(self) -> None:
        if self._coverage is None or self._grid_msg is None:
            return
        # Coverage map (who covered what)
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info = self._grid_msg.info
        # Flip Y back to OccupancyGrid convention (row 0 = bottom) before
        # publishing, since internally we store row 0 = top.
        display = np.flipud(self._coverage.copy()).astype(np.int8)
        msg.data = display.flatten().tolist()
        self._pub_cov_map.publish(msg)

        # Redundancy heatmap (how many distinct robots have visited each cell)
        if self._redundancy is not None:
            rmsg = OccupancyGrid()
            rmsg.header = msg.header
            rmsg.info   = msg.info
            # Mask obstacles → 255 (which becomes -1 when reinterpreted as int8)
            r_internal = np.where(
                self._grid >= 50, 255, self._redundancy
            ).astype(np.uint8)
            # Flip once: internal row-0=top → ROS row-0=bottom
            r_disp = np.flipud(r_internal).view(np.int8)
            rmsg.data = r_disp.flatten().tolist()
            self._pub_redundancy.publish(rmsg)

    # ------------------------------------------------------------------
    # Stats publisher
    # ------------------------------------------------------------------

    def _publish_stats(self, final: bool = False) -> None:
        if self._total_free == 0 or self._coverage is None:
            return
        covered_cells = int(
            np.sum(
                (self._coverage > _UNCOVERED) & (self._coverage < _OBSTACLE)
            )
        )
        pct = 100.0 * covered_cells / self._total_free if self._total_free else 0.0

        # Elapsed time: freeze once the mission completes so downstream
        # consumers (chart, ETA, etc.) don't see the clock keep advancing.
        if self._complete_elapsed is not None:
            elapsed = self._complete_elapsed
        elif self._start_time is not None:
            elapsed = self.get_clock().now().nanoseconds * 1e-9 - self._start_time
        else:
            elapsed = 0.0

        active = sum(
            1
            for i in range(self._n)
            if self._robot_statuses.get(i) == "active"
        )

        msg = CoverageStats()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.algorithm = self._algorithm
        msg.coverage_percentage = float(pct)
        msg.elapsed_time = float(elapsed)
        msg.robots_active = active
        msg.total_robots = self._n
        msg.completed = final or self._state == _State.COMPLETE
        self._pub_stats.publish(msg)

    # ------------------------------------------------------------------
    # Coordinate helpers
    # ------------------------------------------------------------------

    def _world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        """Convert world (x, y) to grid (row, col)."""
        if self._grid_msg is None:
            return (0, 0)
        ox = self._grid_msg.info.origin.position.x
        oy = self._grid_msg.info.origin.position.y
        res = self._resolution
        rows = self._grid_msg.info.height
        col = int((x - ox) / res)
        row = rows - 1 - int((y - oy) / res)
        rows_total = self._grid_msg.info.height
        cols_total = self._grid_msg.info.width
        row = max(0, min(rows_total - 1, row))
        col = max(0, min(cols_total - 1, col))
        return (row, col)

    def _grid_to_world(self, row: int, col: int) -> tuple[float, float]:
        """Convert grid (row, col) to world (x, y) cell centre."""
        if self._grid_msg is None:
            return (0.0, 0.0)
        ox = self._grid_msg.info.origin.position.x
        oy = self._grid_msg.info.origin.position.y
        res = self._resolution
        rows = self._grid_msg.info.height
        x = ox + (col + 0.5) * res
        y = oy + (rows - row - 0.5) * res
        return (x, y)

    def _default_start_positions(self) -> list[tuple[int, int]]:
        """Evenly-spread start positions in safe (free, non-inflated) cells.

        Distributes robots across both X and Y so the workload is naturally
        balanced and they don't all crowd the same corner.  Snaps each
        candidate to the nearest free cell of the inflated grid.
        """
        if self._grid is None:
            return [(5, 5)] * self._n
        rows, cols = self._grid.shape

        # Anchor row near the bottom of the workspace, but pulled up a bit
        # so we're definitely inside the free zone.
        anchor_row = int(rows * 0.85)
        positions: list[tuple[int, int]] = []
        for i in range(self._n):
            # Spread columns evenly across the map width
            target_col = int((i + 1) * cols / (self._n + 1))
            pos = self._snap_to_free(anchor_row, target_col)
            positions.append(pos)
        return positions

    def _snap_to_free(self, row: int, col: int) -> tuple[int, int]:
        """Find the nearest non-obstacle cell to *(row, col)* via BFS."""
        assert self._grid is not None
        rows, cols = self._grid.shape
        row = max(0, min(rows - 1, row))
        col = max(0, min(cols - 1, col))
        if self._grid[row, col] < 50:
            return (row, col)

        from collections import deque
        visited = {(row, col)}
        q = deque([(row, col)])
        while q:
            r, c = q.popleft()
            for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                nr, nc = r + dr, c + dc
                if not (0 <= nr < rows and 0 <= nc < cols):
                    continue
                if (nr, nc) in visited:
                    continue
                visited.add((nr, nc))
                if self._grid[nr, nc] < 50:
                    return (nr, nc)
                q.append((nr, nc))
        return (row, col)   # fallback (shouldn't happen on a sane map)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CoverageCoordinatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
