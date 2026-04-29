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
  algorithm          str    — "boustrophedon" | "frontier"
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
        self._total_free: int = 0

        # Frontier-specific state
        self._frontier_explorer: Optional[FrontierExplorer] = None
        self._claimed_frontiers: set[tuple[int, int]] = set()

        self._timer = self.create_timer(0.5, self._tick)
        self.get_logger().info(
            f"Coordinator ready — algorithm={self._algorithm}, "
            f"robots={self._n}, failure_sim={self._fail_sim}"
        )

    # ------------------------------------------------------------------
    # Map callback
    # ------------------------------------------------------------------

    def _cb_map(self, msg: OccupancyGrid) -> None:
        if self._state != _State.WAITING_FOR_MAP:
            return
        self._grid_msg = msg
        rows = msg.info.height
        cols = msg.info.width
        arr = np.array(msg.data, dtype=np.int8).reshape(rows, cols)
        # Convert to simple 0/100 grid.
        self._grid = np.where(arr > 50, 100, np.where(arr < 0, 0, arr)).astype(
            np.uint8
        )
        self._resolution = msg.info.resolution
        self._coverage = np.zeros((rows, cols), dtype=np.uint8)
        # Mark obstacles in coverage map.
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
        # Mark coverage: paint cells within robot radius as covered.
        r, c = self._world_to_grid(x, y)
        rad = max(1, int(self._radius_m / self._resolution))
        rows, cols = self._grid.shape
        r_min = max(0, r - rad)
        r_max = min(rows - 1, r + rad)
        c_min = max(0, c - rad)
        c_max = min(cols - 1, c + rad)
        stamp = (robot_id + 1) * 10  # 10, 20, 30, …
        for rr in range(r_min, r_max + 1):
            for cc in range(c_min, c_max + 1):
                if (
                    math.hypot(rr - r, cc - c) <= rad
                    and self._grid[rr, cc] < 50
                    and self._coverage[rr, cc] == _UNCOVERED
                ):
                    self._coverage[rr, cc] = stamp

    def _cb_status(self, msg: String, robot_id: int) -> None:
        self._robot_statuses[robot_id] = msg.data
        if msg.data == "failed" and robot_id not in self._failed_robots:
            self._failed_robots.add(robot_id)
            self.get_logger().warn(
                f"Robot {robot_id} reported FAILED — initiating reallocation"
            )
            self._state = _State.FAILURE

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
        else:
            self._plan_frontier()

        self._state = _State.RUNNING

    def _plan_boustrophedon(self) -> None:
        assert self._grid is not None
        cw = max(1, int(self._cov_width_m / self._resolution))
        inflation = max(1, int(self._radius_m / self._resolution))
        decomposer = BoustrophedonDecomposer(
            coverage_width=cw, occupied_threshold=50
        )
        planner = AStar()

        cells = decomposer.decompose(self._grid)
        self.get_logger().info(f"BCD produced {len(cells)} cells")

        assignment = decomposer.assign_to_robots(cells, self._n)
        self._robot_remaining_cells = {
            rid: list(clist) for rid, clist in assignment.items()
        }

        for robot_id, robot_cells in assignment.items():
            path_poses = self._build_full_path(
                robot_id, robot_cells, decomposer, planner, inflation
            )
            self._send_waypoints(robot_id, path_poses)

    def _plan_frontier(self) -> None:
        assert self._grid is not None
        sensor_r = max(10, int(0.5 / self._resolution) * 5)  # ~2.5 m
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
        self._tick_frontier_assignment()

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
    ) -> list[tuple[float, float]]:
        """Concatenate lawnmower + A* inter-cell segments into world coords."""
        assert self._grid is not None
        world_path: list[tuple[float, float]] = []
        prev_grid: Optional[tuple[int, int]] = None

        # Robot starts from a sensible default position.
        starts = self._default_start_positions()
        if robot_id < len(starts):
            prev_grid = starts[robot_id]

        for cell in cells:
            sweep = decomposer.generate_path(cell, self._grid)
            if not sweep:
                continue

            # Navigate from last position to first sweep point via A*.
            if prev_grid is not None and sweep:
                transit = planner.search(
                    self._grid, prev_grid, sweep[0], inflation_radius=inflation
                )
                if transit:
                    for gpt in transit[1:]:
                        world_path.append(self._grid_to_world(*gpt))

            for gpt in sweep:
                world_path.append(self._grid_to_world(*gpt))
            prev_grid = sweep[-1] if sweep else prev_grid

        return world_path

    # ------------------------------------------------------------------
    # Frontier-specific tick (called every timer tick while running)
    # ------------------------------------------------------------------

    def _tick_frontier_assignment(self) -> None:
        if self._frontier_explorer is None or self._grid is None:
            return

        # Update sensor reveals from all current robot positions.
        grid_poses = [
            self._world_to_grid(*self._robot_poses.get(i, (0.0, 0.0)))
            for i in range(self._n)
        ]
        self._frontier_explorer.reveal(grid_poses)

        centroids = self._frontier_explorer.find_frontier_centroids()
        assignments = FrontierExplorer.assign_frontiers(
            centroids, grid_poses, self._claimed_frontiers
        )

        for robot_id, frontier in assignments.items():
            if frontier is None:
                continue
            if self._robot_statuses.get(robot_id) in ("active",):
                continue  # robot still working on previous goal
            planner = AStar()
            inflation = max(1, int(self._radius_m / self._resolution))
            current_grid = self._world_to_grid(
                *self._robot_poses.get(robot_id, (0.0, 0.0))
            )
            path = planner.search(
                self._grid, current_grid, frontier, inflation_radius=inflation
            )
            if path:
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
            inflation = max(1, int(self._radius_m / self._resolution))
            for robot_id, cells in self._robot_remaining_cells.items():
                if robot_id == failed_id:
                    continue
                path_poses = self._build_full_path(
                    robot_id, cells, decomposer, planner, inflation
                )
                if path_poses:
                    self._send_waypoints(robot_id, path_poses)
                    self.get_logger().info(
                        f"Reallocated {len(cells)} cells to robot {robot_id}"
                    )

        self._state = _State.RUNNING

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
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info = self._grid_msg.info
        # Normalise robot stamps to 0-100 range for RViz colour mapping.
        display = self._coverage.copy().astype(np.int8)
        msg.data = display.flatten().tolist()
        self._pub_cov_map.publish(msg)

    # ------------------------------------------------------------------
    # Stats publisher
    # ------------------------------------------------------------------

    def _publish_stats(self, final: bool = False) -> None:
        if self._total_free == 0 or self._coverage is None:
            return
        covered = int(np.sum(self._coverage[(self._coverage > 0) & (self._coverage < _OBSTACLE)]))
        covered_cells = int(
            np.sum(
                (self._coverage > _UNCOVERED) & (self._coverage < _OBSTACLE)
            )
        )
        pct = 100.0 * covered_cells / self._total_free if self._total_free else 0.0
        elapsed = 0.0
        if self._start_time is not None:
            elapsed = self.get_clock().now().nanoseconds * 1e-9 - self._start_time
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
        """Evenly-spaced start positions near the bottom of the free space."""
        if self._grid is None:
            return [(5, 5)] * self._n
        rows, cols = self._grid.shape
        free_cols = [
            c
            for c in range(5, cols - 5, max(1, cols // (self._n + 1)))
            if self._grid[rows - 10, c] < 50
        ]
        positions = []
        for i in range(self._n):
            c = free_cols[i % len(free_cols)] if free_cols else cols // 2
            positions.append((rows - 10, c))
        return positions


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
