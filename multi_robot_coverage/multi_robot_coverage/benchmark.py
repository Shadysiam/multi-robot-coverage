"""Headless benchmark runner for multi-robot coverage algorithms.

Runs each algorithm against each map without any ROS2 or Docker dependency,
producing reproducible JSON metric files in ``results/``.

The benchmark imports the same algorithm classes the ROS2 coordinator uses,
so the numbers reflect identical planning logic — only the runtime layer
differs (pure Python loop vs. rclpy node).

Usage
-----
    # Run all algorithms × all maps
    python3 -m multi_robot_coverage.benchmark

    # Single combination
    python3 -m multi_robot_coverage.benchmark \
        --map obstacle_room --algorithm boustrophedon

    # Custom parameters
    python3 -m multi_robot_coverage.benchmark \
        --num-robots 4 --robot-speed 1.5 --max-time 240

Output
------
``results/<map>_<algorithm>.json`` containing:
  - Run metadata (map, algorithm, parameters)
  - Coverage curve sampled every ``sample_every_s`` seconds
  - Final metrics: coverage %, completion time, total distance, redundancy
"""

from __future__ import annotations

import argparse
import json
import math
import os
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np

from multi_robot_coverage.algorithms.astar import AStar
from multi_robot_coverage.algorithms.boustrophedon import (
    BoustrophedonDecomposer,
    CoverageCell,
)
from multi_robot_coverage.algorithms.frontier_based import FrontierExplorer
from multi_robot_coverage.algorithms.random_walk import RandomWalkPlanner
from multi_robot_coverage.algorithms.simple_boustrophedon import SimpleBoustrophedonPlanner


# ---------------------------------------------------------------------------
# Defaults — matched to the ROS2 launch file
# ---------------------------------------------------------------------------

DEFAULT_RESOLUTION_M = 0.05
DEFAULT_ROBOT_RADIUS_M = 0.2
DEFAULT_COVERAGE_WIDTH_M = 0.4
DEFAULT_ROBOT_SPEED_MPS = 1.0
DEFAULT_NUM_ROBOTS = 3
DEFAULT_DT_S = 0.1
DEFAULT_SAMPLE_EVERY_S = 1.0
DEFAULT_MAX_TIME_S = 300.0

MAP_NAMES = ("simple_room", "obstacle_room", "warehouse")
ALGORITHMS = ("boustrophedon", "simple_boustrophedon", "frontier", "random_walk")

_OCC = 100         # obstacle marker after PGM load
_FREE = 0
_OCC_THRESHOLD = 50


# ---------------------------------------------------------------------------
# Map loading
# ---------------------------------------------------------------------------


def _maps_dir() -> Path:
    """Return the absolute path to the package's maps/ directory."""
    return Path(__file__).resolve().parent.parent / "maps"


def load_map(name: str) -> np.ndarray:
    """Load a PGM map into a 0/100 occupancy grid.

    Free cells = 0, obstacles = 100.  The grid is oriented row-0 = TOP so it
    matches the coordinator's internal convention.
    """
    pgm = _maps_dir() / f"{name}.pgm"
    if not pgm.exists():
        # Try to regenerate by importing the map_generator
        from multi_robot_coverage import map_generator
        map_generator.main()
    if not pgm.exists():
        raise FileNotFoundError(
            f"Map file {pgm} not found. Run map_generator first."
        )

    with open(pgm, "rb") as f:
        magic = f.readline().strip()
        if magic != b"P5":
            raise ValueError(f"Expected P5 PGM, got {magic!r}")
        # Skip any comment lines
        while True:
            line = f.readline()
            if not line.startswith(b"#"):
                break
        cols, rows = (int(x) for x in line.split())
        max_val = int(f.readline().strip())
        if max_val != 255:
            raise ValueError(f"Expected max=255, got {max_val}")
        raw = np.frombuffer(f.read(), dtype=np.uint8).reshape(rows, cols)

    # Pixels < 128 are obstacles (black), the rest are free (white).
    grid = np.where(raw < 128, _OCC, _FREE).astype(np.uint8)
    return grid


def inflate_obstacles(grid: np.ndarray, radius_cells: int) -> np.ndarray:
    """Dilate obstacles by *radius_cells* (binary_dilation)."""
    if radius_cells <= 0:
        return grid.copy()
    try:
        from scipy.ndimage import binary_dilation
    except ImportError as e:
        raise RuntimeError("scipy is required for obstacle inflation") from e
    mask = grid >= _OCC_THRESHOLD
    struct = np.ones((2 * radius_cells + 1, 2 * radius_cells + 1), dtype=bool)
    inflated = binary_dilation(mask, structure=struct)
    out = grid.copy()
    out[inflated] = _OCC
    return out


# ---------------------------------------------------------------------------
# Start positions — mirrors coverage_coordinator._default_start_positions
# ---------------------------------------------------------------------------


def _snap_to_free(grid: np.ndarray, row: int, col: int) -> tuple[int, int]:
    """BFS to nearest non-obstacle cell."""
    rows, cols = grid.shape
    row = max(0, min(rows - 1, row))
    col = max(0, min(cols - 1, col))
    if grid[row, col] < _OCC_THRESHOLD:
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
            if grid[nr, nc] < _OCC_THRESHOLD:
                return (nr, nc)
            q.append((nr, nc))
    return (row, col)


def default_starts(grid: np.ndarray, n_robots: int) -> list[tuple[int, int]]:
    """Spread starts evenly across the bottom 85% of the map."""
    rows, cols = grid.shape
    anchor_row = int(rows * 0.85)
    positions: list[tuple[int, int]] = []
    for i in range(n_robots):
        target_col = int((i + 1) * cols / (n_robots + 1))
        positions.append(_snap_to_free(grid, anchor_row, target_col))
    return positions


# ---------------------------------------------------------------------------
# Path building — mirrors coverage_coordinator path-build logic
# ---------------------------------------------------------------------------


def _segment_clear(grid: np.ndarray, a: tuple[int, int], b: tuple[int, int]) -> bool:
    """Bresenham line check between two grid cells."""
    rows, cols = grid.shape
    r0, c0 = a
    r1, c1 = b
    dr = abs(r1 - r0)
    dc = abs(c1 - c0)
    sr = 1 if r0 < r1 else -1
    sc = 1 if c0 < c1 else -1
    err = dr - dc
    r, c = r0, c0
    while True:
        if not (0 <= r < rows and 0 <= c < cols):
            return False
        if grid[r, c] >= _OCC_THRESHOLD:
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


def _densify_path(
    grid: np.ndarray,
    grid_path: list[tuple[int, int]],
    planner: AStar,
) -> list[tuple[int, int]]:
    """A*-bridge any gap > sqrt(2) cells, then Bresenham-validate."""
    if len(grid_path) < 2:
        return list(grid_path)
    dense: list[tuple[int, int]] = [grid_path[0]]
    for i in range(1, len(grid_path)):
        prev = dense[-1]
        curr = grid_path[i]
        dist = math.hypot(curr[0] - prev[0], curr[1] - prev[1])
        if dist > 1.5:
            bridge = planner.search(grid, prev, curr, inflation_radius=0)
            if bridge and len(bridge) > 1:
                dense.extend(bridge[1:])
        else:
            dense.append(curr)
    # Bresenham validation
    validated: list[tuple[int, int]] = [dense[0]]
    for i in range(1, len(dense)):
        prev = validated[-1]
        curr = dense[i]
        if _segment_clear(grid, prev, curr):
            validated.append(curr)
            continue
        detour = planner.search(grid, prev, curr, inflation_radius=0)
        if detour and len(detour) > 1:
            validated.extend(detour[1:])
    return validated


def _build_bcd_path_for_robot(
    grid: np.ndarray,
    robot_cells: list[CoverageCell],
    decomposer: BoustrophedonDecomposer,
    planner: AStar,
    start_pos: tuple[int, int],
) -> list[tuple[int, int]]:
    """Concatenate lawnmower + A* inter-cell into one robot path."""
    path: list[tuple[int, int]] = []
    prev_grid = start_pos
    for cell in robot_cells:
        sweep = decomposer.generate_path(cell, grid)
        if not sweep:
            continue
        # A* transit to first sweep point
        transit = planner.search(grid, prev_grid, sweep[0], inflation_radius=0)
        if transit:
            path.extend(transit[1:] if path else transit)
        # Dense lawnmower sweep
        dense_sweep = _densify_path(grid, sweep, planner)
        path.extend(dense_sweep[1:] if path and dense_sweep and path[-1] == dense_sweep[0] else dense_sweep)
        prev_grid = sweep[-1]
    return path


def plan_paths(
    algorithm: str,
    grid: np.ndarray,
    n_robots: int,
    starts: list[tuple[int, int]],
    coverage_width_cells: int,
) -> dict[int, list[tuple[int, int]]]:
    """Generate per-robot grid paths for the chosen algorithm.

    Returns
    -------
    dict[int, list[(row, col)]]
        Robot ID -> ordered waypoints in grid coordinates.
    """
    if algorithm == "boustrophedon":
        decomposer = BoustrophedonDecomposer(
            coverage_width=coverage_width_cells, occupied_threshold=_OCC_THRESHOLD
        )
        planner = AStar()
        cells = decomposer.decompose(grid)
        assignment = decomposer.assign_to_robots(
            cells, n_robots, robot_starts=starts, method="hungarian"
        )
        return {
            rid: _build_bcd_path_for_robot(
                grid, list(cs), decomposer, planner, starts[rid]
            )
            for rid, cs in assignment.items()
        }

    if algorithm == "simple_boustrophedon":
        planner_simple = SimpleBoustrophedonPlanner(coverage_width=coverage_width_cells)
        raw = planner_simple.generate_paths(grid, n_robots)
        astar = AStar()
        return {
            rid: _densify_path(grid, path, astar)
            for rid, path in raw.items()
        }

    if algorithm == "random_walk":
        rw = RandomWalkPlanner(num_steps=1500, step_size=2, seed=42)
        return rw.assign_to_robots(grid, n_robots, starts)

    if algorithm == "frontier":
        # Frontier requires interleaved planning; handled separately in
        # the simulator.  Return empty paths here.
        return {rid: [] for rid in range(n_robots)}

    raise ValueError(f"Unknown algorithm: {algorithm}")


# ---------------------------------------------------------------------------
# Simulation
# ---------------------------------------------------------------------------


@dataclass
class _Robot:
    """Robot state during simulation."""
    rid: int
    pos: tuple[float, float]      # (row, col) fractional grid coords
    waypoints: list[tuple[int, int]]
    wp_idx: int = 0
    distance_m: float = 0.0
    done: bool = False


def _paint_coverage(
    coverage: np.ndarray,
    redundancy: np.ndarray,
    grid: np.ndarray,
    rid: int,
    pos: tuple[float, float],
    robot_radius_cells: int,
) -> None:
    """Paint cells within robot footprint as covered by *rid*."""
    rows, cols = grid.shape
    r, c = int(round(pos[0])), int(round(pos[1]))
    rad = robot_radius_cells + 1
    r_min = max(0, r - rad)
    r_max = min(rows - 1, r + rad)
    c_min = max(0, c - rad)
    c_max = min(cols - 1, c + rad)
    stamp = (rid + 1) * 10
    for rr in range(r_min, r_max + 1):
        for cc in range(c_min, c_max + 1):
            if math.hypot(rr - r, cc - c) > rad:
                continue
            if grid[rr, cc] >= _OCC_THRESHOLD:
                continue
            cur = coverage[rr, cc]
            if cur == 0:
                coverage[rr, cc] = stamp
                redundancy[rr, cc] = 1
            elif cur != stamp:
                if redundancy[rr, cc] < 250:
                    redundancy[rr, cc] += 1


def _step_robot(
    robot: _Robot,
    step_m: float,
    resolution_m: float,
) -> None:
    """Advance robot along its waypoint list by *step_m* metres."""
    if robot.done or not robot.waypoints:
        return
    remaining_m = step_m
    while remaining_m > 1e-9 and robot.wp_idx < len(robot.waypoints):
        target = robot.waypoints[robot.wp_idx]
        dr = target[0] - robot.pos[0]
        dc = target[1] - robot.pos[1]
        dist_cells = math.hypot(dr, dc)
        dist_m = dist_cells * resolution_m
        if dist_m < 1e-9:
            robot.wp_idx += 1
            continue
        if dist_m <= remaining_m:
            robot.pos = (float(target[0]), float(target[1]))
            robot.distance_m += dist_m
            remaining_m -= dist_m
            robot.wp_idx += 1
        else:
            frac = remaining_m / dist_m
            robot.pos = (robot.pos[0] + dr * frac, robot.pos[1] + dc * frac)
            robot.distance_m += remaining_m
            remaining_m = 0.0
    if robot.wp_idx >= len(robot.waypoints):
        robot.done = True


def simulate_paths(
    grid: np.ndarray,
    paths: dict[int, list[tuple[int, int]]],
    starts: list[tuple[int, int]],
    robot_radius_cells: int,
    resolution_m: float,
    speed_mps: float,
    dt_s: float,
    sample_every_s: float,
    max_time_s: float,
) -> dict:
    """Run the simulation given pre-planned paths and return metrics."""
    rows, cols = grid.shape
    coverage = np.zeros((rows, cols), dtype=np.uint8)
    redundancy = np.zeros((rows, cols), dtype=np.uint8)
    total_free = int(np.sum(grid < _OCC_THRESHOLD))

    robots: list[_Robot] = [
        _Robot(
            rid=rid,
            pos=(float(starts[rid][0]), float(starts[rid][1])),
            waypoints=paths.get(rid, []),
        )
        for rid in range(len(starts))
    ]

    # Initial coverage from start positions
    for r in robots:
        _paint_coverage(coverage, redundancy, grid, r.rid, r.pos, robot_radius_cells)

    step_m = speed_mps * dt_s
    curve: list[dict] = []
    next_sample = 0.0
    t = 0.0

    while t < max_time_s and not all(r.done for r in robots):
        for r in robots:
            _step_robot(r, step_m, resolution_m)
            _paint_coverage(coverage, redundancy, grid, r.rid, r.pos, robot_radius_cells)
        t += dt_s
        if t >= next_sample:
            covered = int(np.sum((coverage > 0) & (coverage < _OCC)))
            pct = 100.0 * covered / total_free if total_free else 0.0
            curve.append({
                "t": round(t, 2),
                "pct": round(pct, 3),
                "distance_m_total": round(sum(r.distance_m for r in robots), 3),
                "redundant_cells": int(np.sum(redundancy >= 2)),
            })
            next_sample += sample_every_s

    # Final sample
    covered = int(np.sum((coverage > 0) & (coverage < _OCC)))
    pct = 100.0 * covered / total_free if total_free else 0.0
    redundant = int(np.sum(redundancy >= 2))
    distances = {str(r.rid): round(r.distance_m, 3) for r in robots}
    completion_time = t if all(r.done for r in robots) else None

    return {
        "coverage_curve": curve,
        "final_coverage_pct": round(pct, 3),
        "completion_time_s": round(completion_time, 2) if completion_time else None,
        "total_distance_m_per_robot": distances,
        "total_distance_m": round(sum(r.distance_m for r in robots), 3),
        "redundant_cells": redundant,
        "redundancy_ratio": round(redundant / total_free, 4) if total_free else 0.0,
        "total_free_cells": total_free,
        "covered_cells": covered,
    }


# ---------------------------------------------------------------------------
# Frontier — interleaved exploration + planning
# ---------------------------------------------------------------------------


def simulate_frontier(
    grid: np.ndarray,
    starts: list[tuple[int, int]],
    robot_radius_cells: int,
    resolution_m: float,
    speed_mps: float,
    dt_s: float,
    sample_every_s: float,
    max_time_s: float,
) -> dict:
    """Run frontier exploration with re-planning every replan_interval_s."""
    rows, cols = grid.shape
    coverage = np.zeros((rows, cols), dtype=np.uint8)
    redundancy = np.zeros((rows, cols), dtype=np.uint8)
    total_free = int(np.sum(grid < _OCC_THRESHOLD))
    n_robots = len(starts)

    sensor_radius = max(5, int(0.4 / resolution_m))  # ~8 cells
    explorer = FrontierExplorer(grid, sensor_radius=sensor_radius)
    astar = AStar()

    robots: list[_Robot] = [
        _Robot(rid=rid, pos=(float(starts[rid][0]), float(starts[rid][1])), waypoints=[])
        for rid in range(n_robots)
    ]
    explorer.reveal([(int(r.pos[0]), int(r.pos[1])) for r in robots])
    for r in robots:
        _paint_coverage(coverage, redundancy, grid, r.rid, r.pos, robot_radius_cells)

    step_m = speed_mps * dt_s
    curve: list[dict] = []
    next_sample = 0.0
    next_replan = 0.0
    replan_interval_s = 2.0
    t = 0.0
    stalled_iters = 0

    while t < max_time_s:
        # Re-plan periodically OR when any robot is out of waypoints
        any_idle = any(r.wp_idx >= len(r.waypoints) for r in robots)
        if t >= next_replan or any_idle:
            grid_poses = [(int(r.pos[0]), int(r.pos[1])) for r in robots]
            explorer.reveal(grid_poses)
            centroids = explorer.find_frontier_centroids()
            if not centroids:
                stalled_iters += 1
                if stalled_iters > 3:
                    break  # nothing left to explore
            else:
                stalled_iters = 0
                claimed: set[tuple[int, int]] = set()
                assignments = FrontierExplorer.assign_frontiers(
                    centroids, grid_poses, claimed
                )
                for rid, frontier in assignments.items():
                    if frontier is None:
                        continue
                    path = astar.search(
                        grid, grid_poses[rid], frontier, inflation_radius=0
                    )
                    if path and len(path) > 1:
                        robots[rid].waypoints = path
                        robots[rid].wp_idx = 1  # skip start
                        robots[rid].done = False
            next_replan = t + replan_interval_s

        for r in robots:
            _step_robot(r, step_m, resolution_m)
            _paint_coverage(coverage, redundancy, grid, r.rid, r.pos, robot_radius_cells)
        t += dt_s

        if t >= next_sample:
            covered = int(np.sum((coverage > 0) & (coverage < _OCC)))
            pct = 100.0 * covered / total_free if total_free else 0.0
            curve.append({
                "t": round(t, 2),
                "pct": round(pct, 3),
                "distance_m_total": round(sum(r.distance_m for r in robots), 3),
                "redundant_cells": int(np.sum(redundancy >= 2)),
            })
            next_sample += sample_every_s

    covered = int(np.sum((coverage > 0) & (coverage < _OCC)))
    pct = 100.0 * covered / total_free if total_free else 0.0
    redundant = int(np.sum(redundancy >= 2))
    distances = {str(r.rid): round(r.distance_m, 3) for r in robots}

    return {
        "coverage_curve": curve,
        "final_coverage_pct": round(pct, 3),
        "completion_time_s": round(t, 2),
        "total_distance_m_per_robot": distances,
        "total_distance_m": round(sum(r.distance_m for r in robots), 3),
        "redundant_cells": redundant,
        "redundancy_ratio": round(redundant / total_free, 4) if total_free else 0.0,
        "total_free_cells": total_free,
        "covered_cells": covered,
    }


# ---------------------------------------------------------------------------
# Top-level run
# ---------------------------------------------------------------------------


def run_single(
    map_name: str,
    algorithm: str,
    n_robots: int = DEFAULT_NUM_ROBOTS,
    resolution_m: float = DEFAULT_RESOLUTION_M,
    robot_radius_m: float = DEFAULT_ROBOT_RADIUS_M,
    coverage_width_m: float = DEFAULT_COVERAGE_WIDTH_M,
    speed_mps: float = DEFAULT_ROBOT_SPEED_MPS,
    dt_s: float = DEFAULT_DT_S,
    sample_every_s: float = DEFAULT_SAMPLE_EVERY_S,
    max_time_s: float = DEFAULT_MAX_TIME_S,
) -> dict:
    """Run a single (map, algorithm) combination and return the result dict."""
    raw_grid = load_map(map_name)
    radius_cells = max(1, int(robot_radius_m / resolution_m))
    grid = inflate_obstacles(raw_grid, radius_cells)
    coverage_width_cells = max(1, int(coverage_width_m / resolution_m))
    starts = default_starts(grid, n_robots)

    wall_start = time.perf_counter()
    if algorithm == "frontier":
        metrics = simulate_frontier(
            grid, starts, radius_cells, resolution_m,
            speed_mps, dt_s, sample_every_s, max_time_s,
        )
    else:
        paths = plan_paths(algorithm, grid, n_robots, starts, coverage_width_cells)
        metrics = simulate_paths(
            grid, paths, starts, radius_cells, resolution_m,
            speed_mps, dt_s, sample_every_s, max_time_s,
        )
    wall_elapsed = time.perf_counter() - wall_start

    return {
        "map": map_name,
        "algorithm": algorithm,
        "num_robots": n_robots,
        "robot_radius_m": robot_radius_m,
        "robot_speed_mps": speed_mps,
        "coverage_width_m": coverage_width_m,
        "map_resolution_m": resolution_m,
        "starts_grid": [list(s) for s in starts],
        "wall_time_s": round(wall_elapsed, 3),
        "results": metrics,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", default=None, help="Single map (default: all)")
    parser.add_argument("--algorithm", default=None, help="Single algorithm (default: all)")
    parser.add_argument("--num-robots", type=int, default=DEFAULT_NUM_ROBOTS)
    parser.add_argument("--robot-speed", type=float, default=DEFAULT_ROBOT_SPEED_MPS)
    parser.add_argument("--max-time", type=float, default=DEFAULT_MAX_TIME_S)
    parser.add_argument("--output-dir", default=None, help="results/ dir (default: repo results/)")
    args = parser.parse_args()

    maps_to_run = [args.map] if args.map else list(MAP_NAMES)
    algos_to_run = [args.algorithm] if args.algorithm else list(ALGORITHMS)

    if args.output_dir:
        out_dir = Path(args.output_dir)
    else:
        out_dir = Path(__file__).resolve().parent.parent.parent / "results"
    out_dir.mkdir(parents=True, exist_ok=True)

    summary: list[dict] = []
    for map_name in maps_to_run:
        for algorithm in algos_to_run:
            print(f"\n=== {map_name} × {algorithm} ===")
            result = run_single(
                map_name, algorithm,
                n_robots=args.num_robots,
                speed_mps=args.robot_speed,
                max_time_s=args.max_time,
            )
            r = result["results"]
            print(f"  coverage: {r['final_coverage_pct']:.1f}%"
                  f"  time: {r['completion_time_s']}s"
                  f"  distance: {r['total_distance_m']:.1f}m"
                  f"  redundancy: {r['redundancy_ratio']*100:.1f}%"
                  f"  (wall: {result['wall_time_s']:.2f}s)")

            out_file = out_dir / f"{map_name}_{algorithm}.json"
            with open(out_file, "w") as f:
                json.dump(result, f, indent=2)
            print(f"  → {out_file}")

            summary.append({
                "map": map_name,
                "algorithm": algorithm,
                "coverage_pct": r["final_coverage_pct"],
                "completion_time_s": r["completion_time_s"],
                "total_distance_m": r["total_distance_m"],
                "redundancy_ratio": r["redundancy_ratio"],
            })

    # Write summary table
    summary_path = out_dir / "summary.json"
    with open(summary_path, "w") as f:
        json.dump(summary, f, indent=2)
    print(f"\n=== Summary written to {summary_path} ===")

    # Print Markdown table to stdout for easy README paste
    print("\n| Map | Algorithm | Coverage | Time (s) | Distance (m) | Redundancy |")
    print("|---|---|---:|---:|---:|---:|")
    for row in summary:
        ct = f"{row['completion_time_s']:.0f}" if row['completion_time_s'] else "—"
        print(
            f"| {row['map']} | {row['algorithm']} | "
            f"{row['coverage_pct']:.1f}% | {ct} | "
            f"{row['total_distance_m']:.1f} | "
            f"{row['redundancy_ratio']*100:.1f}% |"
        )


if __name__ == "__main__":
    main()
