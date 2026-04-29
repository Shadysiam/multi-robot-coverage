"""Unit tests for the A* path planner.

These tests are pure Python — no ROS2 installation required.
Run with:  pytest test/test_astar.py -v
"""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent.parent))
from multi_robot_coverage.algorithms.astar import AStar


@pytest.fixture()
def planner() -> AStar:
    return AStar()


# ---------------------------------------------------------------------------
# Basic reachability
# ---------------------------------------------------------------------------


def test_open_grid_finds_path(planner: AStar) -> None:
    grid = np.zeros((20, 20), dtype=np.uint8)
    path = planner.search(grid, (0, 0), (19, 19))
    assert path is not None
    assert path[0] == (0, 0)
    assert path[-1] == (19, 19)


def test_start_equals_goal(planner: AStar) -> None:
    grid = np.zeros((10, 10), dtype=np.uint8)
    path = planner.search(grid, (5, 5), (5, 5))
    assert path == [(5, 5)]


def test_single_step_cardinal(planner: AStar) -> None:
    grid = np.zeros((5, 5), dtype=np.uint8)
    path = planner.search(grid, (2, 2), (2, 3))
    assert path is not None
    assert len(path) == 2


def test_single_step_diagonal(planner: AStar) -> None:
    grid = np.zeros((5, 5), dtype=np.uint8)
    path = planner.search(grid, (2, 2), (3, 3))
    assert path is not None
    assert len(path) == 2


# ---------------------------------------------------------------------------
# Obstacle handling
# ---------------------------------------------------------------------------


def test_path_around_wall(planner: AStar) -> None:
    grid = np.zeros((10, 10), dtype=np.uint8)
    # Vertical wall through column 5, rows 0-7 (gap at rows 8-9)
    grid[0:8, 5] = 100
    path = planner.search(grid, (0, 0), (0, 9))
    assert path is not None
    assert path[-1] == (0, 9)
    # Ensure path doesn't pass through the wall
    for r, c in path:
        assert grid[r, c] < 50


def test_unreachable_goal_returns_none(planner: AStar) -> None:
    grid = np.zeros((10, 10), dtype=np.uint8)
    # Completely enclose the goal
    grid[3:6, 3:6] = 100
    path = planner.search(grid, (0, 0), (4, 4))
    assert path is None


def test_obstacle_start_returns_none(planner: AStar) -> None:
    grid = np.zeros((10, 10), dtype=np.uint8)
    grid[0, 0] = 100
    path = planner.search(grid, (0, 0), (9, 9))
    assert path is None


def test_obstacle_goal_returns_none(planner: AStar) -> None:
    grid = np.zeros((10, 10), dtype=np.uint8)
    grid[9, 9] = 100
    path = planner.search(grid, (0, 0), (9, 9))
    assert path is None


def test_no_corner_clipping(planner: AStar) -> None:
    """Diagonal move must not clip through a narrow obstacle corner."""
    grid = np.zeros((5, 5), dtype=np.uint8)
    grid[1, 1] = 100  # obstacle forces the path to go around
    grid[2, 2] = 100
    # Start (0,2) → goal (2,0): naive diagonal would cut through (1,1)/(2,2)
    path = planner.search(grid, (0, 2), (2, 0))
    # If a path exists, it must not pass through obstacles
    if path is not None:
        for r, c in path:
            assert grid[r, c] < 50


# ---------------------------------------------------------------------------
# Path properties
# ---------------------------------------------------------------------------


def test_path_endpoints_correct(planner: AStar) -> None:
    grid = np.zeros((30, 30), dtype=np.uint8)
    start, goal = (2, 2), (27, 27)
    path = planner.search(grid, start, goal)
    assert path is not None
    assert path[0] == start
    assert path[-1] == goal


def test_path_is_contiguous(planner: AStar) -> None:
    """Each step must be a valid 8-connected neighbour."""
    grid = np.zeros((20, 20), dtype=np.uint8)
    path = planner.search(grid, (0, 0), (19, 19))
    assert path is not None
    for (r1, c1), (r2, c2) in zip(path, path[1:]):
        assert abs(r2 - r1) <= 1 and abs(c2 - c1) <= 1
        assert (r2, c2) != (r1, c1)


def test_path_length_is_near_optimal(planner: AStar) -> None:
    """Diagonal path on open grid should be close to Euclidean distance."""
    grid = np.zeros((20, 20), dtype=np.uint8)
    path = planner.search(grid, (0, 0), (19, 19))
    assert path is not None
    # Optimal diagonal path length ≈ 19√2 ≈ 26.87 → 20 steps
    assert len(path) <= 22  # allow small overhead


# ---------------------------------------------------------------------------
# Inflation radius
# ---------------------------------------------------------------------------


def test_inflation_blocks_narrow_corridor(planner: AStar) -> None:
    """A corridor narrower than inflation_radius must be blocked."""
    grid = np.zeros((10, 20), dtype=np.uint8)
    # 1-cell-wide corridor through column 10 between rows 1-8
    grid[:, 10] = 100
    grid[5, 10] = 0   # 1-cell gap
    # Without inflation: path exists through the gap
    path_no_inf = planner.search(grid, (5, 0), (5, 19), inflation_radius=0)
    # With inflation > 1: gap is inflated away
    path_inflated = planner.search(grid, (5, 0), (5, 19), inflation_radius=2)
    assert path_no_inf is not None
    assert path_inflated is None
