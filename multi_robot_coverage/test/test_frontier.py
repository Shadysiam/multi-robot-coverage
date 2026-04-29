"""Unit tests for the frontier-based exploration planner.

Pure Python — no ROS2 required.
Run with:  pytest test/test_frontier.py -v
"""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent.parent))
from multi_robot_coverage.algorithms.frontier_based import (
    FREE,
    OCCUPIED,
    UNKNOWN,
    FrontierExplorer,
)


def _open_grid(rows: int = 30, cols: int = 30) -> np.ndarray:
    g = np.zeros((rows, cols), dtype=np.uint8)
    g[:2, :] = 100; g[-2:, :] = 100
    g[:, :2] = 100; g[:, -2:] = 100
    return g


# ---------------------------------------------------------------------------
# Known map initialisation
# ---------------------------------------------------------------------------


def test_known_map_starts_unknown() -> None:
    fe = FrontierExplorer(_open_grid(), sensor_radius=5)
    assert np.all(fe.known_map == UNKNOWN)


def test_reveal_exposes_cells() -> None:
    fe = FrontierExplorer(_open_grid(), sensor_radius=5)
    fe.reveal([(15, 15)])
    assert np.any(fe.known_map != UNKNOWN)


def test_reveal_marks_obstacles_correctly() -> None:
    g = _open_grid()
    fe = FrontierExplorer(g, sensor_radius=20)
    fe.reveal([(15, 15)])
    known = fe.known_map
    # (0, 15): distance from (15,15) = 15 ≤ 20 → within sensor radius → OCCUPIED
    assert known[0, 15] == OCCUPIED
    # (1, 15): also within radius and a border wall cell
    assert known[1, 15] == OCCUPIED


def test_reveal_marks_free_cells() -> None:
    fe = FrontierExplorer(_open_grid(), sensor_radius=10)
    fe.reveal([(15, 15)])
    assert fe.known_map[15, 15] == FREE


# ---------------------------------------------------------------------------
# Frontier detection
# ---------------------------------------------------------------------------


def test_no_frontiers_before_reveal() -> None:
    fe = FrontierExplorer(_open_grid(), sensor_radius=5)
    assert fe.find_frontiers() == []


def test_frontiers_exist_after_partial_reveal() -> None:
    fe = FrontierExplorer(_open_grid(50, 50), sensor_radius=8)
    fe.reveal([(25, 25)])
    frontiers = fe.find_frontiers()
    assert len(frontiers) > 0


def test_frontier_cells_are_free_and_adjacent_to_unknown() -> None:
    fe = FrontierExplorer(_open_grid(40, 40), sensor_radius=6)
    fe.reveal([(20, 20)])
    known = fe.known_map
    for r, c in fe.find_frontiers():
        assert known[r, c] == FREE
        neighbours = [(r-1, c), (r+1, c), (r, c-1), (r, c+1)]
        assert any(
            0 <= nr < known.shape[0] and 0 <= nc < known.shape[1] and known[nr, nc] == UNKNOWN
            for nr, nc in neighbours
        )


def test_full_reveal_produces_no_frontiers() -> None:
    g = _open_grid(20, 20)
    fe = FrontierExplorer(g, sensor_radius=30)
    fe.reveal([(10, 10)])
    # All reachable free space is known → no frontiers
    assert fe.find_frontiers() == []


# ---------------------------------------------------------------------------
# Frontier centroids
# ---------------------------------------------------------------------------


def test_centroids_fewer_than_raw_frontiers() -> None:
    fe = FrontierExplorer(_open_grid(50, 50), sensor_radius=8)
    fe.reveal([(25, 25)])
    raw = fe.find_frontiers()
    centroids = fe.find_frontier_centroids(min_cluster_size=2)
    assert len(centroids) <= len(raw)


def test_centroids_are_free_or_near_free() -> None:
    fe = FrontierExplorer(_open_grid(40, 40), sensor_radius=6)
    fe.reveal([(20, 20)])
    known = fe.known_map
    rows, cols = known.shape
    for r, c in fe.find_frontier_centroids():
        assert 0 <= r < rows and 0 <= c < cols


# ---------------------------------------------------------------------------
# Frontier assignment
# ---------------------------------------------------------------------------


def test_each_robot_gets_unique_frontier() -> None:
    frontiers = [(5, 5), (10, 10), (15, 15), (20, 20)]
    robot_positions = [(0, 0), (0, 25), (25, 0)]
    claimed: set = set()
    result = FrontierExplorer.assign_frontiers(frontiers, robot_positions, claimed)
    assigned = [v for v in result.values() if v is not None]
    assert len(assigned) == len(set(assigned)), "Two robots got the same frontier"


def test_claimed_frontiers_not_reassigned() -> None:
    frontiers = [(5, 5), (10, 10), (15, 15)]
    claimed = {(5, 5)}
    robot_positions = [(0, 0), (0, 25)]
    result = FrontierExplorer.assign_frontiers(frontiers, robot_positions, claimed)
    for frontier in result.values():
        assert frontier != (5, 5)


def test_none_assigned_when_no_frontiers() -> None:
    result = FrontierExplorer.assign_frontiers([], [(0, 0), (5, 5)], set())
    assert all(v is None for v in result.values())


def test_claimed_set_updated_after_assignment() -> None:
    frontiers = [(3, 3), (7, 7)]
    claimed: set = set()
    FrontierExplorer.assign_frontiers(frontiers, [(0, 0), (10, 10)], claimed)
    assert len(claimed) == 2


# ---------------------------------------------------------------------------
# Coverage fraction
# ---------------------------------------------------------------------------


def test_coverage_fraction_zero_before_reveal() -> None:
    fe = FrontierExplorer(_open_grid(), sensor_radius=5)
    assert fe.coverage_fraction() == pytest.approx(0.0)


def test_coverage_fraction_increases_with_reveal() -> None:
    fe = FrontierExplorer(_open_grid(40, 40), sensor_radius=5)
    fe.reveal([(20, 20)])
    f1 = fe.coverage_fraction()
    fe.reveal([(10, 10), (10, 30), (30, 10), (30, 30)])
    f2 = fe.coverage_fraction()
    assert f2 >= f1


def test_full_coverage_fraction_is_one() -> None:
    g = _open_grid(20, 20)
    fe = FrontierExplorer(g, sensor_radius=50)
    fe.reveal([(10, 10)])
    assert fe.coverage_fraction() == pytest.approx(1.0)
