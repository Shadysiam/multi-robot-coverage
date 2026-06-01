"""Frontier-based multi-robot exploration algorithm.

Each robot independently navigates to the nearest unclaimed frontier cell.
A coordination layer prevents two robots from targeting the same frontier.

Reference
---------
Yamauchi, B. (1997). A frontier-based approach for autonomous exploration.
Proceedings of the IEEE International Symposium on Computational Intelligence
in Robotics and Automation.
"""

from __future__ import annotations

import math
from collections import deque
from typing import Optional

import numpy as np


# Occupancy values used in the known-map
UNKNOWN: int = -1
FREE: int = 0
OCCUPIED: int = 100


class FrontierExplorer:
    """Frontier detection and multi-robot frontier assignment.

    The *known map* starts fully unknown and is updated as robots move,
    revealing a circular region of radius ``sensor_radius`` grid cells
    around each robot's position.

    Parameters
    ----------
    grid:
        Ground-truth occupancy grid (values ≥ 50 = obstacle, 0 = free).
    sensor_radius:
        Sensing radius in grid cells. Cells within this radius of a
        robot's current position are added to the known map.
    """

    def __init__(self, grid: np.ndarray, sensor_radius: int = 20) -> None:
        self._grid = grid.copy()
        self._sensor_radius = sensor_radius
        rows, cols = grid.shape
        # Known map is initialised to UNKNOWN everywhere.
        self._known: np.ndarray = np.full((rows, cols), UNKNOWN, dtype=np.int8)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @property
    def known_map(self) -> np.ndarray:
        """Read-only view of the current known map."""
        return self._known

    def reveal(self, robot_positions: list[tuple[int, int]]) -> None:
        """Update the known map from all robot sensor footprints.

        Parameters
        ----------
        robot_positions:
            List of ``(row, col)`` positions for each active robot.
        """
        rows, cols = self._grid.shape
        for r, c in robot_positions:
            r_min = max(0, r - self._sensor_radius)
            r_max = min(rows - 1, r + self._sensor_radius)
            c_min = max(0, c - self._sensor_radius)
            c_max = min(cols - 1, c + self._sensor_radius)
            for rr in range(r_min, r_max + 1):
                for cc in range(c_min, c_max + 1):
                    if math.hypot(rr - r, cc - c) <= self._sensor_radius:
                        val = self._grid[rr, cc]
                        self._known[rr, cc] = OCCUPIED if val >= 50 else FREE

    def find_frontiers(self) -> list[tuple[int, int]]:
        """Return grid cells that lie on the boundary between free and unknown.

        A frontier cell is a FREE cell in the known map that is 4-adjacent
        to at least one UNKNOWN cell.

        Returns
        -------
        list[tuple[int, int]]
            ``(row, col)`` coordinates of all frontier cells.
        """
        rows, cols = self._known.shape
        frontiers: list[tuple[int, int]] = []
        for r in range(1, rows - 1):
            for c in range(1, cols - 1):
                if self._known[r, c] != FREE:
                    continue
                # Check 4-connectivity for unknown neighbour.
                for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                    if self._known[r + dr, c + dc] == UNKNOWN:
                        frontiers.append((r, c))
                        break
        return frontiers

    def find_frontier_centroids(
        self, min_cluster_size: int = 3
    ) -> list[tuple[int, int]]:
        """Cluster raw frontier cells and return one centroid per cluster.

        Small isolated frontier pixels are filtered out to reduce noise.

        Parameters
        ----------
        min_cluster_size:
            Clusters with fewer cells than this threshold are discarded.

        Returns
        -------
        list[tuple[int, int]]
            One representative ``(row, col)`` per cluster.
        """
        raw = self.find_frontiers()
        if not raw:
            return []

        frontier_set = set(raw)
        visited: set[tuple[int, int]] = set()
        centroids: list[tuple[int, int]] = []

        for seed in raw:
            if seed in visited:
                continue
            # BFS to collect cluster.
            cluster: list[tuple[int, int]] = []
            queue: deque[tuple[int, int]] = deque([seed])
            visited.add(seed)
            while queue:
                pos = queue.popleft()
                cluster.append(pos)
                r, c = pos
                for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                    nbr = (r + dr, c + dc)
                    if nbr in frontier_set and nbr not in visited:
                        visited.add(nbr)
                        queue.append(nbr)

            if len(cluster) >= min_cluster_size:
                # CRITICAL: pick the cluster member closest to the arithmetic
                # mean, NOT the mean itself.  When a cluster curves around an
                # obstacle, the arithmetic mean can land in an obstacle cell —
                # A* then refuses to plan a path to it and the robot stays
                # idle.  Picking the closest actual cluster cell guarantees
                # the goal is a free, reachable frontier cell.
                r_mean = sum(p[0] for p in cluster) / len(cluster)
                c_mean = sum(p[1] for p in cluster) / len(cluster)
                representative = min(
                    cluster,
                    key=lambda p: (p[0] - r_mean) ** 2 + (p[1] - c_mean) ** 2,
                )
                centroids.append(representative)

        return centroids

    @staticmethod
    def assign_frontiers(
        frontiers: list[tuple[int, int]],
        robot_positions: list[tuple[int, int]],
        claimed: set[tuple[int, int]],
    ) -> dict[int, Optional[tuple[int, int]]]:
        """Greedily assign one unique frontier to each active robot.

        Robots are matched to their nearest unclaimed frontier. A frontier
        claimed by one robot is excluded from all subsequent assignments in
        the same call.

        Parameters
        ----------
        frontiers:
            Available frontier centroids (output of ``find_frontier_centroids``).
        robot_positions:
            ``(row, col)`` for each robot (index = robot ID).
        claimed:
            Set of frontiers already claimed in previous calls; updated
            in-place with newly claimed frontiers.

        Returns
        -------
        dict[int, tuple | None]
            Maps robot ID → assigned frontier, or ``None`` if no frontier
            is available for that robot.
        """
        available = [f for f in frontiers if f not in claimed]
        assignments: dict[int, Optional[tuple[int, int]]] = {}

        for robot_id, rpos in enumerate(robot_positions):
            if not available:
                assignments[robot_id] = None
                continue

            nearest = min(
                available,
                key=lambda f: math.hypot(f[0] - rpos[0], f[1] - rpos[1]),
            )
            assignments[robot_id] = nearest
            claimed.add(nearest)
            available.remove(nearest)

        return assignments

    def coverage_fraction(self) -> float:
        """Return fraction of non-obstacle free cells that are known-free."""
        total_free = int(np.sum(self._grid < 50))
        if total_free == 0:
            return 0.0
        known_free = int(np.sum(self._known == FREE))
        return known_free / total_free
