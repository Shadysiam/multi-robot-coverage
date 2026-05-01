"""Simple Boustrophedon (no decomposition) — comparison baseline.

Divides the map into horizontal bands (one per robot) and sweeps each
band with a back-and-forth lawnmower pattern.  Unlike BCD, there is no
attempt to detect or adapt to obstacles — the robot simply skips cells
that fall on obstacles.  This causes missed regions wherever obstacles
cut through strips and leaves coverage gaps compared to BCD.

Used purely for algorithm comparison to demonstrate why Boustrophedon
Cellular Decomposition (with explicit obstacle handling) outperforms
naive whole-map sweeping.
"""

from __future__ import annotations

import numpy as np


class SimpleBoustrophedonPlanner:
    """Naive lawnmower without obstacle-aware cell decomposition.

    Parameters
    ----------
    coverage_width:
        Row spacing between parallel sweep strips (grid cells).
    occupied_threshold:
        Grid values ≥ this are treated as obstacles.
    """

    def __init__(
        self,
        coverage_width: int = 8,
        occupied_threshold: int = 50,
    ) -> None:
        self._cw = max(1, coverage_width)
        self._occ = occupied_threshold

    def generate_paths(
        self,
        grid: np.ndarray,
        num_robots: int,
    ) -> dict[int, list[tuple[int, int]]]:
        """Generate one lawnmower path per robot.

        The map is divided into ``num_robots`` equal horizontal bands.
        Within each band the robot sweeps every ``coverage_width`` rows,
        reversing direction each strip (boustrophedon pattern).
        Obstacle cells are skipped — but the robot still interpolates
        between the waypoints on either side of any gap, which can cross
        through the obstacle.  That coverage loss is intentional: it
        is precisely the problem BCD solves.

        Parameters
        ----------
        grid:
            2-D occupancy array (rows × cols).
        num_robots:
            Number of robots to plan for.

        Returns
        -------
        dict[int, list[(row, col)]]
            Per-robot ordered waypoint lists.
        """
        rows, cols = grid.shape
        free = grid < self._occ
        band = rows // num_robots
        paths: dict[int, list[tuple[int, int]]] = {}

        for rid in range(num_robots):
            r_start = rid * band
            r_end   = (rid + 1) * band if rid < num_robots - 1 else rows
            path: list[tuple[int, int]] = []
            strip_idx = 0

            for row in range(r_start, r_end, self._cw):
                # Alternate left→right and right→left.
                col_iter = (
                    range(0, cols, self._cw)
                    if strip_idx % 2 == 0
                    else range(cols - 1, -1, -self._cw)
                )
                for col in col_iter:
                    if free[row, col]:
                        path.append((row, col))
                strip_idx += 1

            paths[rid] = path

        return paths
