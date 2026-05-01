"""Random Walk coverage planner — baseline algorithm.

Each robot performs a biased random walk: it continues in its current
direction until hitting an obstacle or boundary, then picks a new random
direction.  Serves as a performance baseline that BCD clearly beats.
"""

from __future__ import annotations

import random
from typing import Optional

import numpy as np


# 8-connected movement directions (dr, dc)
_DIRECTIONS = [
    (0, 1), (0, -1), (1, 0), (-1, 0),
    (1, 1), (1, -1), (-1, 1), (-1, -1),
]


class RandomWalkPlanner:
    """Pre-generates a long random walk path for a single robot.

    The walk is biased: it keeps the current direction until blocked,
    then picks a new random direction.  This avoids constant turning and
    produces roughly straight runs — similar to a Roomba's behaviour.

    Parameters
    ----------
    num_steps : int
        Number of grid steps to pre-generate per robot.
    step_size : int
        How many cells to advance per step.
    seed : int | None
        Random seed for reproducibility.
    """

    def __init__(
        self,
        num_steps: int = 8000,
        step_size: int = 2,
        seed: Optional[int] = None,
    ) -> None:
        self._num_steps = num_steps
        self._step_size = step_size
        self._rng = random.Random(seed)

    def generate_path(
        self,
        grid: np.ndarray,
        start: tuple[int, int],
    ) -> list[tuple[int, int]]:
        """Walk the grid from *start* for ``num_steps`` steps.

        Parameters
        ----------
        grid : np.ndarray shape (rows, cols)
            Occupancy grid — values ≥ 50 are obstacles.
        start : (row, col)
            Starting cell.

        Returns
        -------
        list of (row, col) waypoints.
        """
        rows, cols = grid.shape
        path: list[tuple[int, int]] = [start]
        current = start
        direction = self._rng.choice(_DIRECTIONS)
        steps_in_dir = 0
        max_steps_before_turn = 15  # bias toward straight runs

        for _ in range(self._num_steps):
            dr, dc = direction
            nr = current[0] + dr * self._step_size
            nc = current[1] + dc * self._step_size

            # Check if we can continue in current direction
            passable = (
                0 <= nr < rows
                and 0 <= nc < cols
                and grid[nr, nc] < 50
            )

            steps_in_dir += 1
            forced_turn = steps_in_dir >= max_steps_before_turn

            if passable and not forced_turn:
                current = (nr, nc)
                path.append(current)
            else:
                # Try all directions in random order
                directions_shuffled = _DIRECTIONS[:]
                self._rng.shuffle(directions_shuffled)
                moved = False
                for d in directions_shuffled:
                    tr = current[0] + d[0] * self._step_size
                    tc = current[1] + d[1] * self._step_size
                    if (
                        0 <= tr < rows
                        and 0 <= tc < cols
                        and grid[tr, tc] < 50
                    ):
                        direction = d
                        current = (tr, tc)
                        path.append(current)
                        steps_in_dir = 0
                        max_steps_before_turn = self._rng.randint(8, 20)
                        moved = True
                        break
                if not moved:
                    break   # Robot is completely stuck (shouldn't happen)

        return path

    def assign_to_robots(
        self,
        grid: np.ndarray,
        num_robots: int,
        start_positions: list[tuple[int, int]],
    ) -> dict[int, list[tuple[int, int]]]:
        """Generate independent random walk paths for each robot."""
        return {
            rid: self.generate_path(grid, start_positions[rid])
            for rid in range(num_robots)
        }
