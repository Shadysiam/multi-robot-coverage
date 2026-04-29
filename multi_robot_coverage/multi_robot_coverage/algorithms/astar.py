"""A* path planner on a 2-D occupancy grid with optional obstacle inflation."""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np


@dataclass(order=True)
class _Node:
    """Heap node that orders by f-score only."""

    f: float
    g: float = field(compare=False)
    pos: tuple[int, int] = field(compare=False)
    parent: Optional["_Node"] = field(default=None, compare=False)


class AStar:
    """A* search on a 2-D occupancy grid.

    Grid values >= ``occupied_threshold`` are treated as obstacles.
    The search uses 8-connectivity; diagonal moves are blocked if either
    adjacent cardinal neighbour is occupied (to avoid corner-clipping).

    Example
    -------
    >>> planner = AStar()
    >>> path = planner.search(grid, start=(5, 5), goal=(20, 30))
    """

    # (Δrow, Δcol, move-cost)
    _NEIGHBOURS: list[tuple[int, int, float]] = [
        (-1, 0, 1.0),
        (1, 0, 1.0),
        (0, -1, 1.0),
        (0, 1, 1.0),
        (-1, -1, math.sqrt(2)),
        (-1, 1, math.sqrt(2)),
        (1, -1, math.sqrt(2)),
        (1, 1, math.sqrt(2)),
    ]

    def __init__(self, occupied_threshold: int = 50) -> None:
        self._occ_thresh = occupied_threshold

    def search(
        self,
        grid: np.ndarray,
        start: tuple[int, int],
        goal: tuple[int, int],
        inflation_radius: int = 0,
    ) -> Optional[list[tuple[int, int]]]:
        """Find the shortest path from *start* to *goal*.

        Parameters
        ----------
        grid:
            2-D integer array; values >= ``occupied_threshold`` are obstacles.
        start:
            ``(row, col)`` source cell.
        goal:
            ``(row, col)`` destination cell.
        inflation_radius:
            Morphological dilation applied to the obstacle mask (cells).
            Set to ``robot_radius / resolution`` for collision-free paths.

        Returns
        -------
        list[tuple[int, int]] | None
            Ordered waypoints from *start* to *goal*, or ``None`` if the goal
            is unreachable.
        """
        rows, cols = grid.shape
        occupied = self._build_obstacle_mask(grid, inflation_radius)

        if occupied[start] or occupied[goal]:
            return None

        open_heap: list[_Node] = []
        # Maps pos → best-known g-cost to track duplicates in the heap.
        best_g: dict[tuple[int, int], float] = {}
        closed: set[tuple[int, int]] = set()

        start_node = _Node(f=self._h(start, goal), g=0.0, pos=start)
        heapq.heappush(open_heap, start_node)
        best_g[start] = 0.0

        while open_heap:
            current = heapq.heappop(open_heap)
            pos = current.pos

            if pos in closed:
                continue
            closed.add(pos)

            if pos == goal:
                return self._reconstruct(current)

            for dr, dc, cost in self._NEIGHBOURS:
                nr, nc = pos[0] + dr, pos[1] + dc
                nbr = (nr, nc)

                if not (0 <= nr < rows and 0 <= nc < cols):
                    continue
                if occupied[nr, nc] or nbr in closed:
                    continue
                # Block corner-clipping through diagonal gaps.
                if dr != 0 and dc != 0:
                    if occupied[pos[0] + dr, pos[1]] or occupied[pos[0], pos[1] + dc]:
                        continue

                new_g = current.g + cost
                if best_g.get(nbr, math.inf) <= new_g:
                    continue

                best_g[nbr] = new_g
                node = _Node(
                    f=new_g + self._h(nbr, goal),
                    g=new_g,
                    pos=nbr,
                    parent=current,
                )
                heapq.heappush(open_heap, node)

        return None

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _build_obstacle_mask(
        self, grid: np.ndarray, inflation_radius: int
    ) -> np.ndarray:
        """Return boolean obstacle mask, optionally dilated."""
        mask = grid >= self._occ_thresh
        if inflation_radius > 0:
            from scipy.ndimage import binary_dilation

            struct = np.ones(
                (2 * inflation_radius + 1, 2 * inflation_radius + 1), dtype=bool
            )
            mask = binary_dilation(mask, structure=struct)
        return mask

    @staticmethod
    def _h(pos: tuple[int, int], goal: tuple[int, int]) -> float:
        """Euclidean distance heuristic (admissible for 8-connectivity)."""
        return math.hypot(goal[0] - pos[0], goal[1] - pos[1])

    @staticmethod
    def _reconstruct(node: _Node) -> list[tuple[int, int]]:
        """Walk the parent chain to recover the path (start → goal)."""
        path: list[tuple[int, int]] = []
        cur: Optional[_Node] = node
        while cur is not None:
            path.append(cur.pos)
            cur = cur.parent
        return path[::-1]
