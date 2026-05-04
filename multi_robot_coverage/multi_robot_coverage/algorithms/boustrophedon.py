"""Boustrophedon Cellular Decomposition for multi-robot coverage planning.

The algorithm sweeps a vertical line from left to right across the occupancy
grid.  At each column it records the set of *free row-intervals* (slices).
When the connectivity between consecutive columns changes, a critical event
occurs and the active cell is split or closed.  Each resulting cell is a
connected polygon of the free space.  Within each cell a back-and-forth
(lawnmower) path is generated; cells are distributed among N robots by
greedy area-balancing.

Reference
---------
Choset, H. (2001). Coverage for robotics – A survey of recent results.
Annals of Mathematics and Artificial Intelligence, 31(1-4), 113-126.

Gong, X., et al. (2024). Multi-Robot Coverage Path Planning Based on
Boustrophedon Cellular Decomposition with Propagation-Based Task
Reallocation. Sensors, 24(23), 7482. https://doi.org/10.3390/s24237482
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np


@dataclass
class CoverageCell:
    """A contiguous free-space region produced by BCD.

    Attributes
    ----------
    cell_id:
        Unique integer identifier.
    points:
        Set of ``(row, col)`` grid coordinates belonging to this cell.
    """

    cell_id: int
    points: set[tuple[int, int]] = field(default_factory=set)

    # ------------------------------------------------------------------
    # Derived properties
    # ------------------------------------------------------------------

    @property
    def area(self) -> int:
        """Number of free cells in this region."""
        return len(self.points)

    @property
    def centroid(self) -> tuple[float, float]:
        """Mean (row, col) of all member points."""
        if not self.points:
            return (0.0, 0.0)
        pts = np.array(list(self.points))
        return float(pts[:, 0].mean()), float(pts[:, 1].mean())

    @property
    def bounding_box(self) -> tuple[int, int, int, int]:
        """Return ``(r_min, r_max, c_min, c_max)``."""
        pts = np.array(list(self.points))
        return (
            int(pts[:, 0].min()),
            int(pts[:, 0].max()),
            int(pts[:, 1].min()),
            int(pts[:, 1].max()),
        )


@dataclass
class _Slice:
    """Contiguous free row-interval within a single grid column."""

    row_min: int
    row_max: int  # inclusive

    def overlaps(self, other: "_Slice") -> bool:
        return self.row_min <= other.row_max and other.row_min <= self.row_max


class BoustrophedonDecomposer:
    """Decomposes a 2-D occupancy grid into coverage cells.

    Parameters
    ----------
    coverage_width:
        Spacing between parallel lawnmower strips in grid cells.
        Typically set to ``ceil(robot_diameter / resolution)``.
    occupied_threshold:
        Grid values ≥ this are treated as obstacles.
    """

    def __init__(
        self,
        coverage_width: int = 8,
        occupied_threshold: int = 50,
    ) -> None:
        self._coverage_width = max(1, coverage_width)
        self._occ_thresh = occupied_threshold

    # ------------------------------------------------------------------
    # Decomposition
    # ------------------------------------------------------------------

    def decompose(self, grid: np.ndarray) -> list[CoverageCell]:
        """Run BCD on *grid* and return the list of coverage cells.

        Parameters
        ----------
        grid:
            2-D integer occupancy grid (rows × cols).  Values ≥
            ``occupied_threshold`` are obstacles.

        Returns
        -------
        list[CoverageCell]
            Non-empty cells sorted by area (largest first).
        """
        rows, cols = grid.shape
        free = grid < self._occ_thresh  # boolean mask

        # active_cells: maps a representative (row_min, row_max) → cell
        active: dict[tuple[int, int], CoverageCell] = {}
        completed: list[CoverageCell] = []
        cell_counter = 0

        prev_slices: list[_Slice] = []

        for col in range(cols):
            cur_slices = self._column_slices(free[:, col])

            # Determine which previous slices map to which current slices.
            predecessors: dict[int, list[int]] = {
                j: [] for j in range(len(cur_slices))
            }
            successors: dict[int, list[int]] = {
                i: [] for i in range(len(prev_slices))
            }
            for i, ps in enumerate(prev_slices):
                for j, cs in enumerate(cur_slices):
                    if ps.overlaps(cs):
                        predecessors[j].append(i)
                        successors[i].append(j)

            new_active: dict[tuple[int, int], CoverageCell] = {}

            # Handle each current slice according to its event type.
            for j, cs in enumerate(cur_slices):
                preds = predecessors[j]
                key = (cs.row_min, cs.row_max)

                if len(preds) == 0:
                    # IN event – new cell starts.
                    cell = CoverageCell(cell_id=cell_counter)
                    cell_counter += 1
                    new_active[key] = cell

                elif len(preds) == 1:
                    i = preds[0]
                    ps = prev_slices[i]
                    prev_key = (ps.row_min, ps.row_max)

                    if len(successors[i]) == 1:
                        # ONE-TO-ONE: continue the same cell.
                        cell = active.get(prev_key, CoverageCell(cell_counter))
                        if prev_key not in active:
                            cell_counter += 1
                        new_active[key] = cell
                    else:
                        # SPLIT: one predecessor fans out to many successors.
                        # Close predecessor and start a fresh cell per successor.
                        old_cell = active.pop(prev_key, None)
                        if old_cell is not None:
                            completed.append(old_cell)
                        cell = CoverageCell(cell_id=cell_counter)
                        cell_counter += 1
                        new_active[key] = cell

                else:
                    # MERGE: multiple predecessors → one current slice.
                    # Close all predecessors and start one new cell.
                    for i in preds:
                        ps = prev_slices[i]
                        prev_key = (ps.row_min, ps.row_max)
                        old_cell = active.pop(prev_key, None)
                        if old_cell is not None:
                            completed.append(old_cell)
                    cell = CoverageCell(cell_id=cell_counter)
                    cell_counter += 1
                    new_active[key] = cell

            # Add all free cells in this column to their cell.
            for j, cs in enumerate(cur_slices):
                key = (cs.row_min, cs.row_max)
                cell = new_active.get(key)
                if cell is not None:
                    for r in range(cs.row_min, cs.row_max + 1):
                        cell.points.add((r, col))

            # Close any active cells that have no successor.
            for i, ps in enumerate(prev_slices):
                prev_key = (ps.row_min, ps.row_max)
                if prev_key in active and len(successors[i]) == 0:
                    completed.append(active.pop(prev_key))

            prev_slices = cur_slices
            active = new_active

        # Close whatever remains open at the last column.
        for cell in active.values():
            completed.append(cell)

        non_empty = [c for c in completed if c.area > 0]
        non_empty.sort(key=lambda c: c.area, reverse=True)
        return non_empty

    # ------------------------------------------------------------------
    # Path generation
    # ------------------------------------------------------------------

    def generate_path(
        self, cell: CoverageCell, grid: np.ndarray
    ) -> list[tuple[int, int]]:
        """Generate a boustrophedon (lawnmower) path within *cell*.

        Horizontal strips of width ``coverage_width`` are swept row by row.
        Alternate strips are traversed right-to-left to minimise travel.
        We always include a final strip near ``r_max`` so the bottom edge
        of the cell is fully covered (otherwise cells between the last
        strip and r_max get missed when the gap > coverage_width / 2).

        Parameters
        ----------
        cell:
            The coverage cell to sweep.
        grid:
            Original occupancy grid (used to filter obstacle pixels within
            the cell's bounding box).

        Returns
        -------
        list[tuple[int, int]]
            Ordered ``(row, col)`` waypoints for the sweep path.
        """
        if not cell.points:
            return []

        r_min, r_max, c_min, c_max = cell.bounding_box
        free = grid < self._occ_thresh

        # Build strip rows.  Always start at r_min, step by coverage_width,
        # and ensure the final strip is close to r_max (within half-spacing).
        strip_rows = list(range(r_min, r_max + 1, self._coverage_width))
        if not strip_rows:
            strip_rows = [r_min]
        if strip_rows[-1] < r_max - self._coverage_width // 2:
            strip_rows.append(r_max)

        path: list[tuple[int, int]] = []
        for strip_index, row in enumerate(strip_rows):
            strip_cols = [
                c
                for c in range(c_min, c_max + 1)
                if (row, c) in cell.points and free[row, c]
            ]
            if not strip_cols:
                continue
            if strip_index % 2 == 0:
                path.extend((row, c) for c in strip_cols)
            else:
                path.extend((row, c) for c in reversed(strip_cols))

        return path

    # ------------------------------------------------------------------
    # Robot assignment
    # ------------------------------------------------------------------

    def assign_to_robots(
        self,
        cells: list[CoverageCell],
        n_robots: int,
        robot_starts: Optional[list[tuple[int, int]]] = None,
        method: str = "hungarian",
    ) -> dict[int, list[CoverageCell]]:
        """Distribute cells across N robots, balancing area and travel.

        Parameters
        ----------
        cells:
            All coverage cells from ``decompose``.
        n_robots:
            Number of robots to distribute among.
        robot_starts:
            Per-robot starting (row, col) — used to favour assigning each
            robot the cells closest to its start.  When ``None``, the cells
            are simply area-balanced.
        method:
            ``"hungarian"`` (default) — Hungarian-flavoured iterative
            assignment minimising the *maximum* robot workload while
            preferring nearby cells.  Falls back to greedy if scipy isn't
            available.
            ``"greedy"`` — original greedy area-balancing.

        Returns
        -------
        dict[int, list[CoverageCell]]
            Maps robot ID (0-indexed) → list of cells (TSP-ordered).
        """
        if method == "greedy" or robot_starts is None:
            assignment = self._assign_greedy(cells, n_robots)
        else:
            assignment = self._assign_hungarian(cells, n_robots, robot_starts)

        # TSP-order each robot's cells by nearest-neighbour from its start.
        if robot_starts is not None:
            for rid, robot_cells in assignment.items():
                start = robot_starts[rid] if rid < len(robot_starts) else (0, 0)
                assignment[rid] = self._order_cells_nearest_neighbour(
                    robot_cells, start
                )
        return assignment

    @staticmethod
    def _assign_greedy(
        cells: list[CoverageCell], n_robots: int
    ) -> dict[int, list[CoverageCell]]:
        """Original greedy area-balancing baseline."""
        assignment: dict[int, list[CoverageCell]] = {
            i: [] for i in range(n_robots)
        }
        workload = [0] * n_robots
        for cell in sorted(cells, key=lambda c: c.area, reverse=True):
            robot_id = workload.index(min(workload))
            assignment[robot_id].append(cell)
            workload[robot_id] += cell.area
        return assignment

    @staticmethod
    def _assign_hungarian(
        cells: list[CoverageCell],
        n_robots: int,
        robot_starts: list[tuple[int, int]],
    ) -> dict[int, list[CoverageCell]]:
        """Iterative Hungarian — repeatedly pick the optimal matching of
        the next ``n_robots`` un-assigned cells (sorted by area desc) to
        robots, with cost = α·distance + β·current_workload.

        This minimises the *max* workload while preferring nearby cells,
        which is what we actually care about (mission completion time).
        """
        try:
            from scipy.optimize import linear_sum_assignment
        except ImportError:
            return BoustrophedonDecomposer._assign_greedy(cells, n_robots)

        ALPHA = 1.0     # distance weight
        BETA  = 50.0    # workload-balance weight (per robot cell)

        assignment: dict[int, list[CoverageCell]] = {
            i: [] for i in range(n_robots)
        }
        workload = [0] * n_robots
        # Process cells in batches of n_robots (largest first)
        ordered_cells = sorted(cells, key=lambda c: c.area, reverse=True)

        for i in range(0, len(ordered_cells), n_robots):
            batch = ordered_cells[i: i + n_robots]
            if not batch:
                break

            # Pad batch with dummy cells of zero cost if smaller than n_robots
            n_b = len(batch)
            cost = np.zeros((n_robots, max(n_robots, n_b)), dtype=np.float64)
            for r in range(n_robots):
                for c in range(n_b):
                    cell = batch[c]
                    cr, cc = cell.centroid
                    sr, sc = robot_starts[r]
                    dist = math.hypot(cr - sr, cc - sc)
                    cost[r, c] = ALPHA * dist + BETA * workload[r]
                # Pad columns if any
                for c in range(n_b, n_robots):
                    cost[r, c] = 1e9   # avoid the dummy

            row_ind, col_ind = linear_sum_assignment(cost)
            for r, c in zip(row_ind, col_ind):
                if c < n_b:
                    assignment[r].append(batch[c])
                    workload[r] += batch[c].area
        return assignment

    @staticmethod
    def _order_cells_nearest_neighbour(
        cells: list[CoverageCell],
        start: tuple[int, int],
    ) -> list[CoverageCell]:
        """Reorder a robot's cell list using nearest-neighbour TSP from
        the start position.  Greedy but very effective for small N (3-6
        cells per robot)."""
        if not cells:
            return cells
        remaining = list(cells)
        ordered: list[CoverageCell] = []
        cur = start
        while remaining:
            nxt = min(
                remaining,
                key=lambda c: math.hypot(
                    c.centroid[0] - cur[0], c.centroid[1] - cur[1]
                ),
            )
            ordered.append(nxt)
            cur = (int(nxt.centroid[0]), int(nxt.centroid[1]))
            remaining.remove(nxt)
        return ordered

    def reallocate_failed_robot(
        self,
        failed_robot_id: int,
        remaining_cells: list[CoverageCell],
        active_assignments: dict[int, list[CoverageCell]],
        robot_positions: dict[int, tuple[int, int]],
    ) -> dict[int, list[CoverageCell]]:
        """Redistribute a failed robot's remaining cells using propagation.

        Implements the propagation-based reallocation strategy described in
        Gong et al. (2024): remaining cells are sorted by proximity to the
        failed robot's last position, then assigned round-robin to active
        robots sorted by their distance to each cell centroid.

        Parameters
        ----------
        failed_robot_id:
            ID of the robot that failed.
        remaining_cells:
            Cells that the failed robot had not yet completed.
        active_assignments:
            Current cell assignment map (modified in-place).
        robot_positions:
            Current ``(row, col)`` of each robot.

        Returns
        -------
        dict[int, list[CoverageCell]]
            Updated assignment map.
        """
        failed_pos = robot_positions.get(failed_robot_id, (0, 0))
        active_ids = [
            rid for rid in active_assignments if rid != failed_robot_id
        ]

        if not active_ids:
            return active_assignments

        # Sort remaining cells by distance from the failed robot (nearest first).
        def dist_to_failed(c: CoverageCell) -> float:
            cr, cc = c.centroid
            return math.hypot(cr - failed_pos[0], cc - failed_pos[1])

        remaining_sorted = sorted(remaining_cells, key=dist_to_failed)

        # Assign each remaining cell to the nearest active robot.
        for cell in remaining_sorted:
            cr, cc = cell.centroid
            nearest_id = min(
                active_ids,
                key=lambda rid: math.hypot(
                    robot_positions.get(rid, (0, 0))[0] - cr,
                    robot_positions.get(rid, (0, 0))[1] - cc,
                ),
            )
            active_assignments[nearest_id].append(cell)

        # Remove failed robot entry.
        active_assignments.pop(failed_robot_id, None)
        return active_assignments

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _column_slices(free_col: np.ndarray) -> list[_Slice]:
        """Extract contiguous free row-intervals from a single column."""
        slices: list[_Slice] = []
        in_slice = False
        start = 0
        for r, is_free in enumerate(free_col):
            if is_free and not in_slice:
                start = r
                in_slice = True
            elif not is_free and in_slice:
                slices.append(_Slice(row_min=start, row_max=r - 1))
                in_slice = False
        if in_slice:
            slices.append(_Slice(row_min=start, row_max=len(free_col) - 1))
        return slices
