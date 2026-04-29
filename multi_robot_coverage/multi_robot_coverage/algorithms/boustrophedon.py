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

        path: list[tuple[int, int]] = []
        row = r_min
        strip_index = 0

        while row <= r_max:
            # Collect passable columns on this strip row.
            strip_cols = [
                c
                for c in range(c_min, c_max + 1)
                if (row, c) in cell.points and free[row, c]
            ]
            if strip_cols:
                if strip_index % 2 == 0:
                    path.extend((row, c) for c in strip_cols)
                else:
                    path.extend((row, c) for c in reversed(strip_cols))
            row += self._coverage_width
            strip_index += 1

        return path

    # ------------------------------------------------------------------
    # Robot assignment
    # ------------------------------------------------------------------

    def assign_to_robots(
        self, cells: list[CoverageCell], n_robots: int
    ) -> dict[int, list[CoverageCell]]:
        """Distribute cells across N robots by greedy area-balancing.

        The cell list is sorted by area (descending) and assigned to
        whichever robot currently has the smallest total workload.

        Parameters
        ----------
        cells:
            All coverage cells from ``decompose``.
        n_robots:
            Number of robots to distribute among.

        Returns
        -------
        dict[int, list[CoverageCell]]
            Maps robot ID (0-indexed) → ordered list of cells.
        """
        assignment: dict[int, list[CoverageCell]] = {
            i: [] for i in range(n_robots)
        }
        workload = [0] * n_robots

        for cell in sorted(cells, key=lambda c: c.area, reverse=True):
            robot_id = workload.index(min(workload))
            assignment[robot_id].append(cell)
            workload[robot_id] += cell.area

        return assignment

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
