"""Procedural PGM map generator for the multi_robot_coverage package.

Run once before building the workspace to create the three demo maps:

    ros2 run multi_robot_coverage generate_maps

Or directly:

    python3 -m multi_robot_coverage.map_generator

Maps are written to the ``maps/`` directory relative to this file's location.
"""

from __future__ import annotations

import os
import struct
from pathlib import Path

import numpy as np


# ---------------------------------------------------------------------------
# PGM writer
# ---------------------------------------------------------------------------


def _write_pgm_binary(path: Path, img: np.ndarray) -> None:
    """Write a 2-D uint8 array as a binary (P5) PGM file.

    Parameters
    ----------
    path:
        Destination file path.
    img:
        2-D array with dtype uint8.  Row 0 is the top of the image.
    """
    rows, cols = img.shape
    with open(path, "wb") as fh:
        header = f"P5\n{cols} {rows}\n255\n".encode("ascii")
        fh.write(header)
        fh.write(img.tobytes())


def _canvas(rows: int, cols: int, wall: int = 5) -> np.ndarray:
    """Return a blank white canvas with black border walls."""
    img = np.full((rows, cols), 255, dtype=np.uint8)
    img[:wall, :] = 0
    img[-wall:, :] = 0
    img[:, :wall] = 0
    img[:, -wall:] = 0
    return img


def _rect(img: np.ndarray, r0: int, c0: int, r1: int, c1: int) -> None:
    """Fill a rectangle with black (obstacle) pixels (inclusive bounds)."""
    img[r0 : r1 + 1, c0 : c1 + 1] = 0


# ---------------------------------------------------------------------------
# Map definitions
# ---------------------------------------------------------------------------


def make_simple_room(rows: int = 200, cols: int = 200) -> np.ndarray:
    """10 × 10 m open room (at 0.05 m/px)."""
    return _canvas(rows, cols)


def make_obstacle_room(rows: int = 200, cols: int = 200) -> np.ndarray:
    """10 × 10 m room with four interior obstacles."""
    img = _canvas(rows, cols)
    # Top-left block
    _rect(img, 40, 40, 80, 70)
    # Top-right block
    _rect(img, 40, 120, 80, 160)
    # Bottom-centre pillar
    _rect(img, 120, 80, 165, 120)
    # Horizontal divider (partial)
    _rect(img, 95, 30, 105, 90)
    return img


def make_warehouse(rows: int = 200, cols: int = 300) -> np.ndarray:
    """10 × 15 m warehouse with three rows of shelves (at 0.05 m/px)."""
    img = _canvas(rows, cols)
    shelf_height = 10   # pixels (~0.5 m wide shelf)
    shelf_length = 120  # pixels (~6 m long shelf)
    aisle_gap = 50      # pixels (~2.5 m between shelf rows)
    shelf_offset = 30   # distance from left wall

    # Three shelf rows, each containing two shelf units with a gap between.
    row_starts = [30, 30 + aisle_gap + shelf_height, 30 + 2 * (aisle_gap + shelf_height)]
    for r_start in row_starts:
        if r_start + shelf_height >= rows - 5:
            break
        # First shelf unit
        _rect(img, r_start, shelf_offset, r_start + shelf_height, shelf_offset + shelf_length)
        # Second shelf unit (gap in middle for cross-aisle)
        c2 = shelf_offset + shelf_length + 20
        if c2 + shelf_length < cols - 5:
            _rect(img, r_start, c2, r_start + shelf_height, c2 + shelf_length)

    return img


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main() -> None:
    """Generate all three demo maps and write them to the maps directory."""
    maps_dir = Path(__file__).parent.parent / "maps"
    maps_dir.mkdir(parents=True, exist_ok=True)

    specs: list[tuple[str, np.ndarray]] = [
        ("simple_room", make_simple_room()),
        ("obstacle_room", make_obstacle_room()),
        ("warehouse", make_warehouse()),
    ]

    for name, img in specs:
        pgm_path = maps_dir / f"{name}.pgm"
        _write_pgm_binary(pgm_path, img)
        print(f"[map_generator] Wrote {pgm_path}  ({img.shape[1]}×{img.shape[0]} px)")

    print("[map_generator] Done.")


if __name__ == "__main__":
    main()
