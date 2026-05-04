"""Map server node — loads a PGM+YAML map and publishes it as OccupancyGrid.

ROS2 Topics
-----------
Published
~~~~~~~~~
  /map  (nav_msgs/OccupancyGrid)  — full occupancy grid, latched (transient local)

ROS2 Parameters
---------------
  map_name   str   — basename without extension, e.g. "simple_room"
  maps_dir   str   — absolute path to the directory containing PGM+YAML files
"""

from __future__ import annotations

import os
import struct
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from builtin_interfaces.msg import Time


class MapServerNode(Node):
    """Loads a PGM map and publishes it once on /map with transient-local QoS.

    The published grid uses the standard ROS occupancy encoding:
      -1  = unknown
       0  = free
     100  = occupied
    """

    def __init__(self) -> None:
        super().__init__("map_server")

        self.declare_parameter("map_name", "simple_room")
        self.declare_parameter(
            "maps_dir",
            str(Path(__file__).parent.parent.parent / "maps"),
        )

        self._map_name: str = self.get_parameter("map_name").value
        self._maps_dir: str = self.get_parameter("maps_dir").value

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub = self.create_publisher(OccupancyGrid, "/map", qos)

        self._grid_msg: Optional[OccupancyGrid] = None
        self._load_map(self._maps_dir, self._map_name)

        # Live map switching from the dashboard
        from std_msgs.msg import String
        self._sub_set_map = self.create_subscription(
            String, '/set_map', self._cb_set_map, 10
        )

        # Re-publish at 1 Hz so late-joining nodes receive it.
        self._timer = self.create_timer(1.0, self._publish)

    # ------------------------------------------------------------------
    # Live map switch
    # ------------------------------------------------------------------

    def _cb_set_map(self, msg) -> None:
        new_name = (msg.data or "").strip()
        if not new_name or new_name == self._map_name:
            return
        self.get_logger().info(
            f"Switching map: {self._map_name} → {new_name}"
        )
        self._map_name = new_name
        self._load_map(self._maps_dir, new_name)
        # Immediately republish so coordinator picks it up
        self._publish()

    # ------------------------------------------------------------------
    # Map loading
    # ------------------------------------------------------------------

    def _load_map(self, maps_dir: str, map_name: str) -> None:
        yaml_path = Path(maps_dir) / f"{map_name}.yaml"
        if not yaml_path.exists():
            self.get_logger().error(f"Map YAML not found: {yaml_path}")
            return

        with open(yaml_path) as fh:
            meta = yaml.safe_load(fh)

        pgm_path = Path(maps_dir) / meta["image"]
        if not pgm_path.exists():
            self.get_logger().error(f"PGM file not found: {pgm_path}")
            return

        img = self._read_pgm(pgm_path)
        if img is None:
            return

        resolution: float = float(meta.get("resolution", 0.05))
        origin: list[float] = meta.get("origin", [0.0, 0.0, 0.0])
        negate: int = int(meta.get("negate", 0))
        occ_thresh: float = float(meta.get("occupied_thresh", 0.65))
        free_thresh: float = float(meta.get("free_thresh", 0.196))

        if negate:
            img = 255 - img

        # Convert to ROS occupancy values.
        rows, cols = img.shape
        data = np.full(rows * cols, -1, dtype=np.int8)

        # In PGM: 255 = white = free,  0 = black = occupied.
        normalised = img.astype(np.float32) / 255.0
        for r in range(rows):
            for c in range(cols):
                p = float(normalised[r, c])
                # ROS map stores row 0 at bottom, PGM row 0 at top → flip.
                ros_r = rows - 1 - r
                idx = ros_r * cols + c
                if p < free_thresh:
                    data[idx] = 100   # occupied
                elif p > (1.0 - occ_thresh):
                    data[idx] = 0     # free
                # else: unknown (-1)

        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.info.resolution = resolution
        msg.info.width = cols
        msg.info.height = rows
        msg.info.origin.position.x = float(origin[0])
        msg.info.origin.position.y = float(origin[1])
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = data.tolist()

        self._grid_msg = msg
        self.get_logger().info(
            f"Loaded map '{map_name}' ({cols}×{rows} cells, "
            f"{resolution:.3f} m/cell)"
        )

    # ------------------------------------------------------------------
    # PGM reader (supports P5 binary and P2 ASCII)
    # ------------------------------------------------------------------

    @staticmethod
    def _read_pgm(path: Path) -> Optional[np.ndarray]:
        with open(path, "rb") as fh:
            magic = fh.readline().strip()
            # Skip comment lines.
            while True:
                line = fh.readline().strip()
                if not line.startswith(b"#"):
                    break
            dims = line.split()
            cols, rows = int(dims[0]), int(dims[1])
            max_val = int(fh.readline().strip())

            if magic == b"P5":
                raw = fh.read(rows * cols)
                img = np.frombuffer(raw, dtype=np.uint8).reshape(rows, cols)
            elif magic == b"P2":
                values = list(map(int, fh.read().split()))
                img = np.array(values, dtype=np.uint8).reshape(rows, cols)
            else:
                return None

        if max_val != 255:
            img = (img.astype(np.float32) * 255.0 / max_val).astype(np.uint8)
        return img

    # ------------------------------------------------------------------
    # Publisher callback
    # ------------------------------------------------------------------

    def _publish(self) -> None:
        if self._grid_msg is None:
            return
        self._grid_msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(self._grid_msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MapServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
