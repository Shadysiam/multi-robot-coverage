"""RViz2 visualizer node.

Subscribes to robot poses, coverage map, and stats; publishes a rich set of
MarkerArray messages so the operator can see robot positions, coverage
progress, planned paths, and failure events in real time.

ROS2 Topics
-----------
Subscribed
~~~~~~~~~~
  /robot_{id}/pose    (geometry_msgs/PoseStamped)   — robot position
  /robot_{id}/status  (std_msgs/String)              — active|failed|…
  /coverage_stats     (multi_robot_coverage_msgs/CoverageStats)

Published
~~~~~~~~~
  /visualization_marker_array  (visualization_msgs/MarkerArray)

Marker layout
-------------
  ID  0..N-1   : robot arrow markers (coloured by robot ID)
  ID  N..2N-1  : robot trail line strips
  ID  2N       : coverage percentage text overlay
  ID  2N+1     : algorithm label text
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Point, PoseStamped, Vector3
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

from multi_robot_coverage_msgs.msg import CoverageStats


# RGBA colours per robot (up to 6 robots).
_ROBOT_COLOURS: list[tuple[float, float, float]] = [
    (0.18, 0.60, 0.86),   # blue
    (0.20, 0.73, 0.36),   # green
    (0.95, 0.61, 0.07),   # orange
    (0.74, 0.18, 0.70),   # purple
    (0.88, 0.19, 0.19),   # red
    (0.09, 0.72, 0.71),   # teal
]
_FAILED_COLOUR = (0.90, 0.10, 0.10)  # bright red for failed robots

_MAX_TRAIL = 500   # maximum points kept in each trail


class VisualizerNode(Node):
    """Publishes RViz2 markers for all robots and coverage statistics."""

    def __init__(self) -> None:
        super().__init__("visualizer")

        self.declare_parameter("num_robots", 3)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("update_rate", 5.0)

        self._n: int = self.get_parameter("num_robots").value
        self._frame: str = self.get_parameter("map_frame").value
        rate: float = self.get_parameter("update_rate").value

        _tl = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._pub = self.create_publisher(
            MarkerArray, "/visualization_marker_array", 10
        )

        self._robot_poses: dict[int, PoseStamped] = {}
        self._robot_statuses: dict[int, str] = {}
        self._robot_trails: dict[int, list[Point]] = {i: [] for i in range(self._n)}
        self._stats: Optional[CoverageStats] = None

        for i in range(self._n):
            self.create_subscription(
                PoseStamped,
                f"/robot_{i}/pose",
                lambda msg, rid=i: self._cb_pose(msg, rid),
                10,
            )
            self.create_subscription(
                String,
                f"/robot_{i}/status",
                lambda msg, rid=i: self._cb_status(msg, rid),
                _tl,
            )

        self.create_subscription(
            CoverageStats, "/coverage_stats", self._cb_stats, 10
        )

        self._timer = self.create_timer(1.0 / rate, self._publish_markers)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _cb_pose(self, msg: PoseStamped, robot_id: int) -> None:
        self._robot_poses[robot_id] = msg
        trail = self._robot_trails[robot_id]
        pt = Point(
            x=msg.pose.position.x,
            y=msg.pose.position.y,
            z=0.05,
        )
        trail.append(pt)
        if len(trail) > _MAX_TRAIL:
            trail.pop(0)

    def _cb_status(self, msg: String, robot_id: int) -> None:
        self._robot_statuses[robot_id] = msg.data

    def _cb_stats(self, msg: CoverageStats) -> None:
        self._stats = msg

    # ------------------------------------------------------------------
    # Marker construction
    # ------------------------------------------------------------------

    def _publish_markers(self) -> None:
        array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        for i in range(self._n):
            pose = self._robot_poses.get(i)
            status = self._robot_statuses.get(i, "idle")
            failed = status == "failed"
            colour = _FAILED_COLOUR if failed else _ROBOT_COLOURS[i % len(_ROBOT_COLOURS)]

            # Arrow marker for robot body.
            arrow = self._make_arrow(
                marker_id=i,
                pose=pose,
                colour=colour,
                stamp=stamp,
                scale=0.35,
            )
            array.markers.append(arrow)

            # Trail line strip.
            trail_marker = self._make_trail(
                marker_id=self._n + i,
                points=self._robot_trails[i],
                colour=colour,
                stamp=stamp,
            )
            array.markers.append(trail_marker)

            # Robot ID label floating above the arrow.
            if pose is not None:
                label = self._make_text(
                    marker_id=2 * self._n + i,
                    x=pose.pose.position.x,
                    y=pose.pose.position.y,
                    z=0.5,
                    text=f"R{i}" + (" ✗" if failed else ""),
                    colour=colour,
                    stamp=stamp,
                    scale=0.25,
                )
                array.markers.append(label)

        # Coverage percentage HUD.
        hud = self._make_hud(
            marker_id=3 * self._n,
            stamp=stamp,
        )
        array.markers.append(hud)

        self._pub.publish(array)

    def _make_arrow(
        self,
        marker_id: int,
        pose: Optional[PoseStamped],
        colour: tuple[float, float, float],
        stamp,
        scale: float,
    ) -> Marker:
        m = Marker()
        m.header.frame_id = self._frame
        m.header.stamp = stamp
        m.ns = "robots"
        m.id = marker_id
        m.type = Marker.ARROW
        m.action = Marker.ADD if pose is not None else Marker.DELETE
        if pose is not None:
            m.pose = pose.pose
        m.scale.x = scale
        m.scale.y = scale * 0.4
        m.scale.z = scale * 0.4
        m.color = _rgba(*colour)
        m.lifetime.sec = 1
        return m

    def _make_trail(
        self,
        marker_id: int,
        points: list[Point],
        colour: tuple[float, float, float],
        stamp,
    ) -> Marker:
        m = Marker()
        m.header.frame_id = self._frame
        m.header.stamp = stamp
        m.ns = "trails"
        m.id = marker_id
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.04
        m.color = _rgba(*colour, alpha=0.6)
        m.points = list(points)
        m.lifetime.sec = 0  # persist until deleted
        return m

    def _make_text(
        self,
        marker_id: int,
        x: float,
        y: float,
        z: float,
        text: str,
        colour: tuple[float, float, float],
        stamp,
        scale: float = 0.3,
    ) -> Marker:
        m = Marker()
        m.header.frame_id = self._frame
        m.header.stamp = stamp
        m.ns = "labels"
        m.id = marker_id
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z
        m.pose.orientation.w = 1.0
        m.scale.z = scale
        m.color = _rgba(*colour)
        m.text = text
        m.lifetime.sec = 1
        return m

    def _make_hud(self, marker_id: int, stamp) -> Marker:
        """Stats text shown at a fixed position in the scene."""
        pct = 0.0
        elapsed = 0.0
        algo = "—"
        active = 0
        total = self._n
        if self._stats is not None:
            pct = self._stats.coverage_percentage
            elapsed = self._stats.elapsed_time
            algo = self._stats.algorithm
            active = self._stats.robots_active
            total = self._stats.total_robots

        lines = [
            f"Algorithm : {algo}",
            f"Coverage  : {pct:.1f}%",
            f"Elapsed   : {elapsed:.1f}s",
            f"Robots    : {active}/{total} active",
        ]
        text = "\n".join(lines)

        m = Marker()
        m.header.frame_id = self._frame
        m.header.stamp = stamp
        m.ns = "hud"
        m.id = marker_id
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = 0.5
        m.pose.position.y = -1.0
        m.pose.position.z = 0.5
        m.pose.orientation.w = 1.0
        m.scale.z = 0.30
        m.color = _rgba(1.0, 1.0, 1.0)
        m.text = text
        m.lifetime.sec = 2
        return m


# ------------------------------------------------------------------
# Helpers
# ------------------------------------------------------------------


def _rgba(r: float, g: float, b: float, alpha: float = 1.0) -> ColorRGBA:
    c = ColorRGBA()
    c.r = float(r)
    c.g = float(g)
    c.b = float(b)
    c.a = float(alpha)
    return c


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = VisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
