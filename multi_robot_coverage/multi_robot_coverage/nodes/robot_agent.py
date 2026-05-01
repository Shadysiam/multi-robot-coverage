"""Robot agent node — simulates a single coverage robot.

Each robot:
  - Receives a waypoint path from the coordinator via ``/robot_{id}/waypoints``
  - Advances along the path at ``robot_speed`` m/s at 20 Hz
  - Publishes its current pose and the path it is following
  - Can be killed mid-mission to demonstrate failure/reallocation

ROS2 Topics
-----------
Subscribed
~~~~~~~~~~
  /robot_{id}/waypoints     (nav_msgs/Path)       — assigned waypoint list
  /robot_{id}/fail_trigger  (std_msgs/Bool)        — set True to simulate failure

Published
~~~~~~~~~
  /robot_{id}/pose          (geometry_msgs/PoseStamped)  — current pose
  /robot_{id}/path          (nav_msgs/Path)               — current planned path
  /robot_{id}/status        (std_msgs/String)             — idle|active|complete|failed

ROS2 Parameters
---------------
  robot_id     int    — unique robot identifier (0-indexed)
  robot_speed  float  — simulated speed in m/s (default 1.0)
  map_frame    str    — TF frame name for all poses (default "map")
  update_rate  float  — control loop Hz (default 20.0)
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Float64, String
import tf2_ros
from geometry_msgs.msg import TransformStamped


def _yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert a yaw angle (radians) to a ROS Quaternion."""
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class RobotAgentNode(Node):
    """Simulated robot agent that follows a waypoint path.

    The node advances the robot along waypoints at a configurable speed.
    When the robot reaches the final waypoint the status transitions to
    ``complete``.  A ``True`` on the fail_trigger topic stops the robot
    and transitions to ``failed``.
    """

    _STATUS_IDLE = "idle"
    _STATUS_ACTIVE = "active"
    _STATUS_COMPLETE = "complete"
    _STATUS_FAILED = "failed"

    def __init__(self) -> None:
        super().__init__("robot_agent")

        self.declare_parameter("robot_id", 0)
        self.declare_parameter("robot_speed", 1.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("update_rate", 20.0)

        self._robot_id: int = self.get_parameter("robot_id").value
        self._speed: float = self.get_parameter("robot_speed").value
        self._frame: str = self.get_parameter("map_frame").value
        rate: float = self.get_parameter("update_rate").value

        ns = f"robot_{self._robot_id}"

        # Transient-local QoS so late subscribers receive last message.
        _tl_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._pub_pose = self.create_publisher(PoseStamped, f"/{ns}/pose", 10)
        self._pub_path = self.create_publisher(Path, f"/{ns}/path", _tl_qos)
        self._pub_status = self.create_publisher(String, f"/{ns}/status", _tl_qos)

        self._sub_waypoints = self.create_subscription(
            Path,
            f"/{ns}/waypoints",
            self._cb_waypoints,
            _tl_qos,
        )
        self._sub_fail = self.create_subscription(
            Bool,
            f"/{ns}/fail_trigger",
            self._cb_fail,
            10,
        )
        # Live speed control from dashboard
        self._sub_speed = self.create_subscription(
            Float64, '/set_speed', self._cb_set_speed, 10
        )

        # TF broadcaster for RViz visualisation.
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Internal state.
        self._waypoints: list[PoseStamped] = []
        self._wp_index: int = 0
        self._status: str = self._STATUS_IDLE
        self._x: float = 0.0
        self._y: float = 0.0
        self._yaw: float = 0.0
        self._dt: float = 1.0 / rate

        self._timer = self.create_timer(self._dt, self._step)
        self.get_logger().info(
            f"Robot {self._robot_id} agent ready  (speed={self._speed} m/s)"
        )

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------

    def _cb_waypoints(self, msg: Path) -> None:
        """Receive a new waypoint path and start executing it."""
        if self._status == self._STATUS_FAILED:
            return
        # Empty path = reset signal (algorithm switch)
        if not msg.poses:
            self._waypoints = []
            self._wp_index = 0
            self._status = self._STATUS_IDLE
            self._publish_status()
            return
        self._waypoints = list(msg.poses)
        self._wp_index = 0
        if self._waypoints:
            first = self._waypoints[0]
            self._x = first.pose.position.x
            self._y = first.pose.position.y
            self._status = self._STATUS_ACTIVE
            self.get_logger().info(
                f"Robot {self._robot_id}: received path with "
                f"{len(self._waypoints)} waypoints"
            )
            self._pub_path.publish(msg)
        self._publish_status()

    def _cb_fail(self, msg: Bool) -> None:
        """Simulate robot failure."""
        if msg.data and self._status != self._STATUS_FAILED:
            self._status = self._STATUS_FAILED
            self._publish_status()
            self.get_logger().warn(f"Robot {self._robot_id}: FAILED (simulated)")

    def _cb_set_speed(self, msg: Float64) -> None:
        """Update robot speed live from the dashboard."""
        new_speed = max(0.1, min(10.0, float(msg.data)))
        self._speed = new_speed

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def _step(self) -> None:
        """Advance robot position by one timestep along its path."""
        if self._status != self._STATUS_ACTIVE:
            self._publish_pose()
            return

        if self._wp_index >= len(self._waypoints):
            self._status = self._STATUS_COMPLETE
            self._publish_status()
            self.get_logger().info(f"Robot {self._robot_id}: path COMPLETE")
            self._publish_pose()
            return

        target = self._waypoints[self._wp_index]
        tx = target.pose.position.x
        ty = target.pose.position.y

        dx = tx - self._x
        dy = ty - self._y
        dist = math.hypot(dx, dy)
        step = self._speed * self._dt

        if dist <= step:
            self._x = tx
            self._y = ty
            self._wp_index += 1
        else:
            ratio = step / dist
            self._x += dx * ratio
            self._y += dy * ratio
            self._yaw = math.atan2(dy, dx)

        self._publish_pose()

    # ------------------------------------------------------------------
    # Publishers
    # ------------------------------------------------------------------

    def _publish_pose(self) -> None:
        """Publish current pose and broadcast TF transform."""
        stamp = self.get_clock().now().to_msg()
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame
        msg.pose.position.x = self._x
        msg.pose.position.y = self._y
        msg.pose.position.z = 0.0
        msg.pose.orientation = _yaw_to_quaternion(self._yaw)
        self._pub_pose.publish(msg)

        # TF: map → robot_{id}/base_link
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self._frame
        tf.child_frame_id = f"robot_{self._robot_id}/base_link"
        tf.transform.translation.x = self._x
        tf.transform.translation.y = self._y
        tf.transform.translation.z = 0.0
        tf.transform.rotation = _yaw_to_quaternion(self._yaw)
        self._tf_broadcaster.sendTransform(tf)

    def _publish_status(self) -> None:
        msg = String()
        msg.data = self._status
        self._pub_status.publish(msg)

    # ------------------------------------------------------------------
    # Accessors used by coordinator when running in the same process
    # ------------------------------------------------------------------

    @property
    def robot_id(self) -> int:
        return self._robot_id

    @property
    def status(self) -> str:
        return self._status

    @property
    def position(self) -> tuple[float, float]:
        return (self._x, self._y)

    @property
    def wp_index(self) -> int:
        return self._wp_index

    @property
    def total_waypoints(self) -> int:
        return len(self._waypoints)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RobotAgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
