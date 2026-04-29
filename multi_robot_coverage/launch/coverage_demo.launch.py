"""Launch file for the multi_robot_coverage demo.

Usage examples
--------------
  ros2 launch multi_robot_coverage coverage_demo.launch.py

  ros2 launch multi_robot_coverage coverage_demo.launch.py \\
      num_robots:=4 algorithm:=boustrophedon map:=warehouse \\
      enable_failure_sim:=true failure_time:=20.0

  ros2 launch multi_robot_coverage coverage_demo.launch.py \\
      num_robots:=2 algorithm:=frontier map:=obstacle_room robot_speed:=1.5
"""

from __future__ import annotations

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# ---------------------------------------------------------------------------
# Argument declarations
# ---------------------------------------------------------------------------

_ARGS: list[tuple[str, str, str]] = [
    # (name, default, description)
    ("num_robots", "3", "Number of coverage robots (1–6)"),
    ("algorithm", "boustrophedon", "Coverage algorithm: boustrophedon | frontier"),
    ("map", "simple_room", "Map basename: simple_room | obstacle_room | warehouse"),
    ("robot_speed", "1.0", "Simulated robot speed in m/s"),
    ("enable_failure_sim", "false", "Simulate one robot failing mid-mission"),
    ("failure_time", "30.0", "Seconds after start to trigger simulated failure"),
    ("failure_robot_id", "-1", "Robot ID to fail (-1 = random active robot)"),
    ("coverage_width_m", "0.4", "Lawnmower strip width in metres"),
    ("robot_radius_m", "0.2", "Robot radius used for A* obstacle inflation"),
    ("use_rviz", "true", "Launch RViz2 for visualisation"),
]


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    for name, default, description in _ARGS:
        ld.add_action(
            DeclareLaunchArgument(name, default_value=default, description=description)
        )

    ld.add_action(OpaqueFunction(function=_launch_nodes))
    return ld


# ---------------------------------------------------------------------------
# Dynamic node spawning (needs OpaqueFunction to read LaunchConfigurations)
# ---------------------------------------------------------------------------


def _launch_nodes(context, *args, **kwargs) -> list:
    pkg_share = get_package_share_directory("multi_robot_coverage")
    maps_dir = os.path.join(pkg_share, "maps")
    config_dir = os.path.join(pkg_share, "config")
    rviz_cfg = os.path.join(config_dir, "coverage.rviz")

    num_robots = int(LaunchConfiguration("num_robots").perform(context))
    algorithm = LaunchConfiguration("algorithm").perform(context)
    map_name = LaunchConfiguration("map").perform(context)
    robot_speed = LaunchConfiguration("robot_speed").perform(context)
    enable_fail = LaunchConfiguration("enable_failure_sim").perform(context).lower()
    failure_time = LaunchConfiguration("failure_time").perform(context)
    failure_robot = LaunchConfiguration("failure_robot_id").perform(context)
    cov_width = LaunchConfiguration("coverage_width_m").perform(context)
    robot_radius = LaunchConfiguration("robot_radius_m").perform(context)
    use_rviz = LaunchConfiguration("use_rviz").perform(context).lower() == "true"

    nodes: list = []

    # ------------------------------------------------------------------
    # Map server
    # ------------------------------------------------------------------
    nodes.append(
        Node(
            package="multi_robot_coverage",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[
                {
                    "map_name": map_name,
                    "maps_dir": maps_dir,
                }
            ],
        )
    )

    # ------------------------------------------------------------------
    # One robot_agent node per robot
    # ------------------------------------------------------------------
    for i in range(num_robots):
        nodes.append(
            Node(
                package="multi_robot_coverage",
                executable="robot_agent",
                name=f"robot_agent_{i}",
                output="screen",
                parameters=[
                    {
                        "robot_id": i,
                        "robot_speed": float(robot_speed),
                        "map_frame": "map",
                        "update_rate": 20.0,
                    }
                ],
            )
        )

    # ------------------------------------------------------------------
    # Coverage coordinator
    # ------------------------------------------------------------------
    nodes.append(
        Node(
            package="multi_robot_coverage",
            executable="coverage_coordinator",
            name="coverage_coordinator",
            output="screen",
            parameters=[
                {
                    "num_robots": num_robots,
                    "algorithm": algorithm,
                    "map_name": map_name,
                    "robot_speed": float(robot_speed),
                    "enable_failure_sim": enable_fail == "true",
                    "failure_time": float(failure_time),
                    "failure_robot_id": int(failure_robot),
                    "coverage_width_m": float(cov_width),
                    "robot_radius_m": float(robot_radius),
                }
            ],
        )
    )

    # ------------------------------------------------------------------
    # Visualizer
    # ------------------------------------------------------------------
    nodes.append(
        Node(
            package="multi_robot_coverage",
            executable="visualizer",
            name="visualizer",
            output="screen",
            parameters=[
                {
                    "num_robots": num_robots,
                    "map_frame": "map",
                    "update_rate": 5.0,
                }
            ],
        )
    )

    # ------------------------------------------------------------------
    # RViz2 (optional)
    # ------------------------------------------------------------------
    if use_rviz:
        rviz_args = ["-d", rviz_cfg] if os.path.isfile(rviz_cfg) else []
        nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=rviz_args,
                output="screen",
            )
        )

    return nodes
