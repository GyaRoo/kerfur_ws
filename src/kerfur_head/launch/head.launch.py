"""Bring up the head-side ROS nodes: perception + expression bridge + touch bridge.

Runs on the head Pi (native ROS2 via RoboStack, NOT in a container anymore).
All three nodes are head-local:
  - perception_head : Hailo detector -> /head/detection
  - expression_bridge : /kerfur/expression -> Kerferface hub websocket
  - touch_bridge : native GPIO touch handler -> /kerfur/pad_nudge

Config: loads the shared kerfur_params.yaml from kerfur_bringup so the head and
the brain machine read tuning from the SAME file (single source of truth, §6).
perception_head needs its params; the bridges read theirs too if present.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Shared global params - same file the brain machine's launch uses.
    config = os.path.join(
        get_package_share_directory("kerfur_bringup"),
        "config",
        "kerfur_params.yaml",
    )

    return LaunchDescription([
        Node(
            package="kerfur_perception",
            executable="detection_bridge",
            name="detection_bridge",     # must match YAML section + node self-name
            output="screen",
            parameters=[config],
            emulate_tty=True,
        ),
        Node(
            package="kerfur_bridge",
            executable="expression_bridge",
            name="expression_bridge",
            output="screen",
            parameters=[config],
            emulate_tty=True,
        ),
        Node(
            package="kerfur_head",
            executable="touch_bridge",
            name="touch_bridge",
            output="screen",
            parameters=[config],
            emulate_tty=True,
        ),
        Node(
            package="kerfur_behavior",
            executable="attention_selector",
            name="attention_selector",
            output="screen",
            parameters=[config],
            emulate_tty=True,
        ),

    ])
