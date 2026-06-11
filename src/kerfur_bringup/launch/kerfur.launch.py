"""Bring up the full Kerfur ROS2 stack with shared global config.

Note: does NOT launch the FastAPI hub or the browser - those are native
(non-ROS) processes started separately. This launches only the ROS nodes.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Locate the global params file inside this package's installed share dir
    config = os.path.join(
        get_package_share_directory("kerfur_bringup"),
        "config",
        "kerfur_params.yaml",
    )

    return LaunchDescription([
        Node(
            package="kerfur_behavior",
            executable="emotion_engine",
            name="emotion_engine",
            output="screen",
            parameters=[config],
            emulate_tty=True,
        ),
        Node(
            package="kerfur_behavior",
            executable="pad_to_face",
            name="pad_to_expression",     # must match the YAML section + node's self-name
            output="screen",
            parameters=[config],
            emulate_tty=True,
        ),
        Node(
            package="kerfur_cognition",
            executable="subconscious",
            name="subconscious",
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
    ])
