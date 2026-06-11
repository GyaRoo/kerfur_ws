"""Bring up the head-side ROS nodes: expression bridge + touch bridge.

Runs inside the head container. Both nodes connect outward - expression_bridge
to the hub websocket, touch_bridge listens for the native GPIO handler's socket.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="kerfur_bridge",
            executable="expression_bridge",
            name="expression_bridge",
            output="screen",
            emulate_tty=True,
        ),
        Node(
            package="kerfur_head",
            executable="touch_bridge",
            name="touch_bridge",
            output="screen",
            emulate_tty=True,
        ),
    ])
