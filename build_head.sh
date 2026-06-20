#!/bin/bash
# Build only the packages the HEAD PI runs.
# Head owns: messages (contract), perception (Hailo), touch bridge, expression bridge.
#
# Run from the workspace root (~/kerfur_ws). Assumes the head's ROS2 environment
# (RoboStack conda env) is already activated and Hailo bindings are reachable.
#
# Usage:  ./build_head.sh
set -e

# kerfur_msgs MUST build first and on every machine - it is the shared contract.
# kerfur_perception requires the Hailo stack; it only builds here, not on dev.
# kerfur_bringup is built here too - it holds the shared kerfur_params.yaml that
# the head launch file reads (keeps config a single source of truth across machines).
PACKAGES=(
  kerfur_msgs
  kerfur_perception
  kerfur_head
  kerfur_bridge
  kerfur_bringup
  kerfur_behavior
)

echo "=== Building HEAD packages: ${PACKAGES[*]} ==="
colcon build --packages-select "${PACKAGES[@]}"

echo ""
echo "Done. Now source the overlay:"
echo "  source install/setup.bash"
echo "Then launch the head stack:"
echo "  ros2 launch kerfur_head head.launch.py"
