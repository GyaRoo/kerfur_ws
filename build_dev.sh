#!/bin/bash
# Build only the packages the DEV / MAIN (brain) machine runs.
# Dev owns: messages (contract), behavior, cognition, bringup (config + full launch).
#
# Deliberately does NOT build kerfur_perception - that needs the Hailo stack,
# which only exists on the head Pi. Selecting it here would fail the build.
#
# Run from the workspace root (~/kerfur_ws) with the dev ROS2 environment sourced
# (apt-based ROS2 on Kubuntu).
#
# Usage:  ./build_dev.sh
set -e

# kerfur_msgs MUST build first and on every machine - it is the shared contract.
PACKAGES=(
  kerfur_msgs
  kerfur_cognition
  kerfur_bringup
)

echo "=== Building DEV packages: ${PACKAGES[*]} ==="
colcon build --packages-select "${PACKAGES[@]}"

echo ""
echo "Done. Now source the overlay:"
echo "  source install/setup.bash"
echo "Then launch the brain stack:"
echo "  ros2 launch kerfur_bringup kerfur.launch.py"
