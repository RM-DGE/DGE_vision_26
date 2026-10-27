#!/usr/bin/env bash
# Build the ROS2 interface package used by DGE_vision_26.
#
# ROS2/rosidl cannot generate messages when the workspace path contains
# non-ASCII characters (this repository lives under "桌面/DGE_vision_26"),
# so the sources are copied to an ASCII path before running colcon.
#
# Usage:
#   ./ros2_ws/build.sh
#   source ~/dge_vision_ros2_ws/install/setup.bash
set -eo pipefail

SRC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/src"
BUILD_WS="${DGE_ROS2_WS:-$HOME/dge_vision_ros2_ws}"

mkdir -p "${BUILD_WS}/src"
cp -a "${SRC_DIR}/." "${BUILD_WS}/src/"

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

cd "${BUILD_WS}"
colcon build --packages-select sp_msgs dge_nav_bridge "$@"

echo
echo "Built in: ${BUILD_WS}"
echo "Now run: source ${BUILD_WS}/install/setup.bash"
