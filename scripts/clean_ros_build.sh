#!/usr/bin/env bash
# Clean colcon artifacts and rebuild all ROS2 packages.
# Uses canonical --base-paths (workspace root has COLCON_IGNORE). See docs/development/BUILD_AND_TEST.md.
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$PROJECT_ROOT"

if [ -z "${ROS_DISTRO:-}" ]; then
    echo "Sourcing ROS2..."
    if [ -f /opt/ros/jazzy/setup.bash ]; then
        source /opt/ros/jazzy/setup.bash
    elif [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
    else
        echo "Error: ROS2 not found. Install and source e.g. /opt/ros/jazzy/setup.bash"
        exit 1
    fi
fi

echo "Cleaning build, install, log..."
rm -rf build install log

echo "Building all ROS2 packages (autonomy_interfaces, autonomy_core, autonomy_bt, gazebo_simulation, vision_processing)..."
colcon build \
    --base-paths \
        src/autonomy/interfaces/autonomy_interfaces \
        src/autonomy/autonomy_core \
        src/autonomy/bt \
        src/simulation/gazebo_simulation \
        src/vision_processing \
    --symlink-install

echo "Done. Source the workspace: source install/setup.bash"
