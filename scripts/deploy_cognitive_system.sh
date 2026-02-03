#!/usr/bin/env bash
# Deploy/start the cognitive stack (SLAM + navigation + missions + BT) for URC 2026.
#
# Prerequisites:
#   - ROS2 Humble (or Jazzy) and workspace built: colcon build --symlink-install
#   - Source: source /opt/ros/humble/setup.bash && source install/setup.bash
#   - Optional: RealSense D435 for slam_only; CAN interface for full competition
#   - Optional: vision_processing, terrain_intelligence, hardware_interface,
#     competition_safety, competition_bridge, mission_control packages for competition
#
# Usage: ./scripts/deploy_cognitive_system.sh [competition|slam_only]
#   competition - full competition_system.launch.py (SLAM, nav, missions, safety, bridge)
#   slam_only   - only slam.launch.py (RealSense/depth/slam orchestrator)

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
MODE="${1:-slam_only}"

# Optional: isolate from other ROS2 networks
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

cd "$REPO_ROOT"

# Ensure workspace is built (colcon build for ROS2 packages)
if [ ! -d "install" ]; then
    echo "No install/ found. Run: colcon build --symlink-install"
    exit 1
fi

if [ -f "install/setup.bash" ]; then
    source install/setup.bash
fi

case "$MODE" in
    competition)
        echo "Launching competition system (SLAM + nav + missions + safety + bridge)..."
        if [ -f "src/autonomy/launch/competition_system.launch.py" ]; then
            ros2 launch src/autonomy/launch/competition_system.launch.py
        else
            echo "competition_system.launch.py not found. Run from repo root or install launch package."
            exit 1
        fi
        ;;
    slam_only)
        echo "Launching SLAM pipeline only (RealSense, depth_processor, slam_orchestrator)..."
        ros2 launch autonomy_core slam.launch.py
        ;;
    *)
        echo "Usage: $0 [competition|slam_only]"
        exit 1
        ;;
esac
