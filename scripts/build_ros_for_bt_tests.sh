#!/usr/bin/env bash
# Build ROS packages needed for BT/blackboard/state machine live tests (Steps 3-4).
# Requires: ROS2 sourced (e.g. source /opt/ros/jazzy/setup.bash) and ros-jazzy-behaviortree-cpp installed.
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$PROJECT_ROOT"

# 1. Check ROS is sourced
if [ -z "${ROS_DISTRO:-}" ]; then
    echo "Error: ROS2 is not sourced. Run: source /opt/ros/jazzy/setup.bash (or humble)"
    exit 1
fi

# 2. Check behaviortree_cpp is available (required for autonomy_bt)
if ! ros2 pkg list 2>/dev/null | grep -q behaviortree_cpp; then
    echo "Error: behaviortree_cpp is not found. autonomy_bt requires it."
    echo "Install with: sudo apt install ros-${ROS_DISTRO}-behaviortree-cpp"
    echo "If the package is not available for your distro, see: https://github.com/BehaviorTree/BehaviorTree.CPP"
    exit 1
fi

# 3. Colcon build with canonical package paths (autonomy_bt requires behaviortree_cpp)
echo "Building ROS2 packages (autonomy_interfaces, autonomy_core, autonomy_bt, gazebo_simulation, vision_processing)..."
colcon build \
    --base-paths \
        src/autonomy/interfaces/autonomy_interfaces \
        src/autonomy/autonomy_core \
        src/autonomy/bt \
        src/simulation/gazebo_simulation \
        src/vision_processing \
    --symlink-install

# 4. Post-build instructions
echo ""
echo "Build complete. Next steps for live blackboard and BT+state machine tests:"
echo "  1. source install/setup.bash"
echo "  2. Start bt_orchestrator: ros2 run autonomy_bt bt_orchestrator"
echo "     (If it is a lifecycle node: ros2 lifecycle set /bt_orchestrator configure; ros2 lifecycle set /bt_orchestrator activate)"
echo "  3. In another terminal (with source install/setup.bash and PYTHONPATH=src):"
echo "     python3 -m pytest tests/integration/test_unified_blackboard.py -v --tb=short"
echo "  4. For BT+state machine runtime: also start adaptive_state_machine, then run:"
echo "     python3 -m pytest tests/integration/test_bt_state_machine_runtime.py -v --tb=short"
