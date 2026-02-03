#!/bin/bash
# Rebuild and restart hardware_interface with dynamic simulator data

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$PROJECT_ROOT"

echo "Rebuilding ROS2 packages..."
if [ -z "${ROS_DISTRO:-}" ]; then
    source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash
fi
colcon build --base-paths \
    src/autonomy/interfaces/autonomy_interfaces \
    src/autonomy/autonomy_core \
    src/autonomy/bt \
    src/simulation/gazebo_simulation \
    src/vision_processing \
    --symlink-install 2>&1 | tail -5

if [ $? -eq 0 ]; then
    echo "✓ Build successful"
    echo ""
    echo "Restart hardware_interface in Terminal 17 to see dynamic data:"
    echo "  Press Ctrl+C to stop current process"
    echo "  Then run: ./scripts/hardware/start_hardware_interface.sh"
    echo ""
    echo "Or restart the full test:"
    echo "  ./scripts/hardware/run_can_tests.sh"
else
    echo "✗ Build failed"
    exit 1
fi
