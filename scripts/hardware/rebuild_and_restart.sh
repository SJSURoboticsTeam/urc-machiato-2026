#!/bin/bash
# Rebuild and restart hardware_interface with dynamic simulator data

cd /home/durian/urc-machiato-2026

echo "Rebuilding hardware_interface with dynamic simulator..."
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src/autonomy/autonomy_core --symlink-install 2>&1 | tail -5

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
