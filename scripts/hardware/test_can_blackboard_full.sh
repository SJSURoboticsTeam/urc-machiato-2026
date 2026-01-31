#!/bin/bash
# Full end-to-end test: CAN data to blackboard with simulator
# 1. Start mock blackboard service (background)
# 2. Start hardware_interface (background)
# 3. Run dashboard (foreground); Ctrl+C stops all

set -e
WORKSPACE="/home/durian/urc-machiato-2026"
cd "$WORKSPACE"

# Source ROS2 and workspace
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi
source install/setup.bash

echo "=============================================="
echo "  CAN to Blackboard - Full E2E Test"
echo "=============================================="
echo ""
echo "1. Starting mock blackboard service (background)..."
python3 scripts/hardware/mock_blackboard_service.py > /tmp/mock_blackboard.log 2>&1 &
MOCK_PID=$!
echo "   PID: $MOCK_PID  Log: /tmp/mock_blackboard.log"
echo ""

echo "2. Waiting for blackboard services..."
sleep 2
if ros2 service list | grep -q "/blackboard/get_value"; then
    echo "   Blackboard services ready"
else
    echo "   Warning: /blackboard/get_value not found yet (may appear shortly)"
fi
echo ""

echo "3. Starting hardware_interface (background)..."
python3 -c "
import sys
sys.path.insert(0, 'src')
from autonomy.autonomy_core.autonomy_core.control.hardware_interface_node import main
main()
" > /tmp/hw_interface_e2e.log 2>&1 &
HW_PID=$!
echo "   PID: $HW_PID  Log: /tmp/hw_interface_e2e.log"
echo ""

echo "4. Waiting for hardware_interface to publish..."
sleep 3
echo ""

echo "5. Starting CAN testing dashboard (foreground)..."
echo "   Press Ctrl+C to stop all"
echo "=============================================="
echo ""

cleanup() {
    echo ""
    echo "Stopping..."
    kill $MOCK_PID 2>/dev/null || true
    kill $HW_PID 2>/dev/null || true
    echo "Mock blackboard log: /tmp/mock_blackboard.log"
    echo "Hardware interface log: /tmp/hw_interface_e2e.log"
    exit 0
}
trap cleanup SIGINT SIGTERM

python3 scripts/hardware/can_testing_dashboard.py

cleanup
