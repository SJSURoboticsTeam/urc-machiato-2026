#!/bin/bash
# Complete CAN Testing with ROS2 - Final Setup

cd /home/durian/urc-machiato-2026

echo "╔════════════════════════════════════════════════════════════════╗"
echo "║    CAN to Blackboard Testing - Within ROS2 Environment         ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""

# Source ROS2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "✓ ROS2 environment loaded"
echo ""
echo "═══════════════════════════════════════════════════════════════"
echo "Starting Hardware Interface (Dynamic Simulator)"
echo "═══════════════════════════════════════════════════════════════"
echo ""
echo "This process:"
echo "  ✓ Reads mock CAN data"
echo "  ✓ Publishes to /hardware/* topics (watched by dashboard)"
echo "  ✓ Writes directly to blackboard (Option 1)"
echo "  ✓ Generates dynamic data:"
echo "    - Battery discharges slowly"
echo "    - Velocity in sine wave pattern"
echo "    - Voltage varies with noise"
echo "    - Position integrates over time"
echo ""
echo "Logs will appear in Terminal 17 (this window)"
echo "Dashboard in Terminal 18 should show updates"
echo ""
echo "Press Ctrl+C to stop"
echo ""
echo "─────────────────────────────────────────────────────────────────"
echo ""

# Run hardware interface using ROS2 environment
python3 -c "
import sys
sys.path.insert(0, 'src')
from autonomy.autonomy_core.autonomy_core.control.hardware_interface_node import main
main()
" 2>&1 | tee /tmp/hw_interface_dynamic.log

echo ""
echo "Hardware interface stopped"
