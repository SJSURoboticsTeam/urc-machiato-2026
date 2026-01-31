#!/bin/bash
# Start Hardware Interface with ROS2

cd /home/durian/urc-machiato-2026

# Setup ROS2 environment
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "Starting hardware_interface with dynamic simulator data..."
echo "Logs: /tmp/hw_interface_dynamic.log"
echo ""
echo "Watch the dashboard - velocity should change!"
echo "Press Ctrl+C to stop"
echo ""

# Run using python3 directly through ROS2 workspace
python3 -c "
import sys
sys.path.insert(0, 'src')
from autonomy.autonomy_core.autonomy_core.control.hardware_interface_node import main
main()
" 2>&1 | tee /tmp/hw_interface_dynamic.log
