#!/bin/bash
# Start hardware_interface for testing CAN to blackboard

cd /home/durian/urc-machiato-2026

# Source ROS2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "Starting hardware_interface_node (mock CAN mode)"
echo "This will publish to /hardware/* topics and write to blackboard"
echo ""
echo "Press Ctrl+C to stop"
echo ""

# Run hardware interface
ros2 run autonomy_core hardware_interface
