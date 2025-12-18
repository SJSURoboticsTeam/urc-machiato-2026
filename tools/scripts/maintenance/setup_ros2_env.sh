#!/bin/bash
# ROS2 Environment Setup for URC Machiato 2026

set -e

echo " Setting up ROS2 environment..."

# Source ROS2
source /opt/ros/humble/setup.bash

# Set environment variables
export ROS_DOMAIN_ID=42
export ROVER_ENV="production"
export PYTHONPATH="$PWD:$PWD/autonomy/code:$PYTHONPATH"

# Add ROS2 workspace to path if it exists
if [ -d "$PWD/autonomy/ros2_ws/install" ]; then
    source $PWD/autonomy/ros2_ws/install/setup.bash 2>/dev/null || true
fi

# Verify setup
echo "ROS2 Version: $(ros2 --version 2>/dev/null | head -1 || echo 'Not found')"
echo "Python ROS2 packages available: $(python3 -c "import rclpy; print('YES')" 2>/dev/null || echo 'NO')"

echo " ROS2 environment setup complete"
