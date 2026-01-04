#!/bin/bash
# Start all test system components

echo "Starting test system components..."

# Change to project root
cd "$(dirname "$0")/.."

# Source ROS2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Start state machine
echo "Starting state machine..."
ros2 run autonomy_state_management adaptive_state_machine &
STATE_MACHINE_PID=$!

# Wait for state machine to start
sleep 2

# Start sensor simulator
echo "Starting sensor simulator..."
ros2 run autonomy_simulation sensor_simulator &
SENSOR_PID=$!

# Wait for sensors
sleep 1

# Start mission status publisher
echo "Starting mission publishers..."
ros2 topic pub /mission/status std_msgs/String "data: 'IDLE'" -r 1 &
MISSION_PID=$!

# Start SLAM pose publisher
echo "Starting SLAM publishers..."
python3 src/bridges/slam_pose_publisher.py &
SLAM_POSE_PID=$!

# Start map data publisher
python3 src/bridges/map_data_publisher.py &
MAP_PID=$!

# Wait for all publishers to start
sleep 3

echo "All components started. Running comprehensive test..."
echo "Press Ctrl+C to stop all components"

# Run the test
python3 tests/system/comprehensive_system_test.py

# Cleanup on exit
echo "Cleaning up..."
kill $STATE_MACHINE_PID $SENSOR_PID $MISSION_PID $SLAM_POSE_PID $MAP_PID 2>/dev/null
echo "Done."
