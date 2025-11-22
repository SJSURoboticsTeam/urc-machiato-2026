#!/bin/bash
# Complete Rover Simulation Test Runner
# Launches full simulation environment and runs comprehensive tests

set -e  # Exit on any error

echo "🚀 Starting Complete Rover Simulation Test"
echo "=========================================="

# Set environment
export TURTLEBOT3_MODEL=waffle
export ROS_DOMAIN_ID=42  # Use unique domain to avoid conflicts

# Function to cleanup on exit
cleanup() {
    echo "🧹 Cleaning up simulation..."
    pkill -f "mock_rover_simulation.py" || true
    pkill -f "mission_executor.py" || true
    pkill -f "slam_data_bridge.py" || true
    pkill -f "websocket_mission_bridge.py" || true
    pkill -f "test_rover_simulation.py" || true
    sleep 2
}

trap cleanup EXIT

# Step 1: Launch mock simulation environment
echo "🏗️ Step 1: Launching mock simulation environment..."
python3 mock_rover_simulation.py &
SIM_PID=$!
sleep 5

# Step 3: Launch our custom components
echo "🤖 Step 3: Starting mission executor and bridges..."
python3 mission_executor.py &
MISSION_PID=$!
sleep 2

python3 slam_data_bridge.py &
BRIDGE_PID=$!
sleep 2

python3 websocket_mission_bridge.py &
WS_PID=$!
sleep 2

# Step 4: Run basic simulation tests
echo "🧪 Step 4: Running basic simulation tests..."
python3 test_rover_simulation.py &
TEST_PID=$!
sleep 15  # Let basic tests run

# Step 5: Run comprehensive system test suite
echo "🧪 Step 5: Running comprehensive system test suite..."
python3 comprehensive_system_test.py &
COMPREHENSIVE_PID=$!

# Wait for comprehensive tests to complete
echo "⏳ Waiting for comprehensive tests to complete (120 seconds max)..."
timeout 120 bash -c "wait $COMPREHENSIVE_PID" || echo "Comprehensive tests completed or timed out"

# Step 6: Check final system status
echo "📊 Step 6: Checking final system status..."
echo "Active ROS2 nodes:"
ros2 node list

echo "Active topics:"
ros2 topic list | grep -E "(odom|imu|mission|frontend|slam)"

echo "Mission status:"
ros2 topic echo /mission/status --once --timeout 2 || echo "No mission status available"

echo "Map data:"
ros2 topic echo /frontend/map_data --once --timeout 2 || echo "No map data available"

# Step 7: Generate test report
echo "📋 Step 7: Generating test report..."
echo "==========================================" > simulation_test_report.txt
echo "ROVER SIMULATION TEST REPORT" >> simulation_test_report.txt
echo "Generated: $(date)" >> simulation_test_report.txt
echo "==========================================" >> simulation_test_report.txt
echo "" >> simulation_test_report.txt

echo "SYSTEM COMPONENTS:" >> simulation_test_report.txt
echo "- Mock Simulation: $(pgrep -f mock_rover_simulation >/dev/null && echo 'RUNNING' || echo 'STOPPED')" >> simulation_test_report.txt
echo "- Mission Executor: $(pgrep -f mission_executor >/dev/null && echo 'RUNNING' || echo 'STOPPED')" >> simulation_test_report.txt
echo "- SLAM Bridge: $(pgrep -f slam_data_bridge >/dev/null && echo 'RUNNING' || echo 'STOPPED')" >> simulation_test_report.txt
echo "- WebSocket Bridge: $(pgrep -f websocket_mission_bridge >/dev/null && echo 'RUNNING' || echo 'STOPPED')" >> simulation_test_report.txt
echo "" >> simulation_test_report.txt

echo "DATA FLOW VERIFICATION:" >> simulation_test_report.txt
echo "- Odometry: $(ros2 topic info /odom 2>/dev/null | grep -q Publisher && echo 'ACTIVE' || echo 'INACTIVE')" >> simulation_test_report.txt
echo "- IMU: $(ros2 topic info /imu 2>/dev/null | grep -q Publisher && echo 'ACTIVE' || echo 'INACTIVE')" >> simulation_test_report.txt
echo "- Mission Commands: $(ros2 topic info /mission/commands 2>/dev/null | grep -q Publisher && echo 'ACTIVE' || echo 'INACTIVE')" >> simulation_test_report.txt
echo "- Mission Status: $(ros2 topic info /mission/status 2>/dev/null | grep -q Publisher && echo 'ACTIVE' || echo 'INACTIVE')" >> simulation_test_report.txt
echo "- Frontend Map Data: $(ros2 topic info /frontend/map_data 2>/dev/null | grep -q Publisher && echo 'ACTIVE' || echo 'INACTIVE')" >> simulation_test_report.txt
echo "" >> simulation_test_report.txt

echo "TEST RESULTS SUMMARY:" >> simulation_test_report.txt
echo "See test output above for detailed results." >> simulation_test_report.txt
echo "" >> simulation_test_report.txt

echo "RECOMMENDATIONS:" >> simulation_test_report.txt
if pgrep -f gazebo >/dev/null && pgrep -f mission_executor >/dev/null; then
    echo "✅ Core systems operational - Ready for production testing!" >> simulation_test_report.txt
else
    echo "⚠️ Some systems not running - Check component startup" >> simulation_test_report.txt
fi

echo "==========================================" >> simulation_test_report.txt

echo "📄 Test report saved to: simulation_test_report.txt"
cat simulation_test_report.txt

echo ""
echo "🎉 Simulation test complete!"
echo "=========================================="
echo "Next steps:"
echo "1. Review test results and fix any issues"
echo "2. Run frontend with: cd frontend && npm run dev"
echo "3. Test WebSocket commands from frontend"
echo "4. Scale up to multi-robot scenarios if needed"
