#!/usr/bin/env bash
# Run blackboard integration tests against the mock blackboard service (no bt_orchestrator).
# Simpler than Step 3 with real bt_orchestrator: no C++ build, no lifecycle configure/activate.
# Uses ROS_DOMAIN_ID=42 so the mock is isolated from other nodes (bt_orchestrator, CAN bridge).
# Requires: install/setup.bash (colcon build of autonomy_interfaces), no venv.
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$PROJECT_ROOT"

# Isolate mock and tests from other ROS nodes (use high domain to avoid other processes)
export ROS_DOMAIN_ID=99

if [ ! -f "install/setup.bash" ]; then
    echo "install/setup.bash not found. Build ROS packages first:"
    echo "  source /opt/ros/jazzy/setup.bash"
    echo "  ./scripts/build_ros_for_bt_tests.sh"
    exit 1
fi

source install/setup.bash
export PYTHONPATH="${PROJECT_ROOT}/src:${PROJECT_ROOT}:${PYTHONPATH:-}"

if ! python3 -c "import rclpy" 2>/dev/null; then
    echo "rclpy not available. Use a terminal without the project venv (run 'deactivate' first)."
    exit 1
fi

echo "Starting mock blackboard service in background (ROS_DOMAIN_ID=$ROS_DOMAIN_ID for isolation)..."
python3 scripts/hardware/mock_blackboard_service.py &
MOCK_PID=$!
trap "kill $MOCK_PID 2>/dev/null || true" EXIT

echo "Waiting for /blackboard/get_value (up to 10s)..."
for i in $(seq 1 50); do
    if ros2 service list 2>/dev/null | grep -q blackboard/get_value; then
        break
    fi
    sleep 0.2
done
if ! ros2 service list 2>/dev/null | grep -q blackboard/get_value; then
    echo "Mock blackboard service did not appear. Check install and ROS_DOMAIN_ID."
    kill $MOCK_PID 2>/dev/null || true
    exit 1
fi

echo "Running blackboard integration tests..."
python3 -m pytest tests/integration/core/test_unified_blackboard.py -v --tb=short
RESULT=$?
kill $MOCK_PID 2>/dev/null || true
trap - EXIT
exit $RESULT
