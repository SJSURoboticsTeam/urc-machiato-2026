#!/usr/bin/env bash
# Run full testing plan in order: Step 1 -> Step 2 -> (Step 3 -> Step 4 if nodes available).
# If install/setup.bash or bt_orchestrator is not available, prints exact commands for Steps 3-4.
# See docs/development/BUILD_AND_TEST.md#testing-plan-order.
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$PROJECT_ROOT"

echo "=== Testing plan order: Step 1 -> Step 2 -> Step 3 -> Step 4 ==="

# Step 1: Unit tests
echo ""
echo "[Step 1] Unit tests..."
if ! ./scripts/run_blackboard_bt_state_machine_tests.sh; then
    echo "Step 1: FAILED"
    exit 1
fi
echo "Step 1: PASSED"

# Step 2: Contract integration tests
echo ""
echo "[Step 2] Contract integration tests..."
export PYTHONPATH="src:tests:${PYTHONPATH:-}"
if ! python3 -c "
import sys
ros_paths = [p for p in sys.path if 'ros' in p.lower() or 'ament' in p.lower() or 'jazzy' in p.lower()]
for p in ros_paths:
    try: sys.path.remove(p)
    except ValueError: pass
sys.path.insert(0, 'src')
sys.path.insert(0, 'tests')
import pytest
exit(pytest.main([
    'tests/integration/autonomy/test_bt_state_machine_integration.py',
    'tests/integration/autonomy/test_complete_bt_state_machine_flow.py',
    '-v', '--tb=line',
    '-p', 'no:launch_testing', '-p', 'no:launch_testing_ros_pytest_entrypoint',
]))
"; then
    echo "Step 2: FAILED"
    exit 1
fi
echo "Step 2: PASSED"

# Steps 3-4: run if workspace and bt_orchestrator are available; else print commands
echo ""
if [ ! -f "install/setup.bash" ]; then
    echo "[Step 3-4] SKIP - install/setup.bash not found. Build ROS first, then run:"
    echo "  ./scripts/build_ros_for_bt_tests.sh"
    echo "  source install/setup.bash"
    echo "  ros2 run autonomy_bt bt_orchestrator"
    echo "  In another terminal (workspace sourced): ros2 lifecycle set /bt_orchestrator configure && ros2 lifecycle set /bt_orchestrator activate"
    echo "  In test terminal (no venv): source install/setup.bash && export PYTHONPATH=\$(pwd)/src:\$(pwd):\${PYTHONPATH:-} && python3 -m pytest tests/integration/core/test_unified_blackboard.py -v --tb=short"
    echo "  Then (Step 4): start adaptive_state_machine; same terminal: python3 -m pytest tests/integration/autonomy/test_bt_state_machine_runtime.py -v --tb=short"
    echo "Full order: docs/development/BUILD_AND_TEST.md#testing-plan-order"
    exit 0
fi

source install/setup.bash
if ! python3 -c "import rclpy" 2>/dev/null; then
    echo "[Step 3-4] SKIP - rclpy not available from current python3 (often due to project venv)."
    echo "  Use a terminal where the project venv is NOT activated (run 'deactivate' first)."
    echo "  Then: source install/setup.bash && export PYTHONPATH=\$(pwd)/src:\$(pwd):\${PYTHONPATH:-} && python3 -m pytest tests/integration/core/test_unified_blackboard.py -v --tb=short"
    echo "  Then (Step 4): start adaptive_state_machine; same terminal: python3 -m pytest tests/integration/autonomy/test_bt_state_machine_runtime.py -v --tb=short"
    echo "Full order: docs/development/BUILD_AND_TEST.md#testing-plan-order"
    exit 0
fi
if ! ros2 node list 2>/dev/null | grep -q bt_orchestrator; then
    echo "[Step 3-4] SKIP - bt_orchestrator not running. Start it, then run:"
    echo "  ros2 run autonomy_bt bt_orchestrator"
    echo "  In another terminal (workspace sourced): ros2 lifecycle set /bt_orchestrator configure && ros2 lifecycle set /bt_orchestrator activate"
    echo "  In test terminal (no venv): source install/setup.bash && export PYTHONPATH=\$(pwd)/src:\$(pwd):\${PYTHONPATH:-} && python3 -m pytest tests/integration/core/test_unified_blackboard.py -v --tb=short"
    echo "  Then (Step 4): start adaptive_state_machine; same terminal: python3 -m pytest tests/integration/autonomy/test_bt_state_machine_runtime.py -v --tb=short"
    echo "Full order: docs/development/BUILD_AND_TEST.md#testing-plan-order"
    exit 0
fi
if ! ros2 service list 2>/dev/null | grep -q blackboard/get_value; then
    echo "[Step 3-4] SKIP - Blackboard services not found. Ensure bt_orchestrator is running and lifecycle is configured and activated:"
    echo "  ros2 lifecycle set /bt_orchestrator configure && ros2 lifecycle set /bt_orchestrator activate"
    echo "  If you use ROS_DOMAIN_ID, set it in this terminal too."
    echo "  In test terminal (no venv): source install/setup.bash && export PYTHONPATH=\$(pwd)/src:\$(pwd):\${PYTHONPATH:-} && python3 -m pytest tests/integration/core/test_unified_blackboard.py -v --tb=short"
    echo "  Then (Step 4): start adaptive_state_machine; same terminal: python3 -m pytest tests/integration/autonomy/test_bt_state_machine_runtime.py -v --tb=short"
    echo "Full order: docs/development/BUILD_AND_TEST.md#testing-plan-order"
    exit 0
fi

echo "[Step 3] Live blackboard..."
export PYTHONPATH="${PROJECT_ROOT}/src:${PROJECT_ROOT}:${PYTHONPATH:-}"
if ! python3 -m pytest tests/integration/core/test_unified_blackboard.py -v --tb=short; then
    echo "Step 3: FAILED"
    exit 1
fi
echo "Step 3: PASSED"

echo ""
echo "[Step 4] BT + state machine runtime (adaptive_state_machine must be running)..."
export PYTHONPATH="${PROJECT_ROOT}/src:${PROJECT_ROOT}:${PYTHONPATH:-}"
if ! python3 -m pytest tests/integration/autonomy/test_bt_state_machine_runtime.py -v --tb=short; then
    echo "Step 4: FAILED (ensure adaptive_state_machine is running)"
    exit 1
fi
echo "Step 4: PASSED"

echo ""
echo "=== All steps (1-4) completed ==="
