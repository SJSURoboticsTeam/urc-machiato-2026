#!/usr/bin/env bash
# Unifying script: run blackboard/BT/state-machine test plan in one go.
# Usage:
#   ./scripts/test_all_blackboard_bt.sh              # Step 1 + 2; Step 3+4 if nodes already running
#   ./scripts/test_all_blackboard_bt.sh --all       # Step 1 + 2 + 3 (start bt_orchestrator, run live blackboard, stop)
#   ./scripts/test_all_blackboard_bt.sh --all --with-step4  # Step 1-4 (also start adaptive_state_machine, run Step 4, stop)
# See docs/development/BUILD_AND_TEST.md#testing-plan-order.
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$PROJECT_ROOT"

# Clean environment: avoid project venv so ROS2 python (rclpy) is used when we source install/setup.bash
unset VIRTUAL_ENV

RUN_ALL_LIVE=false
WITH_STEP4=false
for arg in "$@"; do
    case "$arg" in
        --all) RUN_ALL_LIVE=true ;;
        --with-step4) WITH_STEP4=true ;;
        -h|--help)
            echo "Usage: $0 [--all] [--with-step4]"
            echo "  (no args)  Step 1 + 2; Step 3+4 only if bt_orchestrator (and adaptive_state_machine) already running."
            echo "  --all      Step 1 + 2 + 3: start bt_orchestrator, run live blackboard tests, stop orchestrator."
            echo "  --with-step4  With --all: also start adaptive_state_machine and run Step 4, then stop both nodes."
            echo "See docs/development/BUILD_AND_TEST.md#testing-plan-order"
            exit 0
            ;;
    esac
done

BT_PID=""
ASM_PID=""
cleanup() {
    if [ -n "$BT_PID" ] && kill -0 "$BT_PID" 2>/dev/null; then
        kill "$BT_PID" 2>/dev/null || true
        wait "$BT_PID" 2>/dev/null || true
    fi
    if [ -n "$ASM_PID" ] && kill -0 "$ASM_PID" 2>/dev/null; then
        kill "$ASM_PID" 2>/dev/null || true
        wait "$ASM_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT

run_step1() {
    echo ""
    echo "[Step 1] Unit tests..."
    if ! ./scripts/run_blackboard_bt_state_machine_tests.sh; then
        echo "Step 1: FAILED"
        return 1
    fi
    echo "Step 1: PASSED"
    return 0
}

run_step2() {
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
        return 1
    fi
    echo "Step 2: PASSED"
    return 0
}

run_step3() {
    echo ""
    echo "[Step 3] Live blackboard..."
    export PYTHONPATH="${PROJECT_ROOT}/src:${PROJECT_ROOT}:${PYTHONPATH:-}"
    if ! python3 -m pytest tests/integration/core/test_unified_blackboard.py -v --tb=short; then
        echo "Step 3: FAILED"
        return 1
    fi
    echo "Step 3: PASSED"
    return 0
}

run_step4() {
    echo ""
    echo "[Step 4] BT + state machine runtime..."
    export PYTHONPATH="${PROJECT_ROOT}/src:${PROJECT_ROOT}:${PYTHONPATH:-}"
    if ! python3 -m pytest tests/integration/autonomy/test_bt_state_machine_runtime.py -v --tb=short; then
        echo "Step 4: FAILED (ensure adaptive_state_machine is running)"
        return 1
    fi
    echo "Step 4: PASSED"
    return 0
}

wait_for_node() {
    local name="$1"
    local max="${2:-20}"
    local i=0
    while [ "$i" -lt "$max" ]; do
        if ros2 node list 2>/dev/null | grep -q "$name"; then
            return 0
        fi
        sleep 1
        i=$((i + 1))
    done
    return 1
}

wait_for_blackboard_services() {
    local max="${1:-10}"
    local i=0
    while [ "$i" -lt "$max" ]; do
        if ros2 service list 2>/dev/null | grep -q blackboard/get_value; then
            return 0
        fi
        sleep 1
        i=$((i + 1))
    done
    return 1
}

# --- Step 1 + 2 always ---
echo "=== Blackboard / BT / State machine test plan ==="
if ! run_step1; then exit 1; fi
if ! run_step2; then exit 1; fi

# --- Step 3 (and optionally 4): need ROS workspace and rclpy ---
if [ ! -f "install/setup.bash" ]; then
    echo ""
    echo "[Step 3-4] SKIP - install/setup.bash not found. Build ROS: ./scripts/build_ros_for_bt_tests.sh"
    echo "  Then run this script again, or start nodes manually (see docs/development/BUILD_AND_TEST.md)."
    echo "=== Steps 1-2 completed ==="
    exit 0
fi

source install/setup.bash
if ! python3 -c "import rclpy" 2>/dev/null; then
    echo ""
    echo "[Step 3-4] SKIP - rclpy not available (use a terminal without the project venv: deactivate)."
    echo "  Then: source install/setup.bash && export PYTHONPATH=\$(pwd)/src:\$(pwd):\${PYTHONPATH:-} && python3 -m pytest tests/integration/core/test_unified_blackboard.py -v --tb=short"
    echo "=== Steps 1-2 completed ==="
    exit 0
fi

if [ "$RUN_ALL_LIVE" = true ]; then
    # Isolate test on a unique DDS domain so client and server are not affected by other ROS processes
    TEST_ROS_DOMAIN_ID="${TEST_ROS_DOMAIN_ID:-42}"
    export ROS_DOMAIN_ID="$TEST_ROS_DOMAIN_ID"
    echo ""
    echo "[Step 3] Using ROS_DOMAIN_ID=$ROS_DOMAIN_ID for isolated blackboard test"
    # Avoid duplicate bt_orchestrator (two instances cause get_value timeouts; client may send requests to wrong/dying server)
    echo "[Step 3] Stopping any existing bt_orchestrator and waiting for full exit..."
    pkill -f "bt_orchestrator" 2>/dev/null || true
    wait_count=0
    while pgrep -f "bt_orchestrator" >/dev/null 2>&1 && [ "$wait_count" -lt 15 ]; do
        sleep 1
        wait_count=$((wait_count + 1))
    done
    if pgrep -f "bt_orchestrator" >/dev/null 2>&1; then
        echo "[Step 3] WARN - bt_orchestrator still running after 15s; forcing kill..."
        pkill -9 -f "bt_orchestrator" 2>/dev/null || true
        sleep 2
    fi
    sleep 1
    echo "[Step 3] Starting bt_orchestrator for live blackboard tests (blackboard-only mode, no BT tick)..."
    ros2 run autonomy_bt bt_orchestrator --ros-args -p run_bt_tick:=false &
    BT_PID=$!
    if ! wait_for_node bt_orchestrator 25; then
        echo "Step 3: FAILED - bt_orchestrator did not appear in ros2 node list"
        exit 1
    fi
    sleep 2
    ros2 lifecycle set /bt_orchestrator configure 2>/dev/null || true
    sleep 1
    ros2 lifecycle set /bt_orchestrator activate 2>/dev/null || true
    sleep 3
    if ! wait_for_blackboard_services 15; then
        echo "Step 3: FAILED - Blackboard services not available (lifecycle configure/activate may be needed)"
        exit 1
    fi
    sleep 2
    if ! run_step3; then
        echo "If Step 3 timed out (get_value timeouts), rebuild autonomy_bt: ./scripts/build_ros_for_bt_tests.sh"
        exit 1
    fi

    if [ "$WITH_STEP4" = true ]; then
        echo ""
        echo "[Step 4] Starting adaptive_state_machine for runtime tests..."
        export PYTHONPATH="${PROJECT_ROOT}/src:${PROJECT_ROOT}:${PYTHONPATH:-}"
        (cd "$PROJECT_ROOT" && python3 -m src.core.adaptive_state_machine) &
        ASM_PID=$!
        if ! wait_for_node adaptive_state_machine 15; then
            echo "Step 4: WARN - adaptive_state_machine did not appear; running Step 4 anyway..."
        fi
        # Give state machine a moment to advertise services
        sleep 2
        if ! run_step4; then exit 1; fi
    else
        echo ""
        echo "[Step 4] SKIP - use --all --with-step4 to run Step 4 (requires adaptive_state_machine)."
    fi

    echo ""
    echo "=== All requested steps completed ==="
    exit 0
fi

# --- No --all: run Step 3+4 only if nodes are already running ---
if ! ros2 node list 2>/dev/null | grep -q bt_orchestrator; then
    echo ""
    echo "[Step 3-4] SKIP - bt_orchestrator not running."
    echo "  Start it in another terminal: ros2 run autonomy_bt bt_orchestrator"
    echo "  Then: ros2 lifecycle set /bt_orchestrator configure && ros2 lifecycle set /bt_orchestrator activate"
    echo "  Or run this script with --all to start bt_orchestrator automatically."
    echo "=== Steps 1-2 completed ==="
    exit 0
fi

if ! ros2 service list 2>/dev/null | grep -q blackboard/get_value; then
    echo ""
    echo "[Step 3-4] SKIP - Blackboard services not found. Run: ros2 lifecycle set /bt_orchestrator configure && ros2 lifecycle set /bt_orchestrator activate"
    echo "=== Steps 1-2 completed ==="
    exit 0
fi

if ! run_step3; then exit 1; fi

if ros2 node list 2>/dev/null | grep -q adaptive_state_machine; then
    if ! run_step4; then exit 1; fi
    echo ""
    echo "=== All steps (1-4) completed ==="
else
    echo ""
    echo "[Step 4] SKIP - adaptive_state_machine not running. Start it to run Step 4, or use --all --with-step4."
    echo "=== Steps 1-3 completed ==="
fi
