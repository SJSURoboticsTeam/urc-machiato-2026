#!/usr/bin/env bash
# Unifying script: run blackboard/BT/state-machine test plan in one go.
# Usage:
#   ./scripts/run_hardware_bt_tests.sh              # Step 1 + 2; Step 3+4 if nodes already running
#   ./scripts/run_hardware_bt_tests.sh --all       # Step 1 + 2 + 3 (start bt_orchestrator, run live blackboard, stop)
#   ./scripts/run_hardware_bt_tests.sh --all --with-step4  # Step 1-4 (also start adaptive_state_machine, run Step 4, stop)
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$PROJECT_ROOT"

unset VIRTUAL_ENV

RUN_ALL_LIVE=false
WITH_STEP4=false
for arg in "$@"; do
    case "$arg" in
        --all) RUN_ALL_LIVE=true ;;
        --with-step4) WITH_STEP4=true ;;
        -h|--help)
            echo "Usage: $0 [--all] [--with-step4]"
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
    export PYTHONPATH="services:shared:tests:${PYTHONPATH:-}"
    python3 -c "
import sys
ros_paths = [p for p in sys.path if 'ros' in p.lower() or 'ament' in p.lower() or 'jazzy' in p.lower()]
for p in ros_paths:
    try: sys.path.remove(p)
    except ValueError: pass
import pytest
exit(pytest.main([
    'tests/unit/core/test_hybrid_controller_logic.py',
    'tests/unit/core/test_blackboard_keys_schema.py',
    'tests/unit/core/test_blackboard_persistence.py',
    'tests/unit/core/test_adaptive_state_machine_hybrid.py',
    '-v', '--tb=short',
    '-p', 'no:launch_testing',
    '-p', 'no:launch_testing_ros_pytest_entrypoint',
]))
"
}

run_step2() {
    echo ""
    echo "[Step 2] Contract integration tests..."
    export PYTHONPATH="services:shared:tests:${PYTHONPATH:-}"
    python3 -c "
import sys
ros_paths = [p for p in sys.path if 'ros' in p.lower() or 'ament' in p.lower() or 'jazzy' in p.lower()]
for p in ros_paths:
    try: sys.path.remove(p)
    except ValueError: pass
import pytest
exit(pytest.main([
    'tests/integration/autonomy/test_bt_state_machine_integration.py',
    'tests/integration/autonomy/test_complete_bt_state_machine_flow.py',
    '-v', '--tb=line',
    '-p', 'no:launch_testing', '-p', 'no:launch_testing_ros_pytest_entrypoint',
]))
"
}

run_step3() {
    echo ""
    echo "[Step 3] Live blackboard..."
    export PYTHONPATH="${PROJECT_ROOT}/services:${PROJECT_ROOT}/shared:${PROJECT_ROOT}:${PYTHONPATH:-}"
    python3 -m pytest tests/integration/core/test_unified_blackboard.py -v --tb=short
}

run_step4() {
    echo ""
    echo "[Step 4] BT + state machine runtime..."
    export PYTHONPATH="${PROJECT_ROOT}/services:${PROJECT_ROOT}/shared:${PROJECT_ROOT}:${PYTHONPATH:-}"
    python3 -m pytest tests/integration/autonomy/test_bt_state_machine_runtime.py -v --tb=short
}

wait_for_node() {
    local name="$1"
    local max="${2:-20}"
    local i=0
    while [ "$i" -lt "$max" ]; do
        if ros2 node list 2>/dev/null | grep -q "$name"; then return 0; fi
        sleep 1
        i=$((i + 1))
    done
    return 1
}

echo "=== Blackboard / BT / State machine test plan ==="
if ! run_step1; then exit 1; fi
if ! run_step2; then exit 1; fi

if [ ! -f "install/setup.bash" ]; then
    echo "[Step 3-4] SKIP - install/setup.bash not found."
    exit 0
fi

source install/setup.bash
if ! python3 -c "import rclpy" 2>/dev/null; then
    echo "[Step 3-4] SKIP - rclpy not available."
    exit 0
fi

if [ "$RUN_ALL_LIVE" = true ]; then
    export ROS_DOMAIN_ID="${TEST_ROS_DOMAIN_ID:-42}"
    pkill -9 -f "bt_orchestrator" 2>/dev/null || true
    sleep 1
    ros2 run autonomy_bt bt_orchestrator --ros-args -p run_bt_tick:=false &
    BT_PID=$!
    if ! wait_for_node bt_orchestrator 25; then exit 1; fi
    sleep 2
    ros2 lifecycle set /bt_orchestrator configure 2>/dev/null || true
    sleep 1
    ros2 lifecycle set /bt_orchestrator activate 2>/dev/null || true
    sleep 3
    if ! run_step3; then exit 1; fi

    if [ "$WITH_STEP4" = true ]; then
        export PYTHONPATH="${PROJECT_ROOT}/services:${PROJECT_ROOT}/shared:${PYTHONPATH:-}"
        (cd "$PROJECT_ROOT" && python3 -m shared.core.adaptive_state_machine) &
        ASM_PID=$!
        sleep 2
        if ! run_step4; then exit 1; fi
    fi
    exit 0
fi

if ! ros2 node list 2>/dev/null | grep -q bt_orchestrator; then
    echo "[Step 3-4] SKIP - bt_orchestrator not running."
    exit 0
fi
if ! run_step3; then exit 1; fi
if ros2 node list 2>/dev/null | grep -q adaptive_state_machine; then
    if ! run_step4; then exit 1; fi
fi
