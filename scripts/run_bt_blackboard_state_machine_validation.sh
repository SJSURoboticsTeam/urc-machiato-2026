#!/usr/bin/env bash
# Validation ladder for BT, blackboard, state machine (before hardware).
# Runs steps that pass without a full ROS/colcon build; documents steps that need live nodes.
set -e
cd "$(dirname "$0")/.."
REPORTS_DIR="tests/reports"
mkdir -p "$REPORTS_DIR"
RESULTS="${REPORTS_DIR}/validation_bt_blackboard_state_machine.txt"
echo "BT / Blackboard / State Machine validation - $(date -Iseconds)" | tee "$RESULTS"
echo "================================================================" | tee -a "$RESULTS"

# Step 1: Unit tests (no ROS)
echo "" | tee -a "$RESULTS"
echo "[Step 1] Unit tests (blackboard keys, persistence, hybrid logic, state machine contract)" | tee -a "$RESULTS"
if ./scripts/run_blackboard_bt_state_machine_tests.sh >> "$RESULTS" 2>&1; then
    echo "Step 1: PASSED" | tee -a "$RESULTS"
else
    echo "Step 1: FAILED" | tee -a "$RESULTS"
    exit 1
fi

# Step 2: Contract integration tests (no live ROS)
echo "" | tee -a "$RESULTS"
echo "[Step 2] Contract integration tests (BT/state machine/blackboard source checks)" | tee -a "$RESULTS"
export PYTHONPATH="src:tests:${PYTHONPATH:-}"
python3 -c "
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
" >> "$RESULTS" 2>&1 && { echo "Step 2: PASSED" | tee -a "$RESULTS"; } || { echo "Step 2: FAILED" | tee -a "$RESULTS"; exit 1; }

# Steps 3-4: run manually when workspace is built and nodes are running (same order as docs/development/BUILD_AND_TEST.md)
echo "" | tee -a "$RESULTS"
echo "[Step 3] Live blackboard - Prereqs: ROS built (./scripts/build_ros_for_bt_tests.sh), source install/setup.bash, ros2 run autonomy_bt bt_orchestrator (lifecycle configure/activate if needed)" | tee -a "$RESULTS"
echo "  In another terminal: source install/setup.bash && export PYTHONPATH=src && python3 -m pytest tests/integration/core/test_unified_blackboard.py -v --tb=short" | tee -a "$RESULTS"
echo "[Step 4] BT + state machine runtime - Prereqs: same as Step 3 plus adaptive_state_machine running" | tee -a "$RESULTS"
echo "  Same terminal: python3 -m pytest tests/integration/autonomy/test_bt_state_machine_runtime.py -v --tb=short" | tee -a "$RESULTS"
echo "Steps 3-4: SKIP (run manually; full testing plan order: docs/development/BUILD_AND_TEST.md#testing-plan-order)" | tee -a "$RESULTS"

echo "" | tee -a "$RESULTS"
echo "================================================================" | tee -a "$RESULTS"
echo "Validation complete. Steps 1-2 passed. Steps 3-4 require live ROS and built autonomy_bt." | tee -a "$RESULTS"
