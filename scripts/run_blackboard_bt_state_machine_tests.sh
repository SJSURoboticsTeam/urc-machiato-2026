#!/usr/bin/env bash
# Run blackboard, BT, and state-machine hybrid unit tests (real tests, traceable).
# Emits JUnit XML to tests/reports/ for CI and traceability.
# Avoids ROS launch_testing_ros_pytest_entrypoint plugin by using a minimal path.
set -e
cd "$(dirname "$0")/.."
REPORTS_DIR="tests/reports"
mkdir -p "$REPORTS_DIR"
JUNIT_XML="${REPORTS_DIR}/junit_blackboard_bt_state_machine.xml"
export PYTHONPATH="src:tests:${PYTHONPATH:-}"
# Strip ROS/ament paths so launch_testing_ros_pytest_entrypoint is not loaded
python3 -c "
import sys
ros_paths = [p for p in sys.path if 'ros' in p.lower() or 'ament' in p.lower() or 'jazzy' in p.lower()]
for p in ros_paths:
    try:
        sys.path.remove(p)
    except ValueError:
        pass
sys.path.insert(0, 'src')
sys.path.insert(0, 'tests')
import pytest
exit(pytest.main([
    'tests/unit/core/test_hybrid_controller_logic.py',
    'tests/unit/core/test_blackboard_keys_schema.py',
    'tests/unit/core/test_blackboard_persistence.py',
    'tests/unit/core/test_adaptive_state_machine_hybrid.py',
    '-v', '--tb=short',
    '--junitxml', 'tests/reports/junit_blackboard_bt_state_machine.xml',
    '-p', 'no:launch_testing',
    '-p', 'no:launch_testing_ros_pytest_entrypoint',
]))
"
exit $?
