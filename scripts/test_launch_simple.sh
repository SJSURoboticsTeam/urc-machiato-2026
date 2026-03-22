#!/bin/bash
# Simple Launch File and WebSocket Test

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "URC 2026 Simple Launch & WebSocket Test"
echo "========================================"
echo ""

# Test 1: Launch File Syntax
echo "TEST 1: Launch File Syntax"
for f in "$PROJECT_ROOT"/services/autonomy/autonomy_core/launch/*.launch.py; do
    if python3 -m py_compile "$f" 2>/dev/null; then
        echo "  [PASS] $(basename "$f")"
    else
        echo "  [FAIL] $(basename "$f")"
    fi
done
echo ""

# Test 2: Docker Compose
echo "TEST 2: Docker Compose Validation"
if command -v docker &>/dev/null; then
    for f in "$PROJECT_ROOT"/docker/docker-compose.*.yml; do
        if docker compose -f "$f" config > /dev/null 2>&1; then
            echo "  [PASS] $(basename "$f")"
        else
            echo "  [FAIL] $(basename "$f")"
        fi
    done
else
    echo "  [SKIP] Docker not available"
fi
echo ""

# Test 3: ROS2 Environment
echo "TEST 3: ROS2 Environment"
if command -v ros2 &>/dev/null; then
    echo "  [PASS] ROS2 available"
    source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null || true
    if [[ -f "$PROJECT_ROOT/install/setup.bash" ]]; then
        echo "  [PASS] Workspace built"
    else
        echo "  [WARN] Workspace not built (run: colcon build)"
    fi
else
    echo "  [FAIL] ROS2 not found"
fi
echo ""

# Test 4: Blackboard Components
echo "TEST 4: Blackboard Components"
if [[ -f "$PROJECT_ROOT/scripts/hardware/lidar_blackboard_smoke_test.py" ]]; then
    echo "  [PASS] lidar_blackboard_smoke_test.py exists"
else
    echo "  [FAIL] lidar_blackboard_smoke_test.py missing"
fi

if [[ -f "$PROJECT_ROOT/scripts/hardware/sensor_topic_smoke.sh" ]]; then
    echo "  [PASS] sensor_topic_smoke.sh exists"
else
    echo "  [FAIL] sensor_topic_smoke.sh missing"
fi

if [[ -f "$PROJECT_ROOT/services/dashboard/src/hooks/useBlackboardState.js" ]]; then
    echo "  [PASS] useBlackboardState.js exists"
else
    echo "  [FAIL] useBlackboardState.js missing"
fi
echo ""

# Test 5: WebSocket Port Check (canonical 9090 matches services/dashboard)
echo "TEST 5: WebSocket Port Check"
if command -v nc &>/dev/null; then
    if nc -z 127.0.0.1 9090 2>/dev/null; then
        echo "  [PASS] Port 9090 open (matches dashboard default ws://localhost:9090)"
    elif nc -z 127.0.0.1 9091 2>/dev/null; then
        echo "  [INFO] Port 9091 open only (legacy integrated_system; point dashboard at :9091 or use rosbridge_stack)"
    else
        echo "  [INFO] No rosbridge port listening. After: source install/setup.bash"
        echo "         ros2 launch autonomy_core rosbridge_stack.launch.py"
        echo "         verify: nc -z 127.0.0.1 9090"
    fi
else
    echo "  [SKIP] nc not available"
fi
echo ""

echo "Test Complete"
echo "============="
echo "Review the README files for next steps:"
echo "  - docs/hardware/skeletal_hil_checklist.rst"
echo "  - docs/operations/deployment.rst"
echo "  - docs/hardware/l2_lidar_blackboard_e2e.rst"
