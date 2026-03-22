#!/bin/bash
# URC 2026 HIL Test: LiDAR → WebSocket Integration
# Run this script in your lab environment with physical hardware connected

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
REPORT_FILE="$PROJECT_ROOT/test_reports/hil_lidar_websocket_test_$(date +%Y%m%d_%H%M%S).md"

# ros2 launch imports launch/frontend which needs system packages (e.g. lark). venv/conda
# often shadows python3 or PYTHONPATH and causes ModuleNotFoundError, while ros2 run may
# still appear to work. Prefer system binaries first for this script.
export PATH="/usr/bin:/bin:/usr/local/bin:${PATH}"
unset VIRTUAL_ENV

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
log_success() { echo -e "${GREEN}[PASS]${NC} $1"; }
log_warning() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[FAIL]${NC} $1"; }

# install/autonomy_interfaces/lib/pythonX.Y must match this python3 or rclpy fails importing services.
ros_python_bindings_match() {
    local lib_root="$PROJECT_ROOT/install/autonomy_interfaces/lib"
    [ -d "$lib_root" ] || return 0
    local bindir
    bindir=$(find "$lib_root" -maxdepth 1 -type d -name 'python*' 2>/dev/null | head -1)
    [ -n "$bindir" ] || return 0
    local ifver runver
    ifver=$(basename "$bindir")
    ifver=${ifver#python}
    runver=$(python3 -c 'import sys; print("%d.%d" % (sys.version_info[0:2]))')
    if [ "$ifver" != "$runver" ]; then
        log_error "Python mismatch: autonomy_interfaces under install/ is for ${ifver} but python3 is ${runver}."
        log_info "Run: ./scripts/clean_ros_build.sh (from a shell without conda), then source install/setup.bash"
        log_info "Doc: docs/development/ros2_python_environment.rst"
        return 1
    fi
    return 0
}

# Create report header
cat > "$REPORT_FILE" << 'HEADER'
# URC 2026 HIL Test Report: LiDAR → WebSocket

**Test Date:** $(date)
**Tester:** URC 2026 Team
**Objective:** Hardware-in-the-Loop integration with physical LiDAR and WebSocket dashboard

---

## Hardware Configuration

| Component | Expected | Status |
|-----------|----------|--------|
| Unitree L2 LiDAR | IP: 192.168.1.62 | ⏳ Pending |
| Host Network | 192.168.1.x/24 | ⏳ Pending |
| STM32 CAN | /dev/ttyACM0 | ⏳ Pending |

---

HEADER

echo "=========================================="
echo "URC 2026 HIL Test: LiDAR → WebSocket"
echo "=========================================="
echo ""

# Step 1: Environment Setup
log_info "Step 1: Environment Setup"
if [ -n "${CONDA_DEFAULT_ENV:-}" ]; then
    log_warning "conda env active (${CONDA_DEFAULT_ENV}). If ros2 launch fails, run: conda deactivate"
fi
export ROS_DOMAIN_ID=42
export URC_REPO_ROOT=$PROJECT_ROOT
source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash
source "$PROJECT_ROOT/install/setup.bash" 2>/dev/null || {
    log_error "Workspace not built. Run: colcon build"
    exit 1
}
log_success "Environment configured (ROS_DOMAIN_ID=$ROS_DOMAIN_ID)"
echo ""

# Step 2: Physical Hardware Verification
echo "=========================================="
echo "PHASE 1: PHYSICAL HARDWARE CONNECTION"
echo "=========================================="
echo ""

log_info "Checking LiDAR network connectivity..."
if ping -c 3 192.168.1.62 > /dev/null 2>&1; then
    log_success "LiDAR reachable at 192.168.1.62"
    LIDAR_PING="✅ PASS"
else
    log_error "LiDAR NOT reachable. Check:"
    log_error "  1. Ethernet cable connected"
    log_error "  2. Host IP in 192.168.1.x subnet"
    log_error "  3. LiDAR powered on"
    LIDAR_PING="❌ FAIL"
fi

log_info "Checking CAN adapter..."
if [ -e /dev/ttyACM0 ]; then
    log_success "CAN adapter found at /dev/ttyACM0"
    CAN_DEV="✅ PASS"
else
    log_warning "CAN adapter not at /dev/ttyACM0"
    log_info "Available serial devices:"
    ls -la /dev/ttyACM* 2>/dev/null || ls -la /dev/ttyUSB* 2>/dev/null || echo "None found"
    CAN_DEV="⚠️ CHECK"
fi

log_info "Checking dashboard..."
if [ -d "$PROJECT_ROOT/services/dashboard" ]; then
    log_success "Dashboard available"
    DASH_AVAIL="✅ PASS"
else
    log_error "Dashboard not found"
    DASH_AVAIL="❌ FAIL"
fi

# Update report
cat >> "$REPORT_FILE" << HARDWARE
## Phase 1 Results: Hardware Connection

| Check | Result |
|-------|--------|
| LiDAR Ping (192.168.1.62) | $LIDAR_PING |
| CAN Adapter (/dev/ttyACM0) | $CAN_DEV |
| Dashboard Available | $DASH_AVAIL |

HARDWARE

# Step 3: Start Core Services
echo ""
echo "=========================================="
echo "PHASE 2: START CORE SERVICES"
echo "=========================================="
echo ""

log_info "Starting BT Orchestrator..."
ros2 run autonomy_bt bt_orchestrator > /tmp/bt_orchestrator.log 2>&1 &
BT_PID=$!
sleep 3

log_info "Configuring lifecycle..."
ros2 lifecycle set /bt_orchestrator configure > /dev/null 2>&1 && \
ros2 lifecycle set /bt_orchestrator activate > /dev/null 2>&1

if ros2 service list | grep -q /blackboard/get_value; then
    log_success "BT Orchestrator active, blackboard services ready"
    BT_STATUS="✅ PASS"
else
    log_error "BT Orchestrator failed to start"
    BT_STATUS="❌ FAIL"
fi

log_info "Starting Rosbridge..."
ros2 launch autonomy_core rosbridge_stack.launch.py > /tmp/rosbridge.log 2>&1 &
ROSBRIDGE_PID=$!
sleep 3

if nc -z 127.0.0.1 9090 2>/dev/null; then
    log_success "Rosbridge active on port 9090"
    ROSBRIDGE_STATUS="✅ PASS"
else
    log_error "Rosbridge failed to start (port 9090 not open)"
    log_info "Last 40 lines of /tmp/rosbridge.log:"
    tail -n 40 /tmp/rosbridge.log 2>/dev/null || true
    log_info "Common fixes: conda deactivate; ensure apt packages:"
    log_info "  sudo apt install ros-\${ROS_DISTRO}-rosbridge-server ros-\${ROS_DISTRO}-rosapi python3-lark"
    log_info "If log shows ModuleNotFoundError (e.g. lark): fix Python env (see script header PATH sanitization)."
    ROSBRIDGE_STATUS="❌ FAIL"
fi

cat >> "$REPORT_FILE" << SERVICES
## Phase 2 Results: Core Services

| Service | PID | Status |
|---------|-----|--------|
| bt_orchestrator | $BT_PID | $BT_STATUS |
| rosbridge_stack | $ROSBRIDGE_PID | $ROSBRIDGE_STATUS |

SERVICES

# Step 4: LiDAR Integration
echo ""
echo "=========================================="
echo "PHASE 3: LiDAR DRIVER & BRIDGE"
echo "=========================================="
echo ""

log_info "IMPORTANT: LiDAR driver in another terminal must use the SAME ROS_DOMAIN_ID as this script:"
echo "  export ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "  ros2 launch unitree_lidar_ros2 launch.py"
echo ""
read -p "Press Enter when LiDAR driver is running..."

log_info "Checking LiDAR topics..."
echo ""
echo "--- /unilidar/cloud topic ---"
ros2 topic hz /unilidar/cloud --window 5 2>&1 | head -10 || {
    log_error "LiDAR not publishing"
    LIDAR_HZ="❌ FAIL"
}

echo ""
log_info "Starting PointCloud2 → LaserScan bridge..."
BRIDGE_PID=""
if ros2 pkg prefix pointcloud_to_laserscan &>/dev/null; then
    ros2 launch autonomy_core unitree_l2_pc2_to_scan.launch.py > /tmp/lidar_bridge.log 2>&1 &
    BRIDGE_PID=$!
    sleep 5

    if ! ros2 node list 2>/dev/null | grep -qE 'unitree_l2_pc2_to_scan'; then
        log_warning "Bridge node not listed; check /tmp/lidar_bridge.log"
        log_info "Last 30 lines of /tmp/lidar_bridge.log:"
        tail -n 30 /tmp/lidar_bridge.log 2>/dev/null || true
    fi
else
    log_error "ROS package 'pointcloud_to_laserscan' not installed (launch will always fail until this is fixed)."
    log_info "Install: sudo apt install ros-\${ROS_DISTRO}-pointcloud-to-laserscan"
    log_info "Example (Jazzy): sudo apt install ros-jazzy-pointcloud-to-laserscan"
    echo "package 'pointcloud_to_laserscan' not found" >/tmp/lidar_bridge.log
fi

echo ""
echo "--- /scan topic ---"
ros2 topic hz /scan --window 5 2>&1 | head -10 || {
    log_warning "LaserScan not publishing (confirm: ros2 pkg list | grep pointcloud_to_laserscan; sudo apt install ros-\${ROS_DISTRO}-pointcloud-to-laserscan)"
    SCAN_HZ="⚠️ CHECK"
}

echo ""
log_info "Starting proximity_monitor..."
ros2 run autonomy_core proximity_monitor > /tmp/proximity.log 2>&1 &
PROXIMITY_PID=$!
sleep 2

cat >> "$REPORT_FILE" << LIDAR
## Phase 3 Results: LiDAR Integration

| Component | Status |
|-----------|--------|
| LiDAR Driver | ⏳ Manual Verification |
| PointCloud2 Hz | See output above |
| LaserScan Hz | See output above |
| proximity_monitor | PID: $PROXIMITY_PID |

LIDAR

# Step 5: Smoke Tests
echo ""
echo "=========================================="
echo "PHASE 4: SMOKE TESTS"
echo "=========================================="
echo ""

log_info "Running lidar_blackboard_smoke_test.py..."
echo ""
if ! ros_python_bindings_match; then
    log_error "Skipping smoke test until workspace is rebuilt with matching Python."
    SMOKE_RESULT="SKIPPED (Python/bindings mismatch)"
elif python3 "$PROJECT_ROOT/scripts/hardware/lidar_blackboard_smoke_test.py" 2>&1; then
    log_success "Smoke test completed"
    SMOKE_RESULT="✅ PASS"
else
    log_error "Smoke test failed (see output)"
    SMOKE_RESULT="❌ FAIL"
fi

cat >> "$REPORT_FILE" << SMOKE
## Phase 4 Results: Smoke Tests

| Test | Result |
|------|--------|
| lidar_blackboard_smoke_test.py | $SMOKE_RESULT |

SMOKE

# Step 6: Dashboard
echo ""
echo "=========================================="
echo "PHASE 5: DASHBOARD VERIFICATION"
echo "=========================================="
echo ""

log_info "Dashboard Instructions:"
echo ""
echo "1. Open browser: http://localhost:5173"
echo "2. Verify connection to: ws://localhost:9090"
echo "3. Check Blackboard tab for values:"
echo "   - closest_obstacle_distance"
echo "   - obstacle_detected"
echo "   - proximity_violation_distance"
echo ""
read -p "Press Enter after dashboard verification..."

cat >> "$REPORT_FILE" << DASHBOARD
## Phase 5 Results: Dashboard

| Check | Status |
|-------|--------|
| WebSocket Connection | ⏳ Manual Verification |
| Blackboard Values | ⏳ Manual Verification |
| Real-time Updates | ⏳ Manual Verification |

DASHBOARD

# Summary
cat >> "$REPORT_FILE" << SUMMARY

---

## Summary

**Test Date:** $(date)
**Report File:** $REPORT_FILE

### Overall Status: ⏳ PENDING MANUAL VERIFICATION

### Next Steps:
1. Review all Phase results above
2. Check log files:
   - /tmp/bt_orchestrator.log
   - /tmp/rosbridge.log
   - /tmp/lidar_bridge.log
   - /tmp/proximity.log
3. If all phases pass: System ready for mission testing
4. If any phase fails: Review troubleshooting in docs/hardware/skeletal_hil_checklist.rst

### PIDs to Monitor:
- BT Orchestrator: $BT_PID
- Rosbridge: $ROSBRIDGE_PID
- LiDAR Bridge: $BRIDGE_PID
- Proximity Monitor: $PROXIMITY_PID

### To Stop All:
```bash
kill $BT_PID $ROSBRIDGE_PID $BRIDGE_PID $PROXIMITY_PID 2>/dev/null
echo "All services stopped"
```

SUMMARY

echo ""
echo "=========================================="
echo "HIL TEST COMPLETE"
echo "=========================================="
echo ""
echo "Report saved to: $REPORT_FILE"
echo ""
echo "Active processes:"
echo "  BT Orchestrator: $BT_PID"
echo "  Rosbridge: $ROSBRIDGE_PID"
echo "  LiDAR Bridge: $BRIDGE_PID"
echo "  Proximity Monitor: $PROXIMITY_PID"
echo ""
echo "To stop all: kill $BT_PID $ROSBRIDGE_PID $BRIDGE_PID $PROXIMITY_PID"
