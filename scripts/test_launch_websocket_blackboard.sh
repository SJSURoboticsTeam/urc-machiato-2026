#!/bin/bash
# Comprehensive Launch File, Connection, and Blackboard WebSocket Test
# URC 2026 - Validates launch files, container startup, ROS2 connections, and blackboard access

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Test results
declare -A TEST_RESULTS
total_tests=0
passed_tests=0
failed_tests=0

# Test report file
REPORT_FILE="$PROJECT_ROOT/test_reports/launch_websocket_test_report_$(date +%Y%m%d_%H%M%S).md"
mkdir -p "$PROJECT_ROOT/test_reports"

log_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
log_success() { echo -e "${GREEN}[PASS]${NC} $1"; }
log_warning() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[FAIL]${NC} $1"; }

record_test() {
    local test_name="$1"
    local result="$2"
    local details="${3:-}"
    TEST_RESULTS["$test_name"]="$result|$details"
    total_tests=$((total_tests + 1))
    if [[ "$result" == "PASS" ]]; then
        passed_tests=$((passed_tests + 1))
    else
        failed_tests=$((failed_tests + 1))
    fi
}

# ==================== TEST 1: Launch File Syntax Validation ====================

test_launch_syntax() {
    log_info "=== TEST 1: Launch File Syntax Validation ==="
    
    local launch_files=(
        "services/autonomy/autonomy_core/launch/unified.launch.py"
        "services/autonomy/autonomy_core/launch/simulation.launch.py"
        "services/autonomy/autonomy_core/launch/slam.launch.py"
        "services/autonomy/autonomy_core/launch/unitree_l2_pc2_to_scan.launch.py"
        "services/autonomy/autonomy_core/launch/competition_system.launch.py"
        "scripts/launch/integrated_system.launch.py"
        "scripts/launch/mission_system.launch.py"
        "scripts/launch/rover_simulation.launch.py"
    )
    
    for launch_file in "${launch_files[@]}"; do
        local full_path="$PROJECT_ROOT/$launch_file"
        if [[ -f "$full_path" ]]; then
            # Test Python syntax
            if python3 -m py_compile "$full_path" 2>/dev/null; then
                log_success "Syntax OK: $launch_file"
                record_test "Launch Syntax: $launch_file" "PASS"
            else
                log_error "Syntax Error: $launch_file"
                record_test "Launch Syntax: $launch_file" "FAIL" "Python syntax error"
            fi
        else
            log_warning "Missing: $launch_file"
            record_test "Launch Syntax: $launch_file" "FAIL" "File not found"
        fi
    done
    echo ""
}

# ==================== TEST 2: Container Launch Capability ====================

test_container_launch() {
    log_info "=== TEST 2: Container Launch Capability ==="
    
    # Check if Docker is available
    if ! command -v docker &>/dev/null; then
        log_warning "Docker not available, skipping container tests"
        record_test "Container Availability" "SKIP" "Docker not installed"
        return
    fi
    
    # Test docker compose files
    local compose_files=(
        "docker/docker-compose.test.yml"
        "docker/docker-compose.dev.yml"
        "docker/docker-compose.unified.yml"
    )
    
    for compose_file in "${compose_files[@]}"; do
        local full_path="$PROJECT_ROOT/$compose_file"
        if [[ -f "$full_path" ]]; then
            # Validate docker-compose syntax
            if docker compose -f "$full_path" config > /dev/null 2>&1; then
                log_success "Compose OK: $compose_file"
                record_test "Compose Config: $compose_file" "PASS"
            else
                log_error "Compose Error: $compose_file"
                record_test "Compose Config: $compose_file" "FAIL" "Invalid docker-compose syntax"
            fi
        else
            log_warning "Missing: $compose_file"
            record_test "Compose Config: $compose_file" "FAIL" "File not found"
        fi
    done
    
    # Test if test image exists or can be built
    if docker image inspect urc2026:test &>/dev/null 2>&1; then
        log_success "Test image exists: urc2026:test"
        record_test "Container Image" "PASS" "Image urc2026:test available"
    else
        log_warning "Test image not found, attempting build..."
        if docker build -t urc2026:test -f "$PROJECT_ROOT/docker/dockerfiles/autonomy.Dockerfile" "$PROJECT_ROOT" > /dev/null 2>&1; then
            log_success "Test image built successfully"
            record_test "Container Image" "PASS" "Built urc2026:test"
        else
            log_error "Failed to build test image"
            record_test "Container Image" "FAIL" "Build failed"
        fi
    fi
    
    echo ""
}

# ==================== TEST 3: ROS2 Environment Check ====================

test_ros2_environment() {
    log_info "=== TEST 3: ROS2 Environment Check ==="
    
    # Check ROS2 installation
    if command -v ros2 &>/dev/null; then
        local ros_version=$(ros2 --version 2>/dev/null || echo "unknown")
        log_success "ROS2 available: $ros_version"
        record_test "ROS2 Installation" "PASS" "Version: $ros_version"
    else
        log_error "ROS2 not found in PATH"
        record_test "ROS2 Installation" "FAIL" "ros2 command not found"
    fi
    
    # Check if workspace is built
    if [[ -f "$PROJECT_ROOT/install/setup.bash" ]]; then
        log_success "Workspace built: install/setup.bash exists"
        record_test "Workspace Build" "PASS"
        
        # Source and check packages
        source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash 2>/dev/null || true
        source "$PROJECT_ROOT/install/setup.bash" 2>/dev/null || true
        
        local pkg_count=$(ros2 pkg list 2>/dev/null | grep -c "^autonomy" || echo "0")
        if [[ $pkg_count -gt 0 ]]; then
            log_success "Found $pkg_count autonomy packages"
            record_test "ROS2 Packages" "PASS" "$pkg_count packages"
        else
            log_warning "No autonomy packages found (may need build)"
            record_test "ROS2 Packages" "WARN" "0 packages"
        fi
    else
        log_warning "Workspace not built: install/setup.bash missing"
        record_test "Workspace Build" "WARN" "Run: colcon build"
    fi
    
    echo ""
}

# ==================== TEST 4: Launch File Dry-Run ====================

test_launch_dry_run() {
    log_info "=== TEST 4: Launch File Dry-Run Tests ==="
    
    # Source ROS if available
    source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash 2>/dev/null || true
    source "$PROJECT_ROOT/install/setup.bash" 2>/dev/null || true
    
    # Check if ros2 is available
    if ! command -v ros2 &>/dev/null; then
        log_warning "ROS2 not available, skipping launch dry-run tests"
        record_test "Launch Dry-Run" "SKIP" "ROS2 not installed"
        echo ""
        return
    fi
    
    # Test launch file argument parsing (show-args)
    local test_launches=(
        "autonomy_core unified.launch.py mode:=simulation"
        "autonomy_core simulation.launch.py"
        "autonomy_core slam.launch.py"
    )
    
    for launch in "${test_launches[@]}"; do
        local pkg=$(echo "$launch" | cut -d' ' -f1)
        local file=$(echo "$launch" | cut -d' ' -f2)
        local args=$(echo "$launch" | cut -d' ' -f3-)
        
        # Quick check if package exists (with timeout)
        if timeout 3 bash -c "ros2 pkg list 2>/dev/null | grep -q '^$pkg'" 2>/dev/null; then
            if timeout 5 ros2 launch "$pkg" "$file" --show-args $args > /dev/null 2>&1; then
                log_success "Launch args OK: $pkg/$file"
                record_test "Launch Args: $file" "PASS"
            else
                log_warning "Launch args issue: $pkg/$file (may need dependencies)"
                record_test "Launch Args: $file" "WARN" "Check dependencies"
            fi
        else
            log_info "Package not available: $pkg (needs colcon build)"
            record_test "Launch Args: $file" "SKIP" "Package $pkg not built"
        fi
    done
    
    echo ""
}

# ==================== TEST 5: Connection and Topic Tests ====================

test_connections() {
    log_info "=== TEST 5: ROS2 Connection and Topic Tests ==="
    
    source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash 2>/dev/null || true
    source "$PROJECT_ROOT/install/setup.bash" 2>/dev/null || true
    
    # Check if ros2 is available
    if ! command -v ros2 &>/dev/null; then
        log_warning "ROS2 not available, skipping connection tests"
        record_test "ROS2 Connections" "SKIP" "ROS2 not installed"
        echo ""
        return
    fi
    
    # Check daemon status
    if timeout 3 ros2 daemon status 2>/dev/null | grep -q "running"; then
        log_success "ROS2 daemon running"
        record_test "ROS2 Daemon" "PASS"
    else
        log_info "ROS2 daemon not running (normal for first run)"
        record_test "ROS2 Daemon" "INFO" "Not running"
    fi
    
    # Check node discovery (if any nodes are running) with timeout
    local node_count=$(timeout 3 ros2 node list 2>/dev/null | wc -l || echo "0")
    if [[ $node_count -gt 0 ]]; then
        log_success "Found $node_count active nodes"
        record_test "Node Discovery" "PASS" "$node_count nodes"
    else
        log_info "No active nodes (expected if no launch running)"
        record_test "Node Discovery" "INFO" "No nodes running"
    fi
    
    # Check topic list with timeout
    local topic_count=$(timeout 3 ros2 topic list 2>/dev/null | wc -l || echo "0")
    if [[ $topic_count -gt 0 ]]; then
        log_success "Found $topic_count topics"
        record_test "Topic Discovery" "PASS" "$topic_count topics"
    else
        log_info "No active topics (expected if no launch running)"
        record_test "Topic Discovery" "INFO" "No topics"
    fi
    
    echo ""
}

# ==================== TEST 6: Blackboard WebSocket Tests ====================

test_blackboard_websocket() {
    log_info "=== TEST 6: Blackboard WebSocket Tests ==="
    
    source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash 2>/dev/null || true
    source "$PROJECT_ROOT/install/setup.bash" 2>/dev/null || true
    
    # Check for bt_orchestrator executable
    if ros2 pkg executables autonomy_bt 2>/dev/null | grep -q "bt_orchestrator"; then
        log_success "bt_orchestrator executable found"
        record_test "BT Orchestrator Binary" "PASS"
    else
        log_warning "bt_orchestrator not found (may need build)"
        record_test "BT Orchestrator Binary" "WARN" "Build autonomy_bt package"
    fi
    
    # Check for blackboard services
    local services=$(ros2 service list 2>/dev/null | grep -c "blackboard" || echo "0")
    if [[ $services -gt 0 ]]; then
        log_success "Blackboard services found ($services)"
        ros2 service list 2>/dev/null | grep "blackboard" | head -5 | while read line; do
            echo "  - $line"
        done
        record_test "Blackboard Services" "PASS" "$services services"
    else
        log_info "No blackboard services (bt_orchestrator not running)"
        record_test "Blackboard Services" "INFO" "Start bt_orchestrator to test"
    fi
    
    # Check rosbridge configuration
    if ros2 pkg list 2>/dev/null | grep -q "rosbridge_server"; then
        log_success "rosbridge_server package available"
        record_test "Rosbridge Package" "PASS"
    else
        log_warning "rosbridge_server not found (install: sudo apt install ros-\$ROS_DISTRO-rosbridge-server)"
        record_test "Rosbridge Package" "WARN" "Not installed"
    fi
    
    # Test WebSocket port availability (if rosbridge is running)
    if command -v nc &>/dev/null; then
        if nc -z localhost 9090 2>/dev/null; then
            log_success "WebSocket port 9090 open (rosbridge running)"
            record_test "WebSocket Port" "PASS" "Port 9090 active"
        elif nc -z localhost 9091 2>/dev/null; then
            log_success "WebSocket port 9091 open (alternative)"
            record_test "WebSocket Port" "PASS" "Port 9091 active"
        else
            log_info "No WebSocket port open (rosbridge not running)"
            record_test "WebSocket Port" "INFO" "Start rosbridge to test"
        fi
    else
        log_warning "nc not available, skipping port check"
        record_test "WebSocket Port" "SKIP" "nc command not found"
    fi
    
    # Test Python blackboard smoke test script
    if [[ -f "$PROJECT_ROOT/scripts/hardware/lidar_blackboard_smoke_test.py" ]]; then
        if python3 -m py_compile "$PROJECT_ROOT/scripts/hardware/lidar_blackboard_smoke_test.py" 2>/dev/null; then
            log_success "lidar_blackboard_smoke_test.py syntax OK"
            record_test "Blackboard Test Script" "PASS"
        else
            log_error "lidar_blackboard_smoke_test.py has syntax errors"
            record_test "Blackboard Test Script" "FAIL" "Syntax error"
        fi
    fi
    
    # Test sensor_topic_smoke.sh
    if [[ -f "$PROJECT_ROOT/scripts/hardware/sensor_topic_smoke.sh" ]]; then
        if bash -n "$PROJECT_ROOT/scripts/hardware/sensor_topic_smoke.sh" 2>/dev/null; then
            log_success "sensor_topic_smoke.sh syntax OK"
            record_test "Sensor Smoke Script" "PASS"
        else
            log_error "sensor_topic_smoke.sh has syntax errors"
            record_test "Sensor Smoke Script" "FAIL" "Syntax error"
        fi
    fi
    
    echo ""
}

# ==================== TEST 7: Dashboard Frontend ====================

test_dashboard_frontend() {
    log_info "=== TEST 7: Dashboard Frontend Tests ==="
    
    local dashboard_dir="$PROJECT_ROOT/services/dashboard"
    
    if [[ -d "$dashboard_dir" ]]; then
        log_success "Dashboard directory exists"
        record_test "Dashboard Directory" "PASS"
        
        # Check for package.json
        if [[ -f "$dashboard_dir/package.json" ]]; then
            log_success "package.json found"
            record_test "Dashboard package.json" "PASS"
        else
            log_warning "package.json missing"
            record_test "Dashboard package.json" "WARN"
        fi
        
        # Check for key source files
        local key_files=(
            "src/utils/rosbridge.js"
            "src/hooks/useROS.js"
            "src/hooks/useBlackboardState.js"
        )
        
        for file in "${key_files[@]}"; do
            if [[ -f "$dashboard_dir/$file" ]]; then
                log_success "Found: $file"
                record_test "Dashboard: $file" "PASS"
            else
                log_warning "Missing: $file"
                record_test "Dashboard: $file" "WARN"
            fi
        done
    else
        log_warning "Dashboard directory not found"
        record_test "Dashboard Directory" "WARN" "Expected at services/dashboard"
    fi
    
    echo ""
}

# ==================== TEST 8: Integration Test Files ====================

test_integration_files() {
    log_info "=== TEST 8: Integration Test Files ==="
    
    local test_files=(
        "tests/integration/core/test_unified_blackboard.py"
        "tests/integration/hardware/test_hil_blackboard_command_path.py"
        "tests/integration/system/test_full_system_integration.py"
    )
    
    for test_file in "${test_files[@]}"; do
        local full_path="$PROJECT_ROOT/$test_file"
        if [[ -f "$full_path" ]]; then
            if python3 -m py_compile "$full_path" 2>/dev/null; then
                log_success "Test file OK: $test_file"
                record_test "Test: $test_file" "PASS"
            else
                log_error "Syntax error: $test_file"
                record_test "Test: $test_file" "FAIL" "Syntax error"
            fi
        else
            log_warning "Missing: $test_file"
            record_test "Test: $test_file" "WARN" "File not found"
        fi
    done
    
    echo ""
}

# ==================== REPORT GENERATION ====================

generate_report() {
    log_info "=== Generating Test Report ==="
    
    cat > "$REPORT_FILE" << 'EOF'
# URC 2026 Launch File, Connection, and Blackboard WebSocket Test Report

**Generated:** $(date)
**Test Script:** scripts/test_launch_websocket_blackboard.sh

## Summary

| Metric | Value |
|--------|-------|
| Total Tests | $total_tests |
| Passed | $passed_tests |
| Failed | $failed_tests |
| Success Rate | $(echo "scale=1; $passed_tests * 100 / $total_tests" | bc)% |

## Test Results by Category

EOF

    # Group results by category
    local categories=("Launch" "Container" "ROS2" "Blackboard" "Dashboard" "Test")
    
    for category in "${categories[@]}"; do
        echo "### $category Tests" >> "$REPORT_FILE"
        echo "" >> "$REPORT_FILE"
        echo "| Test | Result | Details |" >> "$REPORT_FILE"
        echo "|------|--------|---------|" >> "$REPORT_FILE"
        
        for test_name in "${!TEST_RESULTS[@]}"; do
            if [[ "$test_name" == "$category"* ]]; then
                local result="${TEST_RESULTS[$test_name]}"
                local status=$(echo "$result" | cut -d'|' -f1)
                local details=$(echo "$result" | cut -d'|' -f2-)
                echo "| $test_name | $status | $details |" >> "$REPORT_FILE"
            fi
        done
        echo "" >> "$REPORT_FILE"
    done
    
    # Add recommendations
    cat >> "$REPORT_FILE" << 'EOF'
## Recommendations

Based on the test results:

1. **Launch Files**: All launch files should pass Python syntax validation
2. **Docker Environment**: Ensure Docker images are built before running container tests
3. **ROS2 Workspace**: Run `colcon build` to build all packages before testing
4. **bt_orchestrator**: Required for blackboard functionality
5. **rosbridge_server**: Required for WebSocket dashboard connectivity

## Quick Start Commands

```bash
# Build workspace
colcon build --symlink-install

# Source workspace
source install/setup.bash

# Start bt_orchestrator
ros2 run autonomy_bt bt_orchestrator

# Start rosbridge
ros2 run rosbridge_server rosbridge_websocket

# Run blackboard smoke test
python3 scripts/hardware/lidar_blackboard_smoke_test.py
```

EOF

    # Replace variables
    sed -i "s/\$(date)/$(date)/g" "$REPORT_FILE"
    sed -i "s/\$total_tests/$total_tests/g" "$REPORT_FILE"
    sed -i "s/\$passed_tests/$passed_tests/g" "$REPORT_FILE"
    sed -i "s/\$failed_tests/$failed_tests/g" "$REPORT_FILE"
    
    log_success "Report generated: $REPORT_FILE"
}

# ==================== MAIN ====================

main() {
    echo ""
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}URC 2026 Launch & WebSocket Test Suite${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""
    
    # Run all tests
    test_launch_syntax
    test_container_launch
    test_ros2_environment
    test_launch_dry_run
    test_connections
    test_blackboard_websocket
    test_dashboard_frontend
    test_integration_files
    
    # Generate report
    generate_report
    
    # Summary
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}Test Summary${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""
    echo -e "Total Tests: $total_tests"
    echo -e "${GREEN}Passed: $passed_tests${NC}"
    echo -e "${RED}Failed: $failed_tests${NC}"
    echo ""
    
    if [[ $failed_tests -eq 0 ]]; then
        echo -e "${GREEN}✅ All tests passed!${NC}"
        exit 0
    else
        echo -e "${YELLOW}⚠️  Some tests had warnings or failures${NC}"
        echo ""
        echo -e "Review the detailed report:"
        echo "  $REPORT_FILE"
        exit 1
    fi
}

main "$@"
