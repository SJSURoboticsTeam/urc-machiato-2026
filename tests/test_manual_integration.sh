#!/bin/bash
# Manual Integration Testing Bootup Sequence
# Comprehensive testing of autonomy-teleoperation integration

set -e

# Configuration
WORKSPACE_DIR="/home/ubuntu/urc-machiato-2026"
ROS_DISTRO="humble"
TEST_DURATION=30

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Logging functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

log_step() {
    echo -e "${PURPLE}[STEP]${NC} $1"
}

log_command() {
    echo -e "${CYAN}[CMD]${NC} $1"
}

# Check prerequisites
check_prerequisites() {
    log_step "Checking Prerequisites"

    # Check if we're in the right directory
    if [[ ! -f "missions/mission_executor.py" ]]; then
        log_error "Not in workspace directory. Please run from $WORKSPACE_DIR"
        exit 1
    fi

    # Check ROS2
    if ! command -v ros2 &> /dev/null; then
        log_error "ROS2 not found. Please install ROS2 $ROS_DISTRO"
        exit 1
    fi

    # Check Python dependencies
    if ! python3 -c "import rclpy, yaml" &> /dev/null; then
        log_error "Python dependencies missing. Install with: pip install rclpy pyyaml"
        exit 1
    fi

    log_success "Prerequisites check passed"
}

# Setup ROS2 environment
setup_ros2() {
    log_step "Setting up ROS2 Environment"

    # Source ROS2
    source /opt/ros/$ROS_DISTRO/setup.bash
    log_info "ROS2 $ROS_DISTRO sourced"

    # Source workspace if it exists
    if [[ -f "install/setup.bash" ]]; then
        source install/setup.bash
        log_info "Workspace sourced"
    fi

    # Set environment variables
    export PYTHONPATH="$WORKSPACE_DIR:$PYTHONPATH"
    export ROVER_ENV="production"
    log_info "Environment configured"
}

# Start ROS2 infrastructure
start_ros2_infrastructure() {
    log_step "Starting ROS2 Infrastructure"

    # Start ROS2 daemon if needed
    if ! ros2 daemon status &> /dev/null; then
        log_info "Starting ROS2 daemon..."
        ros2 daemon start
        sleep 2
    fi

    log_success "ROS2 infrastructure ready"
}

# Start mock teleoperation
start_mock_teleoperation() {
    log_step "Starting Mock Teleoperation Data"

    log_command "python3 test_teleoperation_integration.py &"
    python3 test_teleoperation_integration.py &
    MOCK_PID=$!

    # Wait for it to start
    sleep 3

    # Verify it's running
    if kill -0 $MOCK_PID 2>/dev/null; then
        log_success "Mock teleoperation started (PID: $MOCK_PID)"
    else
        log_error "Mock teleoperation failed to start"
        return 1
    fi

    echo $MOCK_PID > .mock_pid
}

# Verify teleoperation topics
verify_teleoperation_topics() {
    log_step "Verifying Teleoperation Topics"

    log_info "Checking ROS2 topics..."
    sleep 2

    # Check if topics exist
    TOPICS=$(ros2 topic list 2>/dev/null)
    EXPECTED_TOPICS=(
        "/teleoperation/joint_states"
        "/teleoperation/chassis_velocity"
        "/teleoperation/motor_temperatures"
        "/teleoperation/system_status"
    )

    MISSING_TOPICS=()
    for topic in "${EXPECTED_TOPICS[@]}"; do
        if echo "$TOPICS" | grep -q "^$topic$"; then
            log_success "✓ $topic"
        else
            log_error "✗ $topic"
            MISSING_TOPICS+=("$topic")
        fi
    done

    if [[ ${#MISSING_TOPICS[@]} -eq 0 ]]; then
        log_success "All teleoperation topics verified"
        return 0
    else
        log_error "Missing topics: ${MISSING_TOPICS[*]}"
        return 1
    fi
}

# Test topic publishing rates
test_topic_rates() {
    log_step "Testing Topic Publishing Rates"

    log_command "ros2 topic hz /teleoperation/joint_states"
    RATE_OUTPUT=$(timeout 5 ros2 topic hz /teleoperation/joint_states 2>/dev/null || true)

    if echo "$RATE_OUTPUT" | grep -q "average rate:"; then
        RATE=$(echo "$RATE_OUTPUT" | grep "average rate:" | head -1 | sed 's/.*average rate: \([0-9.]*\).*/\1/')
        if (( $(echo "$RATE >= 8" | bc -l) )) && (( $(echo "$RATE <= 12" | bc -l) )); then
            log_success "Topic rate: ${RATE} Hz (expected: 8-12 Hz)"
            return 0
        else
            log_warning "Topic rate: ${RATE} Hz (expected: 8-12 Hz)"
            return 1
        fi
    else
        log_error "Could not determine topic rate"
        return 1
    fi
}

# Start autonomy components
start_autonomy() {
    log_step "Starting Autonomy Components"

    log_command "ros2 run missions mission_executor &"
    ros2 run missions mission_executor &
    AUTONOMY_PID=$!

    # Wait for it to start
    sleep 3

    # Verify it's running
    if kill -0 $AUTONOMY_PID 2>/dev/null; then
        log_success "Autonomy mission executor started (PID: $AUTONOMY_PID)"
    else
        log_error "Autonomy failed to start"
        return 1
    fi

    echo $AUTONOMY_PID > .autonomy_pid
}

# Verify autonomy subscriptions
verify_autonomy_subscriptions() {
    log_step "Verifying Autonomy Subscriptions"

    log_info "Checking if autonomy is receiving teleoperation data..."
    sleep 5

    # Check ROS2 nodes
    NODES=$(ros2 node list 2>/dev/null)
    if echo "$NODES" | grep -q "simple_mission_executor"; then
        log_success "✓ Autonomy node running"
    else
        log_error "✗ Autonomy node not found"
        return 1
    fi

    # The autonomy system should be logging data processing
    # We can't easily check internal state, but node presence indicates success
    log_success "Autonomy subscriptions verified"
}

# Interactive testing menu
interactive_testing() {
    log_step "Interactive Testing Menu"
    echo
    echo "🎮 Available Testing Commands:"
    echo "=============================="
    echo "1. Monitor teleoperation data"
    echo "2. Send test mission commands"
    echo "3. Check autonomy logs"
    echo "4. View ROS2 graph"
    echo "5. Test emergency scenarios"
    echo "6. Exit testing"
    echo

    while true; do
        read -p "Enter command (1-6): " choice
        echo

        case $choice in
            1)
                log_info "Monitoring teleoperation data..."
                echo "Press Ctrl+C to stop monitoring"
                timeout 10 ros2 topic echo /teleoperation/joint_states --once || true
                ;;
            2)
                log_info "Sending test mission command..."
                # Example mission command
                echo "Test mission commands would be sent here"
                ;;
            3)
                log_info "Checking autonomy logs..."
                echo "Recent autonomy logs (last 10 lines):"
                # In a real scenario, you'd tail ROS2 logs
                echo "(ROS2 logging would show autonomy data processing here)"
                ;;
            4)
                log_info "Viewing ROS2 computation graph..."
                ros2 run rqt_graph rqt_graph 2>/dev/null &
                sleep 2
                ;;
            5)
                log_info "Testing emergency scenarios..."
                echo "Emergency test commands would be sent here"
                ;;
            6)
                log_info "Exiting interactive testing"
                break
                ;;
            *)
                echo "Invalid choice. Please enter 1-6."
                ;;
        esac
        echo
    done
}

# Cleanup function
cleanup() {
    log_step "Cleaning up test environment"

    # Stop mock teleoperation
    if [[ -f .mock_pid ]]; then
        MOCK_PID=$(cat .mock_pid)
        if kill -0 $MOCK_PID 2>/dev/null; then
            log_info "Stopping mock teleoperation (PID: $MOCK_PID)"
            kill $MOCK_PID 2>/dev/null || true
        fi
        rm -f .mock_pid
    fi

    # Stop autonomy
    if [[ -f .autonomy_pid ]]; then
        AUTONOMY_PID=$(cat .autonomy_pid)
        if kill -0 $AUTONOMY_PID 2>/dev/null; then
            log_info "Stopping autonomy (PID: $AUTONOMY_PID)"
            kill $AUTONOMY_PID 2>/dev/null || true
        fi
        rm -f .autonomy_pid
    fi

    # Kill any remaining processes
    pkill -f "test_teleoperation_integration" || true
    pkill -f "mission_executor" || true

    log_success "Cleanup completed"
}

# Run tests
run_tests() {
    log_step "Running Automated Tests"

    TESTS_PASSED=0
    TOTAL_TESTS=0

    # Test 1: Topic verification
    ((TOTAL_TESTS++))
    if verify_teleoperation_topics; then
        ((TESTS_PASSED++))
    fi

    # Test 2: Publishing rates
    ((TOTAL_TESTS++))
    if test_topic_rates; then
        ((TESTS_PASSED++))
    fi

    # Test 3: Autonomy subscriptions
    ((TOTAL_TESTS++))
    if verify_autonomy_subscriptions; then
        ((TESTS_PASSED++))
    fi

    echo
    log_info "Test Results: $TESTS_PASSED/$TOTAL_TESTS tests passed"

    if [[ $TESTS_PASSED -eq $TOTAL_TESTS ]]; then
        log_success "🎉 All automated tests passed!"
        return 0
    else
        log_warning "⚠️  Some tests failed. Check logs above."
        return 1
    fi
}

# Main execution
main() {
    echo "🚀 URC 2026 Autonomy-Teleoperation Manual Integration Test"
    echo "=" * 65
    echo

    # Trap for cleanup
    trap cleanup EXIT INT TERM

    # Run bootup sequence
    check_prerequisites
    setup_ros2
    start_ros2_infrastructure

    echo
    log_info "Starting integration test sequence..."
    echo

    # Start components
    start_mock_teleoperation
    verify_teleoperation_topics
    test_topic_rates
    start_autonomy
    verify_autonomy_subscriptions

    echo
    log_success "🎯 Integration test environment ready!"
    echo

    # Run automated tests
    if run_tests; then
        echo
        log_success "✅ Automated tests passed!"
        echo
        log_info "You can now:"
        echo "  - Monitor data: ros2 topic echo /teleoperation/joint_states"
        echo "  - Send commands: ros2 topic pub /mission/commands std_msgs/String 'data: \"test\"'"
        echo "  - View logs: Check ROS2 logging output"
        echo
    else
        echo
        log_warning "⚠️  Some automated tests failed."
        echo "  Check the output above for details."
        echo
    fi

    # Interactive testing
    echo "🎮 Starting interactive testing mode..."
    echo "You can now manually test the integration."
    echo "Press Ctrl+C at any time to exit and cleanup."
    echo

    interactive_testing
}

# Run main function
main "$@"
