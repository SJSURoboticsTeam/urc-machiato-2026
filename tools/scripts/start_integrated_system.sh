#!/bin/bash
# Start Integrated URC 2026 System
# Launches all real implementations with dashboard integration

set -e

echo "🚀 Starting URC 2026 Integrated System..."
echo "=========================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print status
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    print_error "ROS2 is not sourced. Please run 'source /opt/ros/humble/setup.bash' first"
    exit 1
fi

print_status "ROS2 distribution: $ROS_DISTRO"

# Create log directory
LOG_DIR="./logs"
mkdir -p "$LOG_DIR"

print_status "Starting system components..."

# Start Service Health Monitor (for redundancy)
print_status "Starting Service Health Monitor..."
python3 scripts/monitoring/service_health_monitor.py > "$LOG_DIR/service_health_monitor.log" 2>&1 &
HEALTH_MONITOR_PID=$!
print_success "Service Health Monitor started (PID: $HEALTH_MONITOR_PID)"

# Start Communication Redundancy Manager
print_status "Starting Communication Redundancy Manager..."
python3 src/bridges/communication_redundancy_manager.py > "$LOG_DIR/communication_redundancy_manager.log" 2>&1 &
REDUNDANCY_MANAGER_PID=$!
print_success "Communication Redundancy Manager started (PID: $REDUNDANCY_MANAGER_PID)"

# Start System Monitor
print_status "Starting System Monitor..."
python3 scripts/monitoring/system_monitor.py > "$LOG_DIR/system_monitor.log" 2>&1 &
SYSTEM_MONITOR_PID=$!
print_success "System Monitor started (PID: $SYSTEM_MONITOR_PID)"

# Start ROS2 State Machine Bridge
print_status "Starting ROS2 State Machine Bridge..."
python3 src/bridges/ros2_state_machine_bridge.py > "$LOG_DIR/ros2_state_machine_bridge.log" 2>&1 &
STATE_MACHINE_PID=$!
print_success "ROS2 State Machine Bridge started (PID: $STATE_MACHINE_PID)"

# Wait a moment for ROS2 bridge to initialize
sleep 2

# Start ROS2 Mission Control Bridge
print_status "Starting ROS2 Mission Control Bridge..."
python3 src/bridges/ros2_mission_bridge.py > "$LOG_DIR/ros2_mission_bridge.log" 2>&1 &
MISSION_BRIDGE_PID=$!
print_success "ROS2 Mission Control Bridge started (PID: $MISSION_BRIDGE_PID)"

# Wait for ROS2 bridges to be ready
sleep 3

# Start Communication Bridge
print_status "Starting Communication Bridge..."
python3 src/bridges/communication_bridge.py > "$LOG_DIR/communication_bridge.log" 2>&1 &
COMMUNICATION_BRIDGE_PID=$!
print_success "Communication Bridge started (PID: $COMMUNICATION_BRIDGE_PID)"

# Wait for all bridges to initialize
sleep 5

# Start the web dashboard
print_status "Starting Web Dashboard..."
cd frontend
npm run dev > "../$LOG_DIR/dashboard.log" 2>&1 &
DASHBOARD_PID=$!
cd ..
print_success "Web Dashboard started (PID: $DASHBOARD_PID)"

echo ""
print_success "🎉 All system components started successfully!"
echo ""
echo "🌐 Dashboard URL: http://localhost:5173"
echo "📊 Simulation Bridge: ws://localhost:8766"
echo "🔧 ROS2 State Machine: /state_machine/* topics"
echo "🎯 ROS2 Mission Control: /mission/* topics"
echo ""
echo "📁 Log files:"
echo "  - ROS2 State Machine: $LOG_DIR/ros2_state_machine_bridge.log"
echo "  - ROS2 Mission Bridge: $LOG_DIR/ros2_mission_bridge.log"
echo "  - Communication Bridge: $LOG_DIR/communication_bridge.log"
echo "  - Dashboard: $LOG_DIR/dashboard.log"
echo ""
print_warning "Press Ctrl+C to stop all services"

# Function to cleanup on exit
cleanup() {
    echo ""
    print_status "Shutting down system components..."

    # Kill all background processes
    if [ ! -z "$HEALTH_MONITOR_PID" ]; then
        kill $HEALTH_MONITOR_PID 2>/dev/null && print_success "Service Health Monitor stopped" || print_warning "Service Health Monitor already stopped"
    fi

    if [ ! -z "$REDUNDANCY_MANAGER_PID" ]; then
        kill $REDUNDANCY_MANAGER_PID 2>/dev/null && print_success "Communication Redundancy Manager stopped" || print_warning "Communication Redundancy Manager already stopped"
    fi

    if [ ! -z "$SYSTEM_MONITOR_PID" ]; then
        kill $SYSTEM_MONITOR_PID 2>/dev/null && print_success "System Monitor stopped" || print_warning "System Monitor already stopped"
    fi

    if [ ! -z "$STATE_MACHINE_PID" ]; then
        kill $STATE_MACHINE_PID 2>/dev/null && print_success "ROS2 State Machine Bridge stopped" || print_warning "ROS2 State Machine Bridge already stopped"
    fi

    if [ ! -z "$MISSION_BRIDGE_PID" ]; then
        kill $MISSION_BRIDGE_PID 2>/dev/null && print_success "ROS2 Mission Bridge stopped" || print_warning "ROS2 Mission Bridge already stopped"
    fi

    if [ ! -z "$COMMUNICATION_BRIDGE_PID" ]; then
        kill $COMMUNICATION_BRIDGE_PID 2>/dev/null && print_success "Communication Bridge stopped" || print_warning "Communication Bridge already stopped"
    fi

    if [ ! -z "$DASHBOARD_PID" ]; then
        kill $DASHBOARD_PID 2>/dev/null && print_success "Dashboard stopped" || print_warning "Dashboard already stopped"
    fi

    print_success "System shutdown complete"
    exit 0
}

# Set trap for cleanup on exit
trap cleanup SIGINT SIGTERM

# Wait for user interrupt
wait
