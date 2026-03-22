#!/bin/bash
#
# Quick Start Script for Integrated Testing Dashboard
#
# This script launches all necessary components for visual testing:
# 1. Test Dashboard Backend (WebSocket server)
# 2. Frontend Development Server
# 3. Optional: ROS2 system for full integration
#

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Project root (this file: <repo>/scripts/testing/start_dashboard.sh)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

if [ -d "$PROJECT_ROOT/services/dashboard" ]; then
    FRONTEND_DIR="$PROJECT_ROOT/services/dashboard"
elif [ -d "$PROJECT_ROOT/src/frontend" ]; then
    FRONTEND_DIR="$PROJECT_ROOT/src/frontend"
else
    echo "ERROR: No frontend found (expected services/dashboard or src/frontend)"
    exit 1
fi

# Print colored message
print_msg() {
    local color=$1
    shift
    echo -e "${color}$@${NC}"
}

print_msg $BLUE "═══════════════════════════════════════════════════════"
print_msg $BLUE "   URC Machiato 2026 - Integrated Testing Dashboard"
print_msg $BLUE "═══════════════════════════════════════════════════════"
echo ""

# Check if running with ROS2
ROS2_MODE=false
if [ "$1" == "--ros2" ]; then
    ROS2_MODE=true
    print_msg $YELLOW "Running with full ROS2 integration"
fi

# Function to check if port is in use
check_port() {
    local port=$1
    if lsof -Pi :$port -sTCP:LISTEN -t >/dev/null 2>&1; then
        return 0  # Port is in use
    else
        return 1  # Port is free
    fi
}

# Function to kill process on port
kill_port() {
    local port=$1
    if check_port $port; then
        print_msg $YELLOW "Killing process on port $port..."
        lsof -ti:$port | xargs kill -9 2>/dev/null || true
        sleep 1
    fi
}

# Cleanup function
cleanup() {
    print_msg $YELLOW "\nShutting down services..."

    # Kill background jobs
    jobs -p | xargs kill 2>/dev/null || true

    # Kill specific ports
    kill_port 8766  # Backend
    kill_port 5173  # Frontend

    if [ "$ROS2_MODE" = true ]; then
        print_msg $YELLOW "Stopping ROS2 nodes..."
        pkill -f "ros2 launch" || true
        pkill -f "ros2_state_machine_bridge" || true
    fi

    print_msg $GREEN "Cleanup complete"
    exit 0
}

# Set up cleanup on exit
trap cleanup SIGINT SIGTERM EXIT

# Check dependencies
print_msg $BLUE "Checking dependencies..."

# Check Python
if ! command -v python3 &> /dev/null; then
    print_msg $RED "ERROR: Python 3 not found"
    exit 1
fi

# Check Node.js
if ! command -v node &> /dev/null; then
    print_msg $RED "ERROR: Node.js not found"
    exit 1
fi

# Check npm
if ! command -v npm &> /dev/null; then
    print_msg $RED "ERROR: npm not found"
    exit 1
fi

print_msg $GREEN "✓ All dependencies found"
echo ""

# Install Python dependencies if needed
print_msg $BLUE "Checking Python dependencies..."
if ! python3 -c "import websockets" 2>/dev/null; then
    print_msg $YELLOW "Installing websockets..."
    pip install websockets || {
        print_msg $RED "ERROR: Failed to install websockets"
        exit 1
    }
fi
print_msg $GREEN "Python dependencies ready"
echo ""

# Install frontend dependencies if needed
if [ ! -d "$FRONTEND_DIR/node_modules" ]; then
    print_msg $YELLOW "Installing frontend dependencies..."
    cd "$FRONTEND_DIR"
    npm install || {
        print_msg $RED "ERROR: Failed to install frontend dependencies"
        exit 1
    }
    cd "$PROJECT_ROOT"
fi
print_msg $GREEN "Frontend dependencies ready"
echo ""

# Clean up any existing processes on our ports
print_msg $BLUE "Checking for existing processes..."
kill_port 8766
kill_port 5173
print_msg $GREEN "✓ Ports cleared"
echo ""

# Start ROS2 system if requested
if [ "$ROS2_MODE" = true ]; then
    print_msg $BLUE "Starting ROS2 system..."

    # Check if ROS2 is available
    if ! command -v ros2 &> /dev/null; then
        print_msg $RED "ERROR: ROS2 not found. Install ROS2 or run without --ros2 flag"
        exit 1
    fi

    # Source ROS2 and workspace (same order as deploy.sh)
    if [[ -f /opt/ros/humble/setup.bash ]]; then
        # shellcheck source=/dev/null
        source /opt/ros/humble/setup.bash
    elif [[ -f /opt/ros/jazzy/setup.bash ]]; then
        # shellcheck source=/dev/null
        source /opt/ros/jazzy/setup.bash
    else
        print_msg $RED "ERROR: ROS 2 Humble or Jazzy setup.bash not found"
        exit 1
    fi
    if [[ -f "$PROJECT_ROOT/install/setup.bash" ]]; then
        # shellcheck source=/dev/null
        source "$PROJECT_ROOT/install/setup.bash"
    else
        print_msg $YELLOW "Warning: install/setup.bash not found; launch may fail"
    fi

    cd "$PROJECT_ROOT"
    ros2 launch autonomy_core unified.launch.py mode:=simulation > /tmp/ros2_launch.log 2>&1 &
    ROS2_PID=$!

    print_msg $GREEN "ROS2 system started (PID: $ROS2_PID)"
    print_msg $YELLOW "  Logs: /tmp/ros2_launch.log"
    echo ""

    # Give ROS2 time to start
    sleep 3
fi

# Start Test Dashboard Backend
print_msg $BLUE "Starting Test Dashboard Backend..."
cd "$PROJECT_ROOT"
python3 scripts/testing/test_dashboard_backend.py > /tmp/test_backend.log 2>&1 &
BACKEND_PID=$!

# Wait for backend to start
sleep 2

# Check if backend started successfully
if ! ps -p $BACKEND_PID > /dev/null; then
    print_msg $RED "ERROR: Backend failed to start. Check /tmp/test_backend.log"
    exit 1
fi

print_msg $GREEN "✓ Backend started (PID: $BACKEND_PID)"
print_msg $YELLOW "  WebSocket: ws://localhost:8766"
print_msg $YELLOW "  Logs: /tmp/test_backend.log"
echo ""

# Start Frontend
print_msg $BLUE "Starting Frontend Development Server..."
cd "$FRONTEND_DIR"
npm run dev > /tmp/frontend.log 2>&1 &
FRONTEND_PID=$!

# Wait for frontend to start
sleep 3

# Check if frontend started successfully
if ! ps -p $FRONTEND_PID > /dev/null; then
    print_msg $RED "ERROR: Frontend failed to start. Check /tmp/frontend.log"
    exit 1
fi

print_msg $GREEN "✓ Frontend started (PID: $FRONTEND_PID)"
print_msg $YELLOW "  URL: http://localhost:5173"
print_msg $YELLOW "  Logs: /tmp/frontend.log"
echo ""

# Print success message
print_msg $GREEN "═══════════════════════════════════════════════════════"
print_msg $GREEN "   All services started successfully."
print_msg $GREEN "═══════════════════════════════════════════════════════"
echo ""

print_msg $BLUE "Access the dashboard:"
print_msg $YELLOW "  1. Open http://localhost:5173 in your browser"
print_msg $YELLOW "  2. Click the 'Testing' tab"
print_msg $YELLOW "  3. Click 'Integrated Dashboard' button"
echo ""

if [ "$ROS2_MODE" = true ]; then
    print_msg $BLUE "ROS2 Integration:"
    print_msg $YELLOW "  - State Machine: Active"
    print_msg $YELLOW "  - SLAM Nodes: Active"
    print_msg $YELLOW "  - Navigation: Active"
    echo ""
fi

print_msg $BLUE "Quick Actions:"
print_msg $YELLOW "  - Run high priority tests: Click 'Run All High Priority'"
print_msg $YELLOW "  - View communication flow: Check the central visualization"
print_msg $YELLOW "  - Monitor state changes: Watch the State Machine Monitor"
print_msg $YELLOW "  - Stream CAN data: Enable 'CAN Bus Stream' test"
echo ""

print_msg $BLUE "Logs:"
print_msg $YELLOW "  - Backend: tail -f /tmp/test_backend.log"
print_msg $YELLOW "  - Frontend: tail -f /tmp/frontend.log"
if [ "$ROS2_MODE" = true ]; then
    print_msg $YELLOW "  - ROS2: tail -f /tmp/ros2_launch.log"
fi
echo ""

print_msg $RED "Press Ctrl+C to stop all services"
print_msg $BLUE "═══════════════════════════════════════════════════════"
echo ""

# Wait for user to stop
wait
