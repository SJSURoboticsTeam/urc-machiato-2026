#!/bin/bash
# Convenience script for running Isaac Sim with Docker
# 
# Usage:
#   ./run_isaac_sim_docker.sh [mode] [options]
#
# Modes:
#   gui       - Run Isaac Sim with GUI (requires X11)
#   headless  - Run Isaac Sim in headless mode with VNC
#   dev       - Development mode with full URC workspace
#   test      - Run tests in headless mode
#   fullstack - Run complete stack with ROS2 bridge
#
# Options:
#   -b, --build     - Rebuild images before running
#   -d, --detach    - Run in background
#   -r, --remove    - Remove container after exit
#
# Examples:
#   ./run_isaac_sim_docker.sh gui
#   ./run_isaac_sim_docker.sh headless -d
#   ./run_isaac_sim_docker.sh fullstack -b
#   ./run_isaac_sim_docker.sh dev

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Logging
log_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
log_success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARNING]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
COMPOSE_FILE="$PROJECT_ROOT/docker/docker-compose.isaac-sim.yml"

# Check if docker-compose file exists
if [ ! -f "$COMPOSE_FILE" ]; then
    log_error "Docker compose file not found: $COMPOSE_FILE"
    exit 1
fi

# Default values
MODE=""
BUILD=false
DETACH=false
REMOVE=false

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        gui|headless|dev|test|fullstack)
            MODE="$1"
            shift
            ;;
        -b|--build)
            BUILD=true
            shift
            ;;
        -d|--detach)
            DETACH=true
            shift
            ;;
        -r|--remove)
            REMOVE=true
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [mode] [options]"
            echo ""
            echo "Modes:"
            echo "  gui       - Run Isaac Sim with GUI (requires X11)"
            echo "  headless  - Run Isaac Sim in headless mode with VNC"
            echo "  dev       - Development mode with full URC workspace"
            echo "  test      - Run tests in headless mode"
            echo "  fullstack - Run complete stack with ROS2 bridge"
            echo ""
            echo "Options:"
            echo "  -b, --build     - Rebuild images before running"
            echo "  -d, --detach    - Run in background"
            echo "  -r, --remove    - Remove container after exit"
            echo "  -h, --help      - Show this help"
            echo ""
            echo "Examples:"
            echo "  $0 gui"
            echo "  $0 headless -d"
            echo "  $0 fullstack -b"
            echo "  $0 dev"
            exit 0
            ;;
        *)
            log_error "Unknown option: $1"
            echo "Use -h or --help for usage information"
            exit 1
            ;;
    esac
done

# Validate mode
if [ -z "$MODE" ]; then
    log_error "No mode specified"
    echo "Usage: $0 [gui|headless|dev|test|fullstack] [options]"
    exit 1
fi

# Check prerequisites
log_info "Checking prerequisites..."

# Check Docker
if ! command -v docker &> /dev/null; then
    log_error "Docker not found. Please install Docker first."
    exit 1
fi

# Check Docker Compose
if ! docker compose version &> /dev/null && ! docker-compose version &> /dev/null; then
    log_error "Docker Compose not found. Please install Docker Compose."
    exit 1
fi

# Use docker compose (new) or docker-compose (old)
if docker compose version &> /dev/null; then
    COMPOSE_CMD="docker compose"
else
    COMPOSE_CMD="docker-compose"
fi

# Check NVIDIA runtime for GPU modes
if [ "$MODE" != "test" ]; then
    if ! docker info | grep -q "nvidia"; then
        log_warn "NVIDIA runtime not detected. GPU acceleration may not work."
        log_info "To enable GPU support, install nvidia-docker2"
    fi
fi

# Setup X11 for GUI mode
if [ "$MODE" = "gui" ] || [ "$MODE" = "dev" ]; then
    # Allow X11 forwarding
    if [ -n "$DISPLAY" ]; then
        log_info "Setting up X11 access..."
        xhost +local:docker 2>/dev/null || true
    else
        log_warn "DISPLAY not set. GUI may not work properly."
    fi
fi

# Build if requested
if [ "$BUILD" = true ]; then
    log_info "Building Docker images..."
    
    case $MODE in
        gui|dev)
            $COMPOSE_CMD -f "$COMPOSE_FILE" build isaac-sim-gui
            ;;
        headless|test)
            $COMPOSE_CMD -f "$COMPOSE_FILE" build isaac-sim-headless
            ;;
        fullstack)
            $COMPOSE_CMD -f "$COMPOSE_FILE" build
            ;;
    esac
    
    log_success "Build completed"
fi

# Prepare compose command
COMPOSE_ARGS="-f $COMPOSE_FILE"

# Add profile
COMPOSE_ARGS="$COMPOSE_ARGS --profile $MODE"

# Prepare docker compose arguments
UP_ARGS=""
[ "$DETACH" = true ] && UP_ARGS="$UP_ARGS -d"
[ "$REMOVE" = true ] && UP_ARGS="$UP_ARGS --rm"

# Run
log_info "Starting Isaac Sim in $MODE mode..."
echo ""

case $MODE in
    gui)
        log_info "GUI Mode - Requires X11 display"
        log_info "Access: Local display"
        echo ""
        $COMPOSE_CMD $COMPOSE_ARGS up $UP_ARGS isaac-sim-gui
        ;;
        
    headless)
        log_info "Headless Mode with VNC"
        log_info "VNC Access: vnc://localhost:5900"
        log_info "Web VNC: http://localhost:6080/vnc.html"
        echo ""
        $COMPOSE_CMD $COMPOSE_ARGS up $UP_ARGS isaac-sim-headless
        ;;
        
    dev)
        log_info "Development Mode"
        log_info "Mounting: $PROJECT_ROOT"
        log_info "Access: Interactive bash shell"
        echo ""
        $COMPOSE_CMD $COMPOSE_ARGS up $UP_ARGS isaac-sim-dev
        ;;
        
    test)
        log_info "Test Mode"
        log_info "Running integration tests..."
        echo ""
        $COMPOSE_CMD $COMPOSE_ARGS up $UP_ARGS isaac-sim-test
        TEST_EXIT=$?
        
        # Copy test results
        if [ -d "$PROJECT_ROOT/test_results" ]; then
            log_info "Test results available in: $PROJECT_ROOT/test_results"
        fi
        
        exit $TEST_EXIT
        ;;
        
    fullstack)
        log_info "Full Stack Mode"
        log_info "Starting: Isaac Sim + ROS2 Bridge + Isaac ROS + Dashboard"
        log_info ""
        log_info "Services:"
        log_info "  - Isaac Sim (GUI): http://localhost:8080"
        log_info "  - Dashboard: http://localhost"
        log_info "  - ROS2 Bridge: Port 7400-7500 (DDS)"
        echo ""
        $COMPOSE_CMD $COMPOSE_ARGS up $UP_ARGS
        ;;
esac

# Capture exit code
EXIT_CODE=$?

# Cleanup X11 permissions
if [ "$MODE" = "gui" ] || [ "$MODE" = "dev" ]; then
    xhost -local:docker 2>/dev/null || true
fi

if [ $EXIT_CODE -eq 0 ]; then
    log_success "Isaac Sim completed successfully"
else
    log_error "Isaac Sim exited with code $EXIT_CODE"
fi

exit $EXIT_CODE
