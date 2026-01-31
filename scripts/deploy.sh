#!/bin/bash
# =============================================================================
# URC 2026 Deployment Script
# =============================================================================
#
# Handles deployment validation and system startup.
#
# Usage:
#   ./scripts/deploy.sh [mode]
#
# Modes:
#   simulation  - Deploy in simulation mode (default)
#   hardware    - Deploy with real hardware
#   competition - Deploy in competition mode
#
# Author: URC 2026 Deployment Team
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

MODE="${1:-simulation}"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
log_success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
log_warning() { echo -e "${YELLOW}[WARNING]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# =============================================================================
# Environment Setup
# =============================================================================

setup_environment() {
    log_info "Setting up environment for $MODE mode..."
    
    export URC_ENV="$MODE"
    
    # Source ROS2
    if [[ -f /opt/ros/humble/setup.bash ]]; then
        source /opt/ros/humble/setup.bash
    elif [[ -f /opt/ros/jazzy/setup.bash ]]; then
        source /opt/ros/jazzy/setup.bash
    fi
    
    # Source workspace
    if [[ -f "$PROJECT_ROOT/install/setup.bash" ]]; then
        source "$PROJECT_ROOT/install/setup.bash"
    fi
    
    # Simulation-specific setup
    if [[ "$MODE" == "simulation" ]]; then
        export GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH:$PROJECT_ROOT/src/simulation/models"
        export GAZEBO_RESOURCE_PATH="$GAZEBO_RESOURCE_PATH:$PROJECT_ROOT/src/simulation"
    fi
}

# =============================================================================
# Pre-deployment Checks
# =============================================================================

pre_deployment_checks() {
    log_info "Running pre-deployment checks..."
    
    CHECKS_PASSED=true
    
    # Check Python dependencies
    if ! python3 -c "import rclpy" 2>/dev/null; then
        log_warning "rclpy not available - ROS2 features limited"
    fi
    
    # Check config system
    if python3 -c "from src.infrastructure.config import get_urc_config; get_urc_config()" 2>/dev/null; then
        log_success "Configuration system: OK"
    else
        log_warning "Configuration system: Issues detected (may use fallback)"
    fi
    
    # Check workspace build
    if [[ -d "$PROJECT_ROOT/install" ]]; then
        log_success "Workspace build: OK"
    else
        log_warning "Workspace not built - run ./scripts/build.sh first"
    fi
    
    # Competition-specific checks
    if [[ "$MODE" == "competition" ]]; then
        log_info "Running competition pre-checks..."
        
        # Check safety systems
        if python3 -c "from src.infrastructure.config import get_urc_config; c = get_urc_config(); assert c.safety.emergency_stop_enabled" 2>/dev/null; then
            log_success "Safety system: Emergency stop enabled"
        else
            log_error "Safety system: Emergency stop MUST be enabled for competition"
            CHECKS_PASSED=false
        fi
    fi
    
    if ! $CHECKS_PASSED; then
        log_error "Pre-deployment checks failed!"
        exit 1
    fi
    
    log_success "Pre-deployment checks passed"
}

# =============================================================================
# System Startup
# =============================================================================

start_system() {
    log_info "Starting URC 2026 system in $MODE mode..."
    
    cd "$PROJECT_ROOT"
    
    case $MODE in
        simulation)
            log_info "Launching simulation..."
            # Start simulation launch file
            if command -v ros2 &> /dev/null; then
                ros2 launch autonomy_core unified.launch.py mode:=simulation &
            else
                log_warning "ros2 command not found, starting Python components only"
                python3 -c "from src.infrastructure.config import get_urc_config; print('Simulation mode ready')"
            fi
            ;;
        hardware)
            log_info "Launching with hardware..."
            if command -v ros2 &> /dev/null; then
                ros2 launch autonomy_core unified.launch.py mode:=hardware &
            fi
            ;;
        competition)
            log_info "Launching competition mode..."
            if command -v ros2 &> /dev/null; then
                ros2 launch autonomy_core unified.launch.py mode:=competition &
            fi
            ;;
    esac
    
    log_success "System started"
}

# =============================================================================
# Health Check
# =============================================================================

health_check() {
    log_info "Running health check..."
    
    sleep 2
    
    # Check if nodes are running
    if command -v ros2 &> /dev/null; then
        NODES=$(ros2 node list 2>/dev/null | wc -l)
        if [[ $NODES -gt 0 ]]; then
            log_success "ROS2 nodes running: $NODES"
        else
            log_warning "No ROS2 nodes detected"
        fi
        
        # Check topics
        TOPICS=$(ros2 topic list 2>/dev/null | wc -l)
        log_info "Active topics: $TOPICS"
    fi
    
    log_success "Health check complete"
}

# =============================================================================
# Main
# =============================================================================

main() {
    echo ""
    echo "============================================"
    echo "  URC 2026 Deployment"
    echo "  Mode: $MODE"
    echo "============================================"
    echo ""
    
    setup_environment
    pre_deployment_checks
    start_system
    health_check
    
    echo ""
    log_success "Deployment complete!"
    echo ""
}

main "$@"
