#!/bin/bash
# =============================================================================
# URC 2026 Unified Build System
# =============================================================================
#
# Single entry point for all build operations.
#
# Usage:
#   ./scripts/build.sh [mode] [options]
#
# Modes:
#   dev       - Development build (default, fast)
#   prod      - Production build (optimized)
#   comp      - Competition build (safety-first)
#   clean     - Clean build artifacts
#
# Options:
#   --test    - Run tests after build
#   --deploy  - Deploy after successful build
#   --no-ros  - Skip ROS2 packages (Python only)
#   --verbose - Verbose output
#   --help    - Show this help
#
# Examples:
#   ./scripts/build.sh dev --test
#   ./scripts/build.sh prod --deploy
#   ./scripts/build.sh comp --test --verbose
#
# Author: URC 2026 Build Team
# =============================================================================

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILD_DIR="$PROJECT_ROOT/build"
INSTALL_DIR="$PROJECT_ROOT/install"
LOG_DIR="$PROJECT_ROOT/log"

# Default options
MODE="dev"
RUN_TESTS=false
DEPLOY=false
SKIP_ROS=false
VERBOSE=false
PARALLEL_JOBS=$(nproc)

# =============================================================================
# Helper Functions
# =============================================================================

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

show_help() {
    head -30 "$0" | tail -25
    exit 0
}

# =============================================================================
# Parse Arguments
# =============================================================================

parse_args() {
    while [[ $# -gt 0 ]]; do
        case $1 in
            dev|prod|comp|clean)
                MODE="$1"
                shift
                ;;
            --test)
                RUN_TESTS=true
                shift
                ;;
            --deploy)
                DEPLOY=true
                shift
                ;;
            --no-ros)
                SKIP_ROS=true
                shift
                ;;
            --verbose|-v)
                VERBOSE=true
                shift
                ;;
            --help|-h)
                show_help
                ;;
            *)
                log_error "Unknown option: $1"
                show_help
                ;;
        esac
    done
}

# =============================================================================
# Environment Setup
# =============================================================================

setup_environment() {
    log_info "Setting up environment for '$MODE' mode..."
    
    # Export environment based on mode
    case $MODE in
        dev)
            export URC_ENV="development"
            export CMAKE_BUILD_TYPE="Debug"
            export COLCON_BUILD_ARGS="--symlink-install"
            ;;
        prod)
            export URC_ENV="production"
            export CMAKE_BUILD_TYPE="Release"
            export COLCON_BUILD_ARGS="--cmake-args -DCMAKE_BUILD_TYPE=Release"
            ;;
        comp)
            export URC_ENV="competition"
            export CMAKE_BUILD_TYPE="Release"
            export COLCON_BUILD_ARGS="--cmake-args -DCMAKE_BUILD_TYPE=Release"
            export URC_SAFETY_LEVEL="HIGH"
            ;;
    esac
    
    # Source ROS2 if available
    if [[ -f /opt/ros/humble/setup.bash ]]; then
        source /opt/ros/humble/setup.bash
        log_info "Sourced ROS2 Humble"
    elif [[ -f /opt/ros/jazzy/setup.bash ]]; then
        source /opt/ros/jazzy/setup.bash
        log_info "Sourced ROS2 Jazzy"
    else
        log_warning "No ROS2 installation found"
        SKIP_ROS=true
    fi
    
    # Source workspace if exists
    if [[ -f "$INSTALL_DIR/setup.bash" ]]; then
        source "$INSTALL_DIR/setup.bash"
        log_info "Sourced workspace overlay"
    fi
    
    cd "$PROJECT_ROOT"
}

# =============================================================================
# Clean Build
# =============================================================================

clean_build() {
    log_info "Cleaning build artifacts..."
    
    rm -rf "$BUILD_DIR" 2>/dev/null || true
    rm -rf "$INSTALL_DIR" 2>/dev/null || true
    rm -rf "$LOG_DIR" 2>/dev/null || true
    
    # Clean Python artifacts
    find "$PROJECT_ROOT" -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
    find "$PROJECT_ROOT" -type d -name "*.egg-info" -exec rm -rf {} + 2>/dev/null || true
    find "$PROJECT_ROOT" -type f -name "*.pyc" -delete 2>/dev/null || true
    
    log_success "Build artifacts cleaned"
}

# =============================================================================
# Python Build
# =============================================================================

build_python() {
    log_info "Building Python packages..."
    
    # Install in development mode
    cd "$PROJECT_ROOT"
    
    if $VERBOSE; then
        pip install -e . -v
    else
        pip install -e . -q
    fi
    
    log_success "Python packages built"
}

# =============================================================================
# ROS2 Build
# =============================================================================

build_ros2() {
    if $SKIP_ROS; then
        log_warning "Skipping ROS2 build (--no-ros or no ROS2 found)"
        return 0
    fi
    
    log_info "Building ROS2 packages..."
    
    cd "$PROJECT_ROOT"
    
    # Canonical ROS package paths (workspace root has COLCON_IGNORE)
    COLCON_BASE_PATHS="src/autonomy/interfaces/autonomy_interfaces src/autonomy/autonomy_core src/autonomy/bt src/simulation/gazebo_simulation src/vision_processing"
    
    # Prepare colcon command
    COLCON_CMD="colcon build --base-paths $COLCON_BASE_PATHS"
    COLCON_CMD="$COLCON_CMD $COLCON_BUILD_ARGS"
    COLCON_CMD="$COLCON_CMD --parallel-workers $PARALLEL_JOBS"
    
    # Add event handlers for progress
    if ! $VERBOSE; then
        COLCON_CMD="$COLCON_CMD --event-handlers console_cohesion+"
    fi
    
    # Build specific packages for modes
    case $MODE in
        dev)
            # Build all packages with symlink install
            log_info "Building all packages (development mode)..."
            ;;
        prod|comp)
            # Build optimized
            log_info "Building optimized packages ($MODE mode)..."
            ;;
    esac
    
    # Execute build
    if $VERBOSE; then
        eval $COLCON_CMD
    else
        eval $COLCON_CMD 2>&1 | grep -E "(Starting|Finished|error|warning)" || true
    fi
    
    # Source newly built workspace
    if [[ -f "$INSTALL_DIR/setup.bash" ]]; then
        source "$INSTALL_DIR/setup.bash"
    fi
    
    log_success "ROS2 packages built"
}

# =============================================================================
# Frontend Build
# =============================================================================

build_frontend() {
    DASHBOARD_DIR="$PROJECT_ROOT/src/dashboard"
    
    if [[ ! -d "$DASHBOARD_DIR" ]]; then
        log_warning "Dashboard directory not found, skipping frontend build"
        return 0
    fi
    
    log_info "Building frontend..."
    
    cd "$DASHBOARD_DIR"
    
    # Install dependencies if needed
    if [[ ! -d "node_modules" ]]; then
        log_info "Installing npm dependencies..."
        npm install --silent
    fi
    
    # Build based on mode
    case $MODE in
        dev)
            log_info "Frontend in dev mode - skipping build (use npm run dev)"
            ;;
        prod|comp)
            npm run build --silent
            log_success "Frontend built"
            ;;
    esac
    
    cd "$PROJECT_ROOT"
}

# =============================================================================
# Run Tests
# =============================================================================

run_tests() {
    log_info "Running tests..."
    
    cd "$PROJECT_ROOT"
    
    # Run pytest
    if $VERBOSE; then
        python3 -m pytest tests/ -v --tb=short
    else
        python3 -m pytest tests/unit/ -v --tb=line -q
    fi
    
    log_success "Tests passed"
}

# =============================================================================
# Deploy
# =============================================================================

deploy() {
    log_info "Deploying ($MODE mode)..."
    
    case $MODE in
        dev)
            log_info "Development deployment - no action needed"
            ;;
        prod)
            log_info "Production deployment..."
            # Add production deployment logic here
            ;;
        comp)
            log_info "Competition deployment..."
            # Run pre-competition checklist
            if [[ -f "$SCRIPT_DIR/competition/pre_competition_checklist.py" ]]; then
                python3 "$SCRIPT_DIR/competition/pre_competition_checklist.py" --quick
            fi
            ;;
    esac
    
    log_success "Deployment complete"
}

# =============================================================================
# Main
# =============================================================================

main() {
    echo ""
    echo "============================================"
    echo "  URC 2026 Unified Build System"
    echo "============================================"
    echo ""
    
    parse_args "$@"
    
    # Handle clean mode
    if [[ "$MODE" == "clean" ]]; then
        clean_build
        exit 0
    fi
    
    # Record start time
    START_TIME=$(date +%s)
    
    # Execute build pipeline
    setup_environment
    build_python
    build_ros2
    
    # Optional frontend build (only for prod/comp)
    if [[ "$MODE" != "dev" ]]; then
        build_frontend
    fi
    
    # Optional test run
    if $RUN_TESTS; then
        run_tests
    fi
    
    # Optional deploy
    if $DEPLOY; then
        deploy
    fi
    
    # Calculate elapsed time
    END_TIME=$(date +%s)
    ELAPSED=$((END_TIME - START_TIME))
    
    echo ""
    echo "============================================"
    log_success "Build completed in ${ELAPSED}s"
    echo "  Mode: $MODE"
    echo "  Tests: $RUN_TESTS"
    echo "  Deploy: $DEPLOY"
    echo "============================================"
    echo ""
}

# Run main
main "$@"
