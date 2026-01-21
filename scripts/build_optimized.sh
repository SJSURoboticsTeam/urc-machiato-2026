#!/bin/bash
# Optimized Build Script for URC 2026
# Provides fast incremental builds with performance monitoring

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
NC='\033[0m' # No Color

# Configuration
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_TYPE="${1:-incremental}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
COLCON_HOME="${COLCON_HOME:-$HOME/.colcon}"

# Performance tracking
BUILD_START_TIME=$(date +%s)
BUILD_LOG="$COLCON_HOME/log/latest_build.log"

# Logging functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1" | tee -a "$BUILD_LOG"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1" | tee -a "$BUILD_LOG"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1" | tee -a "$BUILD_LOG"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1" | tee -a "$BUILD_LOG"
}

log_performance() {
    echo -e "${PURPLE}[PERF]${NC} $1" | tee -a "$BUILD_LOG"
}

# Setup environment
setup_environment() {
    log_info "Setting up build environment..."

    # Source ROS2
    if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
        source "/opt/ros/$ROS_DISTRO/setup.bash"
        log_success "ROS2 $ROS_DISTRO sourced"
    else
        log_warning "ROS2 $ROS_DISTRO not found at /opt/ros/$ROS_DISTRO"
    fi

    # Create log directory
    mkdir -p "$COLCON_HOME/log"

    # Set environment variables for optimization
    export COLCON_HOME
    export CMAKE_BUILD_TYPE=Release
    export MAKEFLAGS="-j$(nproc)"

    log_success "Build environment ready"
}

# Clean build
build_clean() {
    log_info "Performing clean build..."

    cd "$PROJECT_ROOT"

    # Remove old build artifacts
    rm -rf build/ install/ log/

    # Full clean build with optimizations
    colcon build \
        --symlink-install \
        --parallel-workers $(nproc) \
        --continue-on-error \
        --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DCMAKE_CXX_FLAGS="-O3 -march=native -DNDEBUG" \
            -DCMAKE_C_FLAGS="-O3 -march=native -DNDEBUG"

    local result=$?
    if [ $result -eq 0 ]; then
        log_success "Clean build completed"
    else
        log_error "Clean build failed with exit code $result"
        return $result
    fi
}

# Incremental build
build_incremental() {
    log_info "Performing incremental build..."

    cd "$PROJECT_ROOT"

    # Check if this is first build
    if [ ! -d "install/" ]; then
        log_info "No previous build found, performing clean build"
        build_clean
        return $?
    fi

    # Incremental build with optimizations
    colcon build \
        --symlink-install \
        --parallel-workers $(nproc) \
        --continue-on-error \
        --cmake-args \
            -DCMAKE_BUILD_TYPE=Release

    local result=$?
    if [ $result -eq 0 ]; then
        log_success "Incremental build completed"
    else
        log_error "Incremental build failed with exit code $result"
        return $result
    fi
}

# Fast development build
build_fast() {
    log_info "Performing fast development build..."

    cd "$PROJECT_ROOT"

    # Build only essential packages quickly
    colcon build \
        --symlink-install \
        --parallel-workers 2 \
        --packages-up-to autonomy_navigation \
        --continue-on-error

    local result=$?
    if [ $result -eq 0 ]; then
        log_success "Fast build completed"
    else
        log_error "Fast build failed with exit code $result"
        return $result
    fi
}

# Package-specific build
build_package() {
    local package="$1"

    if [ -z "$package" ]; then
        log_error "Package name required for package-specific build"
        echo "Usage: $0 package <package_name>"
        return 1
    fi

    log_info "Building package: $package"

    cd "$PROJECT_ROOT"

    colcon build \
        --symlink-install \
        --parallel-workers $(nproc) \
        --packages-select "$package"

    local result=$?
    if [ $result -eq 0 ]; then
        log_success "Package $package built successfully"
    else
        log_error "Package $package build failed"
        return $result
    fi
}

# Show build performance metrics
show_performance() {
    local build_end_time=$(date +%s)
    local build_duration=$((build_end_time - BUILD_START_TIME))

    log_performance "Build completed in ${build_duration}s"

    # Show package build times if available
    if [ -f "$COLCON_HOME/log/latest_build.log" ]; then
        log_performance "Detailed timing in: $COLCON_HOME/log/latest_build.log"
    fi

    # Show disk usage
    local build_size=$(du -sh build/ 2>/dev/null | cut -f1)
    local install_size=$(du -sh install/ 2>/dev/null | cut -f1)

    if [ -n "$build_size" ]; then
        log_performance "Build directory size: $build_size"
    fi

    if [ -n "$install_size" ]; then
        log_performance "Install directory size: $install_size"
    fi
}

# Show usage
show_usage() {
    cat << EOF
URC 2026 Optimized Build Script

USAGE:
    $0 [build_type] [options]

BUILD TYPES:
    clean         - Full clean build (removes build/install dirs)
    incremental   - Smart incremental build (default)
    fast          - Quick development build (essential packages only)
    package       - Build specific package

OPTIONS:
    --help, -h    - Show this help message
    --verbose, -v - Verbose output
    --dry-run     - Show commands without executing

ENVIRONMENT VARIABLES:
    ROS_DISTRO    - ROS2 distribution (default: humble)
    COLCON_HOME   - Colcon cache directory (default: ~/.colcon)

EXAMPLES:
    $0                          # Incremental build
    $0 clean                    # Clean build
    $0 fast                     # Fast development build
    $0 package autonomy_navigation  # Build specific package

PERFORMANCE FEATURES:
    - Parallel compilation using all CPU cores
    - Incremental builds for faster iteration
    - Symlink installs for faster rebuilds
    - Build performance monitoring
    - Optimized compiler flags for Release builds

EOF
}

# Main execution
main() {
    # Parse arguments
    case "${1:-incremental}" in
        clean)
            setup_environment
            build_clean
            ;;
        incremental)
            setup_environment
            build_incremental
            ;;
        fast)
            setup_environment
            build_fast
            ;;
        package)
            setup_environment
            build_package "$2"
            ;;
        --help|-h)
            show_usage
            exit 0
            ;;
        *)
            log_error "Unknown build type: $1"
            show_usage
            exit 1
            ;;
    esac

    local result=$?

    # Show performance metrics
    show_performance

    # Setup environment for use
    if [ $result -eq 0 ] && [ -f "$PROJECT_ROOT/install/setup.bash" ]; then
        log_info "Build successful! Source environment with:"
        log_info "  source $PROJECT_ROOT/install/setup.bash"
    fi

    return $result
}

# Run main function
main "$@"




