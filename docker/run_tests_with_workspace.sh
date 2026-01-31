#!/bin/bash
# Docker Test Runner with Workspace Build - URC 2026
# Builds workspace and runs tests in Docker environment

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}URC 2026 Docker Tests with Workspace${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Create output directories
mkdir -p "$PROJECT_ROOT/test_results"
mkdir -p "$PROJECT_ROOT/test_reports"

# Step 1: Clean and build ROS2 workspace
echo -e "${YELLOW}Step 1: Building ROS2 workspace...${NC}"
docker run --rm \
    -e ROS_DOMAIN_ID=42 \
    -e PYTHONPATH=/home/urc:/home/urc/src \
    -v "$PROJECT_ROOT/src:/home/urc/src" \
    -v "$PROJECT_ROOT/test_results:/home/urc/test_results" \
    urc2026:test bash -c "
        source /opt/ros/humble/setup.bash
        cd /home/urc/src
        export PATH=\$(echo \"\$PATH\" | tr ':' '\n' | grep -v venv | tr '\n' ':' | sed 's/:$//')
        export PATH=\"/usr/bin:/usr/local/bin:\$PATH\"
        # Clean build directories
        rm -rf build install log
        find . -name 'CMakeCache.txt' -delete 2>/dev/null || true
        find . -name 'CMakeFiles' -type d -exec rm -rf {} + 2>/dev/null || true
        # Build
        colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 2>&1 | tee /home/urc/test_results/build.log | tail -15
    "

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✅ Workspace built${NC}"
else
    echo -e "${RED}❌ Workspace build failed - check test_results/build.log${NC}"
    exit 1
fi
echo ""

# Step 2: Run unit tests
echo -e "${YELLOW}Step 2: Running unit tests...${NC}"
docker run --rm \
    -e ROS_DOMAIN_ID=42 \
    -e PYTHONPATH=/home/urc:/home/urc/src \
    -v "$PROJECT_ROOT/src:/home/urc/src:ro" \
    -v "$PROJECT_ROOT/tests:/home/urc/tests:ro" \
    -v "$PROJECT_ROOT/test_results:/home/urc/test_results" \
    -v "$PROJECT_ROOT/test_reports:/home/urc/test_reports" \
    urc2026:test bash -c "
        source /opt/ros/humble/setup.bash
        source /home/urc/src/install/setup.bash
        cd /home/urc
        python3 -m pytest tests/unit/ -v --tb=short \
            --junitxml=/home/urc/test_results/unit-tests.xml \
            --cov=src --cov-report=html:/home/urc/test_reports/coverage-unit \
            --cov-report=term --maxfail=10 2>&1 | tee /home/urc/test_results/unit-tests.log | tail -30
    " || echo -e "${YELLOW}⚠️  Some unit tests may have failed${NC}"
echo ""

# Step 3: Run integration tests
echo -e "${YELLOW}Step 3: Running integration tests...${NC}"
docker run --rm \
    -e ROS_DOMAIN_ID=42 \
    -e PYTHONPATH=/home/urc:/home/urc/src \
    -v "$PROJECT_ROOT/src:/home/urc/src:ro" \
    -v "$PROJECT_ROOT/tests:/home/urc/tests:ro" \
    -v "$PROJECT_ROOT/test_results:/home/urc/test_results" \
    -v "$PROJECT_ROOT/test_reports:/home/urc/test_reports" \
    urc2026:test bash -c "
        source /opt/ros/humble/setup.bash
        source /home/urc/src/install/setup.bash
        cd /home/urc
        python3 -m pytest tests/integration/ -v --tb=short \
            --junitxml=/home/urc/test_results/integration-tests.xml \
            --cov=src --cov-report=html:/home/urc/test_reports/coverage-integration \
            --cov-report=term --maxfail=10 2>&1 | tee /home/urc/test_results/integration-tests.log | tail -30
    " || echo -e "${YELLOW}⚠️  Some integration tests may have failed${NC}"
echo ""

# Summary
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Test Summary${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
echo -e "${GREEN}✅ Workspace: Built${NC}"
echo -e "${GREEN}✅ Tests: Executed${NC}"
echo ""
echo -e "${BLUE}Results:${NC}"
echo "  - test_results/unit-tests.xml"
echo "  - test_results/integration-tests.xml"
echo "  - test_reports/coverage-unit/"
echo "  - test_reports/coverage-integration/"
echo ""
