#!/bin/bash
# Docker Test Runner Script - URC 2026
# Best practices: Consistent testing environment, isolated execution
# Usage: ./docker/run_tests_in_docker.sh [test-type]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Test type selection
TEST_TYPE="${1:-all}"

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}URC 2026 Docker Test Environment${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Create output directories
mkdir -p "$PROJECT_ROOT/test_results"
mkdir -p "$PROJECT_ROOT/test_reports"

# Function to run tests
run_tests() {
    local profile=$1
    local description=$2
    
    echo -e "${YELLOW}Running: $description${NC}"
    echo -e "${YELLOW}Profile: $profile${NC}"
    echo ""
    
    cd "$SCRIPT_DIR"
    
    # Build images if needed
    echo -e "${BLUE}Building test images...${NC}"
    docker-compose -f docker-compose.test.yml build --quiet
    
    # Run tests
    echo -e "${BLUE}Starting test execution...${NC}"
    if docker-compose -f docker-compose.test.yml --profile "$profile" up --abort-on-container-exit --exit-code-from "$(get_service_name $profile)"; then
        echo -e "${GREEN}✅ $description passed!${NC}"
        return 0
    else
        echo -e "${RED}❌ $description failed!${NC}"
        return 1
    fi
}

# Get service name from profile
get_service_name() {
    case $1 in
        unit) echo "unit-tests" ;;
        integration) echo "integration-tests" ;;
        simulation) echo "simulation-tests" ;;
        performance) echo "performance-tests" ;;
        all) echo "all-tests" ;;
        *) echo "all-tests" ;;
    esac
}

# Main execution
case $TEST_TYPE in
    unit)
        run_tests "unit" "Unit Tests"
        ;;
    integration)
        run_tests "integration" "Integration Tests"
        ;;
    simulation)
        run_tests "simulation" "Simulation Tests"
        ;;
    performance)
        run_tests "performance" "Performance Tests"
        ;;
    all)
        echo -e "${BLUE}Running complete test suite...${NC}"
        echo ""
        
        # Run in sequence for better error reporting
        FAILED=0
        
        echo -e "${YELLOW}=== Unit Tests ===${NC}"
        if ! run_tests "unit" "Unit Tests"; then
            FAILED=1
        fi
        
        echo ""
        echo -e "${YELLOW}=== Integration Tests ===${NC}"
        if ! run_tests "integration" "Integration Tests"; then
            FAILED=1
        fi
        
        echo ""
        echo -e "${YELLOW}=== Simulation Tests ===${NC}"
        if ! run_tests "simulation" "Simulation Tests"; then
            FAILED=1
        fi
        
        echo ""
        echo -e "${YELLOW}=== Performance Tests ===${NC}"
        if ! run_tests "performance" "Performance Tests"; then
            FAILED=1
        fi
        
        echo ""
        echo -e "${BLUE}========================================${NC}"
        if [ $FAILED -eq 0 ]; then
            echo -e "${GREEN}✅ All tests passed!${NC}"
            echo ""
            echo -e "${BLUE}Test results:${NC}"
            echo "  - JUnit XML: $PROJECT_ROOT/test_results/"
            echo "  - Coverage: $PROJECT_ROOT/test_reports/"
            exit 0
        else
            echo -e "${RED}❌ Some tests failed!${NC}"
            echo ""
            echo -e "${BLUE}Check logs:${NC}"
            echo "  docker-compose -f docker/docker-compose.test.yml logs"
            exit 1
        fi
        ;;
    *)
        echo -e "${RED}Unknown test type: $TEST_TYPE${NC}"
        echo ""
        echo "Usage: $0 [unit|integration|simulation|performance|all]"
        echo ""
        echo "Test types:"
        echo "  unit         - Run unit tests only"
        echo "  integration  - Run integration tests"
        echo "  simulation   - Run simulation tests (requires Gazebo)"
        echo "  performance  - Run performance benchmarks"
        echo "  all          - Run complete test suite (default)"
        exit 1
        ;;
esac
