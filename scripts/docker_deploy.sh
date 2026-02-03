#!/bin/bash
# Unified Docker Deployment Script for URC 2026
# Usage: ./docker_deploy.sh [dev|test|prod|sim] [additional_options]

set -e

# ==================================================
# SCRIPT CONFIGURATION
# ==================================================
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
DOCKER_DIR="$PROJECT_ROOT/docker"
UNIFIED_COMPOSE="$DOCKER_DIR/docker-compose.unified.yml"
ENV_FILE="$PROJECT_ROOT/.env.unified"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# ==================================================
# HELPER FUNCTIONS
# ==================================================
log() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

error() {
    echo -e "${RED}[ERROR]${NC} $1"
    exit 1
}

debug() {
    if [[ "${DEBUG:-false}" == "true" ]]; then
        echo -e "${BLUE}[DEBUG]${NC} $1"
    fi
}

# ==================================================
# ENVIRONMENT VALIDATION
# ==================================================
validate_environment() {
    local env=$1
    
    case $env in
        dev|development)
            export URC_ENV=development
            ;;
        test|testing)
            export URC_ENV=testing
            ;;
        prod|production)
            export URC_ENV=production
            ;;
        sim|simulation)
            export URC_ENV=simulation
            ;;
        *)
            error "Invalid environment: $env. Use: dev, test, prod, or sim"
            ;;
    esac
    
    log "Environment set to: $URC_ENV"
    
    # Validate required files
    [[ -f "$UNIFIED_COMPOSE" ]] || error "Unified compose file not found: $UNIFIED_COMPOSE"
    [[ -f "$ENV_FILE" ]] || error "Environment file not found: $ENV_FILE"
    
    # Load environment variables
    set -a
    source "$ENV_FILE"
    set +a
    
    debug "Environment variables loaded"
}

# ==================================================
# DOCKER VALIDATION
# ==================================================
validate_docker() {
    log "Validating Docker installation..."
    
    command -v docker >/dev/null 2>&1 || error "Docker is not installed or not in PATH"
    command -v docker >/dev/null 2>&1 || error "Docker is not installed or not in PATH"
    docker compose version >/dev/null 2>&1 || error "Docker Compose is not installed or not in PATH"
    
    # Check if Docker daemon is running
    if ! docker info >/dev/null 2>&1; then
        error "Docker daemon is not running or user doesn't have permissions"
    fi
    
    log "Docker validation passed"
}

# ==================================================
# BUILD FUNCTIONS
# ==================================================
build_images() {
    local target=$1
    log "Building Docker images for target: $target"
    
    cd "$PROJECT_ROOT"
    
    case $target in
        test)
            docker build -f docker/Dockerfile.unified --target test-runner -t urc2026:test .
            docker build -f docker/Dockerfile.unified --target simulation-test -t urc2026:sim-test .
            ;;
        sim)
            docker build -f docker/Dockerfile.unified --target simulation-service -t urc2026:simulation .
            docker build -f docker/Dockerfile.unified --target sensor-simulator-service -t urc2026:sensor-sim .
            ;;
        dev)
            docker build -f docker/Dockerfile.unified --target config-service -t urc2026:config .
            docker build -f docker/Dockerfile.unified --target websocket-bridge-service -t urc2026:websocket .
            ;;
        prod)
            docker build -f docker/Dockerfile.unified --target full-system -t urc2026:prod .
            ;;
        all)
            docker build -f docker/Dockerfile.unified --target test-runner -t urc2026:test .
            docker build -f docker/Dockerfile.unified --target simulation-service -t urc2026:simulation .
            docker build -f docker/Dockerfile.unified --target full-system -t urc2026:prod .
            ;;
    esac
    
    log "Image build completed"
}

# ==================================================
# DEPLOYMENT FUNCTIONS
# ==================================================
deploy_environment() {
    local env=$1
    local profile=$2
    
    log "Deploying $env environment with profile: $profile"
    
    cd "$PROJECT_ROOT"
    
    # Create necessary directories
    mkdir -p test_results test_reports
    
    # Deploy using unified compose
    docker compose -f "$UNIFIED_COMPOSE" --env-file "$ENV_FILE" \
        --project-name "urc2026-$env" \
        --profile "$profile" \
        up -d
    
    log "Deployment completed for $env"
}

# ==================================================
# TESTING FUNCTIONS
# ==================================================
run_tests() {
    local test_type=$1
    local parallel=${2:-4}
    
    log "Running tests: $test_type (parallel workers: $parallel)"
    
    # Set test environment variables
    export TEST_TYPE="$test_type"
    export TEST_PARALLEL_WORKERS="$parallel"
    
    # Run tests using unified compose
    docker compose -f "$UNIFIED_COMPOSE" --env-file "$ENV_FILE" \
        --project-name "urc2026-test" \
        --profile test \
        up test-runner
    
    log "Tests completed"
}

run_performance_tests() {
    log "Running performance tests..."
    
    docker compose -f "$UNIFIED_COMPOSE" --env-file "$ENV_FILE" \
        --project-name "urc2026-test" \
        --profile perf \
        up performance-tester
    
    log "Performance tests completed"
}

# ==================================================
# SIMULATION FUNCTIONS
# ==================================================
run_simulation() {
    local world=$1
    local robot=$2
    
    log "Starting simulation with world: $world, robot: $robot"
    
    # Set simulation environment variables
    export SIMULATION_WORLD="${world:-urc_mars_world}"
    export SIMULATION_ROBOT="${robot:-urc_rover}"
    
    # Deploy simulation services
    deploy_environment "sim" "sim"
    
    log "Simulation started"
}

# ==================================================
# MONITORING FUNCTIONS
# ==================================================
show_status() {
    local env=$1
    log "Showing status for environment: $env"
    
    docker compose -f "$UNIFIED_COMPOSE" --env-file "$ENV_FILE" \
        --project-name "urc2026-$env" \
        ps
    
    log "Showing container logs (last 20 lines)..."
    docker compose -f "$UNIFIED_COMPOSE" --env-file "$ENV_FILE" \
        --project-name "urc2026-$env" \
        logs --tail=20
}

cleanup() {
    local env=$1
    log "Cleaning up environment: $env"
    
    docker compose -f "$UNIFIED_COMPOSE" --env-file "$ENV_FILE" \
        --project-name "urc2026-$env" \
        down -v --remove-orphans
    
    # Remove test artifacts
    if [[ "$env" == "test" ]]; then
        rm -rf test_results test_reports
    fi
    
    log "Cleanup completed"
}

# ==================================================
# HEALTH CHECK FUNCTIONS
# ==================================================
health_check() {
    local env=$1
    log "Performing health check for environment: $env"
    
    # Check if containers are running
    local containers=$(    docker compose -f "$UNIFIED_COMPOSE" --env-file "$ENV_FILE" \
        --project-name "urc2026-$env" \
        ps -q | wc -l)
    
    if [[ $containers -eq 0 ]]; then
        error "No containers running for environment: $env"
    fi
    
    # Check container health
    local unhealthy=$(docker compose -f "$UNIFIED_COMPOSE" --env-file "$ENV_FILE" \
        --project-name "urc2026-$env" \
        ps | grep "unhealthy" | wc -l)
    
    if [[ $unhealthy -gt 0 ]]; then
        warn "$unhealthy containers are unhealthy"
    else
        log "All containers are healthy"
    fi
    
    log "Health check completed"
}

# ==================================================
# MAIN EXECUTION
# ==================================================
main() {
    local command=${1:-help}
    local environment=${2:-dev}
    local option1=${3:-}
    local option2=${4:-}
    
    log "URC 2026 Unified Docker Deployment"
    log "Command: $command, Environment: $environment"
    
    case $command in
        deploy)
            validate_environment "$environment"
            validate_docker
            build_images "$environment"
            deploy_environment "$environment" "$environment"
            ;;
        test)
            validate_environment "testing"
            validate_docker
            build_images "test"
            run_tests "$option1" "$option2"
            ;;
        perf)
            validate_environment "testing"
            validate_docker
            build_images "test"
            run_performance_tests
            ;;
        sim)
            validate_environment "simulation"
            validate_docker
            build_images "sim"
            run_simulation "$option1" "$option2"
            ;;
        build)
            validate_environment "$environment"
            validate_docker
            build_images "$environment"
            ;;
        status)
            validate_environment "$environment"
            show_status "$environment"
            ;;
        health)
            validate_environment "$environment"
            health_check "$environment"
            ;;
        cleanup)
            validate_environment "$environment"
            cleanup "$environment"
            ;;
        logs)
            validate_environment "$environment"
            docker compose -f "$UNIFIED_COMPOSE" --env-file "$ENV_FILE" \
                --project-name "urc2026-$environment" logs -f
            ;;
        shell)
            validate_environment "$environment"
            local container="urc2026-$environment-${option1:-config}"
            docker exec -it "$container" /bin/bash
            ;;
        help|*)
            cat << EOF
URC 2026 Unified Docker Deployment Script

Usage: $0 [command] [environment] [options]

Commands:
    deploy [env]     Deploy full environment
    test [type] [n]  Run tests (unit/integration/performance/all)
    perf             Run performance tests
    sim [world] [robot]  Start simulation environment
    build [env]      Build Docker images
    status [env]     Show deployment status
    health [env]     Perform health check
    cleanup [env]    Clean up environment
    logs [env]       Show container logs
    shell [env] [container]  Open shell in container
    help             Show this help

Environments:
    dev, development   Development environment
    test, testing     Testing environment
    prod, production  Production environment
    sim, simulation   Simulation environment

Examples:
    $0 deploy dev                    # Deploy development environment
    $0 test unit 8                    # Run unit tests with 8 parallel workers
    $0 test all                      # Run all tests
    $0 sim mars_world advanced_rover  # Start simulation with custom world/robot
    $0 status dev                     # Show development environment status
    $0 cleanup test                   # Clean up test environment
    $0 shell dev config               # Open shell in config service container

Environment variables:
    DEBUG=true                        # Enable debug output
    URC_ENV=environment               # Override environment
    TEST_TYPE=test_type               # Override test type
    SIMULATION_WORLD=world_name       # Override simulation world

EOF
            ;;
    esac
}

# ==================================================
# SCRIPT ENTRY POINT
# ==================================================
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi