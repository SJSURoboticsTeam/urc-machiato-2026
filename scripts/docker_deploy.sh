#!/bin/bash
# =============================================================================
# URC 2026 Docker Deployment Script
# =============================================================================
#
# Usage:
#   ./scripts/docker_deploy.sh [mode]
#
# Modes:
#   dev   - Deploy using docker/docker-compose.dev.yml
#   prod  - Deploy using docker/docker-compose.prod.yml
#   test  - Deploy using docker/docker-compose.test.yml
#
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

MODE="${1:-dev}"

COMPOSE_FILE="docker/docker-compose.${MODE}.yml"

cd "$PROJECT_ROOT"

if [[ ! -f "$COMPOSE_FILE" ]]; then
    echo "Error: Compose file $COMPOSE_FILE does not exist."
    exit 1
fi

echo "Deploying URC 2026 with Docker (Mode: $MODE)"
echo "Using file: $COMPOSE_FILE"

docker compose -f "$COMPOSE_FILE" up -d

echo "Deployment started. Run 'docker compose -f $COMPOSE_FILE logs -f' to view output."
