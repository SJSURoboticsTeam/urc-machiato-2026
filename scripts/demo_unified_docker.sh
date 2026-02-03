#!/bin/bash
# Quick Demo of Unified Docker System for URC 2026
# This script demonstrates the unified testing/deployment capabilities

set -e

echo "🚀 URC 2026 Unified Docker System Demo"
echo "====================================="

# Check if we're in the right directory
if [[ ! -f "scripts/docker_deploy.sh" ]]; then
    echo "❌ Error: Please run this script from the project root directory"
    exit 1
fi

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}📋 Available Commands:${NC}"
echo "  ./scripts/docker_deploy.sh deploy dev    # Deploy development environment"
echo "  ./scripts/docker_deploy.sh test unit     # Run unit tests"
echo "  ./scripts/docker_deploy.sh test all      # Run all tests"
echo "  ./scripts/docker_deploy.sh sim           # Start simulation"
echo "  ./scripts/docker_deploy.sh status dev    # Show status"
echo "  ./scripts/docker_deploy.sh cleanup dev   # Clean up"
echo ""

echo -e "${GREEN}🔧 System Features:${NC}"
echo "✅ Unified Docker compose configuration"
echo "✅ Multi-environment support (dev/test/prod/sim)"
echo "✅ Parallel test execution"
echo "✅ Gazebo simulation integration"
echo "✅ Monitoring and health checks"
echo "✅ Environment-specific configurations"
echo ""

echo -e "${YELLOW}📁 Created Files:${NC}"
echo "• docker/docker-compose.unified.yml - Main orchestration file"
echo "• docker/Dockerfile.unified - Multi-stage build with service targets"
echo "• .env.unified - Environment variables"
echo "• config/docker_development.yaml - Development config"
echo "• config/docker_testing.yaml - Testing config"
echo "• config/docker_simulation.yaml - Simulation config"
echo "• scripts/docker_deploy.sh - Deployment script"
echo ""

echo -e "${BLUE}🎯 Quick Test Commands:${NC}"
echo ""

echo "1. Build test environment:"
echo "   ./scripts/docker_deploy.sh build test"
echo ""

echo "2. Run unit tests:"
echo "   ./scripts/docker_deploy.sh test unit 4"
echo ""

echo "3. Run all tests:"
echo "   ./scripts/docker_deploy.sh test all"
echo ""

echo "4. Deploy development environment:"
echo "   ./scripts/docker_deploy.sh deploy dev"
echo ""

echo "5. Start simulation:"
echo "   ./scripts/docker_deploy.sh sim"
echo ""

echo "6. Check status:"
echo "   ./scripts/docker_deploy.sh status dev"
echo ""

echo "7. Open shell in container:"
echo "   ./scripts/docker_deploy.sh shell dev config"
echo ""

echo -e "${GREEN}🎉 Unified Docker System Ready!${NC}"
echo ""
echo "The unified system provides:"
echo "• Single command deployment for any environment"
echo "• Consistent configuration management"
echo "• Parallel testing with performance optimization"
echo "• Simulation with real Mars environment"
echo "• Production-ready containerization"
echo ""
echo "Start by running: ./scripts/docker_deploy.sh deploy dev"