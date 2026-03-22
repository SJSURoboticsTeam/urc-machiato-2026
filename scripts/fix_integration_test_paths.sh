#!/bin/bash
# Fix Integration Test Paths Script
# Updates old src.* paths to new services.* and shared.* paths in integration tests

set -e

echo "=========================================="
echo "Integration Test Path Fix Script"
echo "=========================================="
echo ""

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Check if we're in the right directory
if [ ! -f "pyproject.toml" ] && [ ! -d "tests/integration" ]; then
    echo -e "${RED}Error: Must run from project root directory${NC}"
    exit 1
fi

echo -e "${YELLOW}Phase 1: Updating mock patch paths...${NC}"
echo ""

# Update mock patch targets from src.autonomy to autonomy.autonomy_core
echo "  - Fixing src.autonomy.* mock patches..."
find tests/integration -name "*.py" -exec sed -i 's/patch("src\.autonomy\./patch("autonomy.autonomy_core./g' {} \;

# Update mock patch targets from src.core to shared.core
echo "  - Fixing src.core.* mock patches..."
find tests/integration -name "*.py" -exec sed -i 's/patch("src\.core\./patch("shared.core./g' {} \;

# Update mock patch targets from src.bridges to shared.infrastructure.bridges
echo "  - Fixing src.bridges.* mock patches..."
find tests/integration -name "*.py" -exec sed -i 's/patch("src\.bridges\./patch("shared.infrastructure.bridges./g' {} \;

# Update mock patch targets from src.config to shared.infrastructure.config
echo "  - Fixing src.config.* mock patches..."
find tests/integration -name "*.py" -exec sed -i 's/patch("src\.config\./patch("shared.infrastructure.config./g' {} \;

echo ""
echo -e "${YELLOW}Phase 2: Updating import paths...${NC}"
echo ""

# Update from src.autonomy imports
echo "  - Fixing from src.autonomy imports..."
find tests/integration -name "*.py" -exec sed -i 's/from src\.autonomy\./from autonomy.autonomy_core./g' {} \;

# Update from src.core imports
echo "  - Fixing from src.core imports..."
find tests/integration -name "*.py" -exec sed -i 's/from src\.core\./from shared.core./g' {} \;

# Update from src.bridges imports
echo "  - Fixing from src.bridges imports..."
find tests/integration -name "*.py" -exec sed -i 's/from src\.bridges\./from shared.infrastructure.bridges./g' {} \;

# Update from src.config imports
echo "  - Fixing from src.config imports..."
find tests/integration -name "*.py" -exec sed -i 's/from src\.config\./from shared.infrastructure.config./g' {} \;

# Update from src.shared imports
echo "  - Fixing from src.shared imports..."
find tests/integration -name "*.py" -exec sed -i 's/from src\.shared\./from shared./g' {} \;

echo ""
echo -e "${YELLOW}Phase 3: Updating simulation imports...${NC}"
echo ""

# Update from simulation. imports
echo "  - Fixing from simulation imports..."
find tests/integration -name "*.py" -exec sed -i 's/from simulation\./from services.simulation.python./g' {} \;

# Update from simulation.can imports
echo "  - Fixing from simulation.can imports..."
find tests/integration -name "*.py" -exec sed -i 's/from simulation\.can/from services.simulation.python.can/g' {} \;

# Update from simulation.network imports
echo "  - Fixing from simulation.network imports..."
find tests/integration -name "*.py" -exec sed -i 's/from simulation\.network/from services.simulation.python.network/g' {} \;

# Update from simulation.firmware imports
echo "  - Fixing from simulation.firmware imports..."
find tests/integration -name "*.py" -exec sed -i 's/from simulation\.firmware/from services.simulation.python.firmware/g' {} \;

# Update from simulation.ros2 imports
echo "  - Fixing from simulation.ros2 imports..."
find tests/integration -name "*.py" -exec sed -i 's/from simulation\.ros2/from services.simulation.python.ros2/g' {} \;

# Update from simulation.environments imports
echo "  - Fixing from simulation.environments imports..."
find tests/integration -name "*.py" -exec sed -i 's/from simulation\.environments/from services.simulation.python.environments/g' {} \;

# Update from simulation.core imports
echo "  - Fixing from simulation.core imports..."
find tests/integration -name "*.py" -exec sed -i 's/from simulation\.core/from services.simulation.python.core/g' {} \;

# Update from simulation.integration imports
echo "  - Fixing from simulation.integration imports..."
find tests/integration -name "*.py" -exec sed -i 's/from simulation\.integration/from services.simulation.python.integration/g' {} \;

# Update from simulation.tools imports
echo "  - Fixing from simulation.tools imports..."
find tests/integration -name "*.py" -exec sed -i 's/from simulation\.tools/from services.simulation.python.tools/g' {} \;

echo ""
echo -e "${YELLOW}Phase 4: Updating sys.path insertions...${NC}"
echo ""

# Update sys.path for autonomy/code
echo "  - Fixing autonomy/code sys.path..."
find tests/integration -name "*.py" -exec sed -i 's/\"autonomy\", \"code\"/\"services\", \"autonomy\", \"autonomy_core\"/g' {} \;

# Update sys.path for bridges
echo "  - Fixing bridges sys.path..."
find tests/integration -name "*.py" -exec sed -i 's/\"\.\.\.\", \"\.\.\.\", \"bridges\"/\"\.\.\.\", \"\.\.\.\", \"shared\", \"infrastructure\", \"bridges\"/g' {} \;

echo ""
echo -e "${GREEN}==========================================${NC}"
echo -e "${GREEN}Path fixes complete!${NC}"
echo -e "${GREEN}==========================================${NC}"
echo ""
echo "Next steps:"
echo "  1. Review changes with: git diff tests/integration/"
echo "  2. Run tests to verify: bash docker/run_tests_in_docker.sh integration"
echo "  3. Check for any remaining issues and fix manually"
echo ""
echo "Note: Some paths may need manual review if module names changed during restructuring."
