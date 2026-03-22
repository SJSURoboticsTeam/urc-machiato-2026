#!/bin/bash
# Test Environment Setup Script
# Sets up ROS2 workspace and environment for testing

set -e

echo "🚀 Setting up URC 2026 Test Environment"

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

# Source ROS2
echo "📦 Sourcing ROS2 Humble..."
source /opt/ros/humble/setup.bash

# Navigate to autonomy workspace
cd "${PROJECT_ROOT}/autonomy"

# Source workspace if it exists
if [ -f "install/setup.bash" ]; then
    echo "🔧 Sourcing autonomy workspace..."
    source install/setup.bash
else
    echo "⚠️  Autonomy workspace not built. Building..."
    # Clean and build the workspace
    rm -rf build install log
    colcon build --packages-select autonomy_interfaces
    source install/setup.bash
fi

# Set environment variables
export ROS_DOMAIN_ID=42
export PYTHONPATH="${PROJECT_ROOT}:${PROJECT_ROOT}/autonomy/code:${PYTHONPATH}"
export ROS_PYTHON_LOG_CONFIG_FILE="${PROJECT_ROOT}/config/ros_logging.conf"

# Create log directory
mkdir -p "${PROJECT_ROOT}/logs/test"

echo "✅ Test environment ready!"
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
echo "PYTHONPATH includes: ${PROJECT_ROOT}"
echo "Log directory: ${PROJECT_ROOT}/logs/test"

# Verify key imports work
echo "🔍 Verifying key imports..."
python3 -c "
try:
    from autonomy_interfaces.action import NavigateToPose
    from bridges.ros2_state_machine_bridge import SystemState
    from autonomy.code.state_management.autonomy_state_machine.states import RoverState
    print('✅ All ROS2 imports working!')
except Exception as e:
    print(f'❌ Import failed: {e}')
    exit(1)
"

echo "🎯 Test environment setup complete!"
echo "Run your tests with: python3 -m pytest tests/integration/ -v"
