#!/bin/bash
# Isaac Sim Environment Setup for URC 2026 Integration
# Run this before using Isaac Sim Python APIs

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Isaac Sim installation path
export ISAAC_SIM_PATH="/home/durian/Downloads/isaac-sim-standalone-5.0.0-linux-x86_64"

# Library paths
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ISAAC_SIM_PATH/.:$ISAAC_SIM_PATH/exts/isaacsim.robot.schema/plugins/lib:$ISAAC_SIM_PATH/exts/isaacsim.robot_motion.lula/pip_prebundle:$ISAAC_SIM_PATH/exts/isaacsim.asset.exporter.urdf/pip_prebundle:$ISAAC_SIM_PATH/kit:$ISAAC_SIM_PATH/kit/kernel/plugins:$ISAAC_SIM_PATH/kit/libs/iray:$ISAAC_SIM_PATH/kit/plugins:$ISAAC_SIM_PATH/kit/plugins/bindings-python:$ISAAC_SIM_PATH/kit/plugins/carb_gfx:$ISAAC_SIM_PATH/kit/plugins/rtx:$ISAAC_SIM_PATH/kit/plugins/gpu.foundation

# Python paths
export PYTHONPATH=$PYTHONPATH:$ISAAC_SIM_PATH/kit/python/lib/python3.11:$ISAAC_SIM_PATH/kit/python/lib/python3.11/site-packages:$ISAAC_SIM_PATH/python_packages:$ISAAC_SIM_PATH/exts/isaacsim.simulation_app:$ISAAC_SIM_PATH/extsDeprecated/omni.isaac.kit:$ISAAC_SIM_PATH/kit/kernel/py:$ISAAC_SIM_PATH/kit/plugins/bindings-python:$ISAAC_SIM_PATH/exts/isaacsim.robot_motion.lula/pip_prebundle:$ISAAC_SIM_PATH/exts/isaacsim.asset.exporter.urdf/pip_prebundle

# Add Isaac Sim extensions to path
for ext in $ISAAC_SIM_PATH/exts/isaacsim.*; do
    if [ -d "$ext" ]; then
        export PYTHONPATH=$PYTHONPATH:$ext
    fi
done

# URC environment
export URC_ENV="simulation"
export SIMULATION_BACKEND="isaac"

echo "Isaac Sim environment configured"
echo "ISAAC_SIM_PATH: $ISAAC_SIM_PATH"
echo "PYTHONPATH includes Isaac Sim extensions"

# Test import
python3 -c "import omni.isaac.core; print('Isaac Sim Python API available')" 2>/dev/null || echo "Note: Isaac Sim may need to be running for full API access"
