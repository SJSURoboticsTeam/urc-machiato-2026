#!/bin/bash
# Isaac Sim 5.0 launch with ROS2 bridge for URC 2026
# Usage: ./scripts/isaac_sim_launch.sh
# Set ISAAC_SIM_PATH to override installation path.

set -e

# ROS2 setup (Jazzy recommended; fallback to Humble)
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
    export ROS_DISTRO=jazzy
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    export ROS_DISTRO=humble
else
    echo "No ROS2 (jazzy or humble) found. Source your ROS2 setup first."
    exit 1
fi

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
if [ -f "$HOME/.ros/fastdds.xml" ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.ros/fastdds.xml"
fi

# Isaac Sim path (override with ISAAC_SIM_PATH)
export ISAAC_SIM_PATH="${ISAAC_SIM_PATH:-$HOME/isaac-sim-5.0.0}"
# Common alternate location
if [ ! -d "$ISAAC_SIM_PATH" ] && [ -d "$HOME/Downloads/isaac-sim-standalone-5.0.0-linux-x86_64" ]; then
    export ISAAC_SIM_PATH="$HOME/Downloads/isaac-sim-standalone-5.0.0-linux-x86_64"
fi

if [ ! -f "$ISAAC_SIM_PATH/isaac-sim.sh" ]; then
    echo "Isaac Sim not found at: $ISAAC_SIM_PATH"
    echo "Install Isaac Sim 5.0 or set ISAAC_SIM_PATH to your installation directory."
    exit 1
fi

cd "$ISAAC_SIM_PATH"
exec ./isaac-sim.sh --/isaac/startup/ros_bridge_extension=isaacsim.ros2.bridge
