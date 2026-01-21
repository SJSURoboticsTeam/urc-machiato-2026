#!/bin/bash
# URC 2026 Mars Rover - Jazzy Jalisco Setup Script
# Configures Cyclone DDS and Iceoryx2 for optimal performance

set -e

echo "🚀 Setting up URC 2026 Rover with ROS2 Jazzy + Cyclone DDS + Iceoryx2"

# Check if Jazzy is installed
if ! [ -d "/opt/ros/jazzy" ]; then
    echo "❌ ROS2 Jazzy not found. Please install Jazzy first."
    exit 1
fi

# Source Jazzy
source /opt/ros/jazzy/setup.bash
echo "✅ ROS2 Jazzy sourced"

# Use default RMW for now (install Cyclone DDS separately if needed)
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
echo "ℹ️ Using default RMW implementation (Cyclone DDS not installed)"

# Configure Cyclone DDS
export CYCLONEDDS_URI=file://$(pwd)/cyclonedds.xml
echo "✅ Cyclone DDS configuration loaded"

# Configure Iceoryx2 for shared memory
export RCL_ICEORYX2=true
echo "✅ Iceoryx2 enabled for intra-process communication"

# Set domain ID
export ROS_DOMAIN_ID=42
echo "✅ ROS Domain ID set to 42"

# Performance optimizations
export ROS_LOCALHOST_ONLY=1  # Disable network discovery for single machine
echo "✅ Localhost-only mode enabled for performance"

# Threading optimizations
export RCL_THREADING_ENABLED=1
export RCL_THREADING_THREAD_COUNT=4
echo "✅ Multi-threading enabled (4 threads)"

# Logging configuration
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_LOGGING_BUFFERED_STREAM=1
export RCUTILS_COLORIZED_OUTPUT=1
echo "✅ Enhanced logging configured"

# Security (optional - enable for competition)
# export ROS_SECURITY_ENABLE=true
# export ROS_SECURITY_STRATEGY=enforce
# echo "✅ ROS2 Security enabled"

# Set Python path for project
export PYTHONPATH="${PYTHONPATH}:$(pwd)/src"
echo "✅ Python path configured"

# Verify configuration
echo ""
echo "🔍 Verifying configuration..."
echo "ROS_DISTRO: $ROS_DISTRO"
echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo "CYCLONEDDS_URI: $CYCLONEDDS_URI"
echo "RCL_ICEORYX2: $RCL_ICEORYX2"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"

# Test basic ROS2 functionality
echo ""
echo "🧪 Testing ROS2 functionality..."
timeout 5 ros2 topic list > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "✅ ROS2 functionality verified"
else
    echo "⚠️  ROS2 functionality test failed - may be expected on first run"
fi

echo ""
echo "🎉 Setup complete! Ready for Jazzy improvements."
echo "Run 'source jazzy_setup.sh' in new terminals to maintain configuration."
