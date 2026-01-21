#!/bin/bash

# Simple Jazzy setup for testing
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$(pwd)/cyclonedds.xml"
export RCL_ICEORYX2=true
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
export RCUTILS_LOGGING_USE_STDOUT=1
export PYTHONPATH="$(pwd)/src:$PYTHONPATH"

echo "🚀 Jazzy environment configured"
