#!/bin/bash
# Stop old hardware_interface and start new one with dynamic data

cd /home/durian/urc-machiato-2026

echo "╔════════════════════════════════════════════════════════════════╗"
echo "║  Restart Hardware Interface with Dynamic Simulator Data        ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""

# Kill old process
OLD_PID=$(ps aux | grep "hardware_interface" | grep -v grep | grep -v restart | awk '{print $2}')

if [ ! -z "$OLD_PID" ]; then
    echo "Stopping old hardware_interface (PID: $OLD_PID)..."
    kill $OLD_PID 2>/dev/null
    sleep 1
    echo "✓ Stopped"
    echo ""
fi

# Source ROS2 and workspace
source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash
source install/setup.bash

echo "Starting hardware_interface with DYNAMIC DATA:"
echo "  - Battery: Slowly discharging"
echo "  - Velocity: Sinusoidal motion (changing!)"
echo "  - Voltage: Realistic variations"
echo "  - Position: Integrating over time"
echo ""
echo "Logs: /tmp/hw_interface_dynamic.log"
echo ""
echo "Watch the dashboard - velocity should now be changing!"
echo "Press Ctrl+C to stop"
echo ""
echo "─────────────────────────────────────────────────────────────────"

# Run with output
ros2 run autonomy_core hardware_interface 2>&1 | tee /tmp/hw_interface_dynamic.log
