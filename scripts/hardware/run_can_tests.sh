#!/bin/bash
# Complete CAN Testing Suite - Software First, Then Hardware
# One-stop solution for testing CAN → hardware_interface → blackboard

set -e

WORKSPACE="/home/durian/urc-machiato-2026"
cd "$WORKSPACE"

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo ""
echo "╔════════════════════════════════════════════════════════════════╗"
echo "║         URC 2026 - Complete CAN Testing Suite                 ║"
echo "║         Test in Software First, Then Hardware                 ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""

# Check ROS2
if [ -z "$ROS_DISTRO" ]; then
    echo "Sourcing ROS2..."
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
    elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
        source /opt/ros/jazzy/setup.bash
    else
        echo "❌ ROS2 not found. Please install ROS2."
        exit 1
    fi
fi

# Source workspace
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
fi

echo -e "${GREEN}✓${NC} ROS2 $ROS_DISTRO ready"
echo ""

# Check if packages are built
if [ ! -d "install/autonomy_core" ]; then
    echo "Building ROS2 packages..."
    colcon build --base-paths src/autonomy/autonomy_core src/autonomy/interfaces/autonomy_interfaces --symlink-install
    source install/setup.bash
    echo -e "${GREEN}✓${NC} Packages built"
    echo ""
fi

# Main menu
echo "Select test mode:"
echo ""
echo "  1) Software Test (Simulated CAN - No Hardware)"
echo "  2) Hardware Test (Real CAN Device)"
echo "  3) Monitor Only (hardware_interface already running)"
echo ""
read -p "Enter choice [1-3]: " choice

case $choice in
    1)
        echo ""
        echo "═══════════════════════════════════════════════════════════"
        echo "  Software Test Mode (Simulated CAN)"
        echo "═══════════════════════════════════════════════════════════"
        echo ""
        echo "This will:"
        echo "  1. Start hardware_interface with mock CAN data"
        echo "  2. Monitor messages in real-time dashboard"
        echo "  3. Show CAN → blackboard data flow"
        echo ""
        echo "Starting in 3 seconds... (Ctrl+C to cancel)"
        sleep 3
        echo ""
        
        # Start hardware interface in background
        echo "Starting hardware_interface (mock CAN mode)..."
        ros2 run autonomy_core hardware_interface > /tmp/hw_interface.log 2>&1 &
        HW_PID=$!
        echo "  PID: $HW_PID"
        echo "  Logs: /tmp/hw_interface.log"
        echo ""
        
        # Wait for initialization
        echo "Waiting for initialization..."
        sleep 3
        
        # Start dashboard
        echo "Starting unified dashboard..."
        echo ""
        python3 scripts/hardware/can_testing_dashboard.py
        
        # Cleanup
        echo ""
        echo "Stopping hardware_interface..."
        kill $HW_PID 2>/dev/null || true
        echo "Done"
        ;;
    
    2)
        echo ""
        echo "═══════════════════════════════════════════════════════════"
        echo "  Hardware Test Mode (Real CAN Device)"
        echo "═══════════════════════════════════════════════════════════"
        echo ""
        
        # Check for device
        DEVICES=$(ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo "")
        if [ -z "$DEVICES" ]; then
            echo "❌ No serial devices found!"
            echo ""
            echo "Looking for: /dev/ttyACM* or /dev/ttyUSB*"
            echo ""
            echo "Steps to fix:"
            echo "  1. Connect STM32 CAN device via USB"
            echo "  2. Run: ./scripts/hardware/setup_usbcan_pi5.sh"
            echo "  3. Verify: ls -l /dev/ttyACM*"
            echo ""
            exit 1
        fi
        
        echo "Available devices:"
        ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
        echo ""
        
        # Select device
        read -p "Enter device path [/dev/ttyACM0]: " DEVICE
        DEVICE=${DEVICE:-/dev/ttyACM0}
        
        if [ ! -e "$DEVICE" ]; then
            echo "❌ Device $DEVICE not found"
            exit 1
        fi
        
        echo ""
        echo "Using device: $DEVICE"
        echo ""
        echo "Starting in 3 seconds... (Ctrl+C to cancel)"
        sleep 3
        echo ""
        
        # Start hardware interface with real device
        echo "Starting hardware_interface (real CAN device: $DEVICE)..."
        ros2 run autonomy_core hardware_interface \
            --ros-args \
            -p can_port:=$DEVICE \
            -p can_baudrate:=115200 \
            > /tmp/hw_interface.log 2>&1 &
        HW_PID=$!
        echo "  PID: $HW_PID"
        echo "  Logs: /tmp/hw_interface.log"
        echo ""
        
        # Wait for initialization
        echo "Waiting for CAN connection..."
        sleep 3
        
        # Check if connected
        if grep -q "Connected to CAN serial" /tmp/hw_interface.log 2>/dev/null; then
            echo -e "${GREEN}✓${NC} CAN device connected"
        else
            echo -e "${YELLOW}⚠${NC} CAN connection status unknown (check logs)"
        fi
        echo ""
        
        # Start dashboard in hardware mode
        echo "Starting unified dashboard (hardware mode)..."
        echo ""
        python3 scripts/hardware/can_testing_dashboard.py --hardware
        
        # Cleanup
        echo ""
        echo "Stopping hardware_interface..."
        kill $HW_PID 2>/dev/null || true
        echo "Done"
        ;;
    
    3)
        echo ""
        echo "═══════════════════════════════════════════════════════════"
        echo "  Monitor Only Mode"
        echo "═══════════════════════════════════════════════════════════"
        echo ""
        echo "Monitoring existing hardware_interface..."
        echo ""
        
        # Check if running
        if ros2 node list 2>/dev/null | grep -q "hardware_interface"; then
            echo -e "${GREEN}✓${NC} hardware_interface is running"
        else
            echo -e "${YELLOW}⚠${NC} hardware_interface not detected"
            echo ""
            echo "Start it in another terminal:"
            echo "  ros2 run autonomy_core hardware_interface"
        fi
        echo ""
        
        python3 scripts/hardware/can_testing_dashboard.py
        ;;
    
    *)
        echo "Invalid choice"
        exit 1
        ;;
esac

echo ""
echo "═══════════════════════════════════════════════════════════════"
echo "Test Complete"
echo "═══════════════════════════════════════════════════════════════"
echo ""
echo "Next steps:"
echo "  - Review logs: tail -f /tmp/hw_interface.log"
echo "  - Check blackboard writes in logs"
echo "  - Test emergency stop: ros2 topic pub /emergency_stop std_msgs/Bool \"data: true\""
echo "  - Send velocity command: ros2 topic pub /cmd_vel geometry_msgs/Twist ..."
echo ""
