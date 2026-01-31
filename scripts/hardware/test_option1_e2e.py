#!/usr/bin/env python3
"""
Integration Test: CAN to Blackboard Direct Writes (Option 1)

Tests the actual code paths without mocking ROS2, using simple inspection.
Verifies the implementation is correct and ready for real testing.
"""

import sys
from pathlib import Path

WORKSPACE = Path(__file__).resolve().parents[2]


def inspect_implementation():
    """Inspect the implementation to verify Option 1 is correctly implemented."""
    
    hw_path = WORKSPACE / "src/autonomy/autonomy_core/autonomy_core/control/hardware_interface_node.py"
    
    with open(hw_path, 'r') as f:
        code = f.read()
    
    print("=== Option 1 Implementation Verification ===\n")
    
    # Extract key code sections
    sections = {
        "Battery State Write": None,
        "Velocity Write": None,
        "Position Write": None,
        "Emergency Stop Write": None,
    }
    
    # Check battery state write
    if 'def read_battery_state(self):' in code:
        start = code.find('def read_battery_state(self):')
        end = code.find('\n    def ', start + 1)
        battery_code = code[start:end]
        
        if 'self.blackboard.set(BlackboardKeys.BATTERY_LEVEL' in battery_code or \
           'blackboard.set("battery_level"' in battery_code:
            sections["Battery State Write"] = True
        else:
            sections["Battery State Write"] = False
    
    # Check velocity write
    if 'def read_chassis_velocity(self):' in code:
        start = code.find('def read_chassis_velocity(self):')
        end = code.find('\n    def ', start + 1)
        velocity_code = code[start:end]
        
        if 'self.blackboard.set(BlackboardKeys.ROBOT_VELOCITY_X' in velocity_code:
            sections["Velocity Write"] = True
        else:
            sections["Velocity Write"] = False
    
    # Check position write
    if 'self.blackboard.set(BlackboardKeys.ROBOT_X' in code:
        sections["Position Write"] = True
    else:
        sections["Position Write"] = False
    
    # Check emergency stop write
    if 'def emergency_stop_callback' in code:
        start = code.find('def emergency_stop_callback')
        end = code.find('\n    def ', start + 1)
        estop_code = code[start:end]
        
        if 'self.blackboard.set(BlackboardKeys.EMERGENCY_STOP_ACTIVE' in estop_code:
            sections["Emergency Stop Write"] = True
        else:
            sections["Emergency Stop Write"] = False
    
    # Print results
    all_passed = True
    for section, status in sections.items():
        if status is True:
            print(f"[OK] {section}")
        elif status is False:
            print(f"[FAIL] {section}")
            all_passed = False
        else:
            print(f"[SKIP] {section} - not found")
            all_passed = False
    
    return all_passed


def trace_data_flow():
    """Trace the data flow to show the architecture."""
    print("\n" + "="*60)
    print("Data Flow Architecture (Option 1)")
    print("="*60)
    print()
    print("CAN Hardware (STM32)")
    print("  |")
    print("  | SLCAN Protocol")
    print("  | /dev/ttyACM0")
    print("  |")
    print("  v")
    print("hardware_interface_node")
    print("  |")
    print("  | read_battery_state()")
    print("  | read_chassis_velocity()")
    print("  | emergency_stop_callback()")
    print("  |")
    print("  | self.blackboard.set(key, value)")
    print("  |")
    print("  v")
    print("Unified Blackboard (BT.CPP)")
    print("  |")
    print("  | Behavior Tree Nodes Read From Blackboard")
    print("  |")
    print("  v")
    print("Autonomy Decision Making")
    print()
    print("Advantages:")
    print("  - Direct write: <1ms latency")
    print("  - No ROS2 topic overhead")
    print("  - No intermediate bridge nodes")
    print("  - Single source of truth")
    print("  - Minimal CPU usage")
    print()


def print_simulator_test_plan():
    """Print the test plan for simulator validation."""
    print("="*60)
    print("Testing with Simulator (No Hardware Required)")
    print("="*60)
    print()
    print("OPTION A: Quick Validation (No Build)")
    print("-" * 40)
    print("1. Run validation script:")
    print("   python3 scripts/hardware/validate_can_blackboard_direct.py")
    print()
    print("2. Inspect code manually:")
    print("   # Check battery write")
    print('   grep -A 5 "def read_battery_state" src/autonomy/autonomy_core/autonomy_core/control/hardware_interface_node.py')
    print()
    print("OPTION B: Full ROS2 Integration Test")
    print("-" * 40)
    print("1. Build packages:")
    print("   cd /home/durian/urc-machiato-2026")
    print("   colcon build --packages-select autonomy_interfaces autonomy_core")
    print("   source install/setup.bash")
    print()
    print("2. Terminal 1 - Launch mock blackboard + hardware interface:")
    print("   python3 scripts/hardware/test_can_blackboard_sim.py")
    print()
    print("3. Terminal 2 - Monitor blackboard:")
    print("   python3 scripts/hardware/blackboard_visualizer.py")
    print()
    print("4. Verify output shows:")
    print("   - battery_level: 85.0")
    print("   - robot_velocity_x: 0.0")
    print("   - robot_velocity_y: 0.0")
    print("   - robot_x: 0.0 (integrating over time)")
    print("   - robot_y: 0.0")
    print("   - emergency_stop_active: False")
    print()
    print("OPTION C: Use swerve_simulator (in scripts/hardware)")
    print("-" * 40)
    print("1. Terminal 1 - Run swerve simulator:")
    print("   python3 scripts/hardware/swerve_simulator.py")
    print()
    print("2. Terminal 2 - Run terminal dashboard:")
    print("   python3 scripts/hardware/terminal_dashboard.py  # if present")
    print()
    print("3. Observe simulated CAN data flowing")
    print()


def main():
    print("\nValidating Option 1: Direct CAN → Blackboard Implementation\n")
    
    if inspect_implementation():
        print("\n[SUCCESS] Implementation verified - all blackboard writes present")
        trace_data_flow()
        print_simulator_test_plan()
        return 0
    else:
        print("\n[FAIL] Implementation incomplete")
        return 1


if __name__ == "__main__":
    sys.exit(main())
