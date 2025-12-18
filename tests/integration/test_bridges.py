#!/usr/bin/env python3
"""
Test script to verify bridge configurations work correctly.
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '.'))

def test_simulation_config():
    """Test if simulation config works."""
    print("Testing simulation config...")

    # Import the bridge
    from bridges.dashboard_simulation_bridge import DashboardSimulationBridge

    # Create bridge with default config (should work)
    bridge = DashboardSimulationBridge()

    print("Bridge config:")
    print(f"  Environment: {bridge.simulation_config['environment']}")
    print(f"  Sensors: {[s['type'] for s in bridge.simulation_config['sensors']]}")

    # Try to start simulation
    success = bridge.start_simulation()

    if success:
        print("✅ Simulation started successfully")
        bridge.stop_simulation()
        return True
    else:
        print("❌ Simulation failed to start")
        return False

def test_state_machine_bridge():
    """Test if state machine bridge works."""
    print("\nTesting state machine bridge...")

    try:
        from bridges.ros2_state_machine_bridge import ROS2StateMachineBridge, SystemState

        # Just test imports and basic functionality
        from bridges.ros2_state_machine_bridge import SimpleStateMachine
        state_machine = SimpleStateMachine()
        print(f"Initial state: {state_machine.state.value}")

        # Test transition
        success = state_machine.transition_to(SystemState.IDLE)
        print(f"Transition to IDLE: {'✅' if success else '❌'}")
        print(f"Current state: {state_machine.state.value}")

        return True
    except Exception as e:
        print(f"❌ State machine bridge error: {e}")
        return False

def test_mission_bridge():
    """Test if mission bridge works."""
    print("\nTesting mission bridge imports...")

    try:
        from bridges.ros2_mission_bridge import ROS2MissionBridge
        from missions.mission_executor import MissionExecutor

        print("✅ Mission bridge imports successful")
        return True
    except Exception as e:
        print(f"❌ Mission bridge error: {e}")
        return False

if __name__ == "__main__":
    print("🧪 Testing Bridge Configurations")
    print("=" * 40)

    results = []

    # Test each bridge
    results.append(("Simulation", test_simulation_config()))
    results.append(("State Machine", test_state_machine_bridge()))
    results.append(("Mission", test_mission_bridge()))

    print("\n📊 Test Results:")
    for name, success in results:
        status = "✅ PASS" if success else "❌ FAIL"
        print(f"  {name}: {status}")

    all_pass = all(success for _, success in results)
    print(f"\n{'🎉 All tests passed!' if all_pass else '⚠️ Some tests failed'}")
