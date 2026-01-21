#!/usr/bin/env python3
"""
Full Stack Communication Demo

Demonstrates end-to-end communication testing using the full stack simulator.
Shows how to run pre-built scenarios and create custom tests.

Author: URC 2026 Examples Team
"""

import asyncio
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from simulation.integration.full_stack_simulator import (
    create_full_stack_simulator, ScenarioType
)


async def basic_scenario_demo():
    """Run basic built-in scenarios."""
    print("\n=== Basic Scenario Demo ===")
    
    sim = create_full_stack_simulator('perfect')
    
    # Run basic velocity scenario
    print("\n🚀 Running BASIC_VELOCITY scenario...")
    result = await sim.run_scenario(ScenarioType.BASIC_VELOCITY)
    
    print(f"  Success: {result.success}")
    print(f"  Duration: {result.duration_s:.3f}s")
    print(f"  Messages: {result.messages_sent}/{result.messages_received}")
    
    if not result.success:
        print(f"  Errors: {result.errors}")
    
    # Run emergency stop scenario
    print("\n🛑 Running EMERGENCY_STOP scenario...")
    result = await sim.run_scenario(ScenarioType.EMERGENCY_STOP)
    
    print(f"  Success: {result.success}")
    print(f"  Duration: {result.duration_s*1000:.1f}ms")
    
    if result.duration_s < 0.1:
        print(f"  ✅ E-stop timing requirement met (<100ms)")
    
    sim.shutdown()
    print("✅ Scenarios complete!\n")


async def test_suite_demo():
    """Run complete test suite."""
    print("\n=== Full Test Suite Demo ===")
    
    sim = create_full_stack_simulator('default')
    
    print("🧪 Running all scenarios...\n")
    summary = await sim.run_test_suite()
    
    print(f"📊 Results:")
    print(f"  Total scenarios: {summary['total_scenarios']}")
    print(f"  Passed: {summary['passed']}")
    print(f"  Failed: {summary['failed']}")
    print(f"  Pass rate: {summary['pass_rate']*100:.1f}%")
    print(f"  Duration: {summary['duration_s']:.1f}s")
    
    print(f"\n📋 Individual results:")
    for result in summary['results']:
        status = "✅" if result['success'] else "❌"
        print(f"  {status} {result['scenario']}: {result['duration_s']:.3f}s")
    
    sim.shutdown()
    print("\n✅ Test suite complete!\n")


async def custom_parameters_demo():
    """Run scenarios with custom parameters."""
    print("\n=== Custom Parameters Demo ===")
    
    sim = create_full_stack_simulator('perfect')
    
    # Run with short duration
    print("\n⚡ Quick test (0.5s)...")
    result = await sim.run_scenario(
        ScenarioType.BASIC_VELOCITY,
        params={'duration': 0.5, 'velocity': 0.3}
    )
    print(f"  Duration: {result.duration_s:.3f}s")
    
    # Run with high load
    print("\n📈 High load test...")
    result = await sim.run_scenario(
        ScenarioType.HIGH_LOAD,
        params={'message_rate': 50, 'duration': 2.0}
    )
    print(f"  Messages sent: {result.messages_sent}")
    print(f"  Success: {result.success}")
    
    sim.shutdown()
    print("✅ Custom parameters complete!\n")


async def metrics_collection_demo():
    """Demonstrate metrics collection."""
    print("\n=== Metrics Collection Demo ===")
    
    sim = create_full_stack_simulator('default')
    
    # Run some scenarios
    await sim.run_scenario(ScenarioType.BASIC_VELOCITY)
    await sim.run_scenario(ScenarioType.EMERGENCY_STOP)
    
    # Get status
    status = sim.get_status()
    
    print("\n📊 Component Status:")
    for component, state in status['components'].items():
        print(f"\n  {component.upper()}:")
        if isinstance(state, dict):
            for key, value in list(state.items())[:5]:  # Show first 5
                print(f"    {key}: {value}")
    
    print("\n📈 Simulation Statistics:")
    for key, value in status['stats'].items():
        print(f"  {key}: {value}")
    
    print("\n📜 Scenario History:")
    for scenario in status['scenario_history'][-3:]:
        status_symbol = "✅" if scenario['success'] else "❌"
        print(f"  {status_symbol} {scenario['type']}")
    
    sim.shutdown()
    print("\n✅ Metrics demo complete!\n")


async def communication_path_demo():
    """Test specific communication paths."""
    print("\n=== Communication Path Demo ===")
    
    sim = create_full_stack_simulator('perfect')
    
    # Test teleoperation path
    print("\n🎮 Testing Teleoperation Path...")
    print("  Frontend → WebSocket → ROS2 → SLCAN → Firmware")
    
    client_id = await sim.websocket_sim.connect()
    
    # Send drive command
    await sim.websocket_sim.receive('driveCommands', {
        'linear': 0.5,
        'angular': 0.2
    }, client_id)
    
    await asyncio.sleep(0.2)
    
    # Check propagation at each layer
    print("\n  Layer verification:")
    
    # ROS2 layer
    if sim.ros2_state['cmd_vel_teleop']:
        print(f"    ✅ ROS2: cmd_vel received")
    
    # Firmware layer
    motor_status = sim.firmware_sim.get_motor_status(0)
    if motor_status and motor_status['velocity_actual'] > 0:
        print(f"    ✅ Firmware: motors responding ({motor_status['velocity_actual']:.2f} rad/s)")
    
    # Test emergency stop path
    print("\n🚨 Testing Emergency Stop Path...")
    await sim.websocket_sim.receive('emergencyStop', {}, client_id)
    
    await asyncio.sleep(0.05)
    
    if sim.firmware_sim.emergency_stop_active:
        print("  ✅ E-stop propagated to firmware")
    
    # Check all motors stopped
    all_stopped = all(m.velocity_setpoint == 0.0 for m in sim.firmware_sim.motors.values())
    if all_stopped:
        print("  ✅ All motors stopped")
    
    sim.shutdown()
    print("\n✅ Communication path demo complete!\n")


async def main():
    """Run all demos."""
    print("=" * 60)
    print("Full Stack Communication Simulator Demo")
    print("=" * 60)
    
    try:
        await basic_scenario_demo()
        await test_suite_demo()
        await custom_parameters_demo()
        await metrics_collection_demo()
        await communication_path_demo()
        
        print("=" * 60)
        print("✅ All full stack demos completed!")
        print("=" * 60)
        
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    asyncio.run(main())
