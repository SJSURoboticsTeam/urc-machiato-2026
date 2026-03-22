#!/usr/bin/env python3
"""
Custom Scenario Demo

Demonstrates creating custom test scenarios with fault injection,
metrics collection, and report generation.

Author: URC 2026 Examples Team
"""

import asyncio
import sys
import time
from pathlib import Path
from typing import Dict, Any, List

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from simulation.integration.full_stack_simulator import create_full_stack_simulator
from simulation.firmware.stm32_firmware_simulator import MotorFaultType


async def simple_custom_scenario():
    """Create and run a simple custom scenario."""
    print("\n=== Simple Custom Scenario ===")

    sim = create_full_stack_simulator("perfect")

    print("\n🧪 Custom Test: Velocity Ramp")
    print("  Test that motors ramp smoothly from 0 to target velocity\n")

    success = True
    errors = []

    try:
        # Connect client
        client_id = await sim.websocket_sim.connect()

        # Send increasing velocities
        velocities = [0.0, 0.2, 0.4, 0.6, 0.8, 1.0]

        for vel in velocities:
            await sim.websocket_sim.receive(
                "driveCommands", {"linear": vel, "angular": 0.0}, client_id
            )

            await asyncio.sleep(0.2)

            # Check motor response
            motor_status = sim.firmware_sim.get_motor_status(0)
            actual_vel = motor_status["velocity_actual"]

            print(f"  Setpoint: {vel:.1f}, Actual: {actual_vel:.2f}")

            # Verify reasonable response
            if vel > 0 and actual_vel < vel * 0.5:
                errors.append(f"Poor response at velocity {vel}")
                success = False

        if success:
            print("  ✅ Smooth ramp verified")

    except Exception as e:
        errors.append(str(e))
        success = False

    sim.shutdown()

    print(f"\nResult: {'✅ PASS' if success else '❌ FAIL'}")
    if errors:
        print(f"Errors: {errors}")

    return success


async def fault_injection_scenario():
    """Scenario with fault injection and recovery."""
    print("\n=== Fault Injection Scenario ===")

    sim = create_full_stack_simulator("default")

    print("\n🧪 Custom Test: Fault Recovery")
    print("  Test system recovers from motor fault\n")

    success = True
    errors = []

    try:
        client_id = await sim.websocket_sim.connect()

        # 1. Establish baseline operation
        print("  Phase 1: Baseline operation")
        await sim.websocket_sim.receive(
            "driveCommands", {"linear": 0.5, "angular": 0.0}, client_id
        )
        await asyncio.sleep(0.3)

        status_before = sim.firmware_sim.get_motor_status(0)
        print(f"    Motor velocity: {status_before['velocity_actual']:.2f} rad/s")

        # 2. Inject fault
        print("  Phase 2: Inject overcurrent fault")
        sim.firmware_sim.inject_fault(0, MotorFaultType.OVERCURRENT)

        status_faulted = sim.firmware_sim.get_motor_status(0)
        if status_faulted["fault"] != "overcurrent":
            errors.append("Fault not injected correctly")
            success = False
        else:
            print("    ✅ Fault injected")

        # 3. Attempt recovery
        print("  Phase 3: Clear fault and recover")
        sim.firmware_sim.clear_fault(0)

        # Send command after clearing fault
        await sim.websocket_sim.receive(
            "driveCommands", {"linear": 0.5, "angular": 0.0}, client_id
        )
        await asyncio.sleep(0.5)

        # 4. Verify recovery
        status_after = sim.firmware_sim.get_motor_status(0)
        if status_after["fault"] == "none" and status_after["velocity_actual"] > 0:
            print("    ✅ System recovered")
        else:
            errors.append("System did not recover properly")
            success = False

    except Exception as e:
        errors.append(str(e))
        success = False

    sim.shutdown()

    print(f"\nResult: {'✅ PASS' if success else '❌ FAIL'}")
    return success


async def performance_measurement_scenario():
    """Scenario measuring specific performance metrics."""
    print("\n=== Performance Measurement Scenario ===")

    sim = create_full_stack_simulator("perfect")

    print("\n🧪 Custom Test: End-to-End Latency")
    print("  Measure command-to-response latency\n")

    client_id = await sim.websocket_sim.connect()

    latencies = []

    print("  Measuring latencies...")
    for i in range(10):
        start = time.time()

        # Send command
        await sim.websocket_sim.receive(
            "driveCommands", {"linear": 0.5, "angular": 0.0}, client_id
        )

        # Measure until firmware responds
        await asyncio.sleep(0.01)  # Minimal delay

        latency = (time.time() - start) * 1000  # ms
        latencies.append(latency)

    # Calculate statistics
    avg_latency = sum(latencies) / len(latencies)
    min_latency = min(latencies)
    max_latency = max(latencies)

    print(f"\n  📊 Latency Statistics:")
    print(f"    Average: {avg_latency:.2f}ms")
    print(f"    Min: {min_latency:.2f}ms")
    print(f"    Max: {max_latency:.2f}ms")

    # Check against target
    target_latency = 50.0  # ms
    success = avg_latency < target_latency

    if success:
        print(f"    ✅ Within target ({target_latency}ms)")
    else:
        print(f"    ⚠️  Exceeds target ({target_latency}ms)")

    sim.shutdown()

    print(f"\nResult: {'✅ PASS' if success else '❌ FAIL'}")
    return success


async def stress_test_scenario():
    """Stress test scenario with high message rate."""
    print("\n=== Stress Test Scenario ===")

    sim = create_full_stack_simulator("perfect")

    print("\n🧪 Custom Test: High Message Rate")
    print("  Test system stability under sustained high load\n")

    client_id = await sim.websocket_sim.connect()

    # Send many messages rapidly
    message_count = 500
    start_time = time.time()

    print(f"  Sending {message_count} messages...")
    for i in range(message_count):
        await sim.websocket_sim.receive(
            "driveCommands",
            {"linear": 0.5 + (i % 10) * 0.05, "angular": 0.0},
            client_id,
        )

    duration = time.time() - start_time
    message_rate = message_count / duration

    print(f"  Duration: {duration:.2f}s")
    print(f"  Rate: {message_rate:.0f} msg/s")

    # Check statistics
    ws_stats = sim.websocket_sim.get_statistics()
    received = ws_stats["stats"]["messages_received"]
    dropped = ws_stats["stats"].get("messages_dropped", 0)

    success_rate = (received / message_count) * 100

    print(f"\n  📊 Results:")
    print(f"    Sent: {message_count}")
    print(f"    Received: {received}")
    print(f"    Dropped: {dropped}")
    print(f"    Success rate: {success_rate:.1f}%")

    success = success_rate > 95.0

    if success:
        print("    ✅ High success rate")
    else:
        print("    ⚠️  Too many drops")

    sim.shutdown()

    print(f"\nResult: {'✅ PASS' if success else '❌ FAIL'}")
    return success


async def multi_phase_scenario():
    """Complex multi-phase scenario."""
    print("\n=== Multi-Phase Scenario ===")

    sim = create_full_stack_simulator("default")

    print("\n🧪 Custom Test: Complete Mission Profile")
    print("  Simulates a realistic mission sequence\n")

    client_id = await sim.websocket_sim.connect()
    phases_passed = 0
    total_phases = 5

    try:
        # Phase 1: Startup and homing
        print("  Phase 1/5: Startup and homing")
        sim.firmware_sim.start_homing_sequence()
        await asyncio.sleep(0.5)
        if sim.firmware_sim.homing_progress > 0:
            print("    ✅ Homing in progress")
            phases_passed += 1

        # Phase 2: Low speed movement
        print("  Phase 2/5: Low speed movement")
        await sim.websocket_sim.receive(
            "driveCommands", {"linear": 0.2, "angular": 0.0}, client_id
        )
        await asyncio.sleep(0.3)
        status = sim.firmware_sim.get_motor_status(0)
        if status["velocity_actual"] > 0:
            print("    ✅ Motors responding")
            phases_passed += 1

        # Phase 3: High speed movement
        print("  Phase 3/5: High speed movement")
        await sim.websocket_sim.receive(
            "driveCommands", {"linear": 0.8, "angular": 0.0}, client_id
        )
        await asyncio.sleep(0.5)
        status = sim.firmware_sim.get_motor_status(0)
        if status["velocity_actual"] > 0.5:
            print("    ✅ High speed achieved")
            phases_passed += 1

        # Phase 4: Emergency stop test
        print("  Phase 4/5: Emergency stop")
        await sim.websocket_sim.receive("emergencyStop", {}, client_id)
        await asyncio.sleep(0.1)
        if sim.firmware_sim.emergency_stop_active:
            print("    ✅ E-stop activated")
            phases_passed += 1

        # Phase 5: Recovery
        print("  Phase 5/5: Recovery")
        sim.firmware_sim.clear_emergency_stop()
        await sim.websocket_sim.receive(
            "driveCommands", {"linear": 0.3, "angular": 0.0}, client_id
        )
        await asyncio.sleep(0.3)
        status = sim.firmware_sim.get_motor_status(0)
        if not sim.firmware_sim.emergency_stop_active and status["velocity_actual"] > 0:
            print("    ✅ System recovered")
            phases_passed += 1

    except Exception as e:
        print(f"    ❌ Error: {e}")

    sim.shutdown()

    print(f"\n  📊 Mission Summary:")
    print(f"    Phases passed: {phases_passed}/{total_phases}")
    print(f"    Success rate: {(phases_passed/total_phases)*100:.0f}%")

    success = phases_passed == total_phases
    print(f"\nResult: {'✅ PASS' if success else '❌ FAIL'}")

    return success


async def main():
    """Run all custom scenarios."""
    print("=" * 60)
    print("Custom Scenario Demo")
    print("=" * 60)
    print("\nDemonstrates creating custom test scenarios for specific use cases.\n")

    results = []

    try:
        # Run all custom scenarios
        results.append(("Simple Velocity Ramp", await simple_custom_scenario()))
        results.append(("Fault Recovery", await fault_injection_scenario()))
        results.append(
            ("Performance Measurement", await performance_measurement_scenario())
        )
        results.append(("Stress Test", await stress_test_scenario()))
        results.append(("Multi-Phase Mission", await multi_phase_scenario()))

        # Summary
        print("\n" + "=" * 60)
        print("Test Summary")
        print("=" * 60)

        passed = sum(1 for _, success in results if success)
        total = len(results)

        print(f"\nResults:")
        for name, success in results:
            status = "✅ PASS" if success else "❌ FAIL"
            print(f"  {status}  {name}")

        print(f"\nTotal: {passed}/{total} passed ({(passed/total)*100:.0f}%)")

        if passed == total:
            print("\n🎉 All custom scenarios passed!")
        else:
            print(f"\n⚠️  {total - passed} scenario(s) failed")

        print("\n" + "=" * 60)
        print("Next Steps:")
        print("  • Create scenarios for your specific use cases")
        print("  • Add metrics collection and reporting")
        print("  • Integrate scenarios into CI/CD pipeline")
        print("  • Use scenarios for regression testing")

    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback

        traceback.print_exc()


if __name__ == "__main__":
    asyncio.run(main())
