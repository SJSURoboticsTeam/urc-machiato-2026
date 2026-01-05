#!/usr/bin/env python3
"""
Performance Test for URC 2026 System

Tests speed, memory usage, and performance metrics for all components.
"""

import time
import psutil
import sys
import os

# Add src to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

def measure_memory():
    """Measure current memory usage in MB."""
    process = psutil.Process()
    return process.memory_info().rss / 1024 / 1024

def measure_cpu():
    """Measure current CPU usage."""
    process = psutil.Process()
    return process.cpu_percent()

def main():
    """Run performance tests."""
    print("🧪 URC 2026 Performance Test")
    print("=" * 50)

    # Baseline
    baseline_memory = measure_memory()
    print(f"Baseline Memory: {baseline_memory:.1f} MB")

    # Test 1: Lightweight Core Startup
    print("\n1. Testing Lightweight Core Startup...")
    start_time = time.time()
    from src.core.lightweight_core import get_lightweight_core
    core = get_lightweight_core()
    startup_time = time.time() - start_time
    print(".3f")

    # Test 2: Component Loading
    print("\n2. Testing Component Loading...")
    components = ['state_machine', 'behavior_tree', 'safety_system']
    for comp in components:
        load_start = time.time()
        core.get_component(comp)
        load_time = time.time() - load_start
        print(".3f")

    # Test 3: CAN Bridge
    print("\n3. Testing CAN Bridge...")
    from src.bridges.direct_can_bridge import DirectCANBridge
    can_start = time.time()
    can_bridge = DirectCANBridge()
    can_time = time.time() - can_start
    print(".3f")

    # Test 4: Simple Bridge
    print("\n4. Testing Simple Bridge...")
    from src.bridges.simple_bridge import SimpleBridge
    bridge_start = time.time()
    bridge = SimpleBridge()
    bridge_time = time.time() - bridge_start
    print(".3f")

    # Test 5: Database Throughput
    print("\n5. Testing Database Throughput...")
    db = core.get_database()
    test_data = [{'battery_level': 80 + i, 'timestamp': time.time()} for i in range(10)]

    db_start = time.time()
    for data in test_data:
        db.store_telemetry(data)
    db_time = time.time() - db_start
    throughput = len(test_data) / db_time if db_time > 0 else 0
    print(".1f")

    # Final Memory Usage
    final_memory = measure_memory()
    memory_delta = final_memory - baseline_memory
    print("\nFinal Memory Usage:")
    print(".1f")
    print(".1f")
    # Performance Summary
    print("\n📊 PERFORMANCE SUMMARY:")    print(".3f"    print(".1f"    print(".1f"
    startup_ok = startup_time < 1.0
    memory_ok = final_memory < 150
    throughput_ok = throughput > 5

    print(f"\n✅ Startup (< 1s): {'PASS' if startup_ok else 'FAIL'}")
    print(f"✅ Memory (< 150MB): {'PASS' if memory_ok else 'FAIL'}")
    print(f"✅ Throughput (> 5 msg/s): {'PASS' if throughput_ok else 'FAIL'}")

    status = "🚀 READY FOR COMPETITION" if all([startup_ok, memory_ok, throughput_ok]) else "⚠️ NEEDS OPTIMIZATION"
    print(f"\n{status}")

if __name__ == "__main__":
    main()
