#!/usr/bin/env python3
"""
Simple Performance Test for URC 2026

Focused testing of speed, memory usage, and performance metrics.
"""

import time
import psutil
import sys
import os

# Add src to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

def measure_memory():
    """Measure current memory usage."""
    process = psutil.Process()
    mem_info = process.memory_info()
    return mem_info.rss / 1024 / 1024  # MB

def measure_cpu():
    """Measure current CPU usage."""
    process = psutil.Process()
    return process.cpu_percent()

def run_performance_test():
    """Run comprehensive performance test."""
    print("🧪 URC 2026 Performance Test")
    print("=" * 50)

    # Baseline measurements
    baseline_memory = measure_memory()
    baseline_cpu = measure_cpu()
    print(".1f"    print(".1f"
    # Test 1: Lightweight Core Startup
    print("\n1️⃣ Testing Lightweight Core Startup...")
    start_time = time.time()

    from src.core.lightweight_core import get_lightweight_core
    core = get_lightweight_core()

    startup_time = time.time() - start_time
    startup_memory = measure_memory()
    startup_cpu = measure_cpu()

    print(".3f"    print(".1f"    print(".1f"
    # Test 2: Component Loading
    print("\n2️⃣ Testing Component Loading...")
    components = ['state_machine', 'behavior_tree', 'safety_system', 'database']
    load_times = {}
    load_memory = {}

    for component in components:
        start_time = time.time()
        comp_instance = core.get_component(component)
        load_time = time.time() - start_time
        memory_after = measure_memory()

        load_times[component] = load_time
        load_memory[component] = memory_after

        print(".3f"
    # Test 3: CAN Bridge Performance
    print("\n3️⃣ Testing CAN Bridge Performance...")
    from src.bridges.direct_can_bridge import DirectCANBridge

    can_start = time.time()
    can_bridge = DirectCANBridge()
    can_creation_time = time.time() - can_start
    can_memory = measure_memory()

    print(".3f"    print(".1f"
    # Test 4: Simple Bridge Performance
    print("\n4️⃣ Testing Simple Bridge Performance...")
    from src.bridges.simple_bridge import SimpleBridge

    bridge_start = time.time()
    bridge = SimpleBridge()
    bridge_creation_time = time.time() - bridge_start
    bridge_memory = measure_memory()

    print(".3f"    print(".1f"
    # Test 5: API Performance
    print("\n5️⃣ Testing Simple API Performance...")
    from src.core.simple_api import SimpleAPI

    api_start = time.time()
    api = SimpleAPI()
    api_creation_time = time.time() - api_start
    api_memory = measure_memory()

    print(".3f"    print(".1f"
    # Test 6: Database Throughput
    print("\n6️⃣ Testing Database Throughput...")
    db = core.get_database()

    # Insert test data
    test_data = []
    for i in range(50):
        test_data.append({
            'battery_level': 70 + (i % 30),
            'system_health': 'nominal',
            'current_mode': 'AUTONOMOUS',
            'timestamp': time.time() + i * 0.01
        })

    db_start = time.time()
    for data in test_data:
        db.store_telemetry(data)
    db_time = time.time() - db_start
    db_throughput = len(test_data) / db_time if db_time > 0 else 0
    db_memory = measure_memory()

    print(".1f"    print(".1f"    print(".1f"
    # Test 7: Memory Analysis
    print("\n7️⃣ Memory Analysis...")
    final_memory = measure_memory()
    memory_delta = final_memory - baseline_memory
    memory_pressure = "LOW" if memory_delta < 30 else "MODERATE" if memory_delta < 60 else "HIGH"

    print(".1f"    print(f"   Memory Pressure: {memory_pressure}")
    print(f"   Components Loaded: {len(core.get_status()['loaded_components'])}")

    # Final Report
    print("\n" + "=" * 50)
    print("📊 PERFORMANCE SUMMARY")
    print("=" * 50)

    print("\n⚡ SPEED METRICS:")    print(".3f"    print(".3f"    print(".3f"    print(".3f"    print(".3f"
    

💾 MEMORY METRICS:"    print(".1f"    print(".1f"    print(".1f"    print(f"   Memory Pressure: {memory_pressure}")

    

📈 THROUGHPUT METRICS:"    print(".1f"
    

✅ PERFORMANCE TARGETS:"    startup_ok = startup_time < 1.0
    memory_ok = final_memory < 150
    throughput_ok = db_throughput > 10

    print(f"   Startup Time (< 1s): {'✅ PASS' if startup_ok else '❌ FAIL'}")
    print(f"   Memory Usage (< 150MB): {'✅ PASS' if memory_ok else '❌ FAIL'}")
    print(f"   Database Throughput (> 10 msg/s): {'✅ PASS' if throughput_ok else '❌ FAIL'}")

    overall_status = "✅ READY FOR COMPETITION" if all([startup_ok, memory_ok]) else "⚠️ NEEDS OPTIMIZATION"
    print(f"\n🚀 OVERALL STATUS: {overall_status}")

    print("=" * 50)

if __name__ == "__main__":
    run_performance_test()
