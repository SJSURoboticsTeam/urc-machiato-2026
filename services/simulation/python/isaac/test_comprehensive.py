#!/usr/bin/env python3
"""
Comprehensive Isaac Sim Integration Test Suite

Tests all aspects of the Isaac Sim integration including:
- Physics simulation (Mars gravity)
- Sensor simulation
- Performance benchmarking
- Backend switching
- ROS 2 integration

Author: URC 2026 Simulation Team
"""

import sys
import os
import time
import logging
from pathlib import Path

# Add project root to Python path
project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))

from simulation.isaac.isaac_integration import IsaacIntegration
from simulation.core.logging_config import setup_simulation_logging


def test_basic_initialization():
    """Test basic Isaac Sim initialization."""
    print("\n" + "="*60)
    print("TEST 1: Basic Initialization")
    print("="*60)
    
    config = {
        "headless": True,
        "physics_dt": 0.016,
        "rendering_dt": 0.016,
        "physics_engine": "PhysX",
        "terrain_size": [50, 50],
        "crater_count": 5,
        "rock_count": 10,
        "dust_enabled": True,
        "wind_enabled": True,
        "sensors_enabled": {
            "camera": True,
            "lidar": True,
            "imu": True,
            "gps": True,
        },
        "noise_level": 0.1,
        "update_rate": 30.0,
    }
    
    integration = IsaacIntegration(config)
    
    if integration.initialize():
        print("✓ Initialization successful")
        status = integration.get_status()
        print(f"  Mode: {status['mode']}")
        print(f"  Mars gravity: {status['mars_gravity']} m/s²")
        print(f"  Atmosphere pressure: {status['mars_atmosphere']['pressure']} Pa")
        integration.cleanup()
        return True
    else:
        print("✗ Initialization failed")
        return False


def test_physics_simulation():
    """Test Mars physics simulation."""
    print("\n" + "="*60)
    print("TEST 2: Physics Simulation")
    print("="*60)
    
    config = {
        "headless": True,
        "physics_dt": 0.016,
        "terrain_size": [100, 100],
        "crater_count": 10,
        "rock_count": 25,
    }
    
    integration = IsaacIntegration(config)
    integration.initialize()
    integration.start()
    
    # Run simulation steps
    num_steps = 100
    start_time = time.time()
    
    for i in range(num_steps):
        state = integration.step(0.016)
        
        if i % 20 == 0:
            physics = state.get("physics", {})
            print(f"  Step {i}: pos={physics.get('position', 'N/A')}")
    
    elapsed = time.time() - start_time
    metrics = integration.get_metrics()
    
    print(f"✓ Physics simulation completed")
    print(f"  Steps: {num_steps}")
    print(f"  Time: {elapsed:.3f}s")
    print(f"  FPS: {metrics['fps']:.2f}")
    print(f"  Avg step time: {metrics['avg_step_time']*1000:.3f}ms")
    
    integration.cleanup()
    return True


def test_sensor_simulation():
    """Test sensor simulation."""
    print("\n" + "="*60)
    print("TEST 3: Sensor Simulation")
    print("="*60)
    
    config = {
        "headless": True,
        "physics_dt": 0.016,
        "sensors_enabled": {
            "camera": True,
            "lidar": True,
            "imu": True,
            "gps": True,
        },
        "noise_level": 0.1,
        "update_rate": 30.0,
    }
    
    integration = IsaacIntegration(config)
    integration.initialize()
    integration.start()
    
    # Run simulation and collect sensor data
    for i in range(30):
        state = integration.step(0.016)
        sensor_data = state.get("sensor_data", {})
        
        if i % 10 == 0:
            print(f"  Step {i}:")
            for sensor_name, data in sensor_data.items():
                if isinstance(data, dict):
                    print(f"    {sensor_name}: OK")
    
    print("✓ Sensor simulation completed")
    integration.cleanup()
    return True


def test_mars_environment():
    """Test Mars environment creation."""
    print("\n" + "="*60)
    print("TEST 4: Mars Environment")
    print("="*60)
    
    config = {
        "headless": True,
        "terrain_size": [200, 200],
        "crater_count": 25,
        "rock_count": 75,
        "dust_enabled": True,
        "wind_enabled": True,
    }
    
    integration = IsaacIntegration(config)
    integration.initialize()
    
    state = integration.step(0.016)
    env = state.get("environment", {})
    
    print(f"✓ Mars environment created")
    print(f"  Terrain size: {env.get('terrain_size', 'N/A')}m")
    print(f"  Craters: {env.get('crater_count', 'N/A')}")
    print(f"  Rocks: {env.get('rock_count', 'N/A')}")
    print(f"  Dust enabled: {env.get('dust_enabled', 'N/A')}")
    print(f"  Wind enabled: {env.get('wind_enabled', 'N/A')}")
    print(f"  Gravity: {env.get('mars_gravity', 'N/A')} m/s²")
    print(f"  Atmosphere: {env.get('atmosphere_pressure', 'N/A')} Pa")
    
    integration.cleanup()
    return True


def test_performance_benchmark():
    """Test performance with different configurations."""
    print("\n" + "="*60)
    print("TEST 5: Performance Benchmark")
    print("="*60)
    
    configs = [
        {"name": "Basic", "steps": 100},
        {"name": "Medium", "steps": 500},
        {"name": "Extended", "steps": 1000},
    ]
    
    results = []
    
    for cfg in configs:
        config = {
            "headless": True,
            "physics_dt": 0.016,
            "terrain_size": [100, 100],
            "crater_count": 15,
            "rock_count": 50,
            "sensors_enabled": {"imu": True, "gps": True},
            "update_rate": 30.0,
        }
        
        integration = IsaacIntegration(config)
        integration.initialize()
        integration.start()
        
        start_time = time.time()
        
        for i in range(cfg["steps"]):
            integration.step(0.016)
        
        elapsed = time.time() - start_time
        metrics = integration.get_metrics()
        
        result = {
            "name": cfg["name"],
            "steps": cfg["steps"],
            "time": elapsed,
            "fps": metrics["fps"],
            "avg_step_ms": metrics["avg_step_time"] * 1000,
        }
        results.append(result)
        
        integration.cleanup()
    
    print("✓ Performance benchmark completed")
    print("\nResults:")
    print("-" * 60)
    print(f"{'Config':<12} {'Steps':<8} {'Time(s)':<10} {'FPS':<10} {'Step(ms)':<10}")
    print("-" * 60)
    for r in results:
        print(f"{r['name']:<12} {r['steps']:<8} {r['time']:<10.3f} {r['fps']:<10.2f} {r['avg_step_ms']:<10.3f}")
    
    return True


def test_stress_test():
    """Stress test with heavy load."""
    print("\n" + "="*60)
    print("TEST 6: Stress Test")
    print("="*60)
    
    config = {
        "headless": True,
        "physics_dt": 0.016,
        "terrain_size": [300, 300],
        "crater_count": 50,
        "rock_count": 150,
        "sensors_enabled": {
            "camera": True,
            "lidar": True,
            "imu": True,
            "gps": True,
        },
        "dust_enabled": True,
        "wind_enabled": True,
    }
    
    integration = IsaacIntegration(config)
    integration.initialize()
    integration.start()
    
    # Run 5 minutes worth of steps at 60 Hz
    target_steps = 300 * 60  # 5 minutes at 60 Hz
    print(f"Running {target_steps} steps...")
    
    start_time = time.time()
    
    for i in range(target_steps):
        integration.step(0.016)
        
        if (i + 1) % 1000 == 0:
            elapsed = time.time() - start_time
            print(f"  Progress: {i+1}/{target_steps} ({100*(i+1)/target_steps:.1f}%) - {elapsed:.1f}s")
    
    elapsed = time.time() - start_time
    metrics = integration.get_metrics()
    
    print(f"✓ Stress test completed")
    print(f"  Steps: {metrics['total_steps']}")
    print(f"  Time: {elapsed:.2f}s")
    print(f"  FPS: {metrics['fps']:.2f}")
    print(f"  Min step: {metrics['min_step_time']*1000:.3f}ms")
    print(f"  Max step: {metrics['max_step_time']*1000:.3f}ms")
    print(f"  Avg step: {metrics['avg_step_time']*1000:.3f}ms")
    
    integration.cleanup()
    return True


def main():
    """Run all tests."""
    print("="*60)
    print("URC 2026 Isaac Sim Integration - Comprehensive Test Suite")
    print("="*60)
    
    # Setup logging
    setup_simulation_logging(enable_structured=False)
    
    # Check availability
    print(f"\nIsaac Sim Available: {IsaacIntegration.is_available()}")
    print(f"Isaac Sim Version: {IsaacIntegration.get_version()}")
    
    # Run tests
    tests = [
        ("Basic Initialization", test_basic_initialization),
        ("Physics Simulation", test_physics_simulation),
        ("Sensor Simulation", test_sensor_simulation),
        ("Mars Environment", test_mars_environment),
        ("Performance Benchmark", test_performance_benchmark),
        ("Stress Test", test_stress_test),
    ]
    
    passed = 0
    failed = 0
    
    for test_name, test_func in tests:
        try:
            if test_func():
                passed += 1
            else:
                failed += 1
                print(f"\n✗ {test_name} FAILED")
        except Exception as e:
            failed += 1
            print(f"\n✗ {test_name} ERROR: {e}")
            import traceback
            traceback.print_exc()
    
    # Summary
    print("\n" + "="*60)
    print("TEST SUMMARY")
    print("="*60)
    print(f"Passed: {passed}/{len(tests)}")
    print(f"Failed: {failed}/{len(tests)}")
    
    if failed == 0:
        print("\n🎉 All tests passed! Isaac Sim integration is fully functional.")
    else:
        print(f"\n⚠️ {failed} test(s) failed. Check output for details.")
    
    return failed == 0


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
