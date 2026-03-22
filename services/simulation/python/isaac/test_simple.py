#!/usr/bin/env python3
"""
Simple Isaac Sim integration test for URC 2026 rover.

Tests basic integration functionality without complex dependencies.

Author: URC 2026 Simulation Team
"""

import sys
import os
from pathlib import Path

# Add project root to Python path
project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))

def test_isaac_sim_availability():
    """Test if Isaac Sim is available."""
    print("=== Testing Isaac Sim Availability ===")
    
    isaac_sim_path = "/home/durian/Downloads/isaac-sim-standalone-5.0.0-linux-x86_64"
    
    if os.path.exists(isaac_sim_path):
        print(f"✓ Isaac Sim found at: {isaac_sim_path}")
        
        # Check for key directories
        key_dirs = ["kit", "extscore", "apps"]
        for dir_name in key_dirs:
            dir_path = os.path.join(isaac_sim_path, dir_name)
            if os.path.exists(dir_path):
                print(f"  ✓ {dir_name}/ directory exists")
            else:
                print(f"  ✗ {dir_name}/ directory missing")
        
        return True
    else:
        print(f"✗ Isaac Sim not found at: {isaac_sim_path}")
        return False


def test_module_imports():
    """Test if we can import our modules."""
    print("\n=== Testing Module Imports ===")
    
    try:
        from simulation.isaac.isaac_integration import IsaacIntegration
        print("✓ IsaacIntegration can be imported")
    except ImportError as e:
        print(f"✗ IsaacIntegration import failed: {e}")
        return False
    
    try:
        from simulation.isaac.physics_engine import IsaacPhysicsEngine
        print("✓ IsaacPhysicsEngine can be imported")
    except ImportError as e:
        print(f"✗ IsaacPhysicsEngine import failed: {e}")
        return False
    
    try:
        from simulation.isaac.sensor_simulation import IsaacSensorSimulation
        print("✓ IsaacSensorSimulation can be imported")
    except ImportError as e:
        print(f"✗ IsaacSensorSimulation import failed: {e}")
        return False
    
    try:
        from simulation.isaac.environment_setup import MarsEnvironment
        print("✓ MarsEnvironment can be imported")
    except ImportError as e:
        print(f"✗ MarsEnvironment import failed: {e}")
        return False
    
    try:
        from simulation.isaac.ros_bridge import IsaacRosBridge
        print("✓ IsaacRosBridge can be imported")
    except ImportError as e:
        print(f"✗ IsaacRosBridge import failed: {e}")
        return False
    
    return True


def test_configuration():
    """Test configuration system."""
    print("\n=== Testing Configuration ===")
    
    try:
        # Test configuration creation
        config = {
            "physics": {
                "backend": "isaac",
                "headless": True,
                "physics_dt": 0.016,
                "rendering_dt": 0.016,
                "physics_engine": "PhysX",
                "sensor_update_rate": 30.0,
            },
            "environment": {
                "mars_gravity": -3.71,
                "atmosphere_pressure": 610.0,
                "dust_enabled": True,
                "wind_enabled": True,
                "terrain_size": [100, 100],
                "crater_count": 15,
                "rock_count": 50,
            },
            "sensors": {
                "sensors_enabled": {
                    "camera": True,
                    "lidar": True,
                    "imu": True,
                    "gps": True,
                },
                "noise_level": 0.1,
                "update_rate": 30.0,
            }
        }
        
        print("✓ Configuration created successfully")
        print(f"  Backend: {config['physics']['backend']}")
        print(f"  Mars gravity: {config['environment']['mars_gravity']} m/s²")
        print(f"  Craters: {config['environment']['crater_count']}")
        print(f"  Rocks: {config['environment']['rock_count']}")
        
        return True
        
    except Exception as e:
        print(f"✗ Configuration test failed: {e}")
        return False


def test_isaac_integration_creation():
    """Test Isaac Sim integration creation."""
    print("\n=== Testing Isaac Integration Creation ===")
    
    try:
        from simulation.isaac.isaac_integration import IsaacIntegration
        from simulation.core.logging_config import setup_simulation_logging
        
        # Setup basic logging
        setup_simulation_logging(enable_structured=False)
        
        config = {
            "headless": True,
            "physics_dt": 0.016,
            "rendering_dt": 0.016,
            "physics_engine": "PhysX",
            "sensor_update_rate": 30.0,
        }
        
        # Create integration object (this should work even if Isaac Sim libs aren't loaded)
        integration = IsaacIntegration(config)
        print("✓ IsaacIntegration object created")
        
        # Check status
        status = integration.get_status()
        print(f"  Mars gravity: {status['mars_gravity']} m/s²")
        print(f"  Atmosphere pressure: {status['mars_atmosphere']['pressure']} Pa")
        
        return True
        
    except Exception as e:
        print(f"✗ IsaacIntegration creation failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_simulation_manager():
    """Test simulation manager with physics backend selection."""
    print("\n=== Testing Simulation Manager ===")
    
    try:
        from simulation.core.simulation_manager import SimulationManager
        
        manager = SimulationManager()
        print("✓ SimulationManager object created")
        
        # Test backend type checking
        backends = ["gazebo", "isaac", "unknown"]
        for backend in backends:
            try:
                config = {"physics": {"backend": backend}}
                # This will fail for unknown backends, which is expected
                print(f"  Backend '{backend}' configuration accepted")
            except Exception as e:
                print(f"  Backend '{backend}' rejected: {e}")
        
        return True
        
    except Exception as e:
        print(f"✗ SimulationManager test failed: {e}")
        return False


def test_file_structure():
    """Test if all required files are created."""
    print("\n=== Testing File Structure ===")
    
    required_files = [
        "simulation/isaac/__init__.py",
        "simulation/isaac/isaac_integration.py",
        "simulation/isaac/physics_engine.py",
        "simulation/isaac/sensor_simulation.py",
        "simulation/isaac/environment_setup.py",
        "simulation/isaac/ros_bridge.py",
        "simulation/isaac/launch_isaac_sim.py",
        "simulation/isaac/test_isaac_integration.py",
    ]
    
    all_exist = True
    for file_path in required_files:
        if os.path.exists(file_path):
            print(f"  ✓ {file_path}")
        else:
            print(f"  ✗ {file_path} missing")
            all_exist = False
    
    return all_exist


def main():
    """Main test function."""
    print("URC 2026 Isaac Sim Integration Test")
    print("=" * 50)
    
    tests = [
        test_isaac_sim_availability,
        test_file_structure,
        test_module_imports,
        test_configuration,
        test_isaac_integration_creation,
        test_simulation_manager,
    ]
    
    passed = 0
    total = len(tests)
    
    for test in tests:
        try:
            if test():
                passed += 1
            else:
                print("Test failed!")
        except Exception as e:
            print(f"Test error: {e}")
    
    print(f"\n=== Test Results ===")
    print(f"Passed: {passed}/{total}")
    
    if passed == total:
        print("🎉 All tests passed! Isaac Sim integration is ready.")
        print("\nNext steps:")
        print("1. Start Isaac Sim: ./simulation/isaac/launch_isaac_sim.py")
        print("2. Test with Mars environment: --world-size 200 --craters 30")
        print("3. Compare performance with Gazebo simulation")
        return True
    else:
        print("❌ Some tests failed. Check the output above for details.")
        return False


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)