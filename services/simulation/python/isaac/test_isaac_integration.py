#!/usr/bin/env python3
"""
Isaac Sim integration test for URC 2026 rover.

Tests the Isaac Sim backend functionality and integration with
the existing simulation framework.

Usage:
    python3 test_isaac_integration.py

Author: URC 2026 Simulation Team
"""

import sys
import os
import logging
import time
from pathlib import Path

# Add project root to Python path
project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))

from simulation.core.simulation_manager import SimulationManager


def setup_logging():
    """Setup logging for test."""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )


def create_test_config(backend: str) -> dict:
    """Create test configuration for specified backend.
    
    Args:
        backend: Physics backend ("gazebo" or "isaac")
        
    Returns:
        Test configuration dictionary
    """
    config = {
        "physics": {
            "backend": backend,
            "headless": True,  # Run in headless mode for testing
            "physics_dt": 0.016,
            "rendering_dt": 0.016,
        },
        "environment": {
            "mars_gravity": -3.71 if backend == "isaac" else -9.81,
            "atmosphere_pressure": 610.0,
            "dust_enabled": backend == "isaac",
            "wind_enabled": backend == "isaac",
            "terrain_size": [50, 50],
            "crater_count": 5,
            "rock_count": 10,
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
        },
        "ros2": {
            "namespace": "urc",
            "enabled": False,  # Disable ROS for basic test
        },
        "logging": {
            "enabled": True,
            "level": "INFO",
            "structured": False,
            "file": None,
        },
        "time": {
            "step_size": 0.016,
            "use_sim_time": True,
        },
        "monitoring": {
            "enabled": True,
            "interval": 1.0,
        },
        "tracing": {
            "enabled": False,
            "auto_profile_threshold": 0.1,
        },
    }
    
    return config


def test_backend_initialization(backend: str) -> bool:
    """Test physics backend initialization.
    
    Args:
        backend: Physics backend name
        
    Returns:
        True if test successful
    """
    logger = logging.getLogger(__name__)
    logger.info(f"Testing {backend} backend initialization...")
    
    try:
        # Create configuration
        config = create_test_config(backend)
        
        # Initialize simulation manager
        manager = SimulationManager()
        
        # Test initialization
        if not manager.initialize(config):
            logger.error(f"Failed to initialize {backend} backend")
            return False
        
        # Check state
        state = manager.get_state()
        if not state["is_initialized"]:
            logger.error(f"{backend} backend not marked as initialized")
            return False
        
        # Check physics backend
        physics_backend = state["physics_backend"]
        if physics_backend["type"] != backend:
            logger.error(f"Expected {backend} backend, got {physics_backend['type']}")
            return False
        
        logger.info(f"{backend} backend initialized successfully")
        
        # Test stepping
        logger.info(f"Testing {backend} simulation stepping...")
        
        if not manager.start():
            logger.error(f"Failed to start {backend} simulation")
            return False
        
        # Run a few steps
        for i in range(5):
            try:
                # Step simulation
                manager.step(0.016)
                time.sleep(0.016)
                
                # Get updated state
                state = manager.get_state()
                logger.info(f"Step {i+1}: Step count = {state['step_count']}")
                
            except Exception as e:
                logger.error(f"Step {i+1} failed: {e}")
                manager.stop()
                return False
        
        # Stop simulation
        if not manager.stop():
            logger.error(f"Failed to stop {backend} simulation")
            return False
        
        # Get final statistics
        stats = manager.get_statistics()
        logger.info(f"{backend} statistics: {stats}")
        
        logger.info(f"{backend} backend test passed")
        return True
        
    except Exception as e:
        logger.error(f"{backend} backend test failed: {e}")
        return False


def test_backend_comparison():
    """Test comparison between Gazebo and Isaac Sim backends."""
    logger = logging.getLogger(__name__)
    logger.info("Testing backend comparison...")
    
    # Test both backends
    backends = ["gazebo", "isaac"]
    results = {}
    
    for backend in backends:
        logger.info(f"Testing {backend} backend...")
        success = test_backend_initialization(backend)
        results[backend] = success
        
        if success:
            logger.info(f"✓ {backend} backend test passed")
        else:
            logger.error(f"✗ {backend} backend test failed")
    
    # Summary
    logger.info("=== Backend Test Summary ===")
    for backend, success in results.items():
        status = "✓ PASSED" if success else "✗ FAILED"
        logger.info(f"{backend}: {status}")
    
    # Check if at least one backend works
    if any(results.values()):
        logger.info("At least one backend works - integration successful")
        return True
    else:
        logger.error("No backend works - integration failed")
        return False


def test_backend_switching():
    """Test switching between backends."""
    logger = logging.getLogger(__name__)
    logger.info("Testing backend switching...")
    
    try:
        # Test Gazebo first
        config = create_test_config("gazebo")
        manager = SimulationManager()
        
        if not manager.initialize(config):
            logger.error("Failed to initialize Gazebo backend")
            return False
        
        logger.info("Gazebo backend initialized")
        
        # Run a few steps
        manager.start()
        for i in range(3):
            manager.step(0.016)
            time.sleep(0.01)
        manager.stop()
        
        # Test Isaac Sim
        config_isaac = create_test_config("isaac")
        manager_isaac = SimulationManager()
        
        if not manager_isaac.initialize(config_isaac):
            logger.error("Failed to initialize Isaac Sim backend")
            return False
        
        logger.info("Isaac Sim backend initialized")
        
        # Run a few steps
        manager_isaac.start()
        for i in range(3):
            manager_isaac.step(0.016)
            time.sleep(0.01)
        manager_isaac.stop()
        
        logger.info("Backend switching test passed")
        return True
        
    except Exception as e:
        logger.error(f"Backend switching test failed: {e}")
        return False


def main():
    """Main test function."""
    setup_logging()
    logger = logging.getLogger(__name__)
    
    logger.info("=== URC 2026 Isaac Sim Integration Test ===")
    
    # Test environment setup
    isaac_sim_path = "/home/durian/Downloads/isaac-sim-standalone-5.0.0-linux-x86_64"
    if os.path.exists(isaac_sim_path):
        logger.info("✓ Isaac Sim installation found")
    else:
        logger.warning("Isaac Sim installation not found")
    
    # Test backend comparison
    logger.info("\n--- Testing Backend Comparison ---")
    if not test_backend_comparison():
        logger.error("Backend comparison failed")
        sys.exit(1)
    
    # Test backend switching
    logger.info("\n--- Testing Backend Switching ---")
    if not test_backend_switching():
        logger.error("Backend switching failed")
        sys.exit(1)
    
    logger.info("\n=== All Tests Passed ===")
    logger.info("Isaac Sim integration is working correctly!")
    logger.info("\nNext steps:")
    logger.info("1. Run full simulation: python3 simulation/isaac/launch_isaac_sim.py")
    logger.info("2. Compare with Gazebo: python3 simulation/gazebo/launch_gazebo.py")
    logger.info("3. Test specific scenarios with --duration 60")


if __name__ == "__main__":
    main()