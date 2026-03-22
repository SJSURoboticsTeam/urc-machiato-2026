#!/usr/bin/env python3
"""
Isaac Sim launcher for URC 2026 rover simulation.

Launches Isaac Sim with Mars environment and URC rover integration.
Provides parallel testing capability with existing Gazebo simulation.

Usage:
    python3 launch_isaac_sim.py
    python3 launch_isaac_sim.py --headless
    python3 launch_isaac_sim.py --world mars_desert

Author: URC 2026 Simulation Team
"""

import argparse
import os
import sys
import logging
import time
from pathlib import Path
from typing import Dict, Any, Optional

# Add project root to Python path
project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))

# Add Isaac Sim to Python path
ISAAC_SIM_PATH = "/home/durian/Downloads/isaac-sim-standalone-5.0.0-linux-x86_64"
if ISAAC_SIM_PATH not in sys.path:
    sys.path.append(f"{ISAAC_SIM_PATH}/kit")
    sys.path.append(f"{ISAAC_SIM_PATH}/kit/extscore")

from simulation.core.simulation_manager import SimulationManager
from simulation.core.logging_config import setup_simulation_logging


def setup_environment() -> bool:
    """Setup environment for Isaac Sim.
    
    Returns:
        bool: True if setup successful
    """
    try:
        # Set Isaac Sim environment variables
        os.environ["ISAAC_SIM_PATH"] = ISAAC_SIM_PATH
        os.environ["EXP_PATH"] = f"{ISAAC_SIM_PATH}/apps"
        os.environ["APP_NAME"] = "isaac.sim.base"
        
        # Set URC environment
        os.environ["URC_ENV"] = "simulation"
        os.environ["SIMULATION_BACKEND"] = "isaac"
        
        print(f"Isaac Sim path: {ISAAC_SIM_PATH}")
        print(f"Project root: {project_root}")
        
        return True
        
    except Exception as e:
        print(f"Failed to setup environment: {e}")
        return False


def create_isaac_config(args: argparse.Namespace) -> Dict[str, Any]:
    """Create Isaac Sim configuration.
    
    Args:
        args: Command line arguments
        
    Returns:
        Dict containing configuration
    """
    config = {
        "physics": {
            "backend": "isaac",
            "headless": args.headless,
            "physics_dt": 0.016,
            "rendering_dt": 0.016,
            "physics_engine": "PhysX",
            "sensor_update_rate": args.sensor_rate,
        },
        "environment": {
            "tier": "extreme",
            "type": "mars",
            "mars_gravity": -3.71,
            "atmosphere_pressure": 610.0,
            "dust_enabled": not args.no_dust,
            "wind_enabled": not args.no_wind,
            "terrain_size": [args.world_size, args.world_size],
            "crater_count": args.craters,
            "rock_count": args.rocks,
        },
        "sensors": [
            {
                "name": "imu",
                "type": "imu",
                "enabled": not args.no_imu,
                "noise_level": args.noise_level,
                "update_rate": args.sensor_rate,
            },
            {
                "name": "gps",
                "type": "gps",
                "enabled": not args.no_gps,
                "noise_level": args.noise_level,
                "update_rate": args.sensor_rate,
            },
        ],
        "network": {
            "enabled": False,  # Disable for basic test
            "profile": "perfect",
        },
        "ros2": {
            "namespace": "urc",
            "enabled": not args.no_ros,
        },
        "logging": {
            "enabled": True,
            "level": "INFO" if not args.debug else "DEBUG",
            "structured": True,
            "file": "isaac_simulation.log",
        },
        "time": {
            "step_size": 0.016,
            "use_sim_time": True,
        },
        "recording": {
            "enabled": args.record,
            "format": "bag",
        },
        "monitoring": {
            "enabled": True,
            "interval": 1.0,
        },
        "rover": {
            "model": "urc_rover",
            "enabled": True,
        },
        "bridges": {},  # Empty for basic test
        "tracing": {
            "enabled": args.debug,
            "auto_profile_threshold": 0.1,
        },
    }
    
    return config


def main():
    """Main launcher function."""
    parser = argparse.ArgumentParser(
        description="Launch URC 2026 Isaac Sim simulation"
    )
    
    # Simulation options
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run in headless mode (no GUI)"
    )
    parser.add_argument(
        "--debug",
        action="store_true", 
        help="Enable debug logging"
    )
    
    # Environment options
    parser.add_argument(
        "--world-size",
        type=int,
        default=100,
        help="World size in meters (default: 100)"
    )
    parser.add_argument(
        "--craters",
        type=int,
        default=15,
        help="Number of craters to generate"
    )
    parser.add_argument(
        "--rocks",
        type=int,
        default=50,
        help="Number of rocks to generate"
    )
    parser.add_argument(
        "--no-dust",
        action="store_true",
        help="Disable dust simulation"
    )
    parser.add_argument(
        "--no-wind",
        action="store_true",
        help="Disable wind simulation"
    )
    
    # Sensor options
    parser.add_argument(
        "--sensor-rate",
        type=float,
        default=30.0,
        help="Sensor update rate in Hz (default: 30)"
    )
    parser.add_argument(
        "--noise-level",
        type=float,
        default=0.1,
        help="Sensor noise level (default: 0.1)"
    )
    parser.add_argument(
        "--no-camera",
        action="store_true",
        help="Disable camera sensor"
    )
    parser.add_argument(
        "--no-lidar",
        action="store_true",
        help="Disable lidar sensor"
    )
    parser.add_argument(
        "--no-imu",
        action="store_true",
        help="Disable IMU sensor"
    )
    parser.add_argument(
        "--no-gps",
        action="store_true",
        help="Disable GPS sensor"
    )
    
    # System options
    parser.add_argument(
        "--no-ros",
        action="store_true",
        help="Disable ROS 2 bridge"
    )
    parser.add_argument(
        "--record",
        action="store_true",
        help="Record simulation data"
    )
    parser.add_argument(
        "--duration",
        type=int,
        default=0,
        help="Simulation duration in seconds (0: infinite)"
    )
    
    args = parser.parse_args()
    
    # Setup environment
    if not setup_environment():
        sys.exit(1)
    
    # Setup logging
    log_level_str = "DEBUG" if args.debug else "INFO"
    setup_simulation_logging(
        log_level=log_level_str,
        log_file="isaac_simulation.log",
        enable_structured=False,  # Use basic logging for now
        enable_json=False
    )
    
    logger = logging.getLogger(__name__)
    
    logger.info("=== URC 2026 Isaac Sim Simulation Launcher ===")
    logger.info(f"Arguments: {vars(args)}")
    
    # Create configuration
    config = create_isaac_config(args)
    logger.info("Configuration created")
    
    # Initialize simulation manager
    logger.info("Initializing simulation manager...")
    simulation_manager = SimulationManager()
    
    # Initialize simulation
    logger.info("Initializing Isaac Sim simulation...")
    if not simulation_manager.initialize(config):
        logger.error("Failed to initialize simulation")
        sys.exit(1)
    
    logger.info("Simulation initialized successfully")
    
    # Start simulation
    logger.info("Starting Isaac Sim simulation...")
    if not simulation_manager.start():
        logger.error("Failed to start simulation")
        sys.exit(1)
    
    logger.info("Simulation started successfully")
    
    try:
        # Run simulation
        start_time = time.time()
        step_count = 0
        
        while True:
            # Check duration limit
            if args.duration > 0 and (time.time() - start_time) > args.duration:
                logger.info("Simulation duration reached, stopping...")
                break
            
            # Simulation runs automatically in background thread
            time.sleep(1.0)
            
            # Log status every 10 seconds
            if step_count % 10 == 0:
                state = simulation_manager.get_state()
                logger.info(
                    f"Simulation status - Step: {state['step_count']}, "
                    f"Time: {state['current_time']:.2f}s, "
                    f"Backend: {state['physics_backend']['type']}"
                )
            
            step_count += 1
            
    except KeyboardInterrupt:
        logger.info("Simulation interrupted by user")
    
    finally:
        # Stop simulation
        logger.info("Stopping simulation...")
        simulation_manager.stop()
        
        # Get final statistics
        stats = simulation_manager.get_statistics()
        logger.info(f"Final statistics: {stats}")
        
        logger.info("Isaac Sim simulation stopped")
        print("=== Simulation Complete ===")


if __name__ == "__main__":
    main()