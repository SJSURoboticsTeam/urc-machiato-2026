"""Isaac Sim integration for URC 2026 rover simulation.

This module provides the integration layer between URC 2026 simulation framework
and NVIDIA Isaac Sim 5.0.0 for enhanced physics simulation and Mars environment
modeling.

Author: URC 2026 Simulation Team
"""

import os
import sys
import time
import logging
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple
from threading import Thread, Lock

# Isaac Sim path configuration
ISAAC_SIM_PATH = "/home/durian/Downloads/isaac-sim-standalone-5.0.0-linux-x86_64"

# Try to set up Isaac Sim paths
def _setup_isaac_sim_paths():
    """Set up Isaac Sim Python paths dynamically."""
    if ISAAC_SIM_PATH not in sys.path:
        sys.path.insert(0, f"{ISAAC_SIM_PATH}/kit")
        sys.path.insert(0, f"{ISAAC_SIM_PATH}/kit/extscore")
        
        # Add extensions
        exts_path = f"{ISAAC_SIM_PATH}/exts"
        if os.path.exists(exts_path):
            for item in os.listdir(exts_path):
                if item.startswith("isaacsim"):
                    ext_full_path = os.path.join(exts_path, item)
                    if ext_full_path not in sys.path:
                        sys.path.insert(0, ext_full_path)

# Try to import Isaac Sim modules
ISAAC_SIM_AVAILABLE = False
_isaac_import_error = None

try:
    _setup_isaac_sim_paths()
    import carb
    import omni.kit.app
    from omni.isaac.core import World
    from omni.isaac.core.utils.nucleus import get_assets_root_path
    from omni.isaac.core.simulation_context import SimulationContext
    from pxr import Usd, UsdGeom, Gf
    ISAAC_SIM_AVAILABLE = True
except ImportError as e:
    _isaac_import_error = str(e)
    ISAAC_SIM_AVAILABLE = False
except Exception as e:
    _isaac_import_error = str(e)
    ISAAC_SIM_AVAILABLE = False

# Import our own modules
from simulation.core.logging_config import get_simulation_logger
from .physics_engine import IsaacPhysicsEngine
from .sensor_simulation import IsaacSensorSimulation
from .environment_setup import MarsEnvironment


class IsaacIntegration:
    """Main Isaac Sim integration interface for URC rover simulation.
    
    Provides physics simulation, environment setup, and sensor simulation
    capabilities using Isaac Sim 5.0 with Mars-specific configurations.
    
    Features:
        - Mars gravity (3.71 m/s²)
        - Realistic terrain with craters and rocks
        - Advanced sensor simulation
        - ROS 2 integration
        - GPU acceleration
    """
    
    def __init__(self, config: Dict[str, Any]):
        """Initialize Isaac Sim integration.
        
        Args:
            config: Isaac Sim configuration dictionary
        """
        self.logger = get_simulation_logger(__name__)
        self.config = config
        
        # Isaac Sim components
        self.simulation_app = None
        self.simulation_context = None
        self.world = None
        self.physics_engine = None
        self.sensor_simulation = None
        self.mars_environment = None
        
        # State tracking
        self.is_initialized = False
        self.is_running = False
        self.step_count = 0
        self.last_step_time = 0.0
        self.start_time = 0.0
        
        # Mars parameters
        self.mars_gravity = -3.71  # m/s² (Mars gravity)
        self.mars_atmosphere = {
            "pressure": 610.0,  # Pa
            "temperature": -63.0,  # °C average
            "density": 0.020  # kg/m³
        }
        
        # Performance metrics
        self.metrics = {
            "total_steps": 0,
            "avg_step_time": 0.0,
            "min_step_time": float('inf'),
            "max_step_time": 0.0,
            "fps": 0.0,
        }
        
        self._check_availability()
    
    def _check_availability(self):
        """Check and log Isaac Sim availability."""
        if ISAAC_SIM_AVAILABLE:
            self.logger.info("Isaac Sim Python API is available")
        else:
            self.logger.warning(f"Isaac Sim not available: {_isaac_import_error}")
            self.logger.info("Will use fallback simulation when initialized")
    
    def initialize(self) -> bool:
        """Initialize Isaac Sim simulation context and world.
        
        Returns:
            bool: True if initialization successful
        """
        try:
            self.logger.info("Initializing Isaac Sim 5.0.0 integration...")
            
            if not ISAAC_SIM_AVAILABLE:
                self.logger.warning("Isaac Sim libraries not available - using simulation fallback")
                return self._initialize_fallback()
            
            # Initialize simulation app
            self._initialize_simulation_app()
            
            # Initialize physics engine
            self.physics_engine = IsaacPhysicsEngine(self.world, self.config)
            self.physics_engine.initialize()
            
            # Initialize Mars environment
            self.mars_environment = MarsEnvironment(self.world, self.config)
            self.mars_environment.setup_mars_physics()
            self.mars_environment.create_terrain()
            
            # Initialize sensor simulation
            self.sensor_simulation = IsaacSensorSimulation(self.world, self.config)
            self.sensor_simulation.initialize()
            
            self.is_initialized = True
            self.start_time = time.time()
            self.logger.info("Isaac Sim initialized successfully")
            
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to initialize Isaac Sim: {e}")
            self.logger.info("Falling back to simulation mode")
            return self._initialize_fallback()
    
    def _initialize_simulation_app(self):
        """Initialize Isaac Sim application."""
        self.logger.info("Creating Isaac Sim simulation context...")
        
        # Simulation configuration
        simulation_config = {
            "physics_dt": self.config.get("physics_dt", 0.016),
            "rendering_dt": self.config.get("rendering_dt", 0.016),
            "headless": self.config.get("headless", False),
            "physics_engine": self.config.get("physics_engine", "PhysX"),
        }
        
        # Create simulation context
        self.simulation_context = SimulationContext(
            stage_units_in_meters=1.0,
            physics_dt=simulation_config["physics_dt"],
            rendering_dt=simulation_config["rendering_dt"],
        )
        
        # Initialize
        self.simulation_context.initialize_sim_context()
        
        # Create world
        self.world = World(
            physics_dt=simulation_config["physics_dt"],
            rendering_dt=simulation_config["rendering_dt"],
            stage_units_in_meters=1.0
        )
        
        self.logger.info("Isaac Sim simulation context created")
    
    def _initialize_fallback(self) -> bool:
        """Initialize fallback simulation when Isaac Sim is unavailable.
        
        Returns:
            bool: True if fallback initialization successful
        """
        self.logger.info("Initializing fallback simulation mode...")
        
        try:
            # Initialize physics engine (fallback)
            self.physics_engine = IsaacPhysicsEngine(None, self.config)
            self.physics_engine.initialize_fallback()
            
            # Initialize Mars environment (fallback)
            self.mars_environment = MarsEnvironment(None, self.config)
            self.mars_environment.setup_mars_physics()
            self.mars_environment.create_terrain_fallback()
            
            # Initialize sensor simulation (fallback)
            self.sensor_simulation = IsaacSensorSimulation(None, self.config)
            self.sensor_simulation.initialize_fallback()
            
            self.is_initialized = True
            self.start_time = time.time()
            self.logger.info("Fallback simulation mode initialized successfully")
            
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to initialize fallback: {e}")
            return False
    
    def step(self, dt: float) -> Dict[str, Any]:
        """Execute single simulation step.
        
        Args:
            dt: Time step in seconds
            
        Returns:
            Dict containing simulation state
        """
        if not self.is_initialized:
            raise RuntimeError("Isaac Sim not initialized")
        
        step_start = time.time()
        
        try:
            # Step the simulation
            if ISAAC_SIM_AVAILABLE and self.simulation_context:
                self.simulation_context.render()
                self.world.step(render=True)
            else:
                # Fallback simulation step
                self._fallback_step(dt)
            
            # Get sensor data
            sensor_data = {}
            if self.sensor_simulation:
                sensor_data = self.sensor_simulation.get_sensor_data()
            
            # Get environment state
            env_state = {}
            if self.mars_environment:
                env_state = self.mars_environment.get_environment_state()
            
            # Get physics state
            physics_state = {}
            if self.physics_engine:
                physics_state = self.physics_engine.get_physics_state()
            
            # Calculate metrics
            step_time = time.time() - step_start
            self._update_metrics(step_time)
            
            # Combine state
            simulation_state = {
                "timestamp": time.time() - self.start_time,
                "step_count": self.step_count,
                "dt": dt,
                "sensor_data": sensor_data,
                "environment": env_state,
                "physics": physics_state,
                "step_time": step_time,
                "mode": "isaac" if ISAAC_SIM_AVAILABLE else "fallback",
                "metrics": self.get_metrics(),
            }
            
            self.step_count += 1
            self.last_step_time = time.time()
            
            return simulation_state
            
        except Exception as e:
            self.logger.error(f"Simulation step failed: {e}")
            raise
    
    def _fallback_step(self, dt: float):
        """Execute fallback simulation step."""
        if self.physics_engine:
            self.physics_engine.step_fallback(dt)
        if self.sensor_simulation:
            self.sensor_simulation.step_fallback(dt)
    
    def _update_metrics(self, step_time: float):
        """Update performance metrics."""
        self.metrics["total_steps"] += 1
        self.metrics["min_step_time"] = min(self.metrics["min_step_time"], step_time)
        self.metrics["max_step_time"] = max(self.metrics["max_step_time"], step_time)
        
        # Calculate running average
        n = self.metrics["total_steps"]
        old_avg = self.metrics["avg_step_time"]
        self.metrics["avg_step_time"] = old_avg + (step_time - old_avg) / n
        
        # Calculate FPS
        if step_time > 0:
            self.metrics["fps"] = 1.0 / step_time
    
    def get_metrics(self) -> Dict[str, Any]:
        """Get performance metrics."""
        return self.metrics.copy()
    
    def reset(self) -> bool:
        """Reset simulation to initial state.
        
        Returns:
            bool: True if reset successful
        """
        try:
            if self.world:
                self.world.reset()
            
            if self.mars_environment:
                self.mars_environment.reset()
            
            if self.sensor_simulation:
                self.sensor_simulation.reset()
            
            self.step_count = 0
            self.last_step_time = time.time()
            
            # Reset metrics
            self.metrics = {
                "total_steps": 0,
                "avg_step_time": 0.0,
                "min_step_time": float('inf'),
                "max_step_time": 0.0,
                "fps": 0.0,
            }
            
            self.logger.info("Simulation reset")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to reset: {e}")
            return False
    
    def start(self) -> bool:
        """Start simulation.
        
        Returns:
            bool: True if started successfully
        """
        if not self.is_initialized:
            self.logger.error("Cannot start - not initialized")
            return False
        
        try:
            self.is_running = True
            self.logger.info("Simulation started")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to start: {e}")
            return False
    
    def stop(self) -> bool:
        """Stop simulation.
        
        Returns:
            bool: True if stopped successfully
        """
        try:
            self.is_running = False
            self.logger.info(f"Simulation stopped. Total steps: {self.step_count}")
            self.logger.info(f"Average FPS: {self.metrics['fps']:.2f}")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to stop: {e}")
            return False
    
    def cleanup(self) -> bool:
        """Clean up Isaac Sim resources.
        
        Returns:
            bool: True if cleanup successful
        """
        try:
            self.stop()
            
            if self.sensor_simulation:
                self.sensor_simulation.cleanup()
            
            if self.mars_environment:
                self.mars_environment.cleanup()
            
            if self.physics_engine:
                self.physics_engine.cleanup()
            
            if self.world:
                self.world.clear()
            
            if self.simulation_context:
                self.simulation_context.stop()
            
            self.is_initialized = False
            self.logger.info("Resources cleaned up")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to cleanup: {e}")
            return False
    
    def get_status(self) -> Dict[str, Any]:
        """Get current simulation status.
        
        Returns:
            Dict containing simulation status
        """
        return {
            "is_initialized": self.is_initialized,
            "is_running": self.is_running,
            "step_count": self.step_count,
            "last_step_time": self.last_step_time,
            "mars_gravity": self.mars_gravity,
            "mars_atmosphere": self.mars_atmosphere,
            "world_active": self.world is not None,
            "simulation_context_active": self.simulation_context is not None,
            "isaac_available": ISAAC_SIM_AVAILABLE,
            "mode": "isaac" if ISAAC_SIM_AVAILABLE else "fallback",
            "metrics": self.get_metrics(),
        }
    
    def add_rover(self, rover_config: Dict[str, Any]) -> bool:
        """Add rover to simulation.
        
        Args:
            rover_config: Rover configuration
            
        Returns:
            bool: True if rover added successfully
        """
        try:
            self.logger.info(f"Adding rover: {rover_config.get('name', 'unknown')}")
            # Rover addition would be implemented here
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to add rover: {e}")
            return False
    
    @staticmethod
    def is_available() -> bool:
        """Check if Isaac Sim is available.
        
        Returns:
            bool: True if Isaac Sim Python API is available
        """
        return ISAAC_SIM_AVAILABLE
    
    @staticmethod
    def get_version() -> str:
        """Get Isaac Sim version.
        
        Returns:
            str: Version string or "unavailable"
        """
        if ISAAC_SIM_AVAILABLE:
            try:
                import omni.isaac.core
                return omni.isaac.core.__version__
            except:
                return "5.0.0"
        return "unavailable"
