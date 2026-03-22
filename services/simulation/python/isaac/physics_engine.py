"""Isaac Sim physics engine for Mars rover simulation.

Configures PhysX physics engine with Mars-specific parameters
including gravity, atmosphere, and terrain interaction.

Author: URC 2026 Simulation Team
"""

import logging
import numpy as np
from typing import Any, Dict, Optional

from simulation.core.logging_config import get_simulation_logger


class IsaacPhysicsEngine:
    """Isaac Sim physics engine with Mars-specific configurations.
    
    Provides realistic physics simulation for Mars environment including
    reduced gravity, Mars atmosphere properties, and appropriate
    material properties for Mars terrain.
    """
    
    def __init__(self, world, config: Dict[str, Any]):
        """Initialize physics engine.
        
        Args:
            world: Isaac Sim world instance (None for fallback)
            config: Physics configuration
        """
        self.logger = get_simulation_logger(__name__)
        self.world = world
        self.config = config
        
        # Mars physical constants
        self.mars_gravity = -3.71  # m/s² (38% of Earth's)
        self.mars_atmosphere_density = 0.020  # kg/m³
        self.mars_temperature = 210.0  # K (-63°C)
        self.mars_pressure = 610.0  # Pa
        
        # Physics settings
        self.physics_dt = config.get("physics_dt", 0.016)  # 60 Hz
        self.substeps = config.get("substeps", 2)
        
        self.is_initialized = False
        self.physics_scene = None
        
        # Fallback state
        self._fallback_state = {
            "position": [0.0, 0.0, 0.5],
            "velocity": [0.0, 0.0, 0.0],
            "acceleration": [0.0, 0.0, self.mars_gravity],
            "angular_velocity": [0.0, 0.0, 0.0],
        }
    
    def initialize(self) -> bool:
        """Initialize physics engine with Mars parameters.
        
        Returns:
            bool: True if initialization successful
        """
        try:
            self.logger.info("Initializing Isaac Sim physics engine...")
            
            # Configure physics scene
            self._configure_physics_scene()
            
            # Setup Mars gravity
            self._setup_mars_gravity()
            
            # Configure material properties for Mars
            self._setup_mars_materials()
            
            # Configure atmosphere simulation
            self._setup_mars_atmosphere()
            
            self.is_initialized = True
            self.logger.info("Physics engine initialized with Mars parameters")
            
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to initialize physics engine: {e}")
            return False
    
    def _configure_physics_scene(self):
        """Configure the physics scene settings."""
        try:
            # Get physics scene
            self.physics_scene = self.world.scene.get_physics_context()
            
            # Set physics timestep
            self.physics_scene.set_physics_dt(self.physics_dt)
            
            # Configure PhysX settings
            self.physics_scene.set_gravity([0.0, 0.0, self.mars_gravity])
            
            # Set solver settings for Mars terrain
            self.physics_scene.set_solver_position_iterations(8)
            self.physics_scene.set_solver_velocity_iterations(4)
            
            # Configure collision settings
            self.physics_scene.set_broadphase_type("MBP")
            
            self.logger.info("Physics scene configured")
            
        except Exception as e:
            self.logger.error(f"Failed to configure physics scene: {e}")
            raise
    
    def _setup_mars_gravity(self):
        """Configure Mars-specific gravity."""
        try:
            gravity_vector = [0.0, 0.0, self.mars_gravity]
            self.physics_scene.set_gravity(gravity_vector)
            self.logger.info(f"Mars gravity set to {self.mars_gravity} m/s²")
            
        except Exception as e:
            self.logger.error(f"Failed to setup Mars gravity: {e}")
            raise
    
    def _setup_mars_materials(self):
        """Setup material properties for Mars environment."""
        try:
            # Mars regolith material properties
            self.mars_regolith_properties = {
                "static_friction": 0.8,
                "dynamic_friction": 0.6,
                "restitution": 0.3,
                "density": 1500.0,  # kg/m³
            }
            
            # Mars rock material properties  
            self.mars_rock_properties = {
                "static_friction": 0.9,
                "dynamic_friction": 0.7,
                "restitution": 0.2,
                "density": 2800.0,  # kg/m³
            }
            
            self.logger.info("Mars material properties configured")
            
        except Exception as e:
            self.logger.error(f"Failed to setup Mars materials: {e}")
            raise
    
    def _setup_mars_atmosphere(self):
        """Setup Mars atmosphere for drag calculations."""
        try:
            self.atmosphere_config = {
                "density": self.mars_atmosphere_density,
                "temperature": self.mars_temperature,
                "pressure": self.mars_pressure,
                "drag_coefficient": 0.47,
            }
            
            self.logger.info(f"Mars atmosphere configured: density={self.mars_atmosphere_density} kg/m³")
            
        except Exception as e:
            self.logger.error(f"Failed to setup Mars atmosphere: {e}")
            raise
    
    # Fallback methods
    def initialize_fallback(self) -> bool:
        """Initialize fallback physics engine when Isaac Sim unavailable."""
        self.logger.info("Initializing fallback physics engine with Mars gravity...")
        
        # Mars gravity
        self._fallback_state["acceleration"][2] = self.mars_gravity
        
        self.is_initialized = True
        self.logger.info(f"Fallback physics initialized - Mars gravity: {self.mars_gravity} m/s²")
        return True
    
    def step_fallback(self, dt: float):
        """Execute fallback physics step.
        
        Simple Euler integration with Mars gravity.
        """
        # Apply gravity
        self._fallback_state["acceleration"][2] = self.mars_gravity
        
        # Update velocity (Euler integration)
        for i in range(3):
            self._fallback_state["velocity"][i] += self._fallback_state["acceleration"][i] * dt
        
        # Update position
        for i in range(3):
            self._fallback_state["position"][i] += self._fallback_state["velocity"][i] * dt
        
        # Ground collision (simple)
        if self._fallback_state["position"][2] < 0.0:
            self._fallback_state["position"][2] = 0.0
            self._fallback_state["velocity"][2] = 0.0
    
    def get_physics_state(self) -> Dict[str, Any]:
        """Get current physics engine state.
        
        Returns:
            Dict containing physics state information
        """
        try:
            if not self.is_initialized:
                return {"status": "not_initialized"}
            
            if self.world is None:
                # Return fallback state
                return {
                    "gravity": self.mars_gravity,
                    "physics_dt": self.physics_dt,
                    "mode": "fallback",
                    "position": self._fallback_state["position"],
                    "velocity": self._fallback_state["velocity"],
                    "acceleration": self._fallback_state["acceleration"],
                }
            
            # Return Isaac Sim state
            physics_info = {
                "gravity": self.mars_gravity,
                "physics_dt": self.physics_dt,
                "mode": "isaac",
                "atmosphere": self.atmosphere_config,
                "is_running": self.world is not None,
            }
            
            return physics_info
            
        except Exception as e:
            self.logger.error(f"Failed to get physics state: {e}")
            return {"error": str(e)}
    
    def reset(self) -> bool:
        """Reset physics engine state.
        
        Returns:
            bool: True if reset successful
        """
        try:
            if self.physics_scene:
                self.physics_scene.reset()
            
            # Reset fallback state
            self._fallback_state = {
                "position": [0.0, 0.0, 0.5],
                "velocity": [0.0, 0.0, 0.0],
                "acceleration": [0.0, 0.0, self.mars_gravity],
                "angular_velocity": [0.0, 0.0, 0.0],
            }
            
            self.logger.info("Physics engine reset")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to reset physics engine: {e}")
            return False
    
    def cleanup(self) -> bool:
        """Clean up physics engine resources.
        
        Returns:
            bool: True if cleanup successful
        """
        try:
            self.physics_scene = None
            self.is_initialized = False
            self.logger.info("Physics engine cleaned up")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to cleanup physics engine: {e}")
            return False
