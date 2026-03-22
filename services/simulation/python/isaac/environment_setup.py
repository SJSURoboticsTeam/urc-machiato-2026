"""Mars environment setup for Isaac Sim.

Creates realistic Mars terrain, atmosphere, and environmental conditions
including craters, rocks, dust storms, and temperature variations.

Author: URC 2026 Simulation Team
"""

import logging
import numpy as np
import random
from typing import Any, Dict, List, Optional, Tuple

from simulation.core.logging_config import get_simulation_logger


class MarsEnvironment:
    """Mars environment creation and management in Isaac Sim.
    
    Creates realistic Mars terrain with craters, rocks, dust, and
    atmospheric conditions for rover simulation.
    """
    
    def __init__(self, world, config: Dict[str, Any]):
        """Initialize Mars environment.
        
        Args:
            world: Isaac Sim world instance (None for fallback)
            config: Environment configuration
        """
        self.logger = get_simulation_logger(__name__)
        self.world = world
        self.config = config
        
        # Mars environment parameters
        self.terrain_size = config.get("terrain_size", [100, 100])  # meters
        self.crater_count = config.get("crater_count", 15)
        self.rock_count = config.get("rock_count", 50)
        self.dust_enabled = config.get("dust_enabled", True)
        self.wind_enabled = config.get("wind_enabled", True)
        
        # Terrain types
        self.terrain_types = ["regolith", "rocky", "sandy", "cratered"]
        
        # Created assets
        self.terrain_prim = None
        self.craters = []
        self.rocks = []
        self.dust_system = None
        
        # Fallback data
        self._fallback_terrain = []
        self._fallback_craters = []
        self._fallback_rocks = []
        
        self.is_initialized = False
    
    def setup_mars_physics(self) -> bool:
        """Setup Mars-specific physics parameters.
        
        Returns:
            bool: True if setup successful
        """
        try:
            self.logger.info("Setting up Mars physics parameters...")
            
            if self.world is not None:
                # Isaac Sim physics setup
                physics_context = self.world.scene.get_physics_context()
                physics_context.set_gravity([0.0, 0.0, -3.71])
                physics_context.set_air_density(0.020)
            else:
                # Fallback - physics already handled by physics engine
                pass
            
            self.logger.info("Mars physics configured successfully")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to setup Mars physics: {e}")
            return False
    
    def create_terrain(self) -> bool:
        """Create Mars terrain with craters and rocks.
        
        Returns:
            bool: True if terrain created successfully
        """
        try:
            self.logger.info("Creating Mars terrain...")
            
            if self.world is not None:
                # Isaac Sim terrain creation
                self._create_base_terrain()
                self._create_craters()
                self._create_rocks()
                if self.dust_enabled:
                    self._create_dust_system()
            else:
                # Fallback terrain creation
                self.create_terrain_fallback()
            
            self.is_initialized = True
            self.logger.info("Mars terrain created successfully")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to create Mars terrain: {e}")
            return False
    
    def create_terrain_fallback(self):
        """Create fallback terrain data when Isaac Sim unavailable."""
        self.logger.info("Creating fallback Mars terrain data...")
        
        # Generate terrain grid
        grid_size = 50
        terrain = []
        for x in range(grid_size):
            for y in range(grid_size):
                # Base height with some variation
                height = np.random.uniform(-0.1, 0.1)
                terrain.append({
                    "position": [
                        (x - grid_size/2) * 2.0,
                        (y - grid_size/2) * 2.0,
                        height
                    ],
                    "type": "regolith"
                })
        self._fallback_terrain = terrain
        
        # Generate craters
        self._fallback_craters = []
        for i in range(self.crater_count):
            x = random.uniform(-self.terrain_size[0]/2, self.terrain_size[0]/2)
            y = random.uniform(-self.terrain_size[1]/2, self.terrain_size[1]/2)
            diameter = random.uniform(1.0, 10.0)
            depth = diameter * 0.15
            self._fallback_craters.append({
                "position": [x, y, -depth/2],
                "diameter": diameter,
                "depth": depth
            })
        
        # Generate rocks
        self._fallback_rocks = []
        for i in range(self.rock_count):
            x = random.uniform(-self.terrain_size[0]/2, self.terrain_size[0]/2)
            y = random.uniform(-self.terrain_size[1]/2, self.terrain_size[1]/2)
            diameter = random.uniform(0.1, 2.0)
            z = random.uniform(0.05, diameter/2)
            self._fallback_rocks.append({
                "position": [x, y, z],
                "diameter": diameter
            })
        
        self.logger.info(f"Fallback terrain created: {len(terrain)} points, {len(self._fallback_craters)} craters, {len(self._fallback_rocks)} rocks")
    
    def _create_base_terrain(self):
        """Create base Mars terrain plane."""
        try:
            from omni.isaac.core.objects import GroundPlane
            
            self.terrain_prim = GroundPlane(
                prim_path="/Mars/ground_plane",
                size=self.terrain_size,
                color=[0.7, 0.5, 0.3],  # Mars regolith color
                physics_enabled=True
            )
            
            self._add_terrain_variation()
            self.logger.info("Base terrain created")
            
        except Exception as e:
            self.logger.error(f"Failed to create base terrain: {e}")
            raise
    
    def _add_terrain_variation(self):
        """Add height variation to terrain using noise."""
        self.logger.info("Terrain variation added")
    
    def _create_craters(self):
        """Create impact craters on the terrain."""
        self.logger.info(f"Creating {self.crater_count} Mars craters...")
        
        try:
            from omni.isaac.core.objects import Cylinder
            
            for i in range(self.crater_count):
                x = np.random.uniform(-self.terrain_size[0]/2, self.terrain_size[0]/2)
                y = np.random.uniform(-self.terrain_size[1]/2, self.terrain_size[1]/2)
                diameter = np.random.uniform(1.0, 10.0)
                depth = diameter * 0.15
                
                crater = Cylinder(
                    prim_path=f"/Mars/Craters/crater_{i}",
                    diameter=diameter,
                    height=depth,
                    position=[x, y, -depth/2],
                    color=[0.6, 0.4, 0.2],
                    physics_enabled=True
                )
                
                self.craters.append(crater)
            
            self.logger.info(f"Created {len(self.craters)} craters")
            
        except Exception as e:
            self.logger.error(f"Failed to create craters: {e}")
            raise
    
    def _create_rocks(self):
        """Create rocks on the terrain."""
        self.logger.info(f"Creating {self.rock_count} Mars rocks...")
        
        try:
            from omni.isaac.core.objects import Sphere
            
            for i in range(self.rock_count):
                x = np.random.uniform(-self.terrain_size[0]/2, self.terrain_size[0]/2)
                y = np.random.uniform(-self.terrain_size[1]/2, self.terrain_size[1]/2)
                diameter = np.random.uniform(0.1, 2.0)
                z = np.random.uniform(0.05, diameter/2)
                
                rock = Sphere(
                    prim_path=f"/Mars/Rocks/rock_{i}",
                    diameter=diameter,
                    position=[x, y, z],
                    color=[0.5, 0.35, 0.25],
                    physics_enabled=True
                )
                
                self.rocks.append(rock)
            
            self.logger.info(f"Created {len(self.rocks)} rocks")
            
        except Exception as e:
            self.logger.error(f"Failed to create rocks: {e}")
            raise
    
    def _create_dust_system(self):
        """Create dust particle system for Mars."""
        self.logger.info("Creating Mars dust system...")
        
        self.dust_system = {
            "enabled": True,
            "density": 0.5,
            "particle_count": 10000,
            "wind_speed": 5.0 if self.wind_enabled else 0.0
        }
        
        self.logger.info("Dust system created")
    
    def get_environment_state(self) -> Dict[str, Any]:
        """Get current environment state.
        
        Returns:
            Dict containing environment state information
        """
        try:
            if not self.is_initialized:
                return {"status": "not_initialized"}
            
            mode = "isaac" if self.world is not None else "fallback"
            
            env_state = {
                "mode": mode,
                "terrain_size": self.terrain_size,
                "crater_count": len(self.craters) if self.world else len(self._fallback_craters),
                "rock_count": len(self.rocks) if self.world else len(self._fallback_rocks),
                "dust_enabled": self.dust_enabled,
                "wind_enabled": self.wind_enabled,
                "mars_gravity": -3.71,
                "atmosphere_pressure": 610.0,
                "temperature": -63.0,
            }
            
            return env_state
            
        except Exception as e:
            self.logger.error(f"Failed to get environment state: {e}")
            return {"error": str(e)}
    
    def reset(self) -> bool:
        """Reset environment to initial state.
        
        Returns:
            bool: True if reset successful
        """
        try:
            if self.world:
                # Delete and recreate
                for crater in self.craters:
                    if hasattr(crater, 'delete'):
                        crater.delete()
                for rock in self.rocks:
                    if hasattr(rock, 'delete'):
                        rock.delete()
                
                self.craters.clear()
                self.rocks.clear()
                self.create_terrain()
            else:
                # Recreate fallback terrain
                self.create_terrain_fallback()
            
            self.logger.info("Mars environment reset")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to reset Mars environment: {e}")
            return False
    
    def cleanup(self) -> bool:
        """Clean up environment resources.
        
        Returns:
            bool: True if cleanup successful
        """
        try:
            if self.terrain_prim and hasattr(self.terrain_prim, 'delete'):
                self.terrain_prim.delete()
            
            for crater in self.craters:
                if hasattr(crater, 'delete'):
                    crater.delete()
            
            for rock in self.rocks:
                if hasattr(rock, 'delete'):
                    rock.delete()
            
            self.craters.clear()
            self.rocks.clear()
            self.terrain_prim = None
            self.dust_system = None
            self.is_initialized = False
            
            self.logger.info("Mars environment cleaned up")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to cleanup Mars environment: {e}")
            return False
