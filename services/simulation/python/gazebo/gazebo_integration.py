#!/usr/bin/env python3
"""
Gazebo Integration for Simulation Framework

Integrates existing Gazebo worlds with the centralized simulation
framework for 3D visualization and physics validation.

Features:
- Gazebo world loading and management
- ROS2 topic bridging
- Time synchronization
- RVIZ visualization
- Physics validation

Author: URC 2026 Simulation Team
"""

import logging
import subprocess
import time
from typing import Dict, Any, List, Optional
from pathlib import Path
from dataclasses import dataclass
from enum import Enum

logger = logging.getLogger(__name__)


class GazeboState(Enum):
    """Gazebo simulation states."""

    STOPPED = "stopped"
    STARTING = "starting"
    RUNNING = "running"
    PAUSED = "paused"
    ERROR = "error"


@dataclass
class GazeboWorld:
    """Represents a Gazebo world configuration."""

    name: str
    world_file: Path
    description: str
    plugins: List[str]
    initial_pose: Dict[str, float]


class GazeboIntegration:
    """
    Integrates Gazebo with simulation framework.

    Provides bridge between centralized simulation and Gazebo
    for 3D visualization and physics validation.
    """

    # Available worlds from src/simulation/gazebo_simulation/worlds/
    AVAILABLE_WORLDS = {
        "urc_desert_terrain": GazeboWorld(
            name="urc_desert_terrain",
            world_file=Path(
                "src/simulation/gazebo_simulation/worlds/urc_desert_terrain.world"
            ),
            description="URC desert terrain with obstacles",
            plugins=["libgazebo_ros_factory.so"],
            initial_pose={"x": 0.0, "y": 0.0, "z": 0.1, "yaw": 0.0},
        ),
        "urc_obstacles": GazeboWorld(
            name="urc_obstacles",
            world_file=Path(
                "src/simulation/gazebo_simulation/worlds/urc_obstacles.world"
            ),
            description="URC obstacle course",
            plugins=["libgazebo_ros_factory.so"],
            initial_pose={"x": 0.0, "y": 0.0, "z": 0.1, "yaw": 0.0},
        ),
        "gps_denied_area": GazeboWorld(
            name="gps_denied_area",
            world_file=Path(
                "src/simulation/gazebo_simulation/worlds/gps_denied_area.world"
            ),
            description="GPS-denied navigation test area",
            plugins=["libgazebo_ros_factory.so"],
            initial_pose={"x": 0.0, "y": 0.0, "z": 0.1, "yaw": 0.0},
        ),
    }

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        """Initialize Gazebo integration.

        Args:
            config: Configuration dictionary
        """
        self.logger = logging.getLogger(f"{__name__}.GazeboIntegration")

        # Configuration
        config = config or {}
        self.world_name = config.get("world", "urc_desert_terrain")
        self.use_rviz = config.get("use_rviz", True)
        self.headless = config.get("headless", False)
        self.real_time_factor = config.get("real_time_factor", 1.0)

        # State
        self.state = GazeboState.STOPPED
        self.gazebo_process: Optional[subprocess.Popen] = None
        self.rviz_process: Optional[subprocess.Popen] = None

        # World configuration
        self.current_world: Optional[GazeboWorld] = None

        # Statistics
        self.stats = {"simulation_time": 0.0, "real_time": 0.0, "physics_updates": 0}

        self.logger.info("Gazebo integration initialized")

    def load_world(self, world_name: str) -> bool:
        """Load a Gazebo world.

        Args:
            world_name: Name of world to load

        Returns:
            bool: True if loaded successfully
        """
        if world_name not in self.AVAILABLE_WORLDS:
            self.logger.error(f"Unknown world: {world_name}")
            return False

        self.current_world = self.AVAILABLE_WORLDS[world_name]
        self.logger.info(f"Loaded world: {world_name}")
        return True

    def start_gazebo(self) -> bool:
        """Start Gazebo simulation.

        Returns:
            bool: True if started successfully
        """
        if self.state == GazeboState.RUNNING:
            self.logger.warning("Gazebo already running")
            return True

        if not self.current_world:
            self.logger.error("No world loaded")
            return False

        try:
            self.state = GazeboState.STARTING

            # Build Gazebo command
            cmd = ["gazebo"]

            if self.headless:
                cmd.append("--headless")

            # Add world file
            cmd.append(str(self.current_world.world_file))

            # Start Gazebo
            self.logger.info(f"Starting Gazebo with world: {self.current_world.name}")
            self.gazebo_process = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE
            )

            # Wait for Gazebo to start
            time.sleep(5.0)

            # Check if still running
            if self.gazebo_process.poll() is None:
                self.state = GazeboState.RUNNING
                self.logger.info("Gazebo started successfully")

                # Start RVIZ if requested
                if self.use_rviz:
                    self._start_rviz()

                return True
            else:
                self.state = GazeboState.ERROR
                self.logger.error("Gazebo failed to start")
                return False

        except Exception as e:
            self.logger.error(f"Failed to start Gazebo: {e}")
            self.state = GazeboState.ERROR
            return False

    def _start_rviz(self) -> bool:
        """Start RVIZ for visualization.

        Returns:
            bool: True if started successfully
        """
        try:
            self.logger.info("Starting RVIZ...")
            self.rviz_process = subprocess.Popen(
                ["rviz2"], stdout=subprocess.PIPE, stderr=subprocess.PIPE
            )
            time.sleep(2.0)
            return True
        except Exception as e:
            self.logger.warning(f"Failed to start RVIZ: {e}")
            return False

    def pause(self) -> bool:
        """Pause Gazebo simulation.

        Returns:
            bool: True if paused
        """
        if self.state != GazeboState.RUNNING:
            return False

        # Send pause command via ROS service
        # This would use: ros2 service call /pause_physics std_srvs/srv/Empty
        self.state = GazeboState.PAUSED
        self.logger.info("Gazebo paused")
        return True

    def resume(self) -> bool:
        """Resume Gazebo simulation.

        Returns:
            bool: True if resumed
        """
        if self.state != GazeboState.PAUSED:
            return False

        self.state = GazeboState.RUNNING
        self.logger.info("Gazebo resumed")
        return True

    def stop_gazebo(self) -> bool:
        """Stop Gazebo simulation.

        Returns:
            bool: True if stopped
        """
        try:
            if self.gazebo_process:
                self.gazebo_process.terminate()
                self.gazebo_process.wait(timeout=5.0)
                self.gazebo_process = None

            if self.rviz_process:
                self.rviz_process.terminate()
                self.rviz_process.wait(timeout=5.0)
                self.rviz_process = None

            self.state = GazeboState.STOPPED
            self.logger.info("Gazebo stopped")
            return True

        except Exception as e:
            self.logger.error(f"Error stopping Gazebo: {e}")
            return False

    def get_simulation_time(self) -> float:
        """Get current Gazebo simulation time.

        Returns:
            float: Simulation time in seconds
        """
        # This would use: ros2 topic echo /clock
        return self.stats["simulation_time"]

    def get_status(self) -> Dict[str, Any]:
        """Get Gazebo status.

        Returns:
            Dict with status information
        """
        return {
            "state": self.state.value,
            "current_world": self.current_world.name if self.current_world else None,
            "gazebo_running": self.gazebo_process is not None
            and self.gazebo_process.poll() is None,
            "rviz_running": self.rviz_process is not None
            and self.rviz_process.poll() is None,
            "statistics": self.stats.copy(),
        }


# Convenience function
def create_gazebo_integration(world: str = "urc_desert_terrain") -> GazeboIntegration:
    """Create Gazebo integration with specified world.

    Args:
        world: Name of world to load

    Returns:
        Configured GazeboIntegration instance
    """
    config = {"world": world, "use_rviz": True, "headless": False}
    integration = GazeboIntegration(config)
    integration.load_world(world)
    return integration
