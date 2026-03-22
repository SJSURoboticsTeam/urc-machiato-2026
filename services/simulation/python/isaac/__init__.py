"""Isaac Sim integration for URC 2026 rover simulation.

This module provides the integration layer between URC 2026 simulation framework
and NVIDIA Isaac Sim 5.0.0 for enhanced physics simulation and Mars environment
modeling.

Author: URC 2026 Simulation Team
"""

from .isaac_integration import IsaacIntegration
from .physics_engine import IsaacPhysicsEngine
from .sensor_simulation import IsaacSensorSimulation
from .environment_setup import MarsEnvironment
from .ros_bridge import IsaacRosBridge

__all__ = [
    "IsaacIntegration",
    "IsaacPhysicsEngine", 
    "IsaacSensorSimulation",
    "MarsEnvironment",
    "IsaacRosBridge",
]

__version__ = "1.0.0"