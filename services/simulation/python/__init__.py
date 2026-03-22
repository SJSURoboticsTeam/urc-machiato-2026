"""URC 2026 Simulation Framework

Provides comprehensive simulation capabilities for:
- Sensor data generation (GPS, IMU, Camera, LiDAR)
- Network condition emulation (WiFi, cellular, satellite)
- Rover physics simulation (kinematics, dynamics)
- Environment modeling (terrain, weather, obstacles)
- Mission execution simulation

All components follow consistent interfaces and are designed for
easy testing, validation, and hardware-in-the-loop integration.

Author: URC 2026 Autonomy Team
"""

# Try to import core components, but handle missing dependencies gracefully
try:
    from simulation.core.logging_config import (
        get_simulation_logger,
        setup_simulation_logging,
    )

    CORE_LOGGING_AVAILABLE = True
except ImportError:
    CORE_LOGGING_AVAILABLE = False

try:
    from simulation.core.simulation_manager import SimulationManager

    SIMULATION_MANAGER_AVAILABLE = True
except ImportError:
    SIMULATION_MANAGER_AVAILABLE = False

try:
    from simulation.core.time_manager import TimeManager

    TIME_MANAGER_AVAILABLE = True
except ImportError:
    TIME_MANAGER_AVAILABLE = False

try:
    from simulation.environments.environment_factory import EnvironmentFactory

    ENVIRONMENT_FACTORY_AVAILABLE = True
except ImportError:
    ENVIRONMENT_FACTORY_AVAILABLE = False

try:
    from simulation.network.network_factory import NetworkFactory

    NETWORK_FACTORY_AVAILABLE = True
except ImportError:
    NETWORK_FACTORY_AVAILABLE = False

try:
    from simulation.rover.rover_factory import RoverFactory

    ROVER_FACTORY_AVAILABLE = True
except ImportError:
    ROVER_FACTORY_AVAILABLE = False

try:
    from simulation.sensors.sensor_factory import SensorFactory

    SENSOR_FACTORY_AVAILABLE = True
except ImportError:
    SENSOR_FACTORY_AVAILABLE = False

# Basic component imports with better error handling
try:
    from simulation.config.config_loader import ConfigLoader

    CONFIG_LOADER_AVAILABLE = True
except ImportError:
    CONFIG_LOADER_AVAILABLE = False

# Version
__version__ = "2.0.1"  # Updated with enhanced features

# Export basic components that should always work
__all__ = [
    "get_simulation_logger",
    "setup_simulation_logging",
    "SimulationManager",
    "TimeManager",
    "EnvironmentFactory",
    "NetworkFactory",
    "RoverFactory",
    "SensorFactory",
    "ConfigLoader",
]
