"""URC 2026 Centralized Simulation Framework.

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

from simulation.core.logging_config import (
    get_simulation_logger,
    setup_simulation_logging,
)
from simulation.core.simulation_manager import SimulationManager
from simulation.core.time_manager import TimeManager
from simulation.core.tracer import SimulationTracer
from simulation.environments.environment_factory import EnvironmentFactory
from simulation.network.network_factory import NetworkFactory
from simulation.rover.rover_factory import RoverFactory
from simulation.sensors.sensor_factory import SensorFactory
from simulation.tools.monitoring_dashboard import SimulationMonitor
from simulation.tools.rl_data_recorder import RLDataRecorder

__version__ = "2.0.0"  # Updated with enhanced features
__all__ = [
    "SimulationManager",
    "TimeManager",
    "SimulationTracer",
    "SimulationMonitor",
    "RLDataRecorder",
    "setup_simulation_logging",
    "get_simulation_logger",
    "EnvironmentFactory",
    "NetworkFactory",
    "RoverFactory",
    "SensorFactory",
]
