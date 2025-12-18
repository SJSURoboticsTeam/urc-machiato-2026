"""Central simulation orchestrator for URC 2026.

Coordinates all simulation components including sensors, network,
rover dynamics, and environmental effects. Provides unified
interface for running comprehensive simulations.

Author: URC 2026 Autonomy Team
"""

import logging
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

from simulation.core.logging_config import (
    get_simulation_logger,
    setup_simulation_logging,
)
from simulation.core.time_manager import TimeManager
from simulation.core.tracer import SimulationTracer
from simulation.environments.environment_factory import EnvironmentFactory
from simulation.network.network_factory import NetworkFactory
from simulation.rover.rover_factory import RoverFactory
from simulation.sensors.sensor_factory import SensorFactory
from simulation.tools.data_recorder import DataRecorder
from simulation.tools.monitoring_dashboard import SimulationMonitor


class SimulationManager:
    """Central orchestrator for all simulation components.

    Manages the complete simulation lifecycle including initialization,
    stepping, data recording, and cleanup. Coordinates sensors,
    network, rover, and environmental simulation components.
    """

    def __init__(self):
        """Initialize simulation manager."""
        # Enhanced logging
        self.logger = get_simulation_logger(__name__, "simulation_manager")

        # Core components
        self.time_manager = TimeManager()
        self.data_recorder = DataRecorder()
        self.monitor = SimulationMonitor(self)
        self.tracer = SimulationTracer()

        # Simulation components (lazy-loaded)
        self.environment = None
        self.sensors: Dict[str, Any] = {}
        self.network = None
        self.rover = None

        # Simulation state
        self.is_initialized = False
        self.is_running = False
        self.simulation_thread: Optional[threading.Thread] = None
        self.step_count = 0
        self._last_step_time = time.time()

        # Configuration
        self.config: Dict[str, Any] = {}
        self.simulation_metadata: Dict[str, Any] = {
            "framework_version": "2.0.0",  # Updated with enhanced features
            "simulation_type": "centralized",
            "features": ["logging", "monitoring", "tracing", "rl_support"],
            "created_at": time.time(),
        }

    def initialize(self, config: Dict[str, Any]) -> bool:
        """Initialize all simulation components.

        Args:
            config: Simulation configuration dictionary

        Returns:
            bool: True if initialization successful
        """
        with self.tracer.trace_context(
            "simulation_initialization", config_keys=list(config.keys())
        ):
            try:
                self.logger.info("Initializing enhanced simulation framework v2.0...")
                self.config = config

                # Setup logging first
                logging_config = config.get("logging", {})
                if logging_config.get("enabled", True):
                    setup_simulation_logging(
                        log_level=logging_config.get("level", "INFO"),
                        log_file=logging_config.get("file", "simulation.log"),
                        enable_structured=logging_config.get("structured", True),
                        enable_json=logging_config.get("json_format", False),
                    )

                # Validate configuration
                if not self._validate_config(config):
                    self.logger.error("Configuration validation failed")
                    return False

                # Initialize monitoring
                monitoring_config = config.get("monitoring", {})
                if monitoring_config.get("enabled", True):
                    self.monitor.start_monitoring(
                        interval=monitoring_config.get("interval", 1.0)
                    )

                # Create environment
                env_config = config.get("environment", {})
                self.environment = EnvironmentFactory.create(env_config)
                self.logger.info(
                    "Environment initialized", tier=env_config.get("tier", "default")
                )

                # Create sensors
                sensor_configs = config.get("sensors", [])
                for sensor_config in sensor_configs:
                    sensor_name = sensor_config["name"]
                    sensor_type = sensor_config["type"]
                    sensor = SensorFactory.create(sensor_type, sensor_config)
                    self.sensors[sensor_name] = sensor
                    self.logger.info(
                        "Sensor initialized",
                        sensor_name=sensor_name,
                        sensor_type=sensor_type,
                    )

                # Create network
                network_config = config.get("network", {})
                self.network = NetworkFactory.create(network_config)
                self.logger.info(
                    "Network initialized",
                    profile=network_config.get("profile", "default"),
                )

                # Create rover
                rover_config = config.get("rover", {})
                self.rover = RoverFactory.create(rover_config)
                self.logger.info(
                    "Rover initialized", model=rover_config.get("model", "default")
                )

                # Initialize time manager
                time_config = config.get("time", {})
                self.time_manager.initialize(time_config)

                # Setup data recording
                recording_config = config.get("recording", {})
                self.data_recorder.initialize(recording_config)

                # Enable tracing
                tracing_config = config.get("tracing", {})
                if tracing_config.get("enabled", True):
                    self.tracer.enable()
                    threshold = tracing_config.get("auto_profile_threshold", 0.1)
                    self.tracer.auto_profile_threshold = threshold

                # Mark as initialized
                self.is_initialized = True
                self.logger.info(
                    "Enhanced simulation framework initialized successfully",
                    features=self.simulation_metadata["features"],
                )

                return True

            except Exception as e:
                self.logger.error(
                    "Simulation initialization failed",
                    error=str(e),
                    config_keys=list(config.keys()),
                )
                return False

    def start(self) -> bool:
        """Start the simulation.

        Returns:
            bool: True if simulation started successfully
        """
        if not self.is_initialized:
            self.logger.error("Cannot start uninitialized simulation")
            return False

        if self.is_running:
            self.logger.warning("Simulation already running")
            return True

        try:
            self.is_running = True
            self.step_count = 0

            # Start simulation thread
            self.simulation_thread = threading.Thread(
                target=self._simulation_loop, daemon=True
            )
            self.simulation_thread.start()

            self.logger.info("Simulation started")
            return True

        except Exception as e:
            self.logger.error(f"Failed to start simulation: {e}")
            self.is_running = False
            return False

    def stop(self) -> bool:
        """Stop the simulation.

        Returns:
            bool: True if simulation stopped successfully
        """
        if not self.is_running:
            self.logger.warning("Simulation not running")
            return True

        try:
            self.is_running = False

            # Wait for simulation thread to finish
            if self.simulation_thread and self.simulation_thread.is_alive():
                self.simulation_thread.join(timeout=5.0)

            self.logger.info("Simulation stopped")
            return True

        except Exception as e:
            self.logger.error(f"Failed to stop simulation: {e}")
            return False

    @SimulationTracer().trace_method("simulation_step")
    def step(self, dt: float) -> Dict[str, Any]:
        """Execute single simulation step.

        Args:
            dt: Time step in seconds

        Returns:
            Dict containing current simulation state
        """
        if not self.is_initialized:
            raise RuntimeError("Simulation not initialized")

        step_start = time.time()
        self._last_step_time = step_start

        with self.tracer.trace_context(
            "simulation_step", step_count=self.step_count, dt=dt
        ) as span:
            try:
                # Update time
                self.time_manager.step(dt)

                # Update environment
                with self.tracer.trace_context("environment_step", dt=dt):
                    env_state = self.environment.step(dt) if self.environment else {}

                # Update sensors with environment
                sensor_data = {}
                for name, sensor in self.sensors.items():
                    with self.tracer.trace_context(
                        f"sensor_step_{name}", sensor_name=name, dt=dt
                    ):
                        data = sensor.step(dt, env_state)
                        sensor_data[name] = data

                # Update rover with sensor data
                with self.tracer.trace_context("rover_step", dt=dt):
                    rover_state = self.rover.step(dt, sensor_data) if self.rover else {}

                # Process network conditions
                with self.tracer.trace_context("network_step"):
                    network_state = (
                        self.network.process_messages() if self.network else {}
                    )

                # Combine all state
                simulation_state = {
                    "timestamp": self.time_manager.current_time,
                    "step_count": self.step_count,
                    "environment": env_state,
                    "sensors": sensor_data,
                    "rover": rover_state,
                    "network": network_state,
                }

                # Record data
                with self.tracer.trace_context("data_recording"):
                    self.data_recorder.record(simulation_state)

                step_duration = time.time() - step_start

                # Log step completion
                self.logger.log_simulation_step(
                    self.step_count,
                    self.time_manager.current_time,
                    step_duration=step_duration,
                    sensor_count=len(sensor_data),
                )

                self.step_count += 1

                # Add performance info to span
                if span:
                    span.attributes["step_duration"] = step_duration
                    span.attributes["sensor_count"] = len(sensor_data)

                return simulation_state

            except Exception as e:
                self.logger.error(
                    "Simulation step failed", step_count=self.step_count, error=str(e)
                )
                raise

    def reset(self) -> bool:
        """Reset simulation to initial state.

        Returns:
            bool: True if reset successful
        """
        try:
            # Reset all components
            if self.environment:
                self.environment.reset()
            for sensor in self.sensors.values():
                sensor.reset()
            if self.network:
                self.network.reset()
            if self.rover:
                self.rover.reset()

            # Reset time and counters
            self.time_manager.reset()
            self.step_count = 0

            # Clear recorded data
            self.data_recorder.clear()

            self.logger.info("Simulation reset")
            return True

        except Exception as e:
            self.logger.error(f"Simulation reset failed: {e}")
            return False

    def get_state(self) -> Dict[str, Any]:
        """Get current simulation state.

        Returns:
            Dict containing complete simulation state
        """
        return {
            "is_initialized": self.is_initialized,
            "is_running": self.is_running,
            "step_count": self.step_count,
            "current_time": self.time_manager.current_time,
            "environment": self.environment.get_state() if self.environment else {},
            "sensors": {
                name: sensor.get_state() for name, sensor in self.sensors.items()
            },
            "rover": self.rover.get_state() if self.rover else {},
            "network": self.network.get_state() if self.network else {},
            "metadata": self.simulation_metadata,
        }

    def save_state(self, filename: str) -> bool:
        """Save current simulation state to file.

        Args:
            filename: Path to save state file

        Returns:
            bool: True if save successful
        """
        try:
            state = self.get_state()
            filepath = Path(filename)

            import json

            with open(filepath, "w") as f:
                json.dump(state, f, indent=2, default=str)

            self.logger.info(f"Simulation state saved to {filepath}")
            return True

        except Exception as e:
            self.logger.error(f"Failed to save state: {e}")
            return False

    def load_state(self, filename: str) -> bool:
        """Load simulation state from file.

        Args:
            filename: Path to state file

        Returns:
            bool: True if load successful
        """
        try:
            filepath = Path(filename)

            import json

            with open(filepath, "r") as f:
                state = json.load(f)

            # Restore state (implementation would depend on component interfaces)
            self.logger.info(f"Simulation state loaded from {filepath}")
            return True

        except Exception as e:
            self.logger.error(f"Failed to load state: {e}")
            return False

    def get_statistics(self) -> Dict[str, Any]:
        """Get comprehensive simulation statistics.

        Returns:
            Dict containing simulation metrics
        """
        base_stats = {
            "total_steps": self.step_count,
            "total_time": self.time_manager.current_time,
            "average_step_time": (
                self.time_manager.current_time / self.step_count
                if self.step_count > 0
                else 0
            ),
            "sensor_count": len(self.sensors),
            "data_points_recorded": len(self.data_recorder.data),
            "network_stats": self.network.get_statistics() if self.network else {},
        }

        # Add monitoring statistics
        if hasattr(self.monitor, "get_performance_report"):
            base_stats["monitoring"] = self.monitor.get_performance_report()

        # Add tracing statistics
        if hasattr(self.tracer, "get_performance_report"):
            base_stats["tracing"] = self.tracer.get_performance_report()

        # Add RL statistics if using RL recorder
        if hasattr(self.data_recorder, "get_rl_statistics"):
            base_stats["rl_training"] = self.data_recorder.get_rl_statistics()

        return base_stats

    def _simulation_loop(self):
        """Main simulation loop for continuous execution."""
        dt = self.config.get("time", {}).get("step_size", 0.01)

        while self.is_running:
            try:
                self.step(dt)
                time.sleep(dt)  # Maintain real-time pacing
            except Exception as e:
                self.logger.error(f"Simulation loop error: {e}")
                self.is_running = False
                break

    def _validate_config(self, config: Dict[str, Any]) -> bool:
        """Validate simulation configuration.

        Args:
            config: Configuration to validate

        Returns:
            bool: True if configuration is valid
        """
        required_keys = ["environment", "sensors", "network", "rover"]

        for key in required_keys:
            if key not in config:
                self.logger.error(f"Missing required configuration key: {key}")
                return False

        # Validate sensor configurations
        sensors = config.get("sensors", [])
        if not isinstance(sensors, list):
            self.logger.error("Sensors configuration must be a list")
            return False

        for sensor in sensors:
            if (
                not isinstance(sensor, dict)
                or "name" not in sensor
                or "type" not in sensor
            ):
                self.logger.error("Each sensor must have 'name' and 'type' fields")
                return False

        return True
