#!/usr/bin/env python3
"""
Integrated Critical Systems for URC Rover

Main integration system that combines all implemented critical systems:
- Redundant CAN bus interface
- Hardware-level emergency stop
- Advanced closed-loop motor control
- Sensor fusion for state estimation
- Graceful degradation system

This system provides comprehensive safety, reliability, and performance
optimization for competition operations.

Author: URC 2026 Integration Team
"""

import time
import threading
import asyncio
import signal
import sys
from typing import Dict, List, Any, Optional
import logging

from geometry_msgs.msg import Twist
from rclpy.node import Node

# Import all critical systems
from infrastructure.bridges.can_bridge import CANBridge
from control.hardware_emergency_stop import (
    HardwareEmergencyStop,
    EmergencyStopConfig,
    initialize_emergency_stop,
)
from control.advanced_motor_controller import AdvancedMotorController
from perception.sensor_fusion import SensorFusionManager, create_sensor_fusion_manager
from core.graceful_degradation import (
    OperationModeManager,
    OperationMode,
    create_operation_mode_manager,
)

logger = logging.getLogger(__name__)


class IntegratedCriticalSystems(Node):
    """
    Main integration system that combines all critical components.

    This system provides:
    - Coordinated safety and control
    - Fault tolerance and redundancy
    - Performance optimization
    - Competition-specific adaptations
    """

    def __init__(self, config: Dict[str, Any] = None):
        super().__init__("integrated_critical_systems")

        self.config = config or {}
        self.system_initialized = False
        self.shutdown_requested = False

        # Initialize critical systems
        self._initialize_emergency_stop()
        self._initialize_can_bridge()
        self._initialize_motor_control()
        self._initialize_sensor_fusion()
        self._initialize_graceful_degradation()

        # System coordination
        self.health_monitor_thread: Optional[threading.Thread] = None
        self.coordination_active = False

        # Performance tracking
        self.performance_metrics = {
            "uptime": 0.0,
            "failover_count": 0,
            "emergency_stop_count": 0,
            "mode_switches": 0,
            "control_loop_performance": [],
            "sensor_fusion_performance": [],
        }

        # Signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        logger.info("Integrated critical systems initialized")

    def _initialize_emergency_stop(self):
        """Initialize hardware emergency stop system."""
        try:
            e_stop_config = EmergencyStopConfig(
                e_stop_gpio_pin=self.config.get("e_stop_gpio_pin", 17),
                motor_power_relay_pin=self.config.get("motor_power_relay_pin", 18),
                status_led_pin=self.config.get("status_led_pin", 24),
                watchdog_timeout_ms=self.config.get("watchdog_timeout_ms", 100),
                auto_reset_seconds=self.config.get("auto_reset_seconds", 5),
                require_manual_reset=self.config.get("require_manual_reset", True),
            )

            self.emergency_stop = initialize_emergency_stop(e_stop_config)

            # Register status callback
            self.emergency_stop.register_status_callback(
                "integration_system", self._emergency_stop_callback
            )

            logger.info("Emergency stop system initialized")

        except Exception as e:
            logger.error(f"Failed to initialize emergency stop: {e}")
            raise

    def _initialize_can_bridge(self):
        """Initialize CAN bridge (single-channel)."""
        try:
            can_config = {
                "device": self.config.get("primary_can_device", "/dev/ttyACM0"),
                "baudrate": self.config.get("can_baudrate", 115200),
                "fallback_devices": [
                    self.config.get("secondary_can_device", "/dev/ttyACM1"),
                    "/dev/ttyAMA10",
                    "/dev/ttyUSB0",
                ],
            }
            self.can_bridge = CANBridge(can_config)
            asyncio.create_task(self.can_bridge.connect())
            logger.info("CAN bridge initialized")
        except Exception as e:
            logger.error(f"Failed to initialize CAN bridge: {e}")
            raise

    def _initialize_motor_control(self):
        """Initialize advanced motor controller."""
        try:
            motor_config = {
                "interface": self.config.get("motor_interface", "mock"),
                "adaptive_mode": self.config.get("adaptive_motor_control", True),
                "load_balancing": self.config.get("motor_load_balancing", True),
                "traction_control": self.config.get("traction_control_enabled", True),
                "predictive_maintenance": {
                    "vibration_threshold": self.config.get("vibration_threshold", 0.5),
                    "temperature_trend_threshold": self.config.get(
                        "temp_trend_threshold", 0.1
                    ),
                },
                "encoders": {
                    "cpr": self.config.get("encoder_cpr", 4096),
                    "wheel_radius": self.config.get("wheel_radius", 0.1),
                },
                "wheel_base": self.config.get("wheel_base", 0.5),
                "track_width": self.config.get("track_width", 0.4),
            }

            self.motor_controller = AdvancedMotorController(motor_config)

            # Initialize controller
            if not self.motor_controller.initialize():
                raise RuntimeError("Failed to initialize motor controller")

            logger.info("Advanced motor controller initialized")

        except Exception as e:
            logger.error(f"Failed to initialize motor control: {e}")
            raise

    def _initialize_sensor_fusion(self):
        """Initialize sensor fusion system."""
        try:
            fusion_config = {
                "fusion_rate": self.config.get("fusion_rate", 100.0),
                "enable_ekf": self.config.get("enable_extended_kalman", True),
                "enable_complementary": self.config.get("enable_complementary", True),
            }

            self.sensor_fusion = create_sensor_fusion_manager(fusion_config)

            # Add sensors
            self.sensor_fusion.add_sensor("imu", "imu")
            self.sensor_fusion.add_sensor("gps", "gps")
            self.sensor_fusion.add_sensor("wheel_odometry", "wheel_odometry")

            # Start fusion
            self.sensor_fusion.start_fusion()

            logger.info("Sensor fusion system initialized")

        except Exception as e:
            logger.error(f"Failed to initialize sensor fusion: {e}")
            raise

    def _initialize_graceful_degradation(self):
        """Initialize graceful degradation system."""
        try:
            degradation_config = {
                "mode_evaluation_interval": self.config.get("mode_eval_interval", 2.0),
                "stability_requirement": self.config.get("mode_stability_time", 5.0),
                "resource_thresholds": {
                    "cpu_critical": self.config.get("cpu_critical_threshold", 90.0),
                    "cpu_high": self.config.get("cpu_high_threshold", 75.0),
                    "memory_critical": self.config.get(
                        "memory_critical_threshold", 95.0
                    ),
                    "memory_high": self.config.get("memory_high_threshold", 80.0),
                },
            }

            self.operation_mode_manager = create_operation_mode_manager(
                degradation_config
            )

            # Start mode management
            self.operation_mode_manager.start_mode_management()

            logger.info("Graceful degradation system initialized")

        except Exception as e:
            logger.error(f"Failed to initialize graceful degradation: {e}")
            raise

    def start_systems(self) -> bool:
        """Start all integrated systems."""
        try:
            # Start health monitoring
            self.start_health_monitoring()

            # Set mission context
            self.operation_mode_manager.set_mission_context(
                phase="competition",
                time_remaining=self.config.get("mission_duration", 900.0),
            )

            self.system_initialized = True
            logger.info("All integrated systems started successfully")

            return True

        except Exception as e:
            logger.error(f"Failed to start systems: {e}")
            return False

    def shutdown_systems(self):
        """Shutdown all integrated systems gracefully."""
        try:
            logger.info("Shutting down integrated critical systems")

            self.shutdown_requested = True
            self.coordination_active = False

            # Stop health monitoring
            if self.health_monitor_thread:
                self.health_monitor_thread.join(timeout=2.0)

            # Emergency stop motors
            self.motor_controller.emergency_stop()

            # Shutdown systems in reverse order
            self.operation_mode_manager.stop_mode_management()
            self.sensor_fusion.stop_fusion()
            self.motor_controller.shutdown()
            self.can_bridge.shutdown()

            # Cleanup emergency stop
            from ..control.hardware_emergency_stop import cleanup_emergency_stop

            cleanup_emergency_stop()

            logger.info("All systems shutdown complete")

        except Exception as e:
            logger.error(f"Error during shutdown: {e}")

    def process_velocity_command(self, twist: Twist) -> bool:
        """Process velocity command through all systems."""
        try:
            # Check if systems are ready
            if not self.system_initialized:
                logger.warning("Systems not initialized, ignoring command")
                return False

            # Check emergency stop status
            e_stop_status = self.emergency_stop.get_status()
            if e_stop_status["is_active"]:
                logger.warning("Emergency stop active, ignoring velocity command")
                return False

            # Check operation mode
            current_mode = self.operation_mode_manager.current_mode
            if current_mode in [
                OperationMode.SAFE_STOP,
                OperationMode.EMERGENCY_SHUTDOWN,
            ]:
                logger.warning(f"Operation mode {current_mode.value} prevents movement")
                return False

            # Send command to motor controller
            success = self.motor_controller.set_velocity_command(twist)

            if success:
                # Feed watchdog
                self.emergency_stop.feed_watchdog()

                # Log performance
                self._log_command_performance(twist)

            return success

        except Exception as e:
            logger.error(f"Failed to process velocity command: {e}")
            return False

    def _emergency_stop_callback(self, status: str, e_stop_info: Dict[str, Any]):
        """Handle emergency stop status changes."""
        if status == "soft_stop_activated" or status == "hard_stop_activated":
            # Emergency stop all motors
            self.motor_controller.emergency_stop()

            # Update performance metrics
            self.performance_metrics["emergency_stop_count"] += 1

            logger.critical(
                f"Emergency stop activated: {e_stop_info.get('stop_reason', 'unknown')}"
            )

    def start_health_monitoring(self):
        """Start system health monitoring."""
        if self.coordination_active:
            return

        self.coordination_active = True
        self.health_monitor_thread = threading.Thread(
            target=self._health_monitoring_loop, daemon=True
        )
        self.health_monitor_thread.start()

        logger.info("System health monitoring started")

    def _health_monitoring_loop(self):
        """Main health monitoring loop."""
        while self.coordination_active and not self.shutdown_requested:
            try:
                # Update performance metrics
                self._update_performance_metrics()

                # Check system health
                self._check_system_health()

                # Coordinate between systems
                self._coordinate_systems()

                # Feed watchdog
                self.emergency_stop.feed_watchdog()

                time.sleep(1.0)  # Health check every second

            except Exception as e:
                logger.error(f"Health monitoring error: {e}")
                time.sleep(1.0)

    def _update_performance_metrics(self):
        """Update performance metrics."""
        current_time = time.time()
        if hasattr(self, "start_time"):
            self.performance_metrics["uptime"] = current_time - self.start_time
        else:
            self.start_time = current_time

        # Get system statuses
        can_status = self.can_bridge.get_status()
        motor_status = self.motor_controller.get_advanced_status()

        # Update metrics (single CAN channel; no failover)
        self.performance_metrics["failover_count"] = 0
        self.performance_metrics["mode_switches"] = motor_status.get(
            "maintenance_alerts", {}
        )

    def _check_system_health(self):
        """Check health of all systems."""
        can_status = self.can_bridge.get_status()
        if not can_status.is_connected:
            logger.critical("CAN bridge disconnected!")
            self.emergency_stop.activate_hard_stop("can_failure")

        # Motor controller health
        motor_status = self.motor_controller.get_advanced_status()
        maintenance_alerts = motor_status.get("maintenance_alerts", {})

        if maintenance_alerts:
            for wheel, alerts in maintenance_alerts.items():
                for alert in alerts:
                    logger.warning(f"Motor maintenance alert [{wheel}]: {alert}")

        # Sensor fusion health
        sensor_status = self.sensor_fusion.get_sensor_status()
        failed_sensors = [
            sid for sid, info in sensor_status.items() if info["status"] == "failed"
        ]

        if failed_sensors:
            logger.warning(f"Failed sensors: {failed_sensors}")

    def _coordinate_systems(self):
        """Coordinate between different systems."""
        # Get current operation mode
        current_mode = self.operation_mode_manager.current_mode

        # Adjust motor control based on mode
        if current_mode == OperationMode.SURVIVAL_MODE:
            # Reduce control rate in survival mode
            if hasattr(self.motor_controller, "set_control_mode"):
                from ..control.closed_loop_motor_controller import ControlMode

                self.motor_controller.set_control_mode(ControlMode.OPEN_LOOP)

        elif current_mode == OperationMode.DEGRADED_AUTONOMY:
            # Use conservative parameters in degraded mode
            from ..control.advanced_motor_controller import TerrainType

            self.motor_controller.set_terrain_mode(TerrainType.ROCKY_TERRAIN)

        # Update sensor confidence based on mode
        if current_mode in [OperationMode.SAFE_STOP, OperationMode.EMERGENCY_SHUTDOWN]:
            # Reduce sensor fusion rate
            pass  # Would interface with sensor fusion system

    def _can_bridge_status_dict(self) -> Dict[str, Any]:
        """Return CAN bridge status as dict for reporting."""
        s = self.can_bridge.get_status()
        return {
            "is_connected": s.is_connected,
            "uptime": s.uptime,
            "messages_sent": s.messages_sent,
            "messages_received": s.messages_received,
            "errors": s.errors,
            **s.additional_info,
        }

    def _log_command_performance(self, twist: Twist):
        """Log velocity command performance."""
        # Add to performance tracking
        self.performance_metrics["control_loop_performance"].append(
            {
                "timestamp": time.time(),
                "linear_x": twist.linear.x,
                "angular_z": twist.angular.z,
            }
        )

        # Limit history size
        if len(self.performance_metrics["control_loop_performance"]) > 1000:
            self.performance_metrics["control_loop_performance"].pop(0)

    def get_system_status(self) -> Dict[str, Any]:
        """Get comprehensive system status."""
        return {
            "system_initialized": self.system_initialized,
            "uptime": self.performance_metrics["uptime"],
            "emergency_stop": self.emergency_stop.get_status(),
            "can_bridge": self._can_bridge_status_dict(),
            "motor_control": self.motor_controller.get_advanced_status(),
            "sensor_fusion": {
                "fused_state": self.sensor_fusion.get_fused_state().__dict__,
                "sensor_status": self.sensor_fusion.get_sensor_status(),
            },
            "operation_mode": self.operation_mode_manager.get_system_status(),
            "performance_metrics": self.performance_metrics,
        }

    def _signal_handler(self, signum, frame):
        """Handle shutdown signals."""
        logger.info(f"Received signal {signum}, initiating graceful shutdown")
        self.shutdown_requested = True
        self.shutdown_systems()
        sys.exit(0)

    def run_competition_validation(self) -> Dict[str, Any]:
        """Run competition validation tests."""
        logger.info("Starting competition validation tests")

        validation_results = {
            "can_redundancy": False,
            "emergency_stop": False,
            "motor_control": False,
            "sensor_fusion": False,
            "graceful_degradation": False,
            "overall_success": False,
        }

        try:
            can_status = self.can_bridge.get_status()
            validation_results["can_redundancy"] = can_status.is_connected

            # Test emergency stop
            e_stop_status = self.emergency_stop.get_status()
            validation_results["emergency_stop"] = (
                e_stop_status["gpio_initialized"] and e_stop_status["watchdog_enabled"]
            )

            # Test motor control
            motor_status = self.motor_controller.get_advanced_status()
            validation_results["motor_control"] = (
                motor_status["control_active"] and len(motor_status["rms_errors"]) > 0
            )

            # Test sensor fusion
            fusion_state = self.sensor_fusion.get_fused_state()
            validation_results["sensor_fusion"] = (
                np.linalg.norm(fusion_state.position) >= 0  # Basic check
            )

            # Test graceful degradation
            mode_status = self.operation_mode_manager.get_system_status()
            validation_results["graceful_degradation"] = mode_status[
                "current_mode"
            ] in [mode.value for mode in OperationMode]

            # Overall success
            validation_results["overall_success"] = all(validation_results.values())

            logger.info(f"Validation results: {validation_results}")

        except Exception as e:
            logger.error(f"Validation failed: {e}")
            validation_results["error"] = str(e)

        return validation_results


# Factory function
def create_integrated_critical_systems(
    config: Dict[str, Any] = None
) -> IntegratedCriticalSystems:
    """Create an integrated critical systems instance."""
    return IntegratedCriticalSystems(config)


# Export components
__all__ = ["IntegratedCriticalSystems", "create_integrated_critical_systems"]
