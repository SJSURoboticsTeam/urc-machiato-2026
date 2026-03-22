#!/usr/bin/env python3
"""
Multi-Level Operation Modes for Graceful Degradation

Advanced operation mode management system that provides intelligent
resource allocation and graceful degradation under various conditions.

Features:
- Multiple operation modes with automatic transitions
- Dynamic resource allocation based on system health
- Competition-specific optimization strategies
- Intelligent feature shedding under stress
- Predictive mode switching

Author: URC 2026 Graceful Degradation Team
"""

import time
import threading
import psutil
import numpy as np
from typing import Dict, List, Any, Optional, Tuple
from dataclasses import dataclass
from enum import Enum
import logging

logger = logging.getLogger(__name__)


class OperationMode(Enum):
    """System operation modes."""

    FULL_COMPETITION = "full_competition"
    DEGRADED_AUTONOMY = "degraded_autonomy"
    SURVIVAL_MODE = "survival_mode"
    SAFE_STOP = "safe_stop"
    EMERGENCY_SHUTDOWN = "emergency_shutdown"


class ResourcePriority(Enum):
    """Resource allocation priorities."""

    CRITICAL = 1  # Never disable
    HIGH = 2  # Only disable in emergency
    MEDIUM = 3  # Disable under stress
    LOW = 4  # First to disable
    OPTIONAL = 5  # Always optional


@dataclass
class SystemResources:
    """System resource usage metrics."""

    cpu_usage: float
    memory_usage: float
    disk_usage: float
    network_usage: float
    temperature: float
    power_consumption: float
    available_bandwidth: float


@dataclass
class ComponentResources:
    """Resource requirements for a component."""

    cpu_required: float
    memory_required: float
    network_required: float
    priority: ResourcePriority
    critical_for_mission: bool


@dataclass
class ModeConfiguration:
    """Configuration for an operation mode."""

    control_rate: float
    sensor_fusion_enabled: bool
    navigation_mode: str
    vision_processing: str
    science_payload: str
    telemetry_level: str
    safety_margin: float


class ComponentRegistry:
    """Registry for system components and their resource requirements."""

    def __init__(self):
        self.components = {
            # Safety systems (always critical)
            "emergency_stop": ComponentResources(
                1.0, 10.0, 0.1, ResourcePriority.CRITICAL, True
            ),
            "safety_monitor": ComponentResources(
                2.0, 20.0, 0.2, ResourcePriority.CRITICAL, True
            ),
            "collision_detection": ComponentResources(
                5.0, 50.0, 0.5, ResourcePriority.CRITICAL, True
            ),
            # Core control (high priority)
            "motor_control": ComponentResources(
                3.0, 30.0, 0.3, ResourcePriority.HIGH, True
            ),
            "state_estimation": ComponentResources(
                8.0, 80.0, 0.8, ResourcePriority.HIGH, True
            ),
            "basic_navigation": ComponentResources(
                5.0, 40.0, 0.6, ResourcePriority.HIGH, True
            ),
            # Perception systems (medium priority)
            "sensor_fusion": ComponentResources(
                10.0, 100.0, 1.0, ResourcePriority.MEDIUM, False
            ),
            "obstacle_detection": ComponentResources(
                15.0, 120.0, 1.5, ResourcePriority.MEDIUM, False
            ),
            "visual_odometry": ComponentResources(
                20.0, 150.0, 2.0, ResourcePriority.MEDIUM, False
            ),
            # Advanced features (low priority)
            "slam": ComponentResources(25.0, 200.0, 2.5, ResourcePriority.LOW, False),
            "object_recognition": ComponentResources(
                30.0, 250.0, 3.0, ResourcePriority.LOW, False
            ),
            "path_planning": ComponentResources(
                15.0, 100.0, 1.0, ResourcePriority.LOW, False
            ),
            # Optional features (optional priority)
            "science_analysis": ComponentResources(
                20.0, 180.0, 2.0, ResourcePriority.OPTIONAL, False
            ),
            "detailed_logging": ComponentResources(
                5.0, 50.0, 0.5, ResourcePriority.OPTIONAL, False
            ),
            "telemetry_streaming": ComponentResources(
                8.0, 80.0, 1.2, ResourcePriority.OPTIONAL, False
            ),
        }

        self.active_components = set(self.components.keys())
        self.disabled_components = set()

    def get_component_resources(
        self, component_id: str
    ) -> Optional[ComponentResources]:
        """Get resource requirements for a component."""
        return self.components.get(component_id)

    def get_total_resources(self, component_list: List[str]) -> Dict[str, float]:
        """Calculate total resources required for a list of components."""
        total = {"cpu": 0.0, "memory": 0.0, "network": 0.0}

        for component_id in component_list:
            resources = self.components.get(component_id)
            if resources:
                total["cpu"] += resources.cpu_required
                total["memory"] += resources.memory_required
                total["network"] += resources.network_required

        return total

    def disable_components_by_priority(self, max_priority: ResourcePriority):
        """Disable components at or below the specified priority level."""
        for component_id, resources in self.components.items():
            if resources.priority.value >= max_priority.value:
                if component_id in self.active_components:
                    self.active_components.remove(component_id)
                    self.disabled_components.add(component_id)
                    logger.info(f"Disabled component: {component_id}")

    def enable_components_by_priority(self, min_priority: ResourcePriority):
        """Enable components at or above the specified priority level."""
        for component_id, resources in self.components.items():
            if resources.priority.value <= min_priority.value:
                if component_id in self.disabled_components:
                    self.disabled_components.remove(component_id)
                    self.active_components.add(component_id)
                    logger.info(f"Enabled component: {component_id}")


class ResourceManager:
    """Dynamic resource allocation and management."""

    def __init__(self):
        self.component_registry = get_component_registry()
        self.resource_thresholds = {
            "cpu_critical": 90.0,
            "cpu_high": 75.0,
            "memory_critical": 95.0,
            "memory_high": 80.0,
            "temperature_critical": 85.0,
            "temperature_high": 75.0,
        }

        self.monitoring_active = False
        self.monitoring_thread: Optional[threading.Thread] = None
        self._shutdown_flag = False

        # Resource history for trend analysis
        self.resource_history = []
        self.max_history_size = 100

    def start_monitoring(self):
        """Start resource monitoring."""
        if self.monitoring_active:
            return

        self.monitoring_active = True
        self._shutdown_flag = False
        self.monitoring_thread = threading.Thread(
            target=self._monitoring_loop, daemon=True
        )
        self.monitoring_thread.start()
        logger.info("Resource monitoring started")

    def stop_monitoring(self):
        """Stop resource monitoring."""
        self._shutdown_flag = True
        self.monitoring_active = False

        if self.monitoring_thread:
            self.monitor_thread.join(timeout=1.0)

        logger.info("Resource monitoring stopped")

    def _monitoring_loop(self):
        """Main resource monitoring loop."""
        while not self._shutdown_flag:
            try:
                # Get current system resources
                resources = self._get_system_resources()

                # Store in history
                self.resource_history.append(resources)
                if len(self.resource_history) > self.max_history_size:
                    self.resource_history.pop(0)

                # Check if resource allocation needs adjustment
                self._evaluate_resource_allocation(resources)

                time.sleep(1.0)  # Monitor every second

            except Exception as e:
                logger.error(f"Resource monitoring error: {e}")
                time.sleep(1.0)

    def _get_system_resources(self) -> SystemResources:
        """Get current system resource usage."""
        try:
            # CPU usage
            cpu_usage = psutil.cpu_percent(interval=0.1)

            # Memory usage
            memory = psutil.virtual_memory()
            memory_usage = memory.percent

            # Disk usage
            disk = psutil.disk_usage("/")
            disk_usage = (disk.used / disk.total) * 100

            # Network usage (simplified)
            network = psutil.net_io_counters()
            network_usage = min(
                (network.bytes_sent + network.bytes_recv) / 1024 / 1024, 100.0
            )  # MB

            # Temperature (simplified - would use actual thermal sensors)
            temperature = 50.0  # Placeholder

            # Power consumption (estimated)
            power_consumption = cpu_usage * 0.5 + memory_usage * 0.3  # Watts

            # Available bandwidth (simplified)
            available_bandwidth = max(100.0 - network_usage, 0.0)

            return SystemResources(
                cpu_usage=cpu_usage,
                memory_usage=memory_usage,
                disk_usage=disk_usage,
                network_usage=network_usage,
                temperature=temperature,
                power_consumption=power_consumption,
                available_bandwidth=available_bandwidth,
            )

        except Exception as e:
            logger.error(f"Failed to get system resources: {e}")
            return SystemResources(0, 0, 0, 0, 0, 0, 0)

    def _evaluate_resource_allocation(self, resources: SystemResources):
        """Evaluate if resource allocation needs adjustment."""
        critical_conditions = []
        high_conditions = []

        # Check CPU
        if resources.cpu_usage > self.resource_thresholds["cpu_critical"]:
            critical_conditions.append("cpu")
        elif resources.cpu_usage > self.resource_thresholds["cpu_high"]:
            high_conditions.append("cpu")

        # Check memory
        if resources.memory_usage > self.resource_thresholds["memory_critical"]:
            critical_conditions.append("memory")
        elif resources.memory_usage > self.resource_thresholds["memory_high"]:
            high_conditions.append("memory")

        # Check temperature
        if resources.temperature > self.resource_thresholds["temperature_critical"]:
            critical_conditions.append("temperature")
        elif resources.temperature > self.resource_thresholds["temperature_high"]:
            high_conditions.append("temperature")

        # Take action based on conditions
        if critical_conditions:
            logger.warning(f"Critical resource conditions: {critical_conditions}")
            self._apply_critical_resource_management(critical_conditions)
        elif high_conditions:
            logger.info(f"High resource conditions: {high_conditions}")
            self._apply_high_resource_management(high_conditions)

    def _apply_critical_resource_management(self, conditions: List[str]):
        """Apply critical resource management."""
        # Disable non-critical components
        self.component_registry.disable_components_by_priority(ResourcePriority.LOW)

        # Reduce performance of remaining components
        # This would interface with actual component managers
        logger.critical("Applied critical resource management")

    def _apply_high_resource_management(self, conditions: List[str]):
        """Apply high resource management."""
        # Disable optional components
        self.component_registry.disable_components_by_priority(
            ResourcePriority.OPTIONAL
        )

        logger.warning("Applied high resource management")

    def get_available_resources(self) -> SystemResources:
        """Get current available resources."""
        return self._get_system_resources()

    def get_active_components(self) -> List[str]:
        """Get list of active components."""
        return list(self.component_registry.active_components)

    def get_disabled_components(self) -> List[str]:
        """Get list of disabled components."""
        return list(self.component_registry.disabled_components)


class OperationModeManager:
    """
    Main operation mode manager for graceful degradation.

    Coordinates resource allocation, mode switching, and system
    optimization based on current conditions and mission phase.
    """

    def __init__(self, config: Dict[str, Any] = None):
        self.config = config or {}

        # Current mode
        self.current_mode = OperationMode.FULL_COMPETITION
        self.last_mode_switch = time.time()
        self.mode_switch_count = 0

        # Mode configurations
        self.mode_configs = {
            OperationMode.FULL_COMPETITION: ModeConfiguration(
                control_rate=100.0,
                sensor_fusion=True,
                navigation_mode="global_planning",
                vision_processing="full_object_detection",
                science_payload="full_analysis",
                telemetry_level="detailed",
                safety_margin=0.1,
            ),
            OperationMode.DEGRADED_AUTONOMY: ModeConfiguration(
                control_rate=50.0,
                sensor_fusion=False,
                navigation_mode="local_planning_only",
                vision_processing="obstacle_detection_only",
                science_payload="minimal",
                telemetry_level="basic",
                safety_margin=0.2,
            ),
            OperationMode.SURVIVAL_MODE: ModeConfiguration(
                control_rate=25.0,
                sensor_fusion=False,
                navigation_mode="reactive_avoidance_only",
                vision_processing="none",
                science_payload="disabled",
                telemetry_level="minimal",
                safety_margin=0.5,
            ),
            OperationMode.SAFE_STOP: ModeConfiguration(
                control_rate=10.0,
                sensor_fusion=False,
                navigation_mode="position_hold",
                vision_processing="none",
                science_payload="disabled",
                telemetry_level="emergency",
                safety_margin=1.0,
            ),
            OperationMode.EMERGENCY_SHUTDOWN: ModeConfiguration(
                control_rate=0.0,
                sensor_fusion=False,
                navigation_mode="disabled",
                vision_processing="none",
                science_payload="disabled",
                telemetry_level="critical_only",
                safety_margin=2.0,
            ),
        }

        # Resource manager
        self.resource_manager = ResourceManager()

        # Mission context
        self.mission_phase = "setup"
        self.time_remaining = 900  # 15 minutes default
        self.current_score = 0
        self.potential_score = 1000

        # Mode switching logic
        self.mode_evaluation_interval = 2.0  # seconds
        self.last_evaluation = time.time()
        self.stability_requirement = 5.0  # seconds - stay in mode for minimum time

        # Threading
        self.mode_management_active = False
        self.mode_thread: Optional[threading.Thread] = None
        self._shutdown_flag = False

        # Status tracking
        self.mode_history = []
        self.max_history_size = 50

        logger.info("Operation mode manager initialized")

    def start_mode_management(self):
        """Start operation mode management."""
        if self.mode_management_active:
            return

        self.mode_management_active = True
        self._shutdown_flag = False
        self.resource_manager.start_monitoring()

        self.mode_thread = threading.Thread(
            target=self._mode_management_loop, daemon=True
        )
        self.mode_thread.start()

        logger.info("Operation mode management started")

    def stop_mode_management(self):
        """Stop operation mode management."""
        self._shutdown_flag = True
        self.mode_management_active = False

        self.resource_manager.stop_monitoring()

        if self.mode_thread:
            self.mode_thread.join(timeout=1.0)

        logger.info("Operation mode management stopped")

    def _mode_management_loop(self):
        """Main mode management loop."""
        while not self._shutdown_flag:
            try:
                current_time = time.time()

                # Evaluate mode switching
                if current_time - self.last_evaluation >= self.mode_evaluation_interval:
                    self._evaluate_mode_switching()
                    self.last_evaluation = current_time

                # Apply current mode configuration
                self._apply_mode_configuration()

                time.sleep(0.5)  # Check every 500ms

            except Exception as e:
                logger.error(f"Mode management error: {e}")
                time.sleep(1.0)

    def _evaluate_mode_switching(self):
        """Evaluate if mode switching is needed."""
        # Get current system resources
        resources = self.resource_manager.get_available_resources()

        # Determine appropriate mode based on resources
        recommended_mode = self._determine_recommended_mode(resources)

        # Check mission-specific factors
        if self.mission_phase == "emergency":
            recommended_mode = OperationMode.SAFE_STOP
        elif self.time_remaining < 300:  # Less than 5 minutes
            # More conservative approach in final minutes
            if recommended_mode == OperationMode.FULL_COMPETITION:
                recommended_mode = OperationMode.DEGRADED_AUTONOMY

        # Apply mode switch if needed
        if recommended_mode != self.current_mode:
            current_time = time.time()
            time_in_current_mode = current_time - self.last_mode_switch

            # Require minimum stability time
            if time_in_current_mode >= self.stability_requirement:
                self._switch_mode(recommended_mode)
            else:
                logger.debug(
                    f"Mode switch delayed for stability: {time_in_current_mode:.1f}s"
                )

    def _determine_recommended_mode(self, resources: SystemResources) -> OperationMode:
        """Determine recommended operation mode based on resources."""
        # Check for emergency conditions
        if (
            resources.cpu_usage > 95
            or resources.memory_usage > 98
            or resources.temperature > 90
        ):
            return OperationMode.EMERGENCY_SHUTDOWN

        # Check for critical conditions
        if (
            resources.cpu_usage > 85
            or resources.memory_usage > 90
            or resources.temperature > 80
        ):
            return OperationMode.SAFE_STOP

        # Check for high resource usage
        if (
            resources.cpu_usage > 75
            or resources.memory_usage > 80
            or resources.temperature > 75
        ):
            return OperationMode.SURVIVAL_MODE

        # Check for moderate resource usage
        if resources.cpu_usage > 60 or resources.memory_usage > 70:
            return OperationMode.DEGRADED_AUTONOMY

        # Default to full competition mode
        return OperationMode.FULL_COMPETITION

    def _switch_mode(self, new_mode: OperationMode):
        """Switch to a new operation mode."""
        old_mode = self.current_mode
        self.current_mode = new_mode
        self.last_mode_switch = time.time()
        self.mode_switch_count += 1

        # Record in history
        self.mode_history.append(
            {
                "timestamp": time.time(),
                "from_mode": old_mode.value,
                "to_mode": new_mode.value,
                "reason": "automatic",
            }
        )

        if len(self.mode_history) > self.max_history_size:
            self.mode_history.pop(0)

        logger.warning(f"Operation mode switched: {old_mode.value} → {new_mode.value}")

        # Apply mode-specific resource allocation
        self._apply_mode_resource_allocation(new_mode)

    def _apply_mode_configuration(self):
        """Apply current mode configuration to system."""
        config = self.mode_configs[self.current_mode]

        # This would interface with actual system components
        # For now, we'll log what would be applied
        logger.debug(f"Applying mode configuration for {self.current_mode.value}:")
        logger.debug(f"  Control rate: {config.control_rate} Hz")
        logger.debug(f"  Sensor fusion: {config.sensor_fusion_enabled}")
        logger.debug(f"  Navigation: {config.navigation_mode}")
        logger.debug(f"  Vision: {config.vision_processing}")
        logger.debug(f"  Science: {config.science_payload}")
        logger.debug(f"  Telemetry: {config.telemetry_level}")

    def _apply_mode_resource_allocation(self, mode: OperationMode):
        """Apply resource allocation for specific mode."""
        if mode == OperationMode.FULL_COMPETITION:
            # Enable all components
            self.resource_manager.component_registry.enable_components_by_priority(
                ResourcePriority.OPTIONAL
            )

        elif mode == OperationMode.DEGRADED_AUTONOMY:
            # Disable optional and low priority components
            self.resource_manager.component_registry.disable_components_by_priority(
                ResourcePriority.LOW
            )
            self.resource_manager.component_registry.enable_components_by_priority(
                ResourcePriority.MEDIUM
            )

        elif mode == OperationMode.SURVIVAL_MODE:
            # Keep only critical and high priority components
            self.resource_manager.component_registry.disable_components_by_priority(
                ResourcePriority.MEDIUM
            )
            self.resource_manager.component_registry.enable_components_by_priority(
                ResourcePriority.HIGH
            )

        elif mode == OperationMode.SAFE_STOP:
            # Keep only critical components
            self.resource_manager.component_registry.disable_components_by_priority(
                ResourcePriority.HIGH
            )
            self.resource_manager.component_registry.enable_components_by_priority(
                ResourcePriority.CRITICAL
            )

        elif mode == OperationMode.EMERGENCY_SHUTDOWN:
            # Keep only emergency stop and safety monitor
            for component in list(
                self.resource_manager.component_registry.active_components
            ):
                if component not in ["emergency_stop", "safety_monitor"]:
                    self.resource_manager.component_registry.active_components.remove(
                        component
                    )
                    self.resource_manager.component_registry.disabled_components.add(
                        component
                    )

    def set_mission_context(self, phase: str, time_remaining: float, score: float = 0):
        """Set mission context for intelligent mode switching."""
        self.mission_phase = phase
        self.time_remaining = time_remaining
        self.current_score = score

        logger.info(
            f"Mission context: phase={phase}, time_remaining={time_remaining:.1f}s, score={score}"
        )

    def force_mode_switch(self, mode: OperationMode, reason: str = "manual"):
        """Force a mode switch."""
        self._switch_mode(mode)
        logger.info(f"Manual mode switch to {mode.value}: {reason}")

    def get_system_status(self) -> Dict[str, Any]:
        """Get comprehensive system status."""
        resources = self.resource_manager.get_available_resources()

        return {
            "current_mode": self.current_mode.value,
            "mode_switch_count": self.mode_switch_count,
            "last_mode_switch": self.last_mode_switch,
            "mission_phase": self.mission_phase,
            "time_remaining": self.time_remaining,
            "current_score": self.current_score,
            "system_resources": {
                "cpu_usage": resources.cpu_usage,
                "memory_usage": resources.memory_usage,
                "temperature": resources.temperature,
                "network_usage": resources.network_usage,
            },
            "active_components": self.resource_manager.get_active_components(),
            "disabled_components": self.resource_manager.get_disabled_components(),
            "mode_configuration": self.mode_configs[self.current_mode].__dict__,
        }

    def get_mode_history(self) -> List[Dict[str, Any]]:
        """Get mode switching history."""
        return self.mode_history.copy()


# Factory function
def create_operation_mode_manager(
    config: Dict[str, Any] = None
) -> OperationModeManager:
    """Create an operation mode manager instance."""
    return OperationModeManager(config)


# Export components
__all__ = [
    "OperationModeManager",
    "ResourceManager",
    "ComponentRegistry",
    "OperationMode",
    "ResourcePriority",
    "SystemResources",
    "ComponentResources",
    "ModeConfiguration",
    "create_operation_mode_manager",
]
