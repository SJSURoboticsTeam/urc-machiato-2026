#!/usr/bin/env python3
"""
Mission Resource Manager - Dynamic Component Management for URC 2026

Manages system resources based on mission requirements, dynamically enabling/disabling
components to optimize performance and power consumption.

Key Features:
- Mission-based component orchestration
- Dynamic resource allocation
- Automatic scaling based on system load
- Lightweight fallback modes
- Power consumption optimization

Author: URC 2026 Resource Optimization Team
"""

import threading
import time
import psutil
import logging
from typing import Dict, List, Any, Optional, Callable, Set
from enum import Enum
from dataclasses import dataclass, field

from src.infrastructure.config import get_config

logger = logging.getLogger(__name__)


class ComponentStatus(Enum):
    """Component operational status."""
    DISABLED = "disabled"
    ENABLED = "enabled"
    LIGHTWEIGHT = "lightweight"
    FULL = "full"


class ResourcePriority(Enum):
    """Resource allocation priorities."""
    CRITICAL = "critical"  # Safety, emergency systems
    HIGH = "high"         # Navigation, SLAM
    MEDIUM = "medium"     # Vision, terrain analysis
    LOW = "low"          # Telemetry, logging


@dataclass
class ComponentConfig:
    """Configuration for a system component."""
    name: str
    priority: ResourcePriority
    resource_usage: Dict[str, float]  # CPU%, Memory MB, etc.
    dependencies: List[str] = field(default_factory=list)
    enable_callback: Optional[Callable] = None
    disable_callback: Optional[Callable] = None
    lightweight_callback: Optional[Callable] = None


class MissionResourceManager:
    """
    Dynamic resource management for mission-based optimization.

    Orchestrates component enable/disable based on mission requirements
    and system resource availability.
    """

    def __init__(self):
        """Initialize the resource manager."""
        self.config = get_config()
        self.current_mission_profile = None
        self.component_status: Dict[str, ComponentStatus] = {}
        self.resource_monitoring = False
        self.monitoring_thread: Optional[threading.Thread] = None
        self.adaptive_scaling_enabled = True

        # Component registry
        self.components: Dict[str, ComponentConfig] = {}
        self._register_components()

        # Resource monitoring
        self.resource_history: List[Dict[str, Any]] = []
        self.scaling_thresholds = {
            'cpu_warning': 75.0,
            'cpu_critical': 90.0,
            'memory_warning_mb': 500.0,
            'memory_critical_mb': 700.0
        }

        logger.info("Mission Resource Manager initialized")

    def _register_components(self):
        """Register all system components with their configurations."""

        # Computer Vision Components
        self.components['computer_vision'] = ComponentConfig(
            name='computer_vision',
            priority=ResourcePriority.MEDIUM,
            resource_usage={'cpu_percent': 30.0, 'memory_mb': 200.0},
            enable_callback=self._enable_computer_vision,
            disable_callback=self._disable_computer_vision,
            lightweight_callback=self._enable_lightweight_vision
        )

        # SLAM Components
        self.components['slam_full'] = ComponentConfig(
            name='slam_full',
            priority=ResourcePriority.HIGH,
            resource_usage={'cpu_percent': 25.0, 'memory_mb': 300.0},
            dependencies=['computer_vision'],
            enable_callback=self._enable_full_slam,
            disable_callback=self._disable_slam,
            lightweight_callback=self._enable_lightweight_slam
        )

        self.components['slam_loop_closure'] = ComponentConfig(
            name='slam_loop_closure',
            priority=ResourcePriority.MEDIUM,
            resource_usage={'cpu_percent': 15.0, 'memory_mb': 150.0},
            dependencies=['slam_full'],
            enable_callback=self._enable_loop_closure,
            disable_callback=self._disable_loop_closure
        )

        # Terrain Analysis
        self.components['terrain_analysis'] = ComponentConfig(
            name='terrain_analysis',
            priority=ResourcePriority.MEDIUM,
            resource_usage={'cpu_percent': 20.0, 'memory_mb': 100.0},
            enable_callback=self._enable_terrain_analysis,
            disable_callback=self._disable_terrain_analysis,
            lightweight_callback=self._enable_lightweight_terrain
        )

        # Excavation Systems
        self.components['excavation'] = ComponentConfig(
            name='excavation',
            priority=ResourcePriority.LOW,
            resource_usage={'cpu_percent': 5.0, 'memory_mb': 50.0},
            enable_callback=self._enable_excavation,
            disable_callback=self._disable_excavation
        )

        # Science Payload
        self.components['science_payload'] = ComponentConfig(
            name='science_payload',
            priority=ResourcePriority.LOW,
            resource_usage={'cpu_percent': 3.0, 'memory_mb': 30.0},
            enable_callback=self._enable_science_payload,
            disable_callback=self._disable_science_payload
        )

        # Advanced Path Planning
        self.components['advanced_path_planning'] = ComponentConfig(
            name='advanced_path_planning',
            priority=ResourcePriority.HIGH,
            resource_usage={'cpu_percent': 10.0, 'memory_mb': 80.0},
            enable_callback=self._enable_advanced_planning,
            disable_callback=self._disable_advanced_planning,
            lightweight_callback=self._enable_basic_planning
        )

        # Telemetry Levels
        self.components['telemetry_detailed'] = ComponentConfig(
            name='telemetry_detailed',
            priority=ResourcePriority.LOW,
            resource_usage={'cpu_percent': 2.0, 'memory_mb': 20.0},
            enable_callback=self._enable_detailed_telemetry,
            disable_callback=self._disable_detailed_telemetry,
            lightweight_callback=self._enable_standard_telemetry
        )

    def switch_mission_profile(self, mission_type: str):
        """
        Switch to a specific mission profile, enabling/disabling components accordingly.

        Args:
            mission_type: Mission profile name (e.g., 'waypoint_navigation', 'sample_collection')
        """
        if mission_type not in self.config.get('mission_profiles', {}):
            logger.error(f"Unknown mission profile: {mission_type}")
            return False

        profile = self.config['mission_profiles'][mission_type]
        self.current_mission_profile = mission_type

        logger.info(f"Switching to mission profile: {mission_type}")

        # Apply component settings from profile
        component_mappings = {
            'computer_vision_enabled': 'computer_vision',
            'terrain_analysis_enabled': 'terrain_analysis',
            'slam_loop_closure_enabled': 'slam_loop_closure',
            'excavation_enabled': 'excavation',
            'science_payload_enabled': 'science_payload',
            'advanced_path_planning': 'advanced_path_planning'
        }

        # Disable all components first
        self._disable_all_components()

        # Enable components based on profile
        for config_key, component_name in component_mappings.items():
            if profile.get(config_key, False):
                self._enable_component(component_name, ComponentStatus.FULL)
            else:
                self._disable_component(component_name)

        # Handle telemetry level
        telemetry_level = profile.get('telemetry_level', 'standard')
        if telemetry_level == 'detailed':
            self._enable_component('telemetry_detailed', ComponentStatus.FULL)
        elif telemetry_level == 'minimal':
            self._enable_component('telemetry_detailed', ComponentStatus.LIGHTWEIGHT)
        else:
            self._enable_component('telemetry_detailed', ComponentStatus.LIGHTWEIGHT)

        # Enable base SLAM if any SLAM components are enabled
        if (profile.get('slam_loop_closure_enabled', False) or
            profile.get('computer_vision_enabled', False)):
            self._enable_component('slam_full', ComponentStatus.FULL)

        # Apply vision settings if vision is enabled
        if profile.get('computer_vision_enabled', False):
            self._configure_vision_settings(profile)

        logger.info(f"Successfully switched to mission profile: {mission_type}")
        return True

    def _configure_vision_settings(self, profile: Dict[str, Any]):
        """Configure computer vision settings based on profile."""
        fps = profile.get('vision_fps', 20)
        resolution = profile.get('vision_resolution', '720p')

        # Apply vision configuration (this would integrate with actual vision system)
        logger.info(f"Configuring vision: {fps}fps at {resolution}")

    def start_resource_monitoring(self):
        """Start adaptive resource monitoring and scaling."""
        if self.resource_monitoring:
            return

        self.resource_monitoring = True
        self.monitoring_thread = threading.Thread(
            target=self._resource_monitoring_loop,
            daemon=True,
            name="ResourceMonitor"
        )
        self.monitoring_thread.start()
        logger.info("Resource monitoring started")

    def stop_resource_monitoring(self):
        """Stop resource monitoring."""
        if not self.resource_monitoring:
            return

        self.resource_monitoring = False
        if self.monitoring_thread and self.monitoring_thread.is_alive():
            self.monitoring_thread.join(timeout=2.0)
        logger.info("Resource monitoring stopped")

    def _resource_monitoring_loop(self):
        """Main resource monitoring loop."""
        while self.resource_monitoring:
            try:
                resources = self._measure_resources()

                # Store in history (keep last 10 measurements)
                self.resource_history.append(resources)
                if len(self.resource_history) > 10:
                    self.resource_history.pop(0)

                # Check for scaling triggers
                if self.adaptive_scaling_enabled:
                    self._check_resource_thresholds(resources)

                time.sleep(2.0)  # Monitor every 2 seconds

            except Exception as e:
                logger.error(f"Resource monitoring error: {e}")
                time.sleep(2.0)

    def _measure_resources(self) -> Dict[str, Any]:
        """Measure current system resources."""
        process = psutil.Process()
        memory = psutil.virtual_memory()

        return {
            'timestamp': time.time(),
            'cpu_percent': process.cpu_percent(interval=None),
            'memory_mb': process.memory_info().rss / (1024 * 1024),
            'system_memory_percent': memory.percent,
            'system_memory_available_mb': memory.available / (1024 * 1024)
        }

    def _check_resource_thresholds(self, resources: Dict[str, Any]):
        """Check if resources exceed thresholds and take action."""
        cpu_percent = resources['cpu_percent']
        memory_mb = resources['memory_mb']

        # Critical thresholds - disable components immediately
        if cpu_percent > self.scaling_thresholds['cpu_critical']:
            logger.warning(".1f")
            self._emergency_resource_reduction()

        elif memory_mb > self.scaling_thresholds['memory_critical_mb']:
            logger.warning(".1f")
            self._emergency_resource_reduction()

        # Warning thresholds - scale back gradually
        elif cpu_percent > self.scaling_thresholds['cpu_warning']:
            logger.info(".1f")
            self._scale_back_resources()

        elif memory_mb > self.scaling_thresholds['memory_warning_mb']:
            logger.info(".1f")
            self._scale_back_resources()

    def _emergency_resource_reduction(self):
        """Emergency reduction of resource usage."""
        logger.warning("Emergency resource reduction triggered")

        # Disable lowest priority components first
        low_priority_components = [
            comp for comp in self.components.values()
            if comp.priority == ResourcePriority.LOW and
            self.component_status.get(comp.name) != ComponentStatus.DISABLED
        ]

        for component in low_priority_components:
            self._disable_component(component.name)

        # If still critical, disable medium priority
        if self._check_resources_critical():
            medium_priority_components = [
                comp for comp in self.components.values()
                if comp.priority == ResourcePriority.MEDIUM and
                self.component_status.get(comp.name) != ComponentStatus.DISABLED
            ]

            for component in medium_priority_components:
                self._set_component_lightweight(component.name)

    def _scale_back_resources(self):
        """Gradually scale back resource usage."""
        logger.info("Scaling back resources")

        # Switch high-resource components to lightweight mode
        for component_name, status in self.component_status.items():
            if status == ComponentStatus.FULL:
                component = self.components.get(component_name)
                if component and component.lightweight_callback:
                    self._set_component_lightweight(component_name)

    def _check_resources_critical(self) -> bool:
        """Check if resources are still in critical state."""
        try:
            resources = self._measure_resources()
            return (resources['cpu_percent'] > self.scaling_thresholds['cpu_critical'] or
                    resources['memory_mb'] > self.scaling_thresholds['memory_critical_mb'])
        except:
            return True  # Assume critical if measurement fails

    def _disable_all_components(self):
        """Disable all optional components."""
        for component_name in self.components.keys():
            self._disable_component(component_name)

    def _enable_component(self, component_name: str, status: ComponentStatus):
        """Enable a component with specified status."""
        if component_name not in self.components:
            logger.warning(f"Unknown component: {component_name}")
            return

        component = self.components[component_name]
        self.component_status[component_name] = status

        # Call appropriate callback
        if status == ComponentStatus.FULL and component.enable_callback:
            component.enable_callback()
        elif status == ComponentStatus.LIGHTWEIGHT and component.lightweight_callback:
            component.lightweight_callback()
        elif status == ComponentStatus.ENABLED and component.enable_callback:
            component.enable_callback()

        logger.info(f"Component {component_name} set to {status.value}")

    def _disable_component(self, component_name: str):
        """Disable a component."""
        if component_name not in self.components:
            return

        component = self.components[component_name]
        self.component_status[component_name] = ComponentStatus.DISABLED

        if component.disable_callback:
            component.disable_callback()

        logger.info(f"Component {component_name} disabled")

    def _set_component_lightweight(self, component_name: str):
        """Switch component to lightweight mode."""
        if component_name not in self.components:
            return

        component = self.components[component_name]

        if component.lightweight_callback:
            self.component_status[component_name] = ComponentStatus.LIGHTWEIGHT
            component.lightweight_callback()
            logger.info(f"Component {component_name} switched to lightweight mode")
        else:
            # No lightweight mode available, disable instead
            self._disable_component(component_name)

    # Component control callbacks (implementations would integrate with actual systems)

    def _enable_computer_vision(self):
        """Enable full computer vision processing."""
        logger.info("Enabling full computer vision")

    def _disable_computer_vision(self):
        """Disable computer vision processing."""
        logger.info("Disabling computer vision")

    def _enable_lightweight_vision(self):
        """Enable lightweight computer vision (reduced processing)."""
        logger.info("Enabling lightweight computer vision")

    def _enable_full_slam(self):
        """Enable full SLAM processing."""
        logger.info("Enabling full SLAM")

    def _disable_slam(self):
        """Disable SLAM processing."""
        logger.info("Disabling SLAM")

    def _enable_lightweight_slam(self):
        """Enable lightweight SLAM (visual odometry only)."""
        logger.info("Enabling lightweight SLAM")

    def _enable_loop_closure(self):
        """Enable SLAM loop closure detection."""
        logger.info("Enabling SLAM loop closure")

    def _disable_loop_closure(self):
        """Disable SLAM loop closure detection."""
        logger.info("Disabling SLAM loop closure")

    def _enable_terrain_analysis(self):
        """Enable full terrain analysis."""
        logger.info("Enabling terrain analysis")

    def _disable_terrain_analysis(self):
        """Disable terrain analysis."""
        logger.info("Disabling terrain analysis")

    def _enable_lightweight_terrain(self):
        """Enable lightweight terrain analysis."""
        logger.info("Enabling lightweight terrain analysis")

    def _enable_excavation(self):
        """Enable excavation systems."""
        logger.info("Enabling excavation systems")

    def _disable_excavation(self):
        """Disable excavation systems."""
        logger.info("Disabling excavation systems")

    def _enable_science_payload(self):
        """Enable science payload systems."""
        logger.info("Enabling science payload")

    def _disable_science_payload(self):
        """Disable science payload systems."""
        logger.info("Disabling science payload")

    def _enable_advanced_planning(self):
        """Enable advanced path planning."""
        logger.info("Enabling advanced path planning")

    def _disable_advanced_planning(self):
        """Disable advanced path planning."""
        logger.info("Disabling advanced path planning")

    def _enable_basic_planning(self):
        """Enable basic path planning."""
        logger.info("Enabling basic path planning")

    def _enable_detailed_telemetry(self):
        """Enable detailed telemetry logging."""
        logger.info("Enabling detailed telemetry")

    def _disable_detailed_telemetry(self):
        """Disable detailed telemetry logging."""
        logger.info("Disabling detailed telemetry")

    def _enable_standard_telemetry(self):
        """Enable standard telemetry logging."""
        logger.info("Enabling standard telemetry")

    def get_resource_status(self) -> Dict[str, Any]:
        """Get current resource usage and component status."""
        return {
            'component_status': {name: status.value for name, status in self.component_status.items()},
            'current_resources': self._measure_resources() if self.resource_history else {},
            'mission_profile': self.current_mission_profile,
            'adaptive_scaling': self.adaptive_scaling_enabled,
            'monitoring_active': self.resource_monitoring
        }


# Global instance
_resource_manager = None

def get_mission_resource_manager() -> MissionResourceManager:
    """Get the global mission resource manager instance."""
    global _resource_manager
    if _resource_manager is None:
        _resource_manager = MissionResourceManager()
    return _resource_manager



