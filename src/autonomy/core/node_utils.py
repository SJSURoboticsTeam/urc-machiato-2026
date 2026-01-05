#!/usr/bin/env python3
"""
URC 2026 Node Utilities - Common ROS2 Node Functionality

Extracted common utilities to reduce individual node size and improve maintainability.
Provides shared ROS2 patterns, lifecycle management, and performance monitoring.

Features:
- Intelligent ROS2 environment detection
- Mock implementations for development
- Lazy loading for heavy dependencies
- Performance optimization for embedded systems

Author: URC 2026 ROS2 Team
"""

import asyncio
import logging
import os
import psutil
import threading
import time
from abc import ABC, abstractmethod
from typing import Any, Dict, List, Optional, Callable, Union
from dataclasses import dataclass, field

# Intelligent ROS2 imports with fallbacks
from src.core.ros2_environment import (
    get_ros2_environment_manager,
    safe_ros2_import,
    lazy_ros2_import
)

# Unified systems
from src.core.observability import get_observability_system
from src.core.utilities import get_safety_manager, get_hardware_validator

# Get ROS2 environment manager
ros2_env = get_ros2_environment_manager()

# Safe imports with fallbacks
rclpy = safe_ros2_import('rclpy')
lifecycle = safe_ros2_import('rclpy.lifecycle')
qos = safe_ros2_import('rclpy.qos')

# Extract classes with fallbacks
LifecycleNode = getattr(lifecycle, 'LifecycleNode', None)
LifecycleState = getattr(lifecycle, 'LifecycleState', None)
TransitionCallbackReturn = getattr(lifecycle, 'TransitionCallbackReturn', None)
QoSProfile = getattr(qos, 'QoSProfile', None)
ReliabilityPolicy = getattr(qos, 'ReliabilityPolicy', None)
DurabilityPolicy = getattr(qos, 'DurabilityPolicy', None)
HistoryPolicy = getattr(qos, 'HistoryPolicy', None)

# Additional ROS2 components with fallbacks
try:
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
except ImportError:
    # Mock implementations for development
    class MultiThreadedExecutor:
        def __init__(self, num_threads=4): pass
        def add_node(self, node): pass
        def spin(self): pass
        def shutdown(self): pass

    class MutuallyExclusiveCallbackGroup:
        pass

    class ReentrantCallbackGroup:
        pass

logger = logging.getLogger(__name__)

# Lazy loading registry for heavy dependencies
_lazy_loaders: Dict[str, Callable] = {}
_heavy_dependencies: Dict[str, Dict[str, Any]] = {}


def register_heavy_dependency(name: str, import_path: str):
    """
    Register a heavy dependency for lazy loading.

    Args:
        name: Name to register the dependency under
        import_path: Full import path for the dependency
    """
    _lazy_loaders[name] = lazy_ros2_import(name, import_path)


def get_lazy_loader(name: str = None) -> Union[Callable, Dict[str, Callable]]:
    """
    Get lazy loader for a dependency or all loaders.

    Args:
        name: Specific dependency name, or None for all loaders

    Returns:
        Lazy loader function or dictionary of all loaders
    """
    if name:
        return _lazy_loaders.get(name, lambda: None)
    return _lazy_loaders.copy()


def get_profiler():
    """Get performance profiler instance."""
    # Lightweight profiler for embedded systems
    class LightweightProfiler:
        def __init__(self):
            self.start_times = {}

        def start(self, name: str):
            self.start_times[name] = time.time()

        def stop(self, name: str) -> float:
            if name in self.start_times:
                duration = time.time() - self.start_times[name]
                del self.start_times[name]
                return duration
            return 0.0

        def profile_function(self, func_name: str):
            def decorator(func):
                def wrapper(*args, **kwargs):
                    self.start(func_name)
                    try:
                        return func(*args, **kwargs)
                    finally:
                        duration = self.stop(func_name)
                        logger.debug(".3f")
                return wrapper
            return decorator

    return LightweightProfiler()


def get_memory_monitor():
    """Get memory monitor instance."""
    class LightweightMemoryMonitor:
        def __init__(self):
            self.baseline = 0
            self.process = psutil.Process()

        def set_baseline(self):
            self.baseline = self.process.memory_info().rss / 1024 / 1024

        def get_memory_usage(self) -> float:
            return self.process.memory_info().rss / 1024 / 1024

        def get_memory_delta(self) -> float:
            return self.get_memory_usage() - self.baseline

        def check_memory_leak(self, threshold_mb: float = 50) -> bool:
            delta = self.get_memory_delta()
            return delta > threshold_mb

    monitor = LightweightMemoryMonitor()
    monitor.set_baseline()
    return monitor


@dataclass
class NodeMetrics:
    """Node performance metrics."""
    startup_time: float = 0.0
    memory_usage_mb: float = 0.0
    cpu_percent: float = 0.0
    message_count: int = 0
    error_count: int = 0
    last_heartbeat: float = field(default_factory=time.time)


class BaseURCNode(ABC):
    """
    Base URC node with intelligent ROS2 environment handling.

    Provides:
    - Automatic ROS2 detection and fallback to mock implementations
    - Lifecycle management with development-friendly fallbacks
    - Performance monitoring and optimization for embedded systems
    - Lazy loading for heavy dependencies
    - Health checking and telemetry
    """

    def __init__(self, node_name: str, **kwargs):
        # ROS2 environment detection and setup
        self._ros2_env = ros2_env
        self._ros2_available = self._ros2_env.environment.available and not self._ros2_env.environment.mock_mode
        self._performance_mode = self._ros2_env.is_performance_mode()

        # Initialize based on ROS2 availability
        if self._ros2_available and LifecycleNode:
            # Use real ROS2 lifecycle node
            LifecycleNode.__init__(self, node_name, **kwargs)
            self._using_mock = False
            self.logger = self.get_logger()
        else:
            # Use lightweight mock implementation
            self.node_name = node_name
            self.logger = logging.getLogger(node_name)
            self._using_mock = True
            self._publishers = {}
            self._subscribers = {}
            self._services = {}
            self._actions = {}
            self._timers = []

        # Core components
        self.node_name = node_name

        # Use unified observability system
        self.observability = get_observability_system()

        # Add node health check
        self.observability.add_health_check(
            f"node_{node_name}",
            self._node_health_check,
            interval=30.0
        )

        # QoS profiles (with fallbacks)
        self.qos_profiles = self._create_qos_profiles()

        # Performance tracking
        self.perf_metrics = NodeMetrics()

        # Lazy loading setup
        self._lazy_components = {}
        self._heavy_components_loaded = set()

        # Callback groups for concurrent execution (ROS2 only)
        if self._ros2_available:
            self.callback_groups = {
                'default': ReentrantCallbackGroup(),
                'exclusive': MutuallyExclusiveCallbackGroup()
            }
        else:
            self.callback_groups = {}

        # Embedded system optimizations
        if self._performance_mode:
            self._setup_embedded_optimizations()

        self.observability.info(f"URC Node initialized",
                               component=node_name,
                               ros2_available=self._ros2_available,
                               mock_mode=self._using_mock)

    def _node_health_check(self) -> Dict[str, Any]:
        """Node health check for unified observability system."""
        try:
            # Check if node is responding
            healthy = True
            status_message = "Node operational"

            # Check memory usage
            memory_mb = self._memory_monitor.get_memory_usage()
            if memory_mb > self._memory_monitor.baseline * 2:  # More than 2x baseline
                healthy = False
                status_message = f"High memory usage: {memory_mb:.1f}MB"

            # Check if ROS2 is available and connected (if applicable)
            if self._ros2_available and hasattr(self, '_ros2_env'):
                ros2_healthy = self._ros2_env.environment.available
                if not ros2_healthy:
                    healthy = False
                    status_message = "ROS2 environment unavailable"

            return {
                "healthy": healthy,
                "message": status_message,
                "memory_mb": memory_mb,
                "components_loaded": len(self._heavy_components_loaded),
                "ros2_available": self._ros2_available
            }

        except Exception as e:
            return {
                "healthy": False,
                "message": f"Health check error: {e}",
                "memory_mb": 0,
                "components_loaded": 0,
                "ros2_available": False
            }

    def _setup_embedded_optimizations(self):
        """Setup optimizations for embedded systems."""
        # Reduce logging verbosity in observability system
        # Note: This would be configured globally, not per node

        # Disable expensive health monitoring features
        # Note: Basic health checks remain active but with reduced frequency

        self.observability.info("Embedded optimizations enabled",
                               component=self.node_name)

    def _create_qos_profiles(self) -> Dict[str, Any]:
        """Create standard QoS profiles with fallbacks."""
        if self._ros2_available and QoSProfile and ReliabilityPolicy and DurabilityPolicy and HistoryPolicy:
            # Real ROS2 QoS profiles
            return {
                'sensor': QoSProfile(
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=1
                ),
                'control': QoSProfile(
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=10
                ),
                'telemetry': QoSProfile(
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=50
                ),
                'emergency': QoSProfile(
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    history=HistoryPolicy.KEEP_ALL,
                    depth=100
                ),
            }
        else:
            # Mock QoS profiles for development
            return {
                'sensor': {'reliability': 'best_effort', 'depth': 1},
                'control': {'reliability': 'reliable', 'depth': 10},
                'telemetry': {'reliability': 'reliable', 'depth': 50},
                'emergency': {'reliability': 'reliable', 'depth': 100},
            }

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure lifecycle callback."""
        self.get_logger().info(f"Configuring {self.node_name}")
        self.health_monitor.start_monitoring()
        self.perf_metrics.startup_time = time.time()
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Activate lifecycle callback."""
        self.get_logger().info(f"Activating {self.node_name}")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Deactivate lifecycle callback."""
        self.get_logger().info(f"Deactivating {self.node_name}")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Cleanup lifecycle callback."""
        self.get_logger().info(f"Cleaning up {self.node_name}")
        self.health_monitor.stop_monitoring()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Shutdown lifecycle callback."""
        self.get_logger().info(f"Shutting down {self.node_name}")
        self.health_monitor.stop_monitoring()
        return TransitionCallbackReturn.SUCCESS

    def create_monitored_publisher(self, msg_type, topic: str, qos_profile: str = 'control'):
        """Create publisher with performance monitoring."""
        qos = self.qos_profiles[qos_profile]
        publisher = self.create_publisher(msg_type, topic, qos, callback_group=self.callback_groups['default'])

        # Wrap publisher to track metrics
        original_publish = publisher.publish
        def monitored_publish(msg):
            self.perf_metrics.message_count += 1
            try:
                original_publish(msg)
            except Exception as e:
                self.perf_metrics.error_count += 1
                self.get_logger().error(f"Publish error: {e}")
                raise

        publisher.publish = monitored_publish
        return publisher

    def create_monitored_subscription(self, msg_type, topic: str, callback: Callable,
                                    qos_profile: str = 'control'):
        """Create subscription with performance monitoring."""
        qos = self.qos_profiles[qos_profile]

        def monitored_callback(msg):
            self.perf_metrics.message_count += 1
            try:
                callback(msg)
            except Exception as e:
                self.perf_metrics.error_count += 1
                self.get_logger().error(f"Callback error: {e}")

        return self.create_subscription(
            msg_type, topic, monitored_callback, qos,
            callback_group=self.callback_groups['default']
        )

    def get_node_status(self) -> Dict[str, Any]:
        """Get comprehensive node status."""
        health = self.health_monitor.get_health_status()

        return {
            "node_name": self.node_name,
            "lifecycle_state": str(self._state_machine.current_state),
            "health": health,
            "performance": {
                "startup_time": self.perf_metrics.startup_time,
                "messages_processed": self.perf_metrics.message_count,
                "errors": self.perf_metrics.error_count
            },
            "qos_profiles": list(self.qos_profiles.keys()),
            "timestamp": time.time()
        }

    # Lazy loading methods for heavy dependencies
    def register_lazy_component(self, name: str, loader_function: Callable):
        """
        Register a lazy component loader.

        Args:
            name: Component name
            loader_function: Function that returns the component when called
        """
        self._lazy_components[name] = loader_function

    def get_lazy_component(self, name: str) -> Any:
        """
        Get a component, loading it lazily if not already loaded.

        Args:
            name: Component name

        Returns:
            Loaded component

        Raises:
            KeyError: If component not registered
        """
        if name not in self._lazy_components:
            raise KeyError(f"Lazy component '{name}' not registered")

        # Load component if not already loaded
        if name not in self._heavy_components_loaded:
            start_time = time.time()
            component = self._lazy_components[name]()
            load_time = time.time() - start_time

            self._heavy_components_loaded.add(name)
            logger.info(f"Lazy loaded component '{name}' in {load_time:.3f}s")

            # Store the loaded component
            setattr(self, f"_{name}", component)

            return component

        # Return already loaded component
        return getattr(self, f"_{name}")

    def unload_heavy_component(self, name: str):
        """
        Unload a heavy component to free memory.

        Args:
            name: Component name
        """
        if name in self._heavy_components_loaded:
            if hasattr(self, f"_{name}"):
                # Try to cleanup if component has cleanup method
                component = getattr(self, f"_{name}")
                if hasattr(component, 'cleanup'):
                    component.cleanup()

                delattr(self, f"_{name}")

            self._heavy_components_loaded.remove(name)
            logger.info(f"Unloaded heavy component '{name}'")

    def get_loaded_components(self) -> List[str]:
        """Get list of currently loaded heavy components."""
        return list(self._heavy_components_loaded)

    def get_memory_usage(self) -> Dict[str, Any]:
        """Get detailed memory usage information."""
        base_memory = self._memory_monitor.get_memory_usage()

        component_memory = {}
        for comp_name in self._heavy_components_loaded:
            # Estimate memory per component (rough approximation)
            component_memory[comp_name] = base_memory * 0.1  # Assume 10% per component

        return {
            'total_memory_mb': base_memory,
            'baseline_memory_mb': self._memory_monitor.baseline,
            'memory_delta_mb': self._memory_monitor.get_memory_delta(),
            'loaded_components': len(self._heavy_components_loaded),
            'component_memory_estimate': component_memory
        }


class LazyLoader:
    """Lazy loading utility for heavy dependencies."""

    def __init__(self):
        self._loaded_modules: Dict[str, Any] = {}
        self._load_functions: Dict[str, Callable] = {}

    def register_loader(self, name: str, load_function: Callable):
        """Register a lazy loader function."""
        self._load_functions[name] = load_function

    def get_module(self, name: str) -> Any:
        """Get module, loading it if necessary."""
        if name not in self._loaded_modules:
            if name in self._load_functions:
                self._loaded_modules[name] = self._load_functions[name]()
            else:
                raise ImportError(f"No loader registered for {name}")

        return self._loaded_modules[name]

    def is_loaded(self, name: str) -> bool:
        """Check if module is loaded."""
        return name in self._loaded_modules


# Global lazy loader instance
_lazy_loader = LazyLoader()

def get_lazy_loader() -> LazyLoader:
    """Get global lazy loader instance."""
    return _lazy_loader

def register_heavy_dependency(name: str, import_path: str):
    """Register a heavy dependency for lazy loading."""
    def loader():
        import importlib
        return importlib.import_module(import_path)

    _lazy_loader.register_loader(name, loader)

# Performance profiling utilities
class PerformanceProfiler:
    """Simple performance profiler for ROS2 nodes."""

    def __init__(self):
        self.measurements: Dict[str, List[float]] = {}
        self.start_times: Dict[str, float] = {}

    def start_timer(self, name: str):
        """Start timing an operation."""
        self.start_times[name] = time.time()

    def end_timer(self, name: str) -> float:
        """End timing and return duration."""
        if name not in self.start_times:
            return 0.0

        duration = time.time() - self.start_times[name]
        if name not in self.measurements:
            self.measurements[name] = []
        self.measurements[name].append(duration)

        return duration

    def get_stats(self, name: str) -> Dict[str, float]:
        """Get statistics for a measurement."""
        if name not in self.measurements:
            return {}

        measurements = self.measurements[name]
        return {
            "count": len(measurements),
            "avg": sum(measurements) / len(measurements),
            "min": min(measurements),
            "max": max(measurements),
            "last": measurements[-1] if measurements else 0.0
        }

# Global profiler instance
_profiler = PerformanceProfiler()

def get_profiler() -> PerformanceProfiler:
    """Get global performance profiler."""
    return _profiler

# Memory optimization utilities
class MemoryMonitor:
    """Monitor memory usage and provide optimization hints."""

    def __init__(self):
        self.process = psutil.Process()
        self.baseline_memory = 0

    def set_baseline(self):
        """Set baseline memory usage."""
        self.baseline_memory = self.process.memory_info().rss

    def get_memory_usage(self) -> Dict[str, Any]:
        """Get current memory usage."""
        mem_info = self.process.memory_info()
        return {
            "rss_mb": mem_info.rss / 1024 / 1024,
            "vms_mb": mem_info.vms / 1024 / 1024,
            "baseline_delta_mb": (mem_info.rss - self.baseline_memory) / 1024 / 1024 if self.baseline_memory else 0
        }

    def check_memory_pressure(self) -> str:
        """Check memory pressure level."""
        usage = self.get_memory_usage()
        if usage["rss_mb"] > 500:  # 500MB threshold
            return "CRITICAL"
        elif usage["rss_mb"] > 200:  # 200MB threshold
            return "HIGH"
        elif usage["rss_mb"] > 100:  # 100MB threshold
            return "MODERATE"
        else:
            return "LOW"

# Global memory monitor
_memory_monitor = MemoryMonitor()

def get_memory_monitor() -> MemoryMonitor:
    """Get global memory monitor."""
    return _memory_monitor

# Export key components
__all__ = [
    'BaseURCNode',
    'NodeHealthMonitor',
    'LazyLoader',
    'PerformanceProfiler',
    'MemoryMonitor',
    'get_lazy_loader',
    'get_profiler',
    'get_memory_monitor',
    'register_heavy_dependency'
]
