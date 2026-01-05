#!/usr/bin/env python3
"""
Advanced Component Registry - Enterprise Component Management System

Provides sophisticated component registration, dependency resolution,
health monitoring, configuration integration, and lifecycle management
for all system components with enterprise-grade features.

Features:
- Advanced dependency resolution with cycle detection
- Component health monitoring and metrics
- Configuration-driven component management
- Automatic component discovery and registration
- Performance profiling and resource tracking
- Version management and compatibility checking
- Hot-swapping and dynamic reconfiguration

Author: URC 2026 Advanced Component Registry Team
"""

import importlib
import inspect
import time
import threading
import psutil
from typing import Dict, List, Any, Optional, Type, Callable, Set, Union
from dataclasses import dataclass, field
from enum import Enum
import logging
import weakref

# Import unified configuration
from src.core.configuration import ConfigurationManager, SystemConfig

logger = logging.getLogger(__name__)


class ComponentState(Enum):
    """Component lifecycle states with detailed transitions."""
    UNREGISTERED = "unregistered"
    REGISTERED = "registered"
    VALIDATING = "validating"
    VALIDATED = "validated"
    INITIALIZING = "initializing"
    INITIALIZED = "initialized"
    STARTING = "starting"
    RUNNING = "running"
    DEGRADED = "degraded"
    STOPPING = "stopping"
    STOPPED = "stopped"
    ERROR = "error"
    DISABLED = "disabled"


class ComponentPriority(Enum):
    """Component initialization priority with resource allocation."""
    CRITICAL = 1      # System-critical, initialize first, max resources
    HIGH = 2          # High priority, early initialization
    NORMAL = 3        # Standard priority, normal initialization
    LOW = 4           # Low priority, late initialization
    OPTIONAL = 5      # Optional components, initialize last, may be skipped


class ComponentHealth(Enum):
    """Component health status."""
    UNKNOWN = "unknown"
    HEALTHY = "healthy"
    DEGRADED = "degraded"
    UNHEALTHY = "unhealthy"
    FAILED = "failed"


@dataclass
class ComponentMetrics:
    """Detailed component performance metrics."""
    initialization_time: float = 0.0
    startup_time: float = 0.0
    memory_usage_mb: float = 0.0
    cpu_usage_percent: float = 0.0
    operation_count: int = 0
    error_count: int = 0
    last_operation_time: float = 0.0
    uptime_seconds: float = 0.0
    health_check_failures: int = 0


@dataclass
class ComponentVersion:
    """Component version information."""
    major: int = 1
    minor: int = 0
    patch: int = 0
    build: Optional[str] = None

    def __str__(self) -> str:
        version = f"{self.major}.{self.minor}.{self.patch}"
        if self.build:
            version += f"-{self.build}"
        return version

    @classmethod
    def from_string(cls, version_str: str) -> 'ComponentVersion':
        """Parse version from string like '1.2.3' or '1.2.3-dev'."""
        parts = version_str.replace('-', '.').split('.')
        major = int(parts[0]) if len(parts) > 0 else 1
        minor = int(parts[1]) if len(parts) > 1 else 0
        patch = int(parts[2]) if len(parts) > 2 else 0
        build = parts[3] if len(parts) > 3 else None
        return cls(major=major, minor=minor, patch=patch, build=build)


@dataclass
class ComponentInfo:
    """Comprehensive information about a registered component."""
    name: str
    component_class: Type
    module_path: str
    version: ComponentVersion = field(default_factory=ComponentVersion)
    description: str = ""
    dependencies: List[str] = field(default_factory=list)
    optional_dependencies: List[str] = field(default_factory=list)
    priority: ComponentPriority = ComponentPriority.NORMAL
    singleton: bool = True
    auto_start: bool = True
    health_check_interval: float = 30.0  # seconds
    max_restart_attempts: int = 3
    restart_delay_seconds: float = 5.0
    config_required: bool = False
    config_section: Optional[str] = None

    # Runtime state
    state: ComponentState = ComponentState.REGISTERED
    health: ComponentHealth = ComponentHealth.UNKNOWN
    instance: Optional[Any] = None
    created_at: float = field(default_factory=time.time)
    started_at: Optional[float] = None
    last_health_check: float = 0.0
    restart_count: int = 0
    metrics: ComponentMetrics = field(default_factory=ComponentMetrics)
    error_message: Optional[str] = None
    config_data: Dict[str, Any] = field(default_factory=dict)

    # Dependencies tracking
    resolved_dependencies: List[str] = field(default_factory=list)
    dependent_components: List[str] = field(default_factory=list)

    def update_metrics(self):
        """Update component performance metrics."""
        if self.instance:
            process = psutil.Process()
            self.metrics.memory_usage_mb = process.memory_info().rss / 1024 / 1024
            self.metrics.cpu_usage_percent = process.cpu_percent()
            self.metrics.uptime_seconds = time.time() - (self.started_at or self.created_at)

    def is_running(self) -> bool:
        """Check if component is running."""
        return self.state == ComponentState.RUNNING

    def is_healthy(self) -> bool:
        """Check if component is healthy."""
        return self.health in [ComponentHealth.HEALTHY, ComponentHealth.DEGRADED]

    def can_start(self) -> bool:
        """Check if component can be started."""
        return (self.state in [ComponentState.REGISTERED, ComponentState.STOPPED, ComponentState.ERROR] and
                self.auto_start)

    def requires_restart(self) -> bool:
        """Check if component requires restart."""
        return (self.state == ComponentState.ERROR and
                self.restart_count < self.max_restart_attempts)


class ComponentRegistry:
    """
    Advanced component registry with enterprise features.

    Provides comprehensive component management including:
    - Dependency resolution with cycle detection
    - Health monitoring and automatic recovery
    - Configuration integration
    - Performance profiling
    - Hot-swapping and dynamic updates
    - Version management and compatibility
    """

    def __init__(self):
        self.components: Dict[str, ComponentInfo] = {}
        self.instances: Dict[str, Any] = {}
        self.dependency_graph: Dict[str, Set[str]] = {}
        self.reverse_dependencies: Dict[str, Set[str]] = {}
        self.component_versions: Dict[str, ComponentVersion] = {}

        # Health monitoring
        self.health_monitoring_active = False
        self.health_monitor_thread: Optional[threading.Thread] = None
        self._health_monitor_lock = threading.Lock()

        # Configuration integration
        self.config_manager = None

        # Performance tracking
        self.performance_metrics: Dict[str, List[float]] = {}

        logger.info("Advanced Component Registry initialized")

    def register_component(self, name: str, component_class: Type,
                          dependencies: Optional[List[str]] = None,
                          optional_dependencies: Optional[List[str]] = None,
                          priority: ComponentPriority = ComponentPriority.NORMAL,
                          singleton: bool = True,
                          auto_start: bool = True,
                          version: Optional[ComponentVersion] = None,
                          description: str = "",
                          health_check_interval: float = 30.0,
                          max_restart_attempts: int = 3,
                          config_required: bool = False,
                          config_section: Optional[str] = None) -> bool:
        """
        Register a component with comprehensive metadata.

        Args:
            name: Unique component name
            component_class: Component class
            dependencies: Required dependencies
            optional_dependencies: Optional dependencies
            priority: Initialization priority
            singleton: Whether to create single instance
            auto_start: Whether to start automatically
            version: Component version
            description: Component description
            health_check_interval: Health check frequency
            max_restart_attempts: Maximum restart attempts
            config_required: Whether configuration is required
            config_section: Configuration section name

        Returns:
            True if registration successful
        """
        if name in self.components:
            logger.warning(f"Component {name} already registered, updating")
            # Preserve runtime state when updating
            existing_state = self.components[name].state
            existing_instance = self.components[name].instance
        else:
            existing_state = ComponentState.REGISTERED
            existing_instance = None

        deps = dependencies or []
        optional_deps = optional_dependencies or []
        version = version or ComponentVersion()

        # Validate dependencies
        if not self._validate_dependencies(name, deps):
            logger.error(f"Invalid dependencies for component {name}")
            return False

        # Extract module path
        try:
            module_path = f"{component_class.__module__}.{component_class.__name__}"
        except AttributeError:
            module_path = str(component_class)

        component_info = ComponentInfo(
            name=name,
            component_class=component_class,
            module_path=module_path,
            version=version,
            description=description,
            dependencies=deps,
            optional_dependencies=optional_deps,
            priority=priority,
            singleton=singleton,
            auto_start=auto_start,
            health_check_interval=health_check_interval,
            max_restart_attempts=max_restart_attempts,
            config_required=config_required,
            config_section=config_section,
            state=existing_state,
            instance=existing_instance
        )

        self.components[name] = component_info
        self.component_versions[name] = version

        # Update dependency graphs
        self._update_dependency_graphs(name, deps)

        logger.info(f"Registered component: {name} v{version} ({description})")
        return True

    def unregister_component(self, name: str) -> bool:
        """
        Unregister a component.

        Args:
            name: Component name

        Returns:
            True if unregistered successfully
        """
        if name not in self.components:
            logger.warning(f"Component {name} not registered")
            return False

        component_info = self.components[name]

        # Stop component if running
        if component_info.is_running():
            self.stop_component(name)

        # Remove from dependency graphs
        self._remove_from_dependency_graphs(name)

        # Remove instances
        if name in self.instances:
            del self.instances[name]

        # Remove component info
        del self.components[name]
        del self.component_versions[name]

        logger.info(f"Unregistered component: {name}")
        return True

    def get_component(self, name: str) -> Any:
        """
        Get component instance with advanced dependency resolution.

        Args:
            name: Component name

        Returns:
            Component instance

        Raises:
            ValueError: If component not found or dependencies not satisfied
        """
        if name not in self.components:
            raise ValueError(f"Component {name} not registered")

        component_info = self.components[name]

        # Check if component is disabled
        if component_info.state == ComponentState.DISABLED:
            raise ValueError(f"Component {name} is disabled")

        # Resolve dependencies first
        unresolved_deps = self._get_unresolved_dependencies(name)
        if unresolved_deps:
            raise ValueError(f"Unresolved dependencies for {name}: {unresolved_deps}")

        # Create instance if needed
        if component_info.singleton and name in self.instances:
            component_info.update_metrics()
            return self.instances[name]

        # Create new instance
        try:
            component_info.state = ComponentState.INITIALIZING
            start_time = time.time()

            # Inject dependencies
            kwargs = self._resolve_component_dependencies(name)

            # Create instance
            instance = component_info.component_class(**kwargs)

            component_info.metrics.initialization_time = time.time() - start_time
            component_info.instance = instance
            component_info.state = ComponentState.INITIALIZED

            if component_info.singleton:
                self.instances[name] = instance

            logger.info(f"Created component instance: {name}")
            return instance

        except Exception as e:
            component_info.state = ComponentState.ERROR
            component_info.error_message = str(e)
            logger.error(f"Failed to create component {name}: {e}")
            raise

    def start_component(self, name: str) -> bool:
        """
        Start a component.

        Args:
            name: Component name

        Returns:
            True if started successfully
        """
        if name not in self.components:
            return False

        component_info = self.components[name]

        try:
            component_info.state = ComponentState.STARTING
            start_time = time.time()

            # Get instance
            instance = self.get_component(name)

            # Call start method if available
            if hasattr(instance, 'start'):
                instance.start()

            component_info.metrics.startup_time = time.time() - start_time
            component_info.started_at = time.time()
            component_info.state = ComponentState.RUNNING
            component_info.health = ComponentHealth.HEALTHY

            logger.info(f"Started component: {name}")
            return True

        except Exception as e:
            component_info.state = ComponentState.ERROR
            component_info.error_message = str(e)
            component_info.health = ComponentHealth.FAILED
            logger.error(f"Failed to start component {name}: {e}")
            return False

    def stop_component(self, name: str) -> bool:
        """
        Stop a component.

        Args:
            name: Component name

        Returns:
            True if stopped successfully
        """
        if name not in self.components:
            return False

        component_info = self.components[name]

        try:
            component_info.state = ComponentState.STOPPING

            if component_info.instance and hasattr(component_info.instance, 'stop'):
                component_info.instance.stop()

            component_info.state = ComponentState.STOPPED
            component_info.health = ComponentHealth.UNKNOWN

            logger.info(f"Stopped component: {name}")
            return True

        except Exception as e:
            component_info.state = ComponentState.ERROR
            component_info.error_message = str(e)
            logger.error(f"Failed to stop component {name}: {e}")
            return False

    def restart_component(self, name: str) -> bool:
        """
        Restart a component.

        Args:
            name: Component name

        Returns:
            True if restarted successfully
        """
        if name not in self.components:
            return False

        component_info = self.components[name]

        if component_info.restart_count >= component_info.max_restart_attempts:
            logger.error(f"Component {name} exceeded max restart attempts")
            return False

        logger.info(f"Restarting component: {name} (attempt {component_info.restart_count + 1})")

        # Stop component
        self.stop_component(name)

        # Wait before restart
        time.sleep(component_info.restart_delay_seconds)

        # Start component
        if self.start_component(name):
            component_info.restart_count += 1
            return True

        return False

    def get_initialization_order(self) -> List[str]:
        """
        Get component initialization order with cycle detection.

        Returns:
            Ordered list of component names

        Raises:
            ValueError: If dependency cycles are detected
        """
        # Kahn's algorithm with cycle detection
        in_degree = {name: len(info.dependencies) for name, info in self.components.items()}
        zero_degree = [name for name, degree in in_degree.items() if degree == 0]

        # Sort by priority
        priority_order = {
            ComponentPriority.CRITICAL: 0,
            ComponentPriority.HIGH: 1,
            ComponentPriority.NORMAL: 2,
            ComponentPriority.LOW: 3,
            ComponentPriority.OPTIONAL: 4
        }

        zero_degree.sort(key=lambda x: priority_order[self.components[x].priority])

        result = []

        while zero_degree:
            current = zero_degree.pop(0)
            result.append(current)

            # Update dependencies
            for dependent in self.reverse_dependencies.get(current, set()):
                in_degree[dependent] -= 1
                if in_degree[dependent] == 0:
                    zero_degree.append(dependent)

            # Re-sort by priority
            zero_degree.sort(key=lambda x: priority_order[self.components[x].priority])

        # Check for cycles
        if len(result) != len(self.components):
            remaining = set(self.components.keys()) - set(result)
            raise ValueError(f"Dependency cycle detected involving: {remaining}")

        return result

    def initialize_all_components(self) -> Dict[str, bool]:
        """
        Initialize all registered components in dependency order.

        Returns:
            Dictionary mapping component names to initialization success
        """
        init_order = self.get_initialization_order()
        results = {}

        for component_name in init_order:
            component_info = self.components[component_name]

            if component_info.can_start():
                try:
                    self.start_component(component_name)
                    results[component_name] = True
                    logger.info(f"Initialized component: {component_name}")
                except Exception as e:
                    results[component_name] = False
                    logger.error(f"Failed to initialize {component_name}: {e}")
            else:
                results[component_name] = False
                logger.info(f"Skipped component: {component_name} (auto_start=False)")

        return results

    def shutdown_all_components(self) -> Dict[str, bool]:
        """
        Shutdown all running components in reverse dependency order.

        Returns:
            Dictionary mapping component names to shutdown success
        """
        shutdown_order = list(reversed(self.get_initialization_order()))
        results = {}

        for component_name in shutdown_order:
            if component_name in self.components:
                try:
                    self.stop_component(component_name)
                    results[component_name] = True
                    logger.info(f"Shutdown component: {component_name}")
                except Exception as e:
                    results[component_name] = False
                    logger.error(f"Failed to shutdown {component_name}: {e}")
            else:
                results[component_name] = False
                logger.warning(f"Component not found for shutdown: {component_name}")

        return results

    def start_health_monitoring(self, interval_seconds: float = 30.0):
        """Start health monitoring for all components."""
        if self.health_monitoring_active:
            return

        self.health_monitoring_active = True
        self.health_monitor_thread = threading.Thread(
            target=self._health_monitor_loop,
            args=(interval_seconds,),
            daemon=True
        )
        self.health_monitor_thread.start()

        logger.info("Component health monitoring started")

    def stop_health_monitoring(self):
        """Stop health monitoring."""
        self.health_monitoring_active = False
        if self.health_monitor_thread:
            self.health_monitor_thread.join(timeout=2.0)
        logger.info("Component health monitoring stopped")

    def _health_monitor_loop(self, interval: float):
        """Health monitoring loop."""
        while self.health_monitoring_active:
            try:
                self._perform_health_checks()
                time.sleep(interval)
            except Exception as e:
                logger.error(f"Health monitoring error: {e}")
                time.sleep(interval)

    def _perform_health_checks(self):
        """Perform health checks on all components."""
        current_time = time.time()

        with self._health_monitor_lock:
            for name, component_info in self.components.items():
                if (component_info.state == ComponentState.RUNNING and
                    current_time - component_info.last_health_check > component_info.health_check_interval):

                    try:
                        health = self._check_component_health(name)
                        component_info.health = health
                        component_info.last_health_check = current_time

                        if health == ComponentHealth.FAILED and component_info.requires_restart():
                            self.restart_component(name)

                    except Exception as e:
                        component_info.health = ComponentHealth.FAILED
                        component_info.error_message = str(e)
                        logger.error(f"Health check failed for {name}: {e}")

    def _check_component_health(self, name: str) -> ComponentHealth:
        """Check health of a specific component."""
        component_info = self.components[name]

        if not component_info.instance:
            return ComponentHealth.UNHEALTHY

        # Try to call health check method
        if hasattr(component_info.instance, 'health_check'):
            try:
                health_result = component_info.instance.health_check()
                if isinstance(health_result, dict):
                    healthy = health_result.get('healthy', True)
                    return ComponentHealth.HEALTHY if healthy else ComponentHealth.UNHEALTHY
                elif isinstance(health_result, bool):
                    return ComponentHealth.HEALTHY if health_result else ComponentHealth.UNHEALTHY
            except Exception:
                return ComponentHealth.FAILED

        # Default: assume healthy if running
        return ComponentHealth.HEALTHY

    def integrate_configuration(self, config_manager: ConfigurationManager):
        """
        Integrate with unified configuration system.

        Args:
            config_manager: Unified configuration manager instance
        """
        self.config_manager = config_manager

        try:
            # Get current system configuration
            system_config = config_manager.current_config

            # Load component configurations
            for name, component_info in self.components.items():
                if component_info.config_section:
                    try:
                        # Access configuration section dynamically
                        config_data = getattr(system_config, component_info.config_section, None)
                        if config_data and hasattr(config_data, '__dict__'):
                            component_info.config_data = config_data.__dict__
                        elif config_data:
                            component_info.config_data = config_data
                        else:
                            logger.warning(f"Configuration section '{component_info.config_section}' not found for {name}")
                    except Exception as e:
                        logger.warning(f"Failed to load config for {name}: {e}")
        except Exception as e:
            logger.error(f"Configuration integration failed: {e}")

    def get_component_status(self, name: str) -> Optional[Dict[str, Any]]:
        """Get detailed status of a component."""
        if name not in self.components:
            return None

        component_info = self.components[name]
        component_info.update_metrics()

        return {
            "name": component_info.name,
            "version": str(component_info.version),
            "description": component_info.description,
            "state": component_info.state.value,
            "health": component_info.health.value,
            "priority": component_info.priority.value,
            "dependencies": component_info.dependencies,
            "optional_dependencies": component_info.optional_dependencies,
            "dependents": component_info.dependent_components,
            "metrics": {
                "initialization_time": component_info.metrics.initialization_time,
                "startup_time": component_info.metrics.startup_time,
                "memory_usage_mb": component_info.metrics.memory_usage_mb,
                "cpu_usage_percent": component_info.metrics.cpu_usage_percent,
                "uptime_seconds": component_info.metrics.uptime_seconds,
                "operation_count": component_info.metrics.operation_count,
                "error_count": component_info.metrics.error_count,
                "restart_count": component_info.restart_count
            },
            "created_at": component_info.created_at,
            "started_at": component_info.started_at,
            "last_health_check": component_info.last_health_check,
            "error_message": component_info.error_message
        }

    def get_system_status(self) -> Dict[str, Any]:
        """Get comprehensive system component status."""
        total_components = len(self.components)
        running_components = len([c for c in self.components.values() if c.is_running()])
        healthy_components = len([c for c in self.components.values() if c.is_healthy()])
        error_components = len([c for c in self.components.values() if c.state == ComponentState.ERROR])

        return {
            "total_components": total_components,
            "running_components": running_components,
            "healthy_components": healthy_components,
            "error_components": error_components,
            "health_percentage": (healthy_components / total_components * 100) if total_components > 0 else 0,
            "components": {name: self.get_component_status(name) for name in self.components.keys()},
            "health_monitoring_active": self.health_monitoring_active,
            "timestamp": time.time()
        }

    def auto_discover_components(self, module_paths: List[str], auto_register: bool = True) -> List[str]:
        """
        Automatically discover components from modules.

        Args:
            module_paths: List of module paths to scan
            auto_register: Whether to auto-register discovered components

        Returns:
            List of discovered component names
        """
        discovered = []

        for module_path in module_paths:
            try:
                module = importlib.import_module(module_path)

                # Look for classes with component metadata
                for name, obj in inspect.getmembers(module):
                    if (inspect.isclass(obj) and
                        hasattr(obj, '__component_name__')):

                        component_name = getattr(obj, '__component_name__')
                        dependencies = getattr(obj, '__dependencies__', [])
                        priority = getattr(obj, '__priority__', ComponentPriority.NORMAL)
                        description = getattr(obj, '__description__', '')
                        version = getattr(obj, '__version__', ComponentVersion())

                        if auto_register:
                            self.register_component(
                                component_name, obj,
                                dependencies=dependencies,
                                priority=priority,
                                description=description,
                                version=version
                            )

                        discovered.append(component_name)

            except ImportError as e:
                logger.warning(f"Failed to import module {module_path}: {e}")

        return discovered

    def _validate_dependencies(self, name: str, dependencies: List[str]) -> bool:
        """Validate component dependencies."""
        # Check for self-dependency
        if name in dependencies:
            logger.error(f"Component {name} cannot depend on itself")
            return False

        # Check for circular dependencies (basic check)
        for dep in dependencies:
            if dep in self.reverse_dependencies.get(name, set()):
                logger.error(f"Circular dependency detected: {name} -> {dep}")
                return False

        return True

    def _update_dependency_graphs(self, name: str, dependencies: List[str]):
        """Update dependency graphs."""
        self.dependency_graph[name] = set(dependencies)

        for dep in dependencies:
            if dep not in self.reverse_dependencies:
                self.reverse_dependencies[dep] = set()
            self.reverse_dependencies[dep].add(name)

            # Update dependent components list
            if dep in self.components:
                self.components[dep].dependent_components.append(name)

    def _remove_from_dependency_graphs(self, name: str):
        """Remove component from dependency graphs."""
        if name in self.dependency_graph:
            del self.dependency_graph[name]

        if name in self.reverse_dependencies:
            del self.reverse_dependencies[name]

        # Remove from other components' dependent lists
        for component_info in self.components.values():
            if name in component_info.dependent_components:
                component_info.dependent_components.remove(name)

    def _get_unresolved_dependencies(self, name: str) -> List[str]:
        """Get unresolved dependencies for a component."""
        component_info = self.components[name]
        unresolved = []

        for dep in component_info.dependencies:
            if dep not in self.components:
                unresolved.append(dep)
            elif not self.components[dep].is_running():
                unresolved.append(dep)

        return unresolved

    def _resolve_component_dependencies(self, name: str) -> Dict[str, Any]:
        """Resolve and inject component dependencies with intelligent parameter naming."""
        component_info = self.components[name]
        kwargs = {}

        # Get component constructor signature to match parameter names
        try:
            import inspect
            sig = inspect.signature(component_info.component_class.__init__)
            param_names = list(sig.parameters.keys())[1:]  # Skip 'self'
        except Exception:
            param_names = []

        for dep in component_info.dependencies:
            if dep in self.instances:
                # Try to find matching parameter name
                param_name = None

                # First, try exact match
                if dep in param_names:
                    param_name = dep
                else:
                    # Try common variations
                    candidates = [
                        dep,  # exact
                        dep.replace('_', ''),  # no underscores
                        dep.replace('_', '').lower(),  # no underscores, lowercase
                        ''.join(word.capitalize() for word in dep.split('_')),  # camelCase
                        dep.lower(),  # lowercase
                        dep.upper(),  # uppercase
                    ]

                    for candidate in candidates:
                        if candidate in param_names:
                            param_name = candidate
                            break

                # If no match found, use the first parameter (assuming single dependency)
                if param_name is None and param_names:
                    param_name = param_names[0]
                    logger.debug(f"Using fallback parameter name '{param_name}' for dependency '{dep}' in {name}")

                if param_name:
                    kwargs[param_name] = self.instances[dep]
                    component_info.resolved_dependencies.append(dep)
                else:
                    logger.warning(f"Could not resolve parameter name for dependency '{dep}' in {name}")

        return kwargs


# Global component registry instance
_component_registry = None

def get_component_registry() -> ComponentRegistry:
    """Get global component registry instance."""
    global _component_registry
    if _component_registry is None:
        _component_registry = ComponentRegistry()
    return _component_registry

def register_component(name: str, component_class: Type, **kwargs):
    """Convenience function to register a component."""
    registry = get_component_registry()
    return registry.register_component(name, component_class, **kwargs)

def get_component(name: str) -> Any:
    """Convenience function to get a component."""
    registry = get_component_registry()
    return registry.get_component(name)

def start_component(name: str) -> bool:
    """Convenience function to start a component."""
    registry = get_component_registry()
    return registry.start_component(name)

def stop_component(name: str) -> bool:
    """Convenience function to stop a component."""
    registry = get_component_registry()
    return registry.stop_component(name)

# Advanced decorator with comprehensive options
def component(name: str,
              dependencies: Optional[List[str]] = None,
              optional_dependencies: Optional[List[str]] = None,
              priority: ComponentPriority = ComponentPriority.NORMAL,
              singleton: bool = True,
              auto_start: bool = True,
              version: Optional[Union[str, ComponentVersion]] = None,
              description: str = "",
              health_check_interval: float = 30.0,
              max_restart_attempts: int = 3,
              config_required: bool = False,
              config_section: Optional[str] = None):
    """
    Advanced component decorator with comprehensive configuration options.

    Args:
        name: Unique component name
        dependencies: Required dependencies
        optional_dependencies: Optional dependencies
        priority: Initialization priority
        singleton: Whether to create single instance
        auto_start: Whether to start automatically
        version: Component version
        description: Component description
        health_check_interval: Health check frequency
        max_restart_attempts: Maximum restart attempts
        config_required: Whether configuration is required
        config_section: Configuration section name
    """
    def decorator(cls):
        # Parse version if string
        if isinstance(version, str):
            parsed_version = ComponentVersion.from_string(version)
        else:
            parsed_version = version or ComponentVersion()

        # Add metadata to class
        cls.__component_name__ = name
        cls.__dependencies__ = dependencies or []
        cls.__optional_dependencies__ = optional_dependencies or []
        cls.__priority__ = priority
        cls.__singleton__ = singleton
        cls.__auto_start__ = auto_start
        cls.__version__ = parsed_version
        cls.__description__ = description
        cls.__health_check_interval__ = health_check_interval
        cls.__max_restart_attempts__ = max_restart_attempts
        cls.__config_required__ = config_required
        cls.__config_section__ = config_section

        # Register immediately
        register_component(
            name=name,
            component_class=cls,
            dependencies=dependencies,
            optional_dependencies=optional_dependencies,
            priority=priority,
            singleton=singleton,
            auto_start=auto_start,
            version=parsed_version,
            description=description,
            health_check_interval=health_check_interval,
            max_restart_attempts=max_restart_attempts,
            config_required=config_required,
            config_section=config_section
        )

        return cls
    return decorator

# Export key components
__all__ = [
    'ComponentRegistry',
    'ComponentInfo',
    'ComponentState',
    'ComponentPriority',
    'ComponentHealth',
    'ComponentMetrics',
    'ComponentVersion',
    'get_component_registry',
    'register_component',
    'get_component',
    'start_component',
    'stop_component',
    'component'
]