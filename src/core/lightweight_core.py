#!/usr/bin/env python3
"""
Lightweight Core - Minimal Memory Footprint for Embedded Systems

Ultra-minimal core with lazy loading, minimal imports, and fast startup.
Designed for embedded systems with limited resources.

Author: URC 2026 Lightweight Core Team
"""

import time
from typing import Dict, List, Any, Optional, Callable
from dataclasses import dataclass

# Import unified configuration
from src.infrastructure.config import RoverConfig as SystemConfig, PerformanceConfig


class LightweightCore:
    """
    Minimal core with < 20MB memory footprint and < 1 second startup.

    Features:
    - Lazy loading of all heavy components
    - Minimal initial imports
    - Memory monitoring and cleanup
    - Fast startup time
    """

    def __init__(self, config: Optional[CoreConfig] = None):
        self.config = config or CoreConfig()
        self._components: Dict[str, Any] = {}
        self._loaders: Dict[str, Callable] = {}
        self._start_time = time.time()
        self._memory_baseline = 0

        # Register minimal loaders
        self._register_loaders()

        # Set memory baseline immediately
        self._set_memory_baseline()

    def _register_loaders(self):
        """Register lazy loaders for heavy components."""

        def load_state_machine():
            from src.core.simplified_state_manager import get_state_manager

            state_mgr = get_state_manager()
            return state_mgr.create_state_machine("lightweight_core", "idle")

        def load_behavior_tree():
            from src.core.behavior_tree import URCBehaviorTree

            return URCBehaviorTree()

        def load_safety_system():
            from src.core.utilities import get_safety_manager

            return URCSafetyManager()

        def load_database():
            from src.core.data_manager import get_data_manager

            return get_data_manager()

        def load_bridge():
            from infrastructure.bridges.simple_bridge import get_simple_bridge

            return get_simple_bridge()

        def load_api():
            from src.core.simple_api import get_simple_api

            return get_simple_api()

        def load_network_resilience():
            from src.core.network_resilience import get_network_resilience_manager

            return get_network_resilience_manager()

        # Register loaders
        self._loaders = {
            "state_machine": load_state_machine,
            "behavior_tree": load_behavior_tree,
            "safety_system": load_safety_system,
            "database": load_database,
            "bridge": load_bridge,
            "api": load_api,
            "network": load_network_resilience,
        }

    def _set_memory_baseline(self):
        """Set baseline memory usage."""
        import psutil

        process = psutil.Process()
        self._memory_baseline = process.memory_info().rss / 1024 / 1024  # MB

    def get_component(self, name: str) -> Any:
        """Get component with lazy loading."""
        if name not in self._components:
            if name in self._loaders:
                start_time = time.time()
                self._components[name] = self._loaders[name]()
                load_time = (time.time() - start_time) * 1000  # ms

                if load_time > self.config.lazy_load_threshold_ms:
                    print(f"⚠️  Slow component load: {name} ({load_time:.1f}ms)")
            else:
                raise ValueError(f"Unknown component: {name}")

        return self._components[name]

    def get_memory_usage(self) -> Dict[str, float]:
        """Get current memory usage."""
        import psutil

        process = psutil.Process()
        current_mb = process.memory_info().rss / 1024 / 1024
        delta_mb = current_mb - self._memory_baseline

        return {
            "current_mb": current_mb,
            "baseline_mb": self._memory_baseline,
            "delta_mb": delta_mb,
            "limit_mb": self.config.max_memory_mb,
        }

    def check_memory_pressure(self) -> str:
        """Check memory pressure level."""
        mem = self.get_memory_usage()

        if mem["current_mb"] > self.config.max_memory_mb:
            return "CRITICAL"
        elif mem["delta_mb"] > 30:  # 30MB increase
            return "HIGH"
        elif mem["delta_mb"] > 10:  # 10MB increase
            return "MODERATE"
        else:
            return "LOW"

    def cleanup_unused_components(self):
        """Cleanup unused components to free memory."""
        # Remove components that haven't been accessed recently
        current_time = time.time()
        to_remove = []

        for name, component in self._components.items():
            # Simple heuristic: remove if not accessed in last 5 minutes
            if hasattr(component, "_last_access"):
                if current_time - component._last_access > 300:  # 5 minutes
                    to_remove.append(name)

        for name in to_remove:
            del self._components[name]
            print(f"🧹 Cleaned up unused component: {name}")

    def get_status(self) -> Dict[str, Any]:
        """Get lightweight core status."""
        mem = self.get_memory_usage()

        return {
            "uptime_seconds": time.time() - self._start_time,
            "memory_usage": mem,
            "memory_pressure": self.check_memory_pressure(),
            "loaded_components": list(self._components.keys()),
            "available_components": list(self._loaders.keys()),
            "profiling_enabled": self.config.enable_profiling,
        }

    # Convenience methods for common operations
    def get_state_machine(self):
        """Get state machine (lazy loaded)."""
        return self.get_component("state_machine")

    def get_behavior_tree(self):
        """Get behavior tree (lazy loaded)."""
        return self.get_component("behavior_tree")

    def get_safety_system(self):
        """Get safety system (lazy loaded)."""
        return self.get_component("safety_system")

    def get_database(self):
        """Get database (lazy loaded)."""
        return self.get_component("database")

    def get_bridge(self):
        """Get bridge (lazy loaded)."""
        return self.get_component("bridge")

    def get_network_manager(self):
        """Get network manager (lazy loaded)."""
        return self.get_component("network")

    def get_api(self):
        """Get API (lazy loaded)."""
        return self.get_component("api")


# Global lightweight core instance
_core_instance: Optional[LightweightCore] = None


def get_lightweight_core(config: Optional[CoreConfig] = None) -> LightweightCore:
    """Get global lightweight core instance."""
    global _core_instance
    if _core_instance is None:
        _core_instance = LightweightCore(config)
    return _core_instance


def initialize_lightweight_core(config: Optional[CoreConfig] = None) -> LightweightCore:
    """Initialize the lightweight core."""
    return get_lightweight_core(config)


# Export key components
__all__ = [
    "LightweightCore",
    "CoreConfig",
    "get_lightweight_core",
    "initialize_lightweight_core",
]
