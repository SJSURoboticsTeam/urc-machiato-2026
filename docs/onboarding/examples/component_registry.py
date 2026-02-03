"""
Component Registry Example

Demonstrates registering components with the enterprise-grade component
management system. Use with the autonomy core and infrastructure config.

See: docs/onboarding/PILLAR_2_COGNITION.md, src/core/component_registry.py
"""

from typing import Any, Dict

from src.core.component_registry import (
    ComponentPriority,
    component,
    get_component_registry,
)


@component(
    name="navigation_component",
    priority=ComponentPriority.HIGH,
    version="1.0.0",
    description="Navigation subsystem with health checks",
    health_check_interval=10.0,
    health_check_enabled=True,
)
class NavigationComponent:
    """Example navigation component with health monitoring."""

    def __init__(self) -> None:
        self.status = "initializing"

    def start(self) -> None:
        """Start the component."""
        self.status = "running"

    def stop(self) -> None:
        """Stop the component."""
        self.status = "stopped"

    def health_check(self) -> Dict[str, Any]:
        """Health check for the component."""
        return {
            "healthy": self.status == "running",
            "status": self.status,
        }


# Usage: get the registry and initialize/retrieve components
# registry = get_component_registry()
# registry.initialize_all_components()
# nav = registry.get_component("navigation_component")
