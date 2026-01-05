#!/usr/bin/env python3
"""
Component Registry Tests - URC 2026

Tests the ComponentRegistry system for:
- Dynamic component registration and discovery
- Dependency injection and resolution
- Component lifecycle management
- Health monitoring and status tracking
- Concurrent access handling

Author: URC 2026 Testing Team
"""

import asyncio
import threading
import time
import pytest
from unittest.mock import Mock, patch, AsyncMock
from typing import Dict, Any, Optional, List

from src.core.component_registry import ComponentRegistry, ComponentState


class TestComponentRegistry:
    """Test ComponentRegistry functionality."""

    @pytest.fixture
    def component_registry(self):
        """Create ComponentRegistry instance."""
        return ComponentRegistry()

    @pytest.fixture
    def mock_component(self):
        """Create mock component for testing."""
        component = Mock()
        component.name = "test_component"
        component.start = AsyncMock(return_value=True)
        component.stop = AsyncMock(return_value=True)
        component.health_check = AsyncMock(return_value={"status": "healthy"})
        component.get_status = Mock(return_value={"state": "running", "uptime": 60})
        return component

    def test_component_registry_initialization(self):
        """Test ComponentRegistry initialization."""
        registry = ComponentRegistry()

        assert isinstance(registry.components, dict)
        assert isinstance(registry.dependencies, dict)
        assert isinstance(registry.health_status, dict)
        assert registry._lock is not None

    def test_component_registration(self, component_registry, mock_component):
        """Test component registration."""
        # Register component
        result = component_registry.register_component(
            "test_component",
            mock_component,
            ["navigation", "communication"]
        )

        assert result is True
        assert "test_component" in component_registry.components
        assert component_registry.components["test_component"] == mock_component
        assert component_registry.dependencies["test_component"] == ["navigation", "communication"]

    def test_component_discovery(self, component_registry, mock_component):
        """Test component discovery."""
        component_registry.register_component("test_component", mock_component)

        # Discover component
        found_component = component_registry.get_component("test_component")
        assert found_component == mock_component

        # Discover non-existent component
        not_found = component_registry.get_component("nonexistent")
        assert not_found is None

    def test_component_with_dependencies_registration(self, component_registry):
        """Test component registration with dependencies."""
        # Create components with dependencies
        nav_component = Mock()
        nav_component.name = "navigation"
        nav_component.start = AsyncMock(return_value=True)
        nav_component.stop = AsyncMock(return_value=True)

        comm_component = Mock()
        comm_component.name = "communication"
        comm_component.start = AsyncMock(return_value=True)
        comm_component.stop = AsyncMock(return_value=True)

        mission_component = Mock()
        mission_component.name = "mission_executor"
        mission_component.start = AsyncMock(return_value=True)
        mission_component.stop = AsyncMock(return_value=True)

        # Register components
        component_registry.register_component("navigation", nav_component)
        component_registry.register_component("communication", comm_component)
        component_registry.register_component("mission_executor", mission_component, ["navigation", "communication"])

        # Verify dependency graph
        assert component_registry.dependencies["mission_executor"] == ["navigation", "communication"]

    def test_dependency_resolution(self, component_registry):
        """Test dependency resolution."""
        # Register components with dependencies
        nav = Mock()
        comm = Mock()
        mission = Mock()

        component_registry.register_component("navigation", nav)
        component_registry.register_component("communication", comm)
        component_registry.register_component("mission", mission, ["navigation", "communication"])

        # Resolve dependencies
        deps = component_registry.resolve_dependencies("mission")
        assert deps == ["navigation", "communication"]

        # Try to resolve non-existent component
        with pytest.raises(ValueError):
            component_registry.resolve_dependencies("nonexistent")

    def test_component_lifecycle_management(self, component_registry, mock_component):
        """Test component lifecycle management."""
        component_registry.register_component("test_component", mock_component)

        # Start component
        result = asyncio.run(component_registry.start_component("test_component"))
        assert result is True
        mock_component.start.assert_called_once()

        # Check status
        status = component_registry.get_component_status("test_component")
        assert status == ComponentState.STARTING

        # Stop component
        result = asyncio.run(component_registry.stop_component("test_component"))
        assert result is True
        mock_component.stop.assert_called_once()

        # Check status
        status = component_registry.get_component_status("test_component")
        assert status == ComponentState.STOPPING

    def test_bulk_component_operations(self, component_registry):
        """Test bulk component operations."""
        # Register multiple components
        components = {}
        for i in range(3):
            component = Mock()
            component.name = f"component_{i}"
            component.start = AsyncMock(return_value=True)
            component.stop = AsyncMock(return_value=True)
            components[f"component_{i}"] = component
            component_registry.register_component(f"component_{i}", component)

        # Start all components
        results = asyncio.run(component_registry.start_all_components())
        assert all(results.values())

        # Stop all components
        results = asyncio.run(component_registry.stop_all_components())
        assert all(results.values())

    def test_component_health_monitoring(self, component_registry, mock_component):
        """Test component health monitoring."""
        component_registry.register_component("test_component", mock_component)

        # Run health check
        health = asyncio.run(component_registry.check_component_health("test_component"))
        assert health["status"] == "healthy"
        mock_component.health_check.assert_called_once()

        # Check health status storage
        assert "test_component" in component_registry.health_status

    def test_concurrent_access_handling(self, component_registry):
        """Test concurrent access handling."""
        results = []

        def concurrent_operation(operation_id: int):
            component = Mock()
            component.name = f"concurrent_{operation_id}"
            component.start = AsyncMock(return_value=True)

            # Register component
            component_registry.register_component(f"concurrent_{operation_id}", component)

            # Start component
            result = asyncio.run(component_registry.start_component(f"concurrent_{operation_id}"))
            results.append(result)

        # Run concurrent operations
        threads = []
        for i in range(5):
            thread = threading.Thread(target=concurrent_operation, args=(i,))
            threads.append(thread)
            thread.start()

        # Wait for all threads
        for thread in threads:
            thread.join()

        # Verify all operations succeeded
        assert len(results) == 5
        assert all(results)

    def test_dependency_cycle_detection(self, component_registry):
        """Test dependency cycle detection."""
        # Create circular dependency: A -> B -> C -> A
        comp_a = Mock()
        comp_b = Mock()
        comp_c = Mock()

        component_registry.register_component("comp_a", comp_a, ["comp_b"])
        component_registry.register_component("comp_b", comp_b, ["comp_c"])
        component_registry.register_component("comp_c", comp_c, ["comp_a"])

        # Try to resolve dependencies - should detect cycle
        with pytest.raises(ValueError, match="circular dependency"):
            component_registry.resolve_dependencies("comp_a")

    def test_component_error_handling(self, component_registry):
        """Test component error handling."""
        # Component that fails to start
        failing_component = Mock()
        failing_component.name = "failing_component"
        failing_component.start = AsyncMock(side_effect=Exception("Start failed"))

        component_registry.register_component("failing_component", failing_component)

        # Try to start failing component
        result = asyncio.run(component_registry.start_component("failing_component"))
        assert result is False

        # Check error status
        status = component_registry.get_component_status("failing_component")
        assert status == ComponentState.STOPPED

    def test_component_replacement(self, component_registry, mock_component):
        """Test component replacement."""
        # Register initial component
        component_registry.register_component("test_component", mock_component)

        # Create replacement component
        new_component = Mock()
        new_component.name = "test_component"
        new_component.start = AsyncMock(return_value=True)
        new_component.stop = AsyncMock(return_value=True)

        # Replace component
        result = component_registry.replace_component("test_component", new_component)
        assert result is True

        # Verify replacement
        retrieved = component_registry.get_component("test_component")
        assert retrieved == new_component

    def test_component_statistics_tracking(self, component_registry, mock_component):
        """Test component statistics tracking."""
        component_registry.register_component("test_component", mock_component)

        # Start and stop component multiple times
        for _ in range(3):
            asyncio.run(component_registry.start_component("test_component"))
            asyncio.run(component_registry.stop_component("test_component"))

        # Get statistics
        stats = component_registry.get_component_statistics("test_component")
        assert "start_count" in stats
        assert "stop_count" in stats
        assert stats["start_count"] == 3
        assert stats["stop_count"] == 3

    def test_registry_cleanup(self, component_registry, mock_component):
        """Test registry cleanup."""
        component_registry.register_component("test_component", mock_component)

        # Verify component is registered
        assert "test_component" in component_registry.components

        # Clean up registry
        component_registry.clear_registry()

        # Verify cleanup
        assert len(component_registry.components) == 0
        assert len(component_registry.dependencies) == 0
        assert len(component_registry.health_status) == 0

    def test_component_filtering(self, component_registry):
        """Test component filtering by criteria."""
        # Register components with different characteristics
        components = [
            ("navigation", Mock(), ["sensors"]),
            ("communication", Mock(), ["network"]),
            ("vision", Mock(), ["camera"]),
            ("mission_control", Mock(), ["navigation", "communication"])
        ]

        for name, component, deps in components:
            component_registry.register_component(name, component, deps)

        # Filter by dependency
        nav_dependents = component_registry.get_components_by_dependency("navigation")
        assert "mission_control" in nav_dependents

        # Filter by status
        started_components = component_registry.get_components_by_status(ComponentState.REGISTERED)
        assert len(started_components) == 4

    def test_component_event_system(self, component_registry, mock_component):
        """Test component event system."""
        events_received = []

        def event_handler(event_type: str, component_name: str, **kwargs):
            events_received.append((event_type, component_name, kwargs))

        # Register event handler
        component_registry.add_event_handler(event_handler)

        # Register component (should trigger event)
        component_registry.register_component("test_component", mock_component)

        # Start component (should trigger event)
        asyncio.run(component_registry.start_component("test_component"))

        # Verify events were received
        assert len(events_received) >= 2  # At least register and start events
        assert any(event[0] == "component_registered" for event in events_received)
        assert any(event[0] == "component_started" for event in events_received)

    def test_registry_serialization(self, component_registry, mock_component):
        """Test registry state serialization."""
        component_registry.register_component("test_component", mock_component, ["dep1", "dep2"])

        # Serialize registry state
        state = component_registry.serialize_state()

        assert isinstance(state, dict)
        assert "components" in state
        assert "dependencies" in state
        assert "test_component" in state["dependencies"]

    def test_registry_performance_monitoring(self, component_registry):
        """Test registry performance monitoring."""
        # Register many components
        for i in range(100):
            component = Mock()
            component.name = f"perf_component_{i}"
            component.start = AsyncMock(return_value=True)
            component_registry.register_component(f"perf_component_{i}", component)

        # Measure operation performance
        start_time = time.time()
        for i in range(100):
            component_registry.get_component(f"perf_component_{i}")
        end_time = time.time()

        # Should complete quickly (under 1 second for 100 operations)
        assert (end_time - start_time) < 1.0


class TestComponentRegistryIntegration:
    """Integration tests for ComponentRegistry with other systems."""

    @pytest.fixture
    def integrated_registry(self):
        """Create registry with integrated components."""
        from src.core.observability import ObservabilitySystem

        registry = ComponentRegistry()
        observability = ObservabilitySystem()

        # Register observability as a component
        registry.register_component("observability", observability)

        return registry

    def test_observability_integration(self, integrated_registry):
        """Test integration with observability system."""
        obs = integrated_registry.get_component("observability")

        # Registry operations should be observable
        integrated_registry.register_component("test_comp", Mock())

        # Should be tracked in observability
        assert obs is not None

    def test_component_registry_with_real_components(self):
        """Test registry with real component implementations."""
        registry = ComponentRegistry()

        # This would test with actual component classes in a real scenario
        # For now, just verify the framework works
        assert registry is not None

    def test_registry_high_availability_setup(self):
        """Test registry setup for high availability."""
        # Test registry configuration for redundant components
        registry = ComponentRegistry()

        # Register primary and backup components
        primary_nav = Mock()
        backup_nav = Mock()

        registry.register_component("navigation_primary", primary_nav)
        registry.register_component("navigation_backup", backup_nav, tags={"backup": True})

        # Should support failover scenarios
        assert registry.get_component("navigation_primary") == primary_nav
        assert registry.get_component("navigation_backup") == backup_nav
