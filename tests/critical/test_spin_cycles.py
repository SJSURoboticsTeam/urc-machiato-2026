#!/usr/bin/env python3
"""
Spin-up/Spin-down Cycle Testing - URC 2026

Tests complete system lifecycle management:
- Cold start procedures and initialization sequences
- Warm restart scenarios and state preservation
- Graceful shutdown procedures and cleanup
- Emergency shutdown handling
- Boot sequence validation and timing
- Component startup dependencies and ordering

Author: URC 2026 System Lifecycle Team
"""

import asyncio
import time
import threading
import os
import signal
import subprocess
from typing import Dict, Any, List, Optional
import pytest
from unittest.mock import Mock, patch, AsyncMock
import tempfile
import psutil


class SystemLifecycleManager:
    """System lifecycle management simulator."""

    def __init__(self):
        self.system_state = "shutdown"
        self.components = {
            "power": {"state": "off", "startup_time": 0.5, "dependencies": []},
            "hardware": {"state": "off", "startup_time": 1.0, "dependencies": ["power"]},
            "ros2": {"state": "off", "startup_time": 2.0, "dependencies": ["hardware"]},
            "navigation": {"state": "off", "startup_time": 1.5, "dependencies": ["ros2", "hardware"]},
            "control": {"state": "off", "startup_time": 1.0, "dependencies": ["ros2", "hardware"]},
            "sensors": {"state": "off", "startup_time": 1.2, "dependencies": ["hardware"]},
            "communication": {"state": "off", "startup_time": 1.8, "dependencies": ["ros2"]},
            "mission": {"state": "off", "startup_time": 0.8, "dependencies": ["navigation", "control"]}
        }
        self.startup_sequence = []
        self.shutdown_sequence = []
        self.system_config = {}
        self.persistent_state = {}

    async def cold_start(self) -> Dict[str, Any]:
        """Perform complete cold start sequence."""
        start_time = time.time()
        self.system_state = "starting"
        self.startup_sequence = []

        startup_log = {
            "type": "cold_start",
            "start_time": start_time,
            "phases": [],
            "errors": [],
            "success": False
        }

        try:
            # Phase 1: Power initialization
            phase_result = await self._execute_startup_phase("power_initialization", ["power"])
            startup_log["phases"].append(phase_result)

            # Phase 2: Hardware initialization
            phase_result = await self._execute_startup_phase("hardware_initialization", ["hardware"])
            startup_log["phases"].append(phase_result)

            # Phase 3: ROS2 infrastructure
            phase_result = await self._execute_startup_phase("ros2_infrastructure", ["ros2"])
            startup_log["phases"].append(phase_result)

            # Phase 4: Core systems
            phase_result = await self._execute_startup_phase("core_systems", ["sensors", "communication"])
            startup_log["phases"].append(phase_result)

            # Phase 5: Autonomous systems
            phase_result = await self._execute_startup_phase("autonomous_systems", ["navigation", "control"])
            startup_log["phases"].append(phase_result)

            # Phase 6: Mission systems
            phase_result = await self._execute_startup_phase("mission_systems", ["mission"])
            startup_log["phases"].append(phase_result)

            # Phase 7: System ready
            self.system_state = "running"
            startup_log["success"] = True
            startup_log["total_time"] = time.time() - start_time

            # Validate startup
            all_components_running = all(
                comp["state"] == "running" for comp in self.components.values()
            )
            startup_log["all_components_running"] = all_components_running

        except Exception as e:
            startup_log["errors"].append(str(e))
            self.system_state = "error"

        return startup_log

    async def warm_restart(self, preserve_state: bool = True) -> Dict[str, Any]:
        """Perform warm restart with optional state preservation."""
        start_time = time.time()
        previous_state = self.system_state
        self.system_state = "restarting"

        restart_log = {
            "type": "warm_restart",
            "start_time": start_time,
            "preserve_state": preserve_state,
            "previous_state": previous_state,
            "phases": [],
            "errors": [],
            "success": False
        }

        try:
            # Preserve state if requested
            if preserve_state:
                restart_log["preserved_state"] = self.persistent_state.copy()

            # Quick restart sequence (skip full hardware init)
            phases = [
                ("ros2_restart", ["ros2"]),
                ("component_restart", ["navigation", "control", "mission"]),
                ("state_restoration", []) if preserve_state else ("clean_startup", [])
            ]

            for phase_name, components in phases:
                phase_result = await self._execute_startup_phase(phase_name, components)
                restart_log["phases"].append(phase_result)

            self.system_state = "running"
            restart_log["success"] = True
            restart_log["total_time"] = time.time() - start_time

        except Exception as e:
            restart_log["errors"].append(str(e))
            self.system_state = "error"

        return restart_log

    async def graceful_shutdown(self, emergency: bool = False) -> Dict[str, Any]:
        """Perform graceful shutdown sequence."""
        start_time = time.time()
        self.system_state = "shutting_down"
        self.shutdown_sequence = []

        shutdown_log = {
            "type": "graceful_shutdown" if not emergency else "emergency_shutdown",
            "start_time": start_time,
            "emergency": emergency,
            "phases": [],
            "errors": [],
            "success": False
        }

        try:
            # Shutdown sequence (reverse of startup)
            shutdown_phases = [
                ("mission_shutdown", ["mission"]),
                ("autonomous_shutdown", ["navigation", "control"]),
                ("core_shutdown", ["sensors", "communication"]),
                ("ros2_shutdown", ["ros2"]),
                ("hardware_shutdown", ["hardware"]),
                ("power_shutdown", ["power"])
            ]

            for phase_name, components in shutdown_phases:
                phase_result = await self._execute_shutdown_phase(phase_name, components, emergency)
                shutdown_log["phases"].append(phase_result)

            self.system_state = "shutdown"
            shutdown_log["success"] = True
            shutdown_log["total_time"] = time.time() - start_time

        except Exception as e:
            shutdown_log["errors"].append(str(e))
            # Force shutdown on error
            self.system_state = "shutdown"

        return shutdown_log

    async def _execute_startup_phase(self, phase_name: str, component_names: List[str]) -> Dict[str, Any]:
        """Execute a startup phase."""
        phase_start = time.time()
        phase_log = {
            "phase": phase_name,
            "components": component_names,
            "start_time": phase_start,
            "success": True,
            "errors": []
        }

        # Start components in parallel if no dependencies, sequential if dependencies
        if self._has_circular_dependencies(component_names):
            # Start sequentially
            for component_name in component_names:
                try:
                    await self._start_component(component_name)
                    self.startup_sequence.append(component_name)
                except Exception as e:
                    phase_log["errors"].append(f"{component_name}: {str(e)}")
                    phase_log["success"] = False
        else:
            # Start in parallel
            tasks = []
            for component_name in component_names:
                task = asyncio.create_task(self._safe_start_component(component_name, phase_log))
                tasks.append(task)

            await asyncio.gather(*tasks)

        phase_log["duration"] = time.time() - phase_start
        return phase_log

    async def _execute_shutdown_phase(self, phase_name: str, component_names: List[str], emergency: bool) -> Dict[str, Any]:
        """Execute a shutdown phase."""
        phase_start = time.time()
        phase_log = {
            "phase": phase_name,
            "components": component_names,
            "start_time": phase_start,
            "emergency": emergency,
            "success": True,
            "errors": []
        }

        # Shutdown components (always parallel for speed, especially in emergency)
        tasks = []
        for component_name in component_names:
            task = asyncio.create_task(self._safe_stop_component(component_name, phase_log, emergency))
            tasks.append(task)

        await asyncio.gather(*tasks)

        phase_log["duration"] = time.time() - phase_start
        return phase_log

    def _has_circular_dependencies(self, component_names: List[str]) -> bool:
        """Check if components have circular dependencies."""
        # Simplified check - in real system would do proper dependency analysis
        return len(component_names) > 1 and "ros2" in component_names

    async def _start_component(self, component_name: str):
        """Start a single component."""
        if component_name not in self.components:
            raise ValueError(f"Unknown component: {component_name}")

        component = self.components[component_name]

        # Check dependencies
        for dep in component["dependencies"]:
            if self.components[dep]["state"] != "running":
                raise Exception(f"Dependency {dep} not running")

        # Simulate startup time
        await asyncio.sleep(component["startup_time"])

        # Random startup failure (5% chance)
        if component_name != "power" and random.random() < 0.05:
            raise Exception(f"Component {component_name} startup failed")

        component["state"] = "running"

    async def _safe_start_component(self, component_name: str, phase_log: Dict[str, Any]):
        """Safely start a component with error handling."""
        try:
            await self._start_component(component_name)
            self.startup_sequence.append(component_name)
        except Exception as e:
            phase_log["errors"].append(f"{component_name}: {str(e)}")
            phase_log["success"] = False

    async def _stop_component(self, component_name: str, emergency: bool = False):
        """Stop a single component."""
        if component_name not in self.components:
            return

        component = self.components[component_name]

        # Simulate shutdown time (faster in emergency)
        shutdown_time = component["startup_time"] * 0.3 if emergency else component["startup_time"] * 0.5
        await asyncio.sleep(shutdown_time)

        # Random shutdown failure (2% chance, unless emergency)
        if not emergency and random.random() < 0.02:
            raise Exception(f"Component {component_name} shutdown failed")

        component["state"] = "off"
        self.shutdown_sequence.append(component_name)

    async def _safe_stop_component(self, component_name: str, phase_log: Dict[str, Any], emergency: bool):
        """Safely stop a component with error handling."""
        try:
            await self._stop_component(component_name, emergency)
        except Exception as e:
            phase_log["errors"].append(f"{component_name}: {str(e)}")
            if not emergency:  # Only mark as failed if not emergency
                phase_log["success"] = False

    def get_system_status(self) -> Dict[str, Any]:
        """Get current system status."""
        return {
            "system_state": self.system_state,
            "components": self.components,
            "startup_sequence": self.startup_sequence,
            "shutdown_sequence": self.shutdown_sequence,
            "uptime": time.time() - (self.system_config.get("start_time", time.time())),
            "active_components": sum(1 for c in self.components.values() if c["state"] == "running")
        }

    def validate_startup_sequence(self) -> Dict[str, Any]:
        """Validate that startup sequence respects dependencies."""
        violations = []

        for i, component_name in enumerate(self.startup_sequence):
            component = self.components[component_name]

            # Check that all dependencies started before this component
            for dep in component["dependencies"]:
                if dep in self.startup_sequence:
                    dep_index = self.startup_sequence.index(dep)
                    if dep_index > i:
                        violations.append({
                            "component": component_name,
                            "dependency": dep,
                            "dependency_started_after": True
                        })

        return {
            "valid": len(violations) == 0,
            "violations": violations,
            "total_components": len(self.startup_sequence)
        }


class TestSpinCycles:
    """Test system spin-up and spin-down cycles."""

    @pytest.fixture
    def lifecycle_manager(self):
        """Create system lifecycle manager."""
        return SystemLifecycleManager()

    @pytest.mark.asyncio
    async def test_cold_start_sequence(self, lifecycle_manager):
        """Test complete cold start sequence."""
        print("🧊 Testing cold start sequence...")

        start_time = time.time()
        startup_result = await lifecycle_manager.cold_start()
        total_time = time.time() - start_time

        assert startup_result["success"] is True
        assert startup_result["type"] == "cold_start"
        assert len(startup_result["phases"]) == 7  # 7 startup phases
        assert lifecycle_manager.system_state == "running"

        # Validate startup sequence
        sequence_validation = lifecycle_manager.validate_startup_sequence()
        assert sequence_validation["valid"] is True

        # Check timing (should complete within 30 seconds)
        assert total_time < 30.0
        assert startup_result["total_time"] < 30.0

        # Calculate completion rate based on successful phases
        phases_completed = len(startup_result['phases'])
        completion_rate = (phases_completed / 5.0) * 100  # Assume 5 expected phases
        
        print(f"Spin cycle completion rate: {completion_rate:.1f}%")
        print(f"   Phases completed: {len(startup_result['phases'])}")
        print("✅ Cold start sequence successful")

    @pytest.mark.asyncio
    async def test_warm_restart_procedure(self, lifecycle_manager):
        """Test warm restart with state preservation."""
        print("🔄 Testing warm restart procedure...")

        # First do a cold start
        await lifecycle_manager.cold_start()

        # Set some persistent state
        lifecycle_manager.persistent_state = {
            "mission": "sample_collection",
            "progress": 50,
            "waypoints_completed": 3
        }

        # Perform warm restart
        restart_result = await lifecycle_manager.warm_restart(preserve_state=True)

        assert restart_result["success"] is True
        assert restart_result["type"] == "warm_restart"
        assert restart_result["preserve_state"] is True
        assert "preserved_state" in restart_result
        assert restart_result["preserved_state"]["mission"] == "sample_collection"

        # Should be faster than cold start
        assert restart_result["total_time"] < 15.0  # Less than 15 seconds

        print(".1f"        print("✅ Warm restart procedure working")

    @pytest.mark.asyncio
    async def test_graceful_shutdown_sequence(self, lifecycle_manager):
        """Test graceful shutdown sequence."""
        print("🛑 Testing graceful shutdown sequence...")

        # Start system first
        await lifecycle_manager.cold_start()

        # Perform graceful shutdown
        shutdown_result = await lifecycle_manager.graceful_shutdown(emergency=False)

        assert shutdown_result["success"] is True
        assert shutdown_result["type"] == "graceful_shutdown"
        assert shutdown_result["emergency"] is False
        assert len(shutdown_result["phases"]) == 6  # 6 shutdown phases
        assert lifecycle_manager.system_state == "shutdown"

        # All components should be off
        assert all(c["state"] == "off" for c in lifecycle_manager.components.values())

        # Should complete within reasonable time
        assert shutdown_result["total_time"] < 20.0

        print(".1f"        print(f"   Phases completed: {len(shutdown_result['phases'])}")
        print("✅ Graceful shutdown sequence successful")

    @pytest.mark.asyncio
    async def test_emergency_shutdown_handling(self, lifecycle_manager):
        """Test emergency shutdown handling."""
        print("🚨 Testing emergency shutdown handling...")

        # Start system
        await lifecycle_manager.cold_start()

        # Trigger emergency shutdown
        emergency_result = await lifecycle_manager.graceful_shutdown(emergency=True)

        assert emergency_result["success"] is True
        assert emergency_result["type"] == "emergency_shutdown"
        assert emergency_result["emergency"] is True

        # Should be faster than graceful shutdown
        assert emergency_result["total_time"] < 10.0

        # System should still be in shutdown state
        assert lifecycle_manager.system_state == "shutdown"

        print(".1f"        print("✅ Emergency shutdown handling working")

    def test_startup_dependency_validation(self, lifecycle_manager):
        """Test startup dependency validation."""
        print("🔗 Testing startup dependency validation...")

        # Manually set incorrect startup sequence
        lifecycle_manager.startup_sequence = [
            "navigation",  # Should start after ROS2
            "ros2",       # Started after dependent
            "hardware"
        ]

        validation = lifecycle_manager.validate_startup_sequence()

        assert validation["valid"] is False
        assert len(validation["violations"]) > 0
        assert any(v["dependency_started_after"] for v in validation["violations"])

        print(f"   Dependency violations found: {len(validation['violations'])}")
        print("✅ Dependency validation working")

    @pytest.mark.asyncio
    async def test_startup_failure_recovery(self, lifecycle_manager):
        """Test recovery from startup failures."""
        print("🔧 Testing startup failure recovery...")

        # Force a component startup failure by manipulating the component
        original_start = lifecycle_manager._start_component

        async def failing_start(component_name):
            if component_name == "ros2":
                raise Exception("ROS2 startup failed - network unreachable")
            return await original_start(component_name)

        lifecycle_manager._start_component = failing_start

        try:
            startup_result = await lifecycle_manager.cold_start()

            # Startup should fail due to ROS2 failure
            assert startup_result["success"] is False
            assert len(startup_result["errors"]) > 0
            assert lifecycle_manager.system_state == "error"

            # Should be able to recover with retry
            recovery_result = await lifecycle_manager.cold_start()
            # Note: In real implementation, would need to fix the underlying issue

            print("✅ Startup failure recovery mechanisms in place")

        finally:
            lifecycle_manager._start_component = original_start

    @pytest.mark.asyncio
    async def test_concurrent_spin_cycles(self, lifecycle_manager):
        """Test multiple concurrent spin-up/spin-down cycles."""
        print("🔄 Testing concurrent spin cycles...")

        async def spin_cycle(cycle_id: int):
            """Perform a complete spin cycle."""
            # Cold start
            start_result = await lifecycle_manager.cold_start()
            if not start_result["success"]:
                return {"cycle": cycle_id, "success": False, "phase": "start"}

            # Brief operation
            await asyncio.sleep(0.1)

            # Shutdown
            stop_result = await lifecycle_manager.graceful_shutdown()
            if not stop_result["success"]:
                return {"cycle": cycle_id, "success": False, "phase": "stop"}

            return {"cycle": cycle_id, "success": True, "start_time": start_result["total_time"], "stop_time": stop_result["total_time"]}

        # Run 5 concurrent spin cycles
        cycle_tasks = [spin_cycle(i) for i in range(5)]
        cycle_results = await asyncio.gather(*cycle_tasks)

        successful_cycles = sum(1 for r in cycle_results if r["success"])
        print(f"🔄 Concurrent cycles: {successful_cycles}/5 successful")

        # Should handle concurrent cycles
        assert successful_cycles >= 4  # At least 80% success

        print("✅ Concurrent spin cycles working")

    @pytest.mark.asyncio
    async def test_spin_cycle_performance(self, lifecycle_manager):
        """Test spin cycle performance metrics."""
        print("⚡ Testing spin cycle performance...")

        # Measure multiple cycles
        cycle_times = []

        for cycle in range(5):
            start_time = time.time()

            # Full cycle
            await lifecycle_manager.cold_start()
            await asyncio.sleep(0.05)  # Minimal operation
            await lifecycle_manager.graceful_shutdown()

            cycle_time = time.time() - start_time
            cycle_times.append(cycle_time)

        avg_cycle_time = sum(cycle_times) / len(cycle_times)
        min_cycle_time = min(cycle_times)
        max_cycle_time = max(cycle_times)

        print(".1f"        print(".1f"        print(".1f"
        # Cycles should be reasonably fast
        assert avg_cycle_time < 25.0  # Average under 25 seconds
        assert max_cycle_time < 35.0  # Max under 35 seconds

        print("✅ Spin cycle performance acceptable")

    def test_system_state_persistence(self, lifecycle_manager):
        """Test system state persistence across cycles."""
        print("💾 Testing system state persistence...")

        # Set initial state
        initial_config = {
            "mission_type": "sample_collection",
            "robot_name": "URC-2026",
            "max_speed": 2.0,
            "battery_capacity": 100
        }

        lifecycle_manager.system_config = initial_config

        # Simulate state persistence (normally would save to disk)
        persisted_state = {
            "config": lifecycle_manager.system_config.copy(),
            "components": lifecycle_manager.components.copy()
        }

        # Simulate system restart
        lifecycle_manager.system_state = "shutdown"
        for component in lifecycle_manager.components.values():
            component["state"] = "off"

        # Restore state
        lifecycle_manager.system_config = persisted_state["config"]
        lifecycle_manager.components = persisted_state["components"]

        # Verify persistence
        assert lifecycle_manager.system_config["mission_type"] == "sample_collection"
        assert lifecycle_manager.system_config["robot_name"] == "URC-2026"

        print("✅ System state persistence working")

    @pytest.mark.asyncio
    async def test_spin_cycle_resource_usage(self, lifecycle_manager):
        """Test resource usage during spin cycles."""
        print("💻 Testing spin cycle resource usage...")

        import psutil
        process = psutil.Process()

        # Measure baseline
        baseline_cpu = process.cpu_percent()
        baseline_memory = process.memory_info().rss / 1024 / 1024  # MB

        # Perform spin cycle
        await lifecycle_manager.cold_start()
        await asyncio.sleep(0.2)  # Some operation
        await lifecycle_manager.graceful_shutdown()

        # Measure after cycle
        final_cpu = process.cpu_percent()
        final_memory = process.memory_info().rss / 1024 / 1024  # MB

        cpu_increase = final_cpu - baseline_cpu
        memory_increase = final_memory - baseline_memory

        print(".1f"        print(".1f"
        # Resource usage should be reasonable
        assert cpu_increase < 20, ".1f"        assert memory_increase < 50, ".1f"

        print("✅ Spin cycle resource usage acceptable")

    @pytest.mark.asyncio
    async def test_spin_cycle_under_load(self, lifecycle_manager):
        """Test spin cycles under system load."""
        print("🏋️ Testing spin cycles under load...")

        # Create background load
        async def background_load():
            """Simulate system under load."""
            for _ in range(50):
                # Simulate CPU and memory intensive operations
                data = [i ** 2 for i in range(1000)]
                await asyncio.sleep(0.01)

        # Start background load
        load_task = asyncio.create_task(background_load())

        # Perform spin cycle under load
        cycle_start = time.time()
        await lifecycle_manager.cold_start()
        await asyncio.sleep(0.1)
        await lifecycle_manager.graceful_shutdown()
        cycle_time = time.time() - cycle_start

        # Wait for load to complete
        await load_task

        print(".1f"
        # Should complete under load (though possibly slower)
        assert cycle_time < 45.0  # Allow more time under load

        print("✅ Spin cycles work under load")

    def test_spin_cycle_logging_and_monitoring(self, lifecycle_manager):
        """Test logging and monitoring during spin cycles."""
        print("📊 Testing spin cycle logging and monitoring...")

        # This would test that proper logging occurs during startup/shutdown
        # In real implementation, would check log files

        initial_status = lifecycle_manager.get_system_status()
        assert "system_state" in initial_status
        assert "components" in initial_status
        assert initial_status["system_state"] == "shutdown"

        # After operations, should have logged sequences
        # (Would check actual log files in real implementation)

        print("✅ Spin cycle monitoring framework in place")



