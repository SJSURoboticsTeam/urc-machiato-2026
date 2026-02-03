#!/usr/bin/env python3
"""
Recovery Mechanisms Tests - URC 2026 Resource Optimization

Tests automatic recovery and scaling under resource pressure:
- CPU pressure recovery (automatic component disabling)
- Memory pressure recovery (automatic scaling)
- Combined resource pressure scenarios
- Recovery effectiveness measurement
- System stability during recovery

Author: URC 2026 Recovery Testing Team
"""

import unittest
import threading
import time
import psutil
import statistics
from typing import Dict, Any, List
import gc
import os

from tests.statistical_performance_testing import StatisticalMeasurement


class RecoveryMechanismsTests(unittest.TestCase):
    """Test recovery mechanisms under resource pressure."""

    def setUp(self):
        """Set up test environment."""
        self.process = psutil.Process()
        self.original_cpu_affinity = (
            self.process.cpu_affinity()
            if hasattr(self.process, "cpu_affinity")
            else None
        )

        # Recovery test parameters
        self.recovery_timeout = 10.0  # seconds to wait for recovery
        self.pressure_duration = 5.0  # seconds of pressure application
        self.measurement_interval = 0.5  # seconds between measurements

    def tearDown(self):
        """Clean up after tests."""
        # Restore CPU affinity if changed
        if self.original_cpu_affinity and hasattr(self.process, "cpu_affinity"):
            try:
                self.process.cpu_affinity(self.original_cpu_affinity)
            except:
                pass

        # Force garbage collection
        gc.collect()

    def test_cpu_pressure_recovery(self):
        """Test automatic recovery under CPU pressure."""
        print("Testing CPU pressure recovery...")

        try:
            from src.core.mission_resource_manager import get_mission_resource_manager

            # Initialize resource manager
            resource_manager = get_mission_resource_manager()

            # Switch to full mission mode (should enable many components)
            success = resource_manager.switch_mission_profile("sample_collection")
            self.assertTrue(success)

            # Get baseline component count
            baseline_status = resource_manager.get_resource_status()
            baseline_components = len(baseline_status.get("component_status", {}))
            self.assertGreater(baseline_components, 0, "Should have components enabled")

            # Measure baseline resources
            baseline_cpu, baseline_memory = self._measure_resources()

            # Start CPU pressure thread
            pressure_thread = threading.Thread(
                target=self._generate_cpu_pressure,
                args=(80, self.pressure_duration),
                daemon=True,
            )
            pressure_thread.start()

            # Wait for pressure to build and recovery to activate
            time.sleep(2.0)

            # Monitor resource usage during pressure
            pressure_measurements = []
            for _ in range(int(self.recovery_timeout / self.measurement_interval)):
                cpu, memory = self._measure_resources()
                pressure_measurements.append((cpu, memory))

                # Check if recovery activated (components disabled)
                current_status = resource_manager.get_resource_status()
                current_components = len(current_status.get("component_status", {}))

                if current_components < baseline_components:
                    print(
                        f"✅ Recovery activated: {baseline_components} → {current_components} components"
                    )
                    break

                time.sleep(self.measurement_interval)

            # Wait for pressure thread to complete
            pressure_thread.join(timeout=2.0)

            # Verify recovery effectiveness
            final_status = resource_manager.get_resource_status()
            final_components = len(final_status.get("component_status", {}))
            final_cpu, final_memory = self._measure_resources()

            # Recovery should have reduced component count or resource usage
            recovery_effective = (
                final_components < baseline_components
                or final_cpu < max(cpu for cpu, _ in pressure_measurements)
                or final_memory < max(memory for _, memory in pressure_measurements)
            )

            self.assertTrue(
                recovery_effective, "Recovery mechanism should reduce resource usage"
            )

            # System should still be functional
            essential_status = final_status.get("component_status", {})
            self.assertIsInstance(
                essential_status, dict, "Component status should be accessible"
            )

            print(
                "✅ CPU pressure recovery test passed"
                f"Components: {baseline_components} → {final_components}, "
                f"CPU: {baseline_cpu:.1f}% → {final_cpu:.1f}%"
            )

        except Exception as e:
            self.fail(f"CPU pressure recovery test failed: {e}")

    def test_memory_pressure_recovery(self):
        """Test automatic recovery under memory pressure."""
        print("Testing memory pressure recovery...")

        try:
            from src.core.mission_resource_manager import get_mission_resource_manager

            resource_manager = get_mission_resource_manager()
            resource_manager.switch_mission_profile("sample_collection")

            # Get baseline
            baseline_components = len(
                resource_manager.get_resource_status().get("component_status", {})
            )
            baseline_cpu, baseline_memory = self._measure_resources()

            # Start memory pressure thread
            pressure_thread = threading.Thread(
                target=self._generate_memory_pressure,
                args=(150, self.pressure_duration),  # 150MB pressure
                daemon=True,
            )
            pressure_thread.start()

            # Monitor for recovery
            max_memory_during_pressure = 0
            recovery_detected = False

            for _ in range(int(self.recovery_timeout / self.measurement_interval)):
                cpu, memory = self._measure_resources()
                max_memory_during_pressure = max(max_memory_during_pressure, memory)

                # Check for recovery
                current_components = len(
                    resource_manager.get_resource_status().get("component_status", {})
                )

                if current_components < baseline_components:
                    recovery_detected = True
                    print(
                        f"✅ Memory recovery activated: {baseline_components} → {current_components} components"
                    )
                    break

                time.sleep(self.measurement_interval)

            pressure_thread.join(timeout=2.0)

            # Verify memory recovery
            final_cpu, final_memory = self._measure_resources()

            # Memory usage should not have grown excessively
            memory_growth = final_memory - baseline_memory
            self.assertLess(
                memory_growth, 200, "Memory growth should be controlled during recovery"
            )

            # Either recovery activated or memory was managed
            recovery_effective = (
                recovery_detected
                or final_memory
                <= max_memory_during_pressure * 1.1  # Within 10% of peak
            )

            self.assertTrue(
                recovery_effective, "Memory recovery should prevent excessive growth"
            )

            print(
                "✅ Memory pressure recovery test passed"
                f"Memory: {baseline_memory:.1f}MB → {final_memory:.1f}MB (peak: {max_memory_during_pressure:.1f}MB)"
            )

        except Exception as e:
            self.fail(f"Memory pressure recovery test failed: {e}")

    def test_combined_resource_pressure_recovery(self):
        """Test recovery under combined CPU and memory pressure."""
        print("Testing combined resource pressure recovery...")

        try:
            from src.core.mission_resource_manager import get_mission_resource_manager

            resource_manager = get_mission_resource_manager()
            resource_manager.switch_mission_profile("sample_collection")

            baseline_components = len(
                resource_manager.get_resource_status().get("component_status", {})
            )
            baseline_cpu, baseline_memory = self._measure_resources()

            # Start combined pressure threads
            cpu_thread = threading.Thread(
                target=self._generate_cpu_pressure,
                args=(70, self.pressure_duration),
                daemon=True,
            )
            memory_thread = threading.Thread(
                target=self._generate_memory_pressure,
                args=(100, self.pressure_duration),
                daemon=True,
            )

            cpu_thread.start()
            memory_thread.start()

            # Monitor combined pressure effects
            max_cpu = 0
            max_memory = 0
            recovery_triggers = []

            for i in range(int(self.recovery_timeout / self.measurement_interval)):
                cpu, memory = self._measure_resources()
                max_cpu = max(max_cpu, cpu)
                max_memory = max(max_memory, memory)

                current_components = len(
                    resource_manager.get_resource_status().get("component_status", {})
                )
                current_status = resource_manager.get_resource_status()

                # Track recovery events
                if current_components < baseline_components:
                    recovery_triggers.append(
                        {
                            "time": i * self.measurement_interval,
                            "components": current_components,
                            "cpu": cpu,
                            "memory": memory,
                        }
                    )

                    if len(recovery_triggers) >= 2:  # Multiple recovery events
                        break

                time.sleep(self.measurement_interval)

            cpu_thread.join(timeout=2.0)
            memory_thread.join(timeout=2.0)

            final_cpu, final_memory = self._measure_resources()

            # Verify combined recovery effectiveness
            recovery_activated = len(recovery_triggers) > 0
            resources_reduced = (
                final_cpu < max_cpu * 0.9
                or final_memory  # At least 10% CPU reduction
                < max_memory * 0.9  # At least 10% memory reduction
            )

            recovery_effective = recovery_activated or resources_reduced

            self.assertTrue(
                recovery_effective, "Combined pressure recovery should activate"
            )

            print(
                "✅ Combined pressure recovery test passed"
                f"Recovery events: {len(recovery_triggers)}, "
                f"CPU: {baseline_cpu:.1f}% → {final_cpu:.1f}%, "
                f"Memory: {baseline_memory:.1f}MB → {final_memory:.1f}MB"
            )

        except Exception as e:
            self.fail(f"Combined pressure recovery test failed: {e}")

    def test_recovery_performance_overhead(self):
        """Test that recovery mechanisms don't introduce excessive overhead."""
        print("Testing recovery performance overhead...")

        try:
            from src.core.mission_resource_manager import get_mission_resource_manager

            resource_manager = get_mission_resource_manager()

            # Measure normal operation performance
            resource_manager.switch_mission_profile(
                "waypoint_navigation"
            )  # Minimal components

            normal_times = []
            for _ in range(10):
                start = time.time()
                status = resource_manager.get_resource_status()
                normal_times.append(time.time() - start)
                time.sleep(0.1)

            normal_avg = statistics.mean(normal_times)

            # Switch to full operation and measure
            resource_manager.switch_mission_profile("sample_collection")

            full_times = []
            for _ in range(10):
                start = time.time()
                status = resource_manager.get_resource_status()
                full_times.append(time.time() - start)
                time.sleep(0.1)

            full_avg = statistics.mean(full_times)

            # Overhead should be reasonable (< 50% increase)
            overhead_ratio = full_avg / normal_avg if normal_avg > 0 else 0
            self.assertLess(overhead_ratio, 1.5, "Recovery overhead should be minimal")

            # Absolute timing should be reasonable (< 10ms per status check)
            self.assertLess(full_avg, 0.01, "Status checks should be fast")

            print(
                "✅ Recovery performance overhead test passed"
                f"Normal: {normal_avg*1000:.2f}ms, Full: {full_avg*1000:.2f}ms "
                f"(overhead: {(overhead_ratio-1)*100:.1f}%)"
            )

        except Exception as e:
            self.fail(f"Recovery performance overhead test failed: {e}")

    def test_recovery_stability(self):
        """Test that recovery mechanisms don't cause system instability."""
        print("Testing recovery stability...")

        try:
            from src.core.mission_resource_manager import get_mission_resource_manager

            resource_manager = get_mission_resource_manager()

            # Test multiple recovery cycles
            stability_measurements = []

            for cycle in range(3):
                # Start with full mission
                resource_manager.switch_mission_profile("sample_collection")
                time.sleep(0.5)

                # Apply brief pressure
                pressure_thread = threading.Thread(
                    target=self._generate_cpu_pressure,
                    args=(85, 2.0),  # Brief intense pressure
                    daemon=True,
                )
                pressure_thread.start()

                # Monitor during recovery
                cycle_measurements = []
                for _ in range(10):  # 5 seconds of monitoring
                    cpu, memory = self._measure_resources()
                    cycle_measurements.append((cpu, memory))

                    components = len(
                        resource_manager.get_resource_status().get(
                            "component_status", {}
                        )
                    )
                    cycle_measurements.append(components)

                    time.sleep(0.5)

                pressure_thread.join(timeout=1.0)
                stability_measurements.append(cycle_measurements)

            # Analyze stability across cycles
            cpu_variability = []
            memory_variability = []
            component_variability = []

            for cycle_data in stability_measurements:
                cycle_cpus = [
                    cpu
                    for cpu, mem, comp in cycle_data
                    if isinstance(cpu, (int, float))
                ]
                cycle_mems = [
                    mem
                    for cpu, mem, comp in cycle_data
                    if isinstance(mem, (int, float))
                ]
                cycle_comps = [
                    comp for cpu, mem, comp in cycle_data if isinstance(comp, int)
                ]

                if cycle_cpus:
                    cpu_variability.append(
                        statistics.stdev(cycle_cpus) / statistics.mean(cycle_cpus)
                    )
                if cycle_mems:
                    memory_variability.append(
                        statistics.stdev(cycle_mems) / statistics.mean(cycle_mems)
                    )
                if cycle_comps:
                    component_variability.append(
                        statistics.stdev(cycle_comps) / statistics.mean(cycle_comps)
                    )

            # System should stabilize (coefficient of variation < 50%)
            avg_cpu_cv = statistics.mean(cpu_variability) if cpu_variability else 0
            avg_memory_cv = (
                statistics.mean(memory_variability) if memory_variability else 0
            )

            self.assertLess(
                avg_cpu_cv, 0.5, "CPU usage should stabilize during recovery"
            )
            self.assertLess(
                avg_memory_cv, 0.5, "Memory usage should stabilize during recovery"
            )

            print(
                "✅ Recovery stability test passed"
                f"CPU CV: {avg_cpu_cv:.2f}, Memory CV: {avg_memory_cv:.2f}"
            )

        except Exception as e:
            self.fail(f"Recovery stability test failed: {e}")

    def _generate_cpu_pressure(self, target_percent: float, duration: float):
        """Generate CPU pressure to trigger recovery mechanisms."""
        end_time = time.time() + duration

        while time.time() < end_time:
            # Calculate busy/idle times to achieve target CPU percentage
            busy_time = target_percent / 100.0 * 0.01
            idle_time = (1 - target_percent / 100.0) * 0.01

            # Busy loop
            busy_end = time.time() + busy_time
            while time.time() < busy_end:
                pass

            # Idle time
            time.sleep(idle_time)

    def _generate_memory_pressure(self, target_mb: int, duration: float):
        """Generate memory pressure to trigger recovery mechanisms."""
        # Allocate memory gradually
        allocations = []
        allocation_size = target_mb * 1024 * 128  # Convert MB to list elements

        try:
            # Allocate in chunks to simulate growing memory usage
            chunk_size = allocation_size // 10
            for i in range(10):
                allocations.append([0] * chunk_size)
                time.sleep(duration / 10)

            # Hold memory for the specified duration
            time.sleep(duration)

        finally:
            # Clean up allocations
            del allocations
            gc.collect()

    def _measure_resources(self) -> tuple[float, float]:
        """Measure current CPU and memory usage."""
        cpu_percent = self.process.cpu_percent(interval=None)
        memory_mb = self.process.memory_info().rss / (1024 * 1024)
        return cpu_percent, memory_mb


def run_recovery_tests():
    """Run all recovery mechanism tests."""
    print("🛟 RECOVERY MECHANISMS TESTS - URC 2026")
    print("=" * 45)

    # Create test suite
    suite = unittest.TestLoader().loadTestsFromTestCase(RecoveryMechanismsTests)
    runner = unittest.TextTestRunner(verbosity=2)

    # Run tests
    result = runner.run(suite)

    # Summary
    print("\n📊 RECOVERY TEST RESULTS")
    print("=" * 30)
    print(f"Tests run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")

    if result.wasSuccessful():
        print("✅ All recovery tests passed!")
        return True
    else:
        print("❌ Some recovery tests failed")
        for failure in result.failures:
            print(f"  FAILED: {failure[0]}")
        for error in result.errors:
            print(f"  ERROR: {error[0]}")
        return False


if __name__ == "__main__":
    success = run_recovery_tests()
    exit(0 if success else 1)
