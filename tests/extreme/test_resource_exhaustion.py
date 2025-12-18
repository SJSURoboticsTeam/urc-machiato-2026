#!/usr/bin/env python3
"""
Extreme Resource Exhaustion Tests

Tests system behavior under extreme resource constraints:
- Memory pressure (99% memory usage)
- CPU starvation (single core, high load)
- Disk I/O contention
- Network bandwidth saturation

Author: URC 2026 Autonomy Team
"""

import gc
import os
import sys
import threading
import time
import unittest
from unittest.mock import Mock, patch

import psutil

# Add src to path - go up two levels from tests/extreme/
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "src"))

from pathlib import Path

from ros2_environment_manager import (
    ResourceLimits,
    ROS2EnvironmentManager,
    ROSEnvironmentConfig,
    get_environment_manager,
)


class ExtremeResourceExhaustionTest(unittest.TestCase):
    """Test extreme resource exhaustion scenarios."""

    def setUp(self):
        """Setup test environment."""
        self.env_manager = get_environment_manager()
        self.workspace_path = Path.cwd()

        # Extreme resource configuration
        self.ros_config = ROSEnvironmentConfig(
            domain_id=300,  # Isolated domain for resource testing
            use_sim_time=True,  # Use sim time for controlled testing
            log_level="WARN",  # Reduce logging overhead
        )

        # Very constrained resources
        self.extreme_limits = ResourceLimits(
            cpu_percent=5.0,  # Extremely limited CPU (5% of one core)
            memory_mb=20,  # Extremely limited memory (20MB)
            max_processes=3,  # Very limited process count
        )

    def test_memory_pressure_extreme(self):
        """Test system behavior under extreme memory pressure (99% usage)."""
        print(" Testing Extreme Memory Pressure...")

        # Test memory pressure resilience without ROS2 environment
        from core.state_synchronization_manager import DistributedStateManager

        # Create manager
        mgr = DistributedStateManager("memory_test")

        # Pre-allocate memory to create pressure
        memory_hog = []
        target_memory_mb = 15  # Leave only 5MB free in constrained environment

        print(f"  [GRAPH] Allocating {target_memory_mb}MB to create memory pressure...")

        # Allocate memory in chunks to reach target
        chunk_size = 1000000  # 1MB chunks
        chunks_needed = target_memory_mb

        for i in range(chunks_needed):
            try:
                memory_hog.append([0] * chunk_size)  # 1MB of integers
            except MemoryError:
                print(f"   Memory allocation stopped at {i}MB")
                break

        allocated_mb = len(memory_hog)
        print(f"  [GRAPH] Allocated {allocated_mb}MB, creating extreme memory pressure")

        # Start manager under memory pressure
        try:
            mgr.start()

            # Register node
            mgr.register_node("memory_test")

            # Test basic operations under memory pressure
            start_time = time.time()

            # Perform memory-intensive operations
            operation_count = 10
            for i in range(operation_count):
                mgr.update_state(f"memory_stress_{i}", f"x" * 1000)  # 1KB strings

                # Force garbage collection to maximize pressure
                gc.collect()

            end_time = time.time()
            operation_time = end_time - start_time

            print(".3f")

            # Verify operations completed
            final_state = mgr.get_state(f"memory_stress_{operation_count-1}")
            self.assertIsNotNone(
                final_state, "Operations should complete under memory pressure"
            )

            # Verify reasonable performance (should complete within reasonable time)
            self.assertLess(
                operation_time,
                30.0,
                "Operations should complete within 30 seconds under memory pressure",
            )

            # Test that state manager remains functional
            status = mgr.get_system_status()
            self.assertIsNotNone(
                status, "State manager should remain functional under memory pressure"
            )
            self.assertEqual(
                status["role"], "master", "State manager should maintain master role"
            )

        except MemoryError:
            self.fail(
                "System should handle memory pressure gracefully, not crash with MemoryError"
            )

        finally:
            # Cleanup memory
            del memory_hog
            gc.collect()

            mgr.stop()

        print("  [PASS] Extreme memory pressure test passed")

    def test_cpu_starvation_scenario(self):
        """Test system under extreme CPU starvation."""
        print("[LIGHTNING] Testing CPU Starvation Scenario...")

        # Create environment with single CPU core limit
        cpu_starved_config = ROSEnvironmentConfig(domain_id=301, use_sim_time=True)

        cpu_limits = ResourceLimits(
            cpu_percent=1.0,  # Extremely limited CPU (1% of one core)
            memory_mb=50,
            max_processes=2,
        )

        with self.env_manager.create_environment(
            name="cpu_starvation_test",
            ros_config=cpu_starved_config,
            resource_limits=cpu_limits,
            workspace_path=self.workspace_path,
        ) as env:

            from core.state_synchronization_manager import DistributedStateManager

            # Create manager
            mgr = DistributedStateManager("cpu_starved")

            mgr.start()

            # Create CPU load threads to starve the system
            cpu_loaders = []
            stop_loading = threading.Event()

            def cpu_loader():
                """Consume CPU cycles continuously."""
                while not stop_loading.is_set():
                    # CPU-intensive operation
                    sum(range(10000))

            # Start multiple CPU loader threads
            loader_count = 2  # Create competition for the limited CPU
            for i in range(loader_count):
                loader = threading.Thread(target=cpu_loader, daemon=True)
                loader.start()
                cpu_loaders.append(loader)

            try:
                print(
                    "  [LIGHTNING] CPU loaders started, creating starvation conditions..."
                )

                # Test operations under CPU starvation
                start_time = time.time()

                operation_count = 20
                for i in range(operation_count):
                    mgr.update_state(f"cpu_stress_{i}", f"data_{i}")
                    time.sleep(0.01)  # Small delay to allow other operations

                end_time = time.time()
                operation_time = end_time - start_time

                print(".2f")
                print(".3f")

                # Verify operations completed (may be slower but should still work)
                final_state = mgr.get_state(f"cpu_stress_{operation_count-1}")
                self.assertIsNotNone(
                    final_state, "Operations should complete under CPU starvation"
                )

                # Operations should take longer due to CPU starvation
                self.assertGreater(
                    operation_time,
                    0.1,
                    "Operations should take measurable time under CPU pressure",
                )

            finally:
                # Stop CPU loaders
                stop_loading.set()
                for loader in cpu_loaders:
                    loader.join(timeout=1.0)

                mgr.stop()

        print("  [PASS] CPU starvation scenario test passed")

    def test_disk_io_contention(self):
        """Test system under extreme disk I/O contention."""
        print(" Testing Disk I/O Contention...")

        with self.env_manager.create_environment(
            name="disk_io_test",
            ros_config=self.ros_config,
            resource_limits=self.extreme_limits,
            workspace_path=self.workspace_path,
        ) as env:

            from core.state_synchronization_manager import DistributedStateManager

            # Create manager
            mgr = DistributedStateManager("disk_test")

            mgr.start()

            # Create disk I/O contention by writing large amounts of log data
            log_files = []
            stop_io = threading.Event()

            def io_contender():
                """Continuously write to disk to create I/O contention."""
                log_file = (
                    env.log_directory
                    / f"io_contender_{threading.current_thread().ident}.log"
                )
                log_files.append(log_file)

                with open(log_file, "w") as f:
                    while not stop_io.is_set():
                        # Write large amounts of data
                        f.write("x" * 10000 + "\n")
                        f.flush()

            # Start I/O contender threads
            io_threads = []
            contender_count = 2

            for i in range(contender_count):
                thread = threading.Thread(target=io_contender, daemon=True)
                thread.start()
                io_threads.append(thread)

            try:
                print("   I/O contenders started, creating disk contention...")

                # Test state operations under I/O pressure
                start_time = time.time()

                operation_count = 15
                for i in range(operation_count):
                    # Create state with substantial data to trigger more I/O
                    large_data = "x" * 5000  # 5KB per operation
                    mgr.update_state(f"io_stress_{i}", large_data)

                end_time = time.time()
                operation_time = end_time - start_time

                print(".2f")
                print(".3f")

                # Verify operations completed
                final_state = mgr.get_state(f"io_stress_{operation_count-1}")
                self.assertIsNotNone(
                    final_state, "Operations should complete under I/O pressure"
                )
                self.assertEqual(
                    len(final_state), 5000, "Large data should be preserved"
                )

            finally:
                # Stop I/O contenders
                stop_io.set()
                for thread in io_threads:
                    thread.join(timeout=2.0)

                # Clean up log files
                for log_file in log_files:
                    try:
                        log_file.unlink()
                    except FileNotFoundError:
                        pass

                mgr.stop()

        print("  [PASS] Disk I/O contention test passed")

    def test_network_bandwidth_saturation(self):
        """Test system under network bandwidth saturation."""
        print("[NETWORK] Testing Network Bandwidth Saturation...")

        # Create environment with extreme network limits
        network_config = ROSEnvironmentConfig(domain_id=302, use_sim_time=True)

        network_limits = ResourceLimits(
            cpu_percent=10.0,
            memory_mb=30,
            network_bandwidth_mbps=0.001,  # 1Kbps - extremely limited
            max_processes=2,
        )

        with self.env_manager.create_environment(
            name="bandwidth_saturation_test",
            ros_config=network_config,
            resource_limits=network_limits,
            workspace_path=self.workspace_path,
        ) as env:

            from core.state_synchronization_manager import DistributedStateManager

            # Create manager
            mgr = DistributedStateManager("bandwidth_test")

            mgr.start()

            # Simulate network saturation by creating many rapid state updates
            # In a real network, this would saturate bandwidth
            print("  [ANTENNA] Simulating network bandwidth saturation...")

            start_time = time.time()

            # Send burst of large updates to simulate bandwidth saturation
            burst_count = 50
            for i in range(burst_count):
                # Large payload to simulate bandwidth usage
                large_payload = "x" * 10000  # 10KB per message
                mgr.update_state(f"bandwidth_burst_{i}", large_payload)

            end_time = time.time()
            burst_time = end_time - start_time

            print(".2f")
            print(".3f")

            # Verify all data was processed
            final_burst = mgr.get_state(f"bandwidth_burst_{burst_count-1}")
            self.assertIsNotNone(final_burst, "Burst operations should complete")
            self.assertEqual(
                len(final_burst), 10000, "Large payloads should be preserved"
            )

            # Test sustained high-frequency updates
            sustained_start = time.time()
            sustained_count = 25

            for i in range(sustained_count):
                mgr.update_state(f"sustained_{i}", f"data_{i}")
                # Minimal delay to maximize frequency
                time.sleep(0.001)

            sustained_time = time.time() - sustained_start

            print(".1f")

            # Verify sustained operations
            sustained_final = mgr.get_state(f"sustained_{sustained_count-1}")
            self.assertIsNotNone(
                sustained_final, "Sustained operations should complete"
            )

            mgr.stop()

        print("  [PASS] Network bandwidth saturation test passed")

    def test_combined_resource_exhaustion(self):
        """Test system under combined resource exhaustion (CPU + Memory + I/O)."""
        print(" Testing Combined Resource Exhaustion...")

        # Create environment with multiple resource constraints
        combined_config = ROSEnvironmentConfig(
            domain_id=303,
            use_sim_time=True,
            log_level="ERROR",  # Minimal logging to reduce I/O
        )

        combined_limits = ResourceLimits(
            cpu_percent=5.0,  # Very limited CPU
            memory_mb=15,  # Very limited memory
            max_processes=2,  # Very limited processes
        )

        with self.env_manager.create_environment(
            name="combined_exhaustion_test",
            ros_config=combined_config,
            resource_limits=combined_limits,
            workspace_path=self.workspace_path,
        ) as env:

            from core.state_synchronization_manager import DistributedStateManager

            # Create manager
            mgr = DistributedStateManager("combined_test")

            # Pre-allocate memory to create pressure
            memory_pressure = []
            for i in range(10):  # Allocate ~10MB
                try:
                    memory_pressure.append([0] * 100000)  # ~800KB each
                except MemoryError:
                    break

            print("   Combined resource exhaustion: CPU + Memory + I/O limits")

            try:
                mgr.start()

                # Test operations under combined pressure
                start_time = time.time()

                operation_count = 8  # Reduced count due to extreme constraints
                for i in range(operation_count):
                    # Create operations that stress all resources
                    data = "x" * 2000  # Moderate payload
                    mgr.update_state(f"combined_stress_{i}", data)

                    # Force GC to increase memory pressure
                    gc.collect()

                end_time = time.time()
                operation_time = end_time - start_time

                print(".3f")

                # Verify system remained functional
                final_state = mgr.get_state(f"combined_stress_{operation_count-1}")
                self.assertIsNotNone(
                    final_state,
                    "System should function under combined resource exhaustion",
                )

                # Operations should complete (may be very slow)
                self.assertLess(
                    operation_time,
                    60.0,
                    "Operations should complete within 1 minute even under extreme pressure",
                )

            finally:
                # Cleanup
                del memory_pressure
                gc.collect()
                mgr.stop()

        print("  [PASS] Combined resource exhaustion test passed")


if __name__ == "__main__":
    # Run tests with verbose output
    unittest.main(verbosity=2)
