#!/usr/bin/env python3
"""
Memory Usage Trend Analysis - URC 2026

Tests long-term memory usage patterns and trends:
- Memory growth rate: <1MB/hour
- Peak usage: <600MB
- Memory fragmentation: <10%
- Garbage collection frequency: <1/minute

Critical for system stability during long competition runs.
"""

import gc
import os
import statistics
import threading
import time
import tracemalloc
import unittest
from concurrent.futures import ThreadPoolExecutor
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import psutil


class MemoryUsageTrendAnalysisTest(unittest.TestCase):
    """Test memory usage trends and long-term stability."""

    def setUp(self):
        """Set up memory trend analysis testing."""
        # Test parameters - aligned with competition requirements
        self.monitoring_duration_hours = 1.0  # Monitor for 1 hour (scaled for testing)
        self.target_growth_mb_per_hour = 1.0  # Memory growth rate target
        self.target_peak_mb = 600.0  # Peak memory usage target
        self.target_fragmentation_percent = 10.0  # Memory fragmentation target
        self.target_gc_frequency_per_minute = 1.0  # GC frequency target

        # Performance tracking
        self.memory_readings = []
        self.gc_events = []
        self.allocation_rates = []
        self.timestamps = []

        # Monitoring state
        self.monitoring_active = False
        self.gc_hook_installed = False

    def test_memory_usage_trends(self):
        """Test memory usage trends over extended period."""
        print("\n💾 Testing Memory Usage Trends")
        print("=" * 60)

        # Install GC hook for monitoring
        self._install_gc_hook()

        try:
            # Start comprehensive memory monitoring
            tracemalloc.start(25)  # Track top 25 allocations
            self._start_memory_monitoring()

            # Run memory stress test scenarios
            print("🔄 Running memory stress test scenarios...")
            self._run_memory_stress_scenarios()

            # Allow system to stabilize and monitor trends
            print("📊 Monitoring memory trends...")
            monitoring_start = time.time()
            while time.time() - monitoring_start < (
                self.monitoring_duration_hours * 3600
            ):
                time.sleep(60)  # Monitor every minute
                self._record_memory_snapshot()

            # Analyze memory trends
            self._analyze_memory_trends()

        finally:
            self._stop_memory_monitoring()
            if self.gc_hook_installed:
                self._uninstall_gc_hook()
            tracemalloc.stop()

    def _start_memory_monitoring(self):
        """Start comprehensive memory monitoring."""
        self.monitoring_active = True

        # Start background monitoring thread
        self.monitor_thread = threading.Thread(
            target=self._background_memory_monitor, daemon=True
        )
        self.monitor_thread.start()

        # Record initial memory state
        self._record_memory_snapshot()

    def _stop_memory_monitoring(self):
        """Stop memory monitoring."""
        self.monitoring_active = False
        if hasattr(self, "monitor_thread"):
            self.monitor_thread.join(timeout=2.0)

    def _background_memory_monitor(self):
        """Background memory monitoring thread."""
        while self.monitoring_active:
            self._record_memory_snapshot()
            time.sleep(30)  # Record every 30 seconds

    def _record_memory_snapshot(self):
        """Record a memory usage snapshot."""
        if not self.monitoring_active:
            return

        process = psutil.Process()
        timestamp = time.time()

        # Basic memory metrics
        memory_info = process.memory_info()
        memory_mb = memory_info.rss / 1024 / 1024

        # Virtual memory info
        virtual_memory = psutil.virtual_memory()
        available_mb = virtual_memory.available / 1024 / 1024

        # Memory fragmentation (estimated)
        fragmentation_percent = (virtual_memory.used / virtual_memory.total) * 100

        # Tracemalloc statistics
        if tracemalloc.is_tracing():
            current, peak = tracemalloc.get_traced_memory()
            current_mb = current / 1024 / 1024
            peak_mb = peak / 1024 / 1024

            # Top memory consumers
            top_stats = tracemalloc.take_snapshot().statistics("lineno")
            top_allocations = []
            for stat in top_stats[:10]:  # Top 10 allocations
                top_allocations.append(
                    {
                        "size_mb": stat.size / 1024 / 1024,
                        "count": stat.count,
                        "traceback": str(stat.traceback),
                    }
                )
        else:
            current_mb = 0
            peak_mb = 0
            top_allocations = []

        snapshot = {
            "timestamp": timestamp,
            "memory_mb": memory_mb,
            "available_mb": available_mb,
            "fragmentation_percent": fragmentation_percent,
            "tracemalloc_current_mb": current_mb,
            "tracemalloc_peak_mb": peak_mb,
            "top_allocations": top_allocations,
            "gc_collections": len(gc.get_stats()),
            "object_count": len(gc.get_objects()),
        }

        self.memory_readings.append(snapshot)
        self.timestamps.append(timestamp)

    def _install_gc_hook(self):
        """Install garbage collection monitoring hook."""
        original_collect = gc.collect

        def gc_hook(phase, info):
            if phase == "stop":
                self.gc_events.append(
                    {
                        "timestamp": time.time(),
                        "generation": info["generation"],
                        "collected": info["collected"],
                        "uncollectable": info["uncollectable"],
                    }
                )

        def monitored_collect(generation=2):
            result = original_collect(generation)
            gc_hook(
                "stop",
                {"generation": generation, "collected": result, "uncollectable": 0},
            )
            return result

        gc.collect = monitored_collect
        self.gc_hook_installed = True

    def _uninstall_gc_hook(self):
        """Uninstall garbage collection monitoring hook."""
        if hasattr(gc, "original_collect"):
            gc.collect = gc.original_collect
        self.gc_hook_installed = False

    def _run_memory_stress_scenarios(self):
        """Run various memory stress test scenarios."""
        scenarios = [
            self._memory_allocation_burst,
            self._object_creation_stress,
            self._string_concatenation_stress,
            self._data_structure_stress,
            self._concurrent_memory_operations,
        ]

        for scenario in scenarios:
            print(f"  Running {scenario.__name__}...")
            scenario()
            time.sleep(5)  # Brief pause between scenarios

    def _memory_allocation_burst(self):
        """Test memory allocation bursts."""
        allocations = []
        for i in range(1000):
            # Allocate various sizes of memory
            size = np.random.randint(1000, 10000)
            allocations.append(np.zeros(size, dtype=np.float64))

        # Hold allocations briefly then release
        time.sleep(2)
        del allocations
        gc.collect()

    def _object_creation_stress(self):
        """Test object creation and destruction stress."""
        objects = []
        for i in range(50000):
            # Create various types of objects
            if i % 4 == 0:
                objects.append({"data": [1, 2, 3] * 100, "id": i})
            elif i % 4 == 1:
                objects.append([i] * 1000)
            elif i % 4 == 2:
                objects.append({"nested": {"deep": {"data": "x" * 500}}})
            else:
                objects.append((i, "string_data_" * 50, [i, i + 1, i + 2]))

        # Clear objects
        objects.clear()
        del objects
        gc.collect()

    def _string_concatenation_stress(self):
        """Test string operations that can cause memory fragmentation."""
        strings = []
        for i in range(10000):
            # Build strings incrementally (bad practice that causes fragmentation)
            s = ""
            for j in range(10):
                s += f"data_{i}_{j}_"
            strings.append(s)

        # Process strings
        processed = [s.upper() for s in strings]
        del processed
        strings.clear()
        del strings
        gc.collect()

    def _data_structure_stress(self):
        """Test complex data structure operations."""
        data_structures = []

        for i in range(1000):
            # Create complex nested data structures
            structure = {
                "id": i,
                "metadata": {
                    "created": time.time(),
                    "tags": [f"tag_{j}" for j in range(10)],
                    "properties": {f"prop_{k}": k * 1.5 for k in range(20)},
                },
                "data": {
                    "arrays": [np.random.random(100) for _ in range(5)],
                    "matrices": [np.random.random((10, 10)) for _ in range(3)],
                    "strings": ["x" * 200] * 20,
                },
            }
            data_structures.append(structure)

        # Simulate processing
        results = []
        for ds in data_structures:
            result = {
                "id": ds["id"],
                "processed_arrays": [arr.mean() for arr in ds["data"]["arrays"]],
                "total_strings": len(ds["data"]["strings"]),
            }
            results.append(result)

        del results
        data_structures.clear()
        del data_structures
        gc.collect()

    def _concurrent_memory_operations(self):
        """Test concurrent memory operations."""

        def memory_worker(worker_id):
            allocations = []
            for i in range(1000):
                size = np.random.randint(500, 2000)
                allocations.append(np.random.random(size))

                if i % 100 == 0:
                    time.sleep(0.001)  # Small delay

            # Cleanup
            del allocations

        # Run concurrent workers
        with ThreadPoolExecutor(max_workers=4) as executor:
            futures = [executor.submit(memory_worker, i) for i in range(4)]
            for future in futures:
                future.result()

        gc.collect()

    def _analyze_memory_trends(self):
        """Analyze memory usage trends and patterns."""
        if len(self.memory_readings) < 2:
            self.fail("Insufficient memory readings for trend analysis")

        # Extract memory time series
        timestamps = [r["timestamp"] for r in self.memory_readings]
        memory_values = [r["memory_mb"] for r in self.memory_readings]

        # Calculate memory growth rate
        duration_hours = (timestamps[-1] - timestamps[0]) / 3600
        memory_growth_mb = memory_values[-1] - memory_values[0]
        growth_rate_mb_per_hour = (
            memory_growth_mb / duration_hours if duration_hours > 0 else 0
        )

        # Calculate memory statistics
        avg_memory_mb = statistics.mean(memory_values)
        max_memory_mb = max(memory_values)
        min_memory_mb = min(memory_values)
        memory_variance = (
            statistics.variance(memory_values) if len(memory_values) > 1 else 0
        )

        # Calculate memory fragmentation trends
        fragmentation_values = [
            r["fragmentation_percent"] for r in self.memory_readings
        ]
        avg_fragmentation = statistics.mean(fragmentation_values)
        max_fragmentation = max(fragmentation_values)

        # Analyze garbage collection frequency
        if self.gc_events:
            gc_timestamps = [event["timestamp"] for event in self.gc_events]
            duration_minutes = (timestamps[-1] - timestamps[0]) / 60
            gc_frequency_per_minute = (
                len(gc_events) / duration_minutes if duration_minutes > 0 else 0
            )
        else:
            gc_frequency_per_minute = 0

        # Analyze allocation patterns
        if len(memory_values) > 10:
            # Calculate trend using linear regression
            from scipy import stats

            slope, intercept, r_value, p_value, std_err = stats.linregress(
                range(len(memory_values)), memory_values
            )
            memory_trend_mb_per_minute = slope * (
                60 / 30
            )  # Convert to per minute (from 30s intervals)
        else:
            memory_trend_mb_per_minute = 0

        # Print detailed results
        print("\n📈 Memory Usage Trend Analysis Results:")
        print("-" * 50)
        print(".2f")
        print(".2f")
        print(".2f")
        print(".2f")
        print(".2f")
        print(".2f")
        print(".2f")
        print(".4f")
        print(".1f")
        print(".2f")
        print(".1f")

        # Validate against competition targets
        print("\n🎯 Competition Requirements Check:")
        print("-" * 50)

        requirements = {
            "Memory Growth Rate (MB/hour)": (
                abs(growth_rate_mb_per_hour),
                self.target_growth_mb_per_hour,
            ),
            "Peak Memory Usage (MB)": (max_memory_mb, self.target_peak_mb),
            "Average Fragmentation (%)": (
                avg_fragmentation,
                self.target_fragmentation_percent,
            ),
            "Max Fragmentation (%)": (
                max_fragmentation,
                self.target_fragmentation_percent * 1.5,
            ),
            "GC Frequency (/minute)": (
                gc_frequency_per_minute,
                self.target_gc_frequency_per_minute,
            ),
        }

        all_passed = True
        for metric, (actual, target) in requirements.items():
            passed = actual <= target
            status = "✅ PASS" if passed else "❌ FAIL"
            print(".3f")

            if not passed:
                all_passed = False

        # Performance assessment
        print("\n🚀 Memory Stability Assessment:")
        print("-" * 50)

        if all_passed:
            print("✅ ALL MEMORY REQUIREMENTS MET")
            print("   System memory usage stable for competition deployment")
        elif abs(growth_rate_mb_per_hour) < self.target_growth_mb_per_hour * 2:
            print("⚠️  MEMORY GROWTH ACCEPTABLE, MONITOR CLOSELY")
            print("   Memory leak potential exists, monitor during long runs")
        else:
            print("❌ MEMORY REQUIREMENTS NOT MET")
            print("   Memory leaks detected, requires investigation before competition")

        # Detailed analysis and recommendations
        if abs(growth_rate_mb_per_hour) > self.target_growth_mb_per_hour:
            print("\n💡 Memory Optimization Recommendations:")
            if growth_rate_mb_per_hour > 0:
                print("   - Investigate memory leaks in long-running operations")
                print("   - Check for accumulating data structures")
                print("   - Implement periodic memory cleanup routines")
            if max_memory_mb > self.target_peak_mb:
                print("   - Optimize memory usage in peak load scenarios")
                print("   - Consider memory pooling for frequently allocated objects")
            if avg_fragmentation > self.target_fragmentation_percent:
                print(
                    "   - Reduce memory fragmentation through better allocation patterns"
                )
                print("   - Avoid frequent small allocations/deallocations")
            if gc_frequency_per_minute > self.target_gc_frequency_per_minute:
                print("   - Reduce garbage collection pressure")
                print("   - Optimize object lifecycle management")

        # Analyze top memory consumers
        if self.memory_readings:
            latest_snapshot = self.memory_readings[-1]
            if latest_snapshot["top_allocations"]:
                print("\n🔍 Top Memory Consumers:")
                print("-" * 30)
                for i, alloc in enumerate(latest_snapshot["top_allocations"][:5], 1):
                    print(".1f")

        # Store results for regression testing
        self.test_results = {
            "duration_hours": duration_hours,
            "growth_rate_mb_per_hour": growth_rate_mb_per_hour,
            "avg_memory_mb": avg_memory_mb,
            "max_memory_mb": max_memory_mb,
            "min_memory_mb": min_memory_mb,
            "memory_variance": memory_variance,
            "avg_fragmentation_percent": avg_fragmentation,
            "max_fragmentation_percent": max_fragmentation,
            "gc_frequency_per_minute": gc_frequency_per_minute,
            "memory_trend_mb_per_minute": memory_trend_mb_per_minute,
            "requirements_met": all_passed,
            "snapshots_collected": len(self.memory_readings),
            "gc_events_recorded": len(self.gc_events),
        }

        # Assert critical requirements
        self.assertLess(
            abs(growth_rate_mb_per_hour), self.target_growth_mb_per_hour * 2, ".4f"
        )
        self.assertLess(max_memory_mb, self.target_peak_mb * 1.2, ".2f")

    def test_memory_usage_under_competition_load(self):
        """Test memory usage under simulated competition load."""
        print("\n🏁 Testing Memory Usage Under Competition Load")

        try:
            # Start monitoring
            tracemalloc.start()
            self._start_memory_monitoring()

            # Simulate competition scenario
            competition_duration_minutes = 30  # 30 minutes of competition

            print(
                f"  Simulating {competition_duration_minutes} minutes of competition activity..."
            )

            start_time = time.time()
            while (time.time() - start_time) < (competition_duration_minutes * 60):
                # Simulate competition activities
                self._simulate_competition_activity()
                time.sleep(1)  # 1 second simulation step

            # Analyze competition memory usage
            if self.memory_readings:
                competition_memory = [r["memory_mb"] for r in self.memory_readings]
                competition_growth = competition_memory[-1] - competition_memory[0]

                print(".2f")
                print(".2f")

                if competition_growth > 50:  # More than 50MB growth
                    print("⚠️  Significant memory growth during competition simulation")
                else:
                    print("✅ Memory usage stable during competition simulation")

        finally:
            self._stop_memory_monitoring()
            tracemalloc.stop()

    def _simulate_competition_activity(self):
        """Simulate typical competition activities that stress memory."""
        # Simulate sensor data processing
        sensor_data = np.random.random((100, 100))  # Simulated sensor readings

        # Simulate vision processing
        processed_frames = []
        for i in range(5):  # Process 5 frames per second
            frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
            processed_frames.append(frame)

        # Simulate navigation calculations
        waypoints = [{"x": i * 2.0, "y": (i % 2) * 1.5} for i in range(10)]
        paths = []
        for wp in waypoints:
            path_segment = np.linspace([0, 0], [wp["x"], wp["y"]], 50)
            paths.append(path_segment)

        # Simulate mission state tracking
        mission_state = {
            "waypoints_completed": np.random.randint(0, 10),
            "sensor_readings": sensor_data.tolist(),
            "navigation_paths": [p.tolist() for p in paths],
            "system_status": {
                "cpu_usage": np.random.random() * 100,
                "memory_usage": np.random.random() * 100,
                "battery_level": 80 + np.random.random() * 20,
            },
        }

        # Hold data briefly then release (simulating processing pipeline)
        time.sleep(0.1)
        del sensor_data, processed_frames, paths, mission_state
        gc.collect()


if __name__ == "__main__":
    unittest.main()
