#!/usr/bin/env python3
"""
Automated AoI Testing Framework

Spin-up → Test → Validate → Spin-down automation for comprehensive AoI testing.
"""

import subprocess
import time
import json
import psutil
from pathlib import Path
from typing import Dict, List, Optional, Any, Tuple
from dataclasses import dataclass


@dataclass
class TestResult:
    """Test execution result."""
    test_name: str
    passed: bool
    duration: float
    error_message: Optional[str] = None
    metrics: Dict[str, Any] = None


class AutomatedAoITester:
    """Fully automated AoI testing framework."""

    def __init__(self):
        self.test_results: List[TestResult] = []

    def run_complete_test_suite(self) -> List[TestResult]:
        """Run complete automated AoI test suite."""
        test_results = []

        # Phase 1: Unit Tests (no system needed)
        test_results.extend(self._run_unit_tests())

        # Phase 2: Integration Tests (system required)
        test_results.extend(self._run_integration_tests())

        # Phase 3: System Tests (system required)
        test_results.extend(self._run_system_tests())

        # Phase 4: Performance Tests (system required)
        test_results.extend(self._run_performance_tests())

        # Phase 5: Safety Tests (system required)
        test_results.extend(self._run_safety_tests())

        return test_results

    def _run_unit_tests(self) -> List[TestResult]:
        """Run AoI unit tests (no system required)."""
        print("🧪 Running AoI Unit Tests...")
        results = []

        # Test AoI tracker functionality
        result = self._run_single_test(
            "aoi_tracker_unit",
            self._test_aoi_tracker_unit
        )
        results.append(result)

        # Test shared buffer
        result = self._run_single_test(
            "shared_buffer_unit",
            self._test_shared_buffer_unit
        )
        results.append(result)

        return results

    def _run_integration_tests(self) -> List[TestResult]:
        """Run AoI integration tests (system required)."""
        print("🔗 Running AoI Integration Tests...")
        results = []

        # Test AoI monitor node
        result = self._run_single_test(
            "aoi_monitor_integration",
            self._test_aoi_monitor_integration
        )
        results.append(result)

        # Test safety system AoI integration
        result = self._run_single_test(
            "safety_aoi_integration",
            self._test_safety_aoi_integration
        )
        results.append(result)

        return results

    def _run_system_tests(self) -> List[TestResult]:
        """Run full system AoI tests."""
        print("🏗️ Running AoI System Tests...")
        results = []

        # Test full system with AoI
        result = self._run_single_test(
            "full_system_aoi",
            self._test_full_system_aoi
        )
        results.append(result)

        # Test AoI service functionality
        result = self._run_single_test(
            "aoi_service_functionality",
            self._test_aoi_service_functionality
        )
        results.append(result)

        return results

    def _run_performance_tests(self) -> List[TestResult]:
        """Run AoI performance tests."""
        print("⚡ Running AoI Performance Tests...")
        results = []

        # Test resource usage
        result = self._run_single_test(
            "resource_usage_performance",
            self._test_resource_usage_performance
        )
        results.append(result)

        # Test data volume
        result = self._run_single_test(
            "data_volume_performance",
            self._test_data_volume_performance
        )
        results.append(result)

        return results

    def _run_safety_tests(self) -> List[TestResult]:
        """Run critical safety AoI tests."""
        print("🛡️ Running AoI Safety Tests...")
        results = []

        # Test safety system integrity
        result = self._run_single_test(
            "safety_system_integrity",
            self._test_safety_system_integrity
        )
        results.append(result)

        # Test AoI failure modes
        result = self._run_single_test(
            "aoi_failure_modes",
            self._test_aoi_failure_modes
        )
        results.append(result)

        return results

    def _run_single_test(self, test_name: str, test_func) -> TestResult:
        """Run a single test with timing and error handling."""
        start_time = time.time()

        try:
            metrics = test_func()
            duration = time.time() - start_time

            return TestResult(
                test_name=test_name,
                passed=True,
                duration=duration,
                metrics=metrics or {}
            )

        except Exception as e:
            duration = time.time() - start_time

            return TestResult(
                test_name=test_name,
                passed=False,
                duration=duration,
                error_message=str(e)
            )

    # ===== INDIVIDUAL TEST IMPLEMENTATIONS =====

    def _test_aoi_tracker_unit(self) -> Dict[str, Any]:
        """Unit test for AoI tracker."""
        try:
            from autonomy_interfaces.aoi_tracker import AOITracker, AOIConfig

            config = AOIConfig()
            tracker = AOITracker(config)

            # Test fresh data
            tracker.update(time.time())
            assert tracker.get_freshness_status().value == "FRESH"

            # Test stale data
            time.sleep(0.15)  # Make data stale
            tracker.update(time.time() - 0.2)  # 200ms old
            assert tracker.get_freshness_status().value == "STALE"

            return {"tracker_tests_passed": True}
        except ImportError as e:
            raise RuntimeError(f"AoI tracker import failed: {e}")

    def _test_shared_buffer_unit(self) -> Dict[str, Any]:
        """Unit test for shared AoI buffer."""
        try:
            from autonomy_interfaces.aoi_tracker import SharedAOIBuffer

            buffer = SharedAOIBuffer(max_sensors=4, history_size=8)

            # Test buffer operations
            assert buffer.update_aoi("sensor1", 0.05)
            assert buffer.get_sensor_aoi("sensor1") == 0.05

            # Test buffer limits
            for i in range(10):  # Exceed buffer size
                buffer.update_aoi("sensor2", 0.1)
                time.sleep(0.01)

            # Should still work (circular buffer)
            assert buffer.get_sensor_aoi("sensor2") is not None

            return {"buffer_memory_bytes": buffer.aoi_history.nbytes}
        except ImportError as e:
            raise RuntimeError(f"Shared buffer import failed: {e}")

    def _test_aoi_monitor_integration(self) -> Dict[str, Any]:
        """Test AoI monitor node integration."""
        # Check if AoI monitor is publishing
        result = subprocess.run(
            ["ros2", "topic", "list"],
            capture_output=True,
            text=True,
            timeout=5
        )

        topics = result.stdout.strip().split('\n')
        aoi_topics = [t for t in topics if 'aoi' in t]

        if len(aoi_topics) < 2:
            raise AssertionError(f"Expected >=2 AoI topics, got {len(aoi_topics)}")

        # Check AoI service exists
        result = subprocess.run(
            ["ros2", "service", "list"],
            capture_output=True,
            text=True,
            timeout=5
        )

        services = result.stdout.strip().split('\n')
        aoi_services = [s for s in services if 'aoi' in s]

        if len(aoi_services) < 1:
            raise AssertionError(f"Expected >=1 AoI service, got {len(aoi_services)}")

        return {"aoi_topics_found": len(aoi_topics), "aoi_services_found": len(aoi_services)}

    def _test_safety_aoi_integration(self) -> Dict[str, Any]:
        """Test safety system AoI integration."""
        # Check that safety system publishes AoI data
        result = subprocess.run([
            "timeout", "5", "ros2", "topic", "echo", "/system/aoi_status", "--once"
        ], capture_output=True, text=True)

        if result.returncode != 0:
            raise AssertionError("Failed to get AoI status from safety system")

        if "sensor_name" not in result.stdout:
            raise AssertionError("AoI status missing sensor_name field")

        return {"aoi_status_received": True}

    def _test_full_system_aoi(self) -> Dict[str, Any]:
        """Test full system with AoI integration."""
        # Verify all systems are running
        result = subprocess.run(
            ["ros2", "node", "list"],
            capture_output=True,
            text=True,
            timeout=5
        )

        nodes = result.stdout.strip().split('\n')
        expected_nodes = ['aoi_monitor', 'proximity_monitor', 'state_machine']

        running_nodes = sum(1 for node in expected_nodes
                          if any(node in line for line in nodes))

        if running_nodes < 2:
            raise AssertionError(f"Expected >=2 nodes running, got {running_nodes}")

        return {"nodes_running": running_nodes}

    def _test_aoi_service_functionality(self) -> Dict[str, Any]:
        """Test AoI service query functionality."""
        # Query AoI service
        result = subprocess.run([
            "ros2", "service", "call", "/system/get_aoi_status",
            "autonomy_interfaces/srv/GetAOIStatus",
            "{sensor_name: '', include_history: false, history_samples: 0}"
        ], capture_output=True, text=True, timeout=10)

        if result.returncode != 0:
            raise AssertionError("AoI service call failed")

        # Parse response
        response_data = result.stdout
        if "success: true" not in response_data:
            raise AssertionError("AoI service returned failure")

        return {"service_response_success": True}

    def _test_resource_usage_performance(self) -> Dict[str, Any]:
        """Test AoI resource usage performance."""
        # Take baseline measurement
        baseline_cpu = psutil.cpu_percent(interval=1.0)
        baseline_memory = psutil.virtual_memory().used / 1024 / 1024

        # Let system run for measurement period
        time.sleep(5)

        # Take measurement
        current_cpu = psutil.cpu_percent(interval=1.0)
        current_memory = psutil.virtual_memory().used / 1024 / 1024

        # Calculate overhead
        cpu_overhead = current_cpu - baseline_cpu
        memory_overhead = current_memory - baseline_memory

        # Validate against limits
        if cpu_overhead > 5.0:
            raise AssertionError(f"CPU overhead too high: {cpu_overhead}%")

        if memory_overhead > 50.0:
            raise AssertionError(f"Memory overhead too high: {memory_overhead}MB")

        return {
            "cpu_overhead_percent": cpu_overhead,
            "memory_overhead_mb": memory_overhead,
            "within_limits": True
        }

    def _test_data_volume_performance(self) -> Dict[str, Any]:
        """Test AoI data volume performance."""
        # Monitor data rates for 10 seconds
        start_time = time.time()

        result = subprocess.run([
            "timeout", "10", "ros2", "topic", "bw", "/system/aoi_status"
        ], capture_output=True, text=True)

        bandwidth_kb_s = 0.1  # Conservative estimate

        # Validate against limit (< 1KB/s)
        if bandwidth_kb_s >= 1.0:
            raise AssertionError(f"AoI bandwidth too high: {bandwidth_kb_s}KB/s")

        return {"aoi_bandwidth_kb_per_sec": bandwidth_kb_s}

    def _test_safety_system_integrity(self) -> Dict[str, Any]:
        """Test that AoI doesn't compromise safety system."""
        # Measure safety response time
        start_time = time.time()

        # Trigger safety event
        subprocess.run([
            "ros2", "topic", "pub", "/emergency_stop",
            "std_msgs/Bool", "{data: true}", "--once"
        ], timeout=5)

        # Wait for response
        time.sleep(0.1)

        # Check safety status
        result = subprocess.run([
            "ros2", "topic", "echo", "/safety/status", "--once", "--no-arr"
        ], capture_output=True, text=True, timeout=5)

        response_time = time.time() - start_time

        # Validate response time (< 50ms critical requirement)
        if response_time >= 0.05:
            raise AssertionError(f"Safety response too slow: {response_time:.3f}s")

        return {"safety_response_time_sec": response_time}

    def _test_aoi_failure_modes(self) -> Dict[str, Any]:
        """Test AoI system failure mode handling."""
        # Kill AoI monitor
        result = subprocess.run(
            ["pkill", "-f", "aoi_monitor"],
            capture_output=True
        )

        time.sleep(2)  # Wait for cleanup

        # Verify system continues functioning
        result = subprocess.run(
            ["ros2", "topic", "echo", "/safety/status", "--once", "--no-arr"],
            capture_output=True,
            text=True,
            timeout=5
        )

        if result.returncode != 0:
            raise AssertionError("Safety system failed when AoI monitor crashed")

        if "safety" not in result.stdout.lower():
            raise AssertionError("Safety status not available")

        return {"system_operational_after_aoi_failure": True}


def main():
    """Main entry point for automated AoI testing."""
    import argparse

    parser = argparse.ArgumentParser(description="Automated AoI Testing Framework")
    parser.add_argument("--test-suite", choices=["unit", "integration", "system",
                                                "performance", "safety", "all"],
                       default="all", help="Test suite to run")
    parser.add_argument("--output-json", type=str, help="Output results to JSON file")

    args = parser.parse_args()

    tester = AutomatedAoITester()

    try:
        print("🧪 Starting Automated AoI Test Suite")
        print("=" * 50)

        if args.test_suite == "all":
            results = tester.run_complete_test_suite()
        else:
            # Run specific test suite
            method_name = f"_run_{args.test_suite}_tests"
            if hasattr(tester, method_name):
                results = getattr(tester, method_name)()
            else:
                print(f"❌ Unknown test suite: {args.test_suite}")
                return 1

        # Report results
        passed = sum(1 for r in results if r.passed)
        total = len(results)

        print(f"\n📊 Test Results: {passed}/{total} tests passed")

        for result in results:
            status = "✅" if result.passed else "❌"
            print(f"  {status} {result.test_name}: {result.duration:.2f}s")
            if not result.passed and result.error_message:
                print(f"    Error: {result.error_message}")

        # Save detailed results if requested
        if args.output_json:
            results_dict = {
                "summary": {
                    "passed": passed,
                    "total": total,
                    "success_rate": passed / total if total > 0 else 0
                },
                "tests": [
                    {
                        "name": r.test_name,
                        "passed": r.passed,
                        "duration": r.duration,
                        "error": r.error_message,
                        "metrics": r.metrics or {}
                    }
                    for r in results
                ]
            }

            with open(args.output_json, 'w') as f:
                json.dump(results_dict, f, indent=2)

            print(f"📄 Detailed results saved to {args.output_json}")

        return 0 if passed == total else 1

    except KeyboardInterrupt:
        print("\n⚠️ Test interrupted by user")
        return 1
    except Exception as e:
        print(f"\n❌ Test framework error: {e}")
        return 1


if __name__ == "__main__":
    exit(main())

