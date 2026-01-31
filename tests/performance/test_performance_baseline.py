#!/usr/bin/env python3
"""
Performance Baseline Tests - URC 2026 Mars Rover

Comprehensive performance testing suite for validating communication architecture
against critical requirements. Tests deterministic latency, network resilience,
and system resource usage.
"""

import pytest
import time
import statistics
import threading
from typing import Dict, List, Any
from dataclasses import dataclass
import json

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

try:
    from src.comms.binary_sensor_protocol import BinarySensorProtocol, IMUData
    from src.sensors.timestamp_provider import get_timestamp_provider, SensorType
    from src.motion.ipc_motion_bridge import (
        create_motion_bridge_server,
        create_motion_bridge_client,
        VelocityCommand,
    )
    from src.comms.network_partition_detector import NetworkPartitionDetector
    from src.comms.adaptive_circuit_breaker import get_adaptive_circuit_breaker
    from src.testing.performance_profiling import get_performance_profiler, PerformanceProfiler
    _PERF_DEPS_AVAILABLE = True
except ImportError as e:
    _PERF_DEPS_AVAILABLE = False
    _PERF_IMPORT_ERROR = e

if not _PERF_DEPS_AVAILABLE:
    pytest.skip(
        f"Performance baseline deps unavailable: {_PERF_IMPORT_ERROR}",
        allow_module_level=True,
    )


@dataclass
class PerformanceRequirement:
    """Performance requirement specification."""
    name: str
    description: str
    max_p99_ms: float
    max_violation_rate_percent: float = 0.0
    enabled: bool = True


class PerformanceBaselineTester:
    """Comprehensive performance baseline tester."""

    REQUIREMENTS = [
        PerformanceRequirement(
            name="binary_protocol",
            description="Binary protocol serialization/deserialization",
            max_p99_ms=1.0
        ),
        PerformanceRequirement(
            name="sensor_timestamps",
            description="Sensor timestamp tagging with latency correction",
            max_p99_ms=1.0
        ),
        PerformanceRequirement(
            name="ipc_motion_bridge",
            description="IPC motion control bridge roundtrip",
            max_p99_ms=5.0
        ),
        PerformanceRequirement(
            name="motion_control_loop",
            description="Complete motion control loop (50Hz)",
            max_p99_ms=20.0,
            max_violation_rate_percent=1.0
        ),
        PerformanceRequirement(
            name="end_to_end_pipeline",
            description="Complete sensor pipeline: raw → timestamped → binary",
            max_p99_ms=5.0
        )
    ]

    def __init__(self):
        self.profiler = PerformanceProfiler()
        self.results = {}
        self.test_bridge_name = f"perf_test_{int(time.time() * 1000)}"

    def run_all_baselines(self) -> Dict[str, Any]:
        """Run all performance baseline tests."""
        print("🧪 Running URC 2026 Performance Baseline Tests")

        self.results = {}

        # Core component tests
        self.results['binary_protocol'] = self.test_binary_protocol_baseline()
        self.results['sensor_timestamps'] = self.test_sensor_timestamp_baseline()
        self.results['ipc_bridge'] = self.test_ipc_bridge_baseline()
        self.results['motion_control'] = self.test_motion_control_baseline()
        self.results['end_to_end'] = self.test_end_to_end_pipeline()

        # Stress tests
        self.results['stress_tests'] = self.run_stress_tests()

        # Generate comprehensive report
        report = self.generate_performance_report()
        return report

    def test_binary_protocol_baseline(self) -> Dict[str, Any]:
        """Test binary protocol performance baseline."""
        print("  📊 Testing Binary Protocol Baseline...")

        imu_data = IMUData(
            measurement_timestamp_ns=int(time.time_ns()),
            reception_timestamp_ns=int(time.time_ns()),
            accel_x=0.1, accel_y=0.2, accel_z=9.8,
            gyro_x=0.01, gyro_y=0.02, gyro_z=0.03,
            orientation_x=0.0, orientation_y=0.0, orientation_z=0.0, orientation_w=1.0
        )

        iterations = 5000
        latencies = []

        # Test encoding/decoding cycle
        for _ in range(iterations):
            start = time.perf_counter()
            binary_msg = BinarySensorProtocol.encode_imu(imu_data)
            decoded = BinarySensorProtocol.decode_imu(binary_msg)
            end = time.perf_counter()

            assert decoded is not None
            latencies.append((end - start) * 1000)

        # Calculate statistics
        latencies.sort()
        result = self._calculate_statistics(latencies, iterations)

        # Compare with JSON baseline
        json_latencies = self._benchmark_json_protocol(imu_data, iterations)
        result['json_comparison'] = {
            'json_p99_ms': sorted(json_latencies)[int(len(json_latencies) * 0.99)],
            'improvement_factor': sorted(json_latencies)[int(len(json_latencies) * 0.99)] / result['p99_ms']
        }

        print(f"Binary protocol p99 latency: {result['p99_ms']:.3f}ms")
        return result

    def _benchmark_json_protocol(self, imu_data: IMUData, iterations: int) -> List[float]:
        """Benchmark JSON protocol for comparison."""
        import json

        json_data = {
            "measurement_timestamp_ns": imu_data.measurement_timestamp_ns,
            "reception_timestamp_ns": imu_data.reception_timestamp_ns,
            "accel": [imu_data.accel_x, imu_data.accel_y, imu_data.accel_z],
            "gyro": [imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z],
            "orientation": [imu_data.orientation_x, imu_data.orientation_y,
                           imu_data.orientation_z, imu_data.orientation_w]
        }

        latencies = []
        for _ in range(iterations):
            start = time.perf_counter()
            json_str = json.dumps(json_data).encode('utf-8')
            decoded = json.loads(json_str.decode('utf-8'))
            end = time.perf_counter()
            latencies.append((end - start) * 1000)

        return latencies

    def test_sensor_timestamp_baseline(self) -> Dict[str, Any]:
        """Test sensor timestamp provider performance."""
        print("  📊 Testing Sensor Timestamp Baseline...")

        timestamp_provider = get_timestamp_provider()
        iterations = 3000
        latencies = []

        for _ in range(iterations):
            raw_data = {"accel_x": 0.1, "accel_y": 0.2, "accel_z": 9.8}

            start = time.perf_counter()
            timestamped = timestamp_provider.tag_sensor_data(SensorType.IMU, raw_data)
            end = time.perf_counter()

            assert timestamped.measurement_timestamp_ns > 0
            assert timestamped.quality_score > 0
            latencies.append((end - start) * 1000)

        result = self._calculate_statistics(latencies, iterations)
        print(f"Sensor timestamp p99 latency: {result['p99_ms']:.3f}ms")
        return result

    def test_ipc_bridge_baseline(self) -> Dict[str, Any]:
        """Test IPC motion bridge performance."""
        print("  📊 Testing IPC Bridge Baseline...")

        server = create_motion_bridge_server(f"{self.test_bridge_name}_baseline")
        client = create_motion_bridge_client(f"{self.test_bridge_name}_baseline")

        try:
            iterations = 1000
            latencies = []

            for i in range(iterations):
                cmd = VelocityCommand(
                    linear_x=0.5 + i * 0.001,
                    angular_z=0.1 + i * 0.0005
                )

                start = time.perf_counter()
                success = client.send_velocity_command(cmd)
                assert success

                result = server.read_pending_command()
                assert result is not None
                end = time.perf_counter()

                latencies.append((end - start) * 1000)

            result = self._calculate_statistics(latencies, iterations)
            print(f"IPC bridge p99 latency: {result['p99_ms']:.3f}ms")
            return result

        finally:
            server.cleanup()
            client.cleanup()

    def test_motion_control_baseline(self) -> Dict[str, Any]:
        """Test motion control loop performance."""
        print("  📊 Testing Motion Control Baseline...")

        server = create_motion_bridge_server(f"{self.test_bridge_name}_motion")
        client = create_motion_bridge_client(f"{self.test_bridge_name}_motion")

        try:
            target_period_ms = 20.0  # 50Hz
            iterations = 500
            violations = 0
            latencies = []

            start_time = time.time()

            for i in range(iterations):
                loop_start = time.perf_counter()

                # Send command
                cmd = VelocityCommand(linear_x=0.5, angular_z=0.1)
                client.send_velocity_command(cmd)

                # Read response
                result = server.read_pending_command()
                assert result is not None

                # Simulate minimal processing
                for _ in range(100):  # Light computation
                    pass

                loop_end = time.perf_counter()
                loop_time_ms = (loop_end - loop_start) * 1000
                latencies.append(loop_time_ms)

                if loop_time_ms > target_period_ms:
                    violations += 1

                # Maintain timing
                elapsed = time.time() - start_time
                expected = (i + 1) * (target_period_ms / 1000)
                if elapsed < expected:
                    time.sleep(expected - elapsed)

            result = self._calculate_statistics(latencies, iterations)
            result['deadline_violations'] = violations
            result['violation_rate_percent'] = (violations / iterations) * 100
            result['target_period_ms'] = target_period_ms

            print(f"Motion control p99 latency: {result['p99_ms']:.3f}ms, violations: {result['violation_rate_percent']:.1f}%")
            return result

        finally:
            server.cleanup()
            client.cleanup()

    def test_end_to_end_pipeline(self) -> Dict[str, Any]:
        """Test complete sensor data pipeline."""
        print("  📊 Testing End-to-End Pipeline...")

        timestamp_provider = get_timestamp_provider()
        iterations = 2000
        latencies = []

        for i in range(iterations):
            # Raw sensor data
            raw_data = {
                "accel_x": 0.1 + i * 0.001,
                "accel_y": 0.2 + i * 0.0005,
                "accel_z": 9.8
            }

            start = time.perf_counter()

            # 1. Add timestamps
            timestamped = timestamp_provider.tag_sensor_data(SensorType.IMU, raw_data)

            # 2. Convert to IMU data structure
            imu_data = IMUData(
                measurement_timestamp_ns=timestamped.measurement_timestamp_ns,
                reception_timestamp_ns=timestamped.reception_timestamp_ns,
                accel_x=raw_data["accel_x"], accel_y=raw_data["accel_y"], accel_z=raw_data["accel_z"],
                gyro_x=0.01, gyro_y=0.02, gyro_z=0.03
            )

            # 3. Encode to binary
            binary_msg = BinarySensorProtocol.encode_imu(imu_data, timestamped.sequence_number)

            # 4. Decode (simulating receiver)
            decoded = BinarySensorProtocol.decode_imu(binary_msg)

            end = time.perf_counter()

            assert decoded is not None
            assert abs(decoded.accel_x - raw_data["accel_x"]) < 1e-6
            latencies.append((end - start) * 1000)

        result = self._calculate_statistics(latencies, iterations)
        print(f"End-to-end pipeline p99 latency: {result['p99_ms']:.3f}ms")
        return result

    def run_stress_tests(self) -> Dict[str, Any]:
        """Run comprehensive stress tests."""
        print("  🏋️ Running Stress Tests...")

        stress_results = {}

        # IPC bridge stress test
        stress_results['ipc_bridge_concurrent'] = self._stress_test_ipc_bridge_concurrent()

        # Memory leak test
        stress_results['memory_stress'] = self._stress_test_memory_usage()

        # High frequency test
        stress_results['high_frequency'] = self._stress_test_high_frequency()

        return stress_results

    def _stress_test_ipc_bridge_concurrent(self) -> Dict[str, Any]:
        """Stress test IPC bridge with concurrent operations."""
        server = create_motion_bridge_server(f"{self.test_bridge_name}_stress")
        client = create_motion_bridge_client(f"{self.test_bridge_name}_stress")

        try:
            num_threads = 4
            operations_per_thread = 500
            results = []

            def client_worker(thread_id):
                """Worker thread for concurrent operations."""
                thread_results = []
                for i in range(operations_per_thread):
                    cmd = VelocityCommand(
                        linear_x=0.1 * thread_id,
                        angular_z=0.05 * thread_id
                    )

                    start = time.perf_counter()
                    success = client.send_velocity_command(cmd)
                    if success:
                        result = server.read_pending_command()
                        end = time.perf_counter()

                        if result:
                            thread_results.append((end - start) * 1000)

                results.extend(thread_results)

            # Start concurrent threads
            threads = []
            for i in range(num_threads):
                thread = threading.Thread(target=client_worker, args=(i+1,))
                threads.append(thread)
                thread.start()

            # Wait for completion
            for thread in threads:
                thread.join(timeout=30.0)

            if results:
                results.sort()
                return {
                    'concurrent_operations': len(results),
                    'p99_latency_ms': results[int(len(results) * 0.99)],
                    'mean_latency_ms': statistics.mean(results),
                    'success': True
                }
            else:
                return {'success': False, 'error': 'No operations completed'}

        finally:
            server.cleanup()
            client.cleanup()

    def _stress_test_memory_usage(self) -> Dict[str, Any]:
        """Stress test for memory leaks and usage."""
        import psutil
        import os

        process = psutil.Process(os.getpid())
        initial_memory = process.memory_info().rss / (1024 * 1024)  # MB

        # Perform many operations
        timestamp_provider = get_timestamp_provider()

        for i in range(10000):
            raw_data = {"accel_x": 0.1, "accel_y": 0.2, "accel_z": 9.8}
            timestamped = timestamp_provider.tag_sensor_data(SensorType.IMU, raw_data)

            imu_data = IMUData(
                measurement_timestamp_ns=timestamped.measurement_timestamp_ns,
                reception_timestamp_ns=timestamped.reception_timestamp_ns,
                accel_x=raw_data["accel_x"], accel_y=raw_data["accel_y"], accel_z=raw_data["accel_z"],
                gyro_x=0.01, gyro_y=0.02, gyro_z=0.03
            )

            binary_msg = BinarySensorProtocol.encode_imu(imu_data)
            decoded = BinarySensorProtocol.decode_imu(binary_msg)

        final_memory = process.memory_info().rss / (1024 * 1024)  # MB
        memory_increase = final_memory - initial_memory

        return {
            'initial_memory_mb': initial_memory,
            'final_memory_mb': final_memory,
            'memory_increase_mb': memory_increase,
            'acceptable_increase_mb': 50.0,  # Allow up to 50MB increase
            'memory_leak_detected': memory_increase > 50.0
        }

    def _stress_test_high_frequency(self) -> Dict[str, Any]:
        """Stress test with high-frequency operations."""
        server = create_motion_bridge_server(f"{self.test_bridge_name}_hf")
        client = create_motion_bridge_client(f"{self.test_bridge_name}_hf")

        try:
            # Test at 200Hz (5ms intervals) for 5 seconds
            target_interval_ms = 5.0
            duration_seconds = 5.0
            expected_operations = int(duration_seconds * 1000 / target_interval_ms)

            operations_completed = 0
            latencies = []
            start_time = time.time()

            while time.time() - start_time < duration_seconds:
                loop_start = time.perf_counter()

                cmd = VelocityCommand(linear_x=1.0, angular_z=0.5)
                success = client.send_velocity_command(cmd)
                if success:
                    result = server.read_pending_command()
                    if result:
                        operations_completed += 1
                        loop_end = time.perf_counter()
                        latencies.append((loop_end - loop_start) * 1000)

                # Maintain frequency
                elapsed = time.perf_counter() - loop_start
                remaining = (target_interval_ms / 1000) - elapsed
                if remaining > 0:
                    time.sleep(remaining)

            if latencies:
                latencies.sort()
                return {
                    'target_frequency_hz': 1000 / target_interval_ms,
                    'operations_completed': operations_completed,
                    'expected_operations': expected_operations,
                    'completion_rate_percent': (operations_completed / expected_operations) * 100,
                    'p99_latency_ms': latencies[int(len(latencies) * 0.99)],
                    'success': True
                }
            else:
                return {'success': False, 'error': 'No operations completed'}

        finally:
            server.cleanup()
            client.cleanup()

    def _calculate_statistics(self, latencies: List[float], iterations: int) -> Dict[str, Any]:
        """Calculate comprehensive latency statistics."""
        if not latencies:
            return {'error': 'No latency measurements'}

        latencies.sort()

        return {
            'iterations': iterations,
            'mean_ms': statistics.mean(latencies),
            'median_ms': statistics.median(latencies),
            'p50_ms': latencies[int(len(latencies) * 0.5)],
            'p90_ms': latencies[int(len(latencies) * 0.9)],
            'p95_ms': latencies[int(len(latencies) * 0.95)],
            'p99_ms': latencies[int(len(latencies) * 0.99)],
            'p999_ms': latencies[int(len(latencies) * 0.999)] if len(latencies) > 1000 else latencies[-1],
            'min_ms': min(latencies),
            'max_ms': max(latencies),
            'std_dev_ms': statistics.stdev(latencies) if len(latencies) > 1 else 0
        }

    def generate_performance_report(self) -> Dict[str, Any]:
        """Generate comprehensive performance report."""
        report = {
            'timestamp': time.time(),
            'test_suite': 'URC 2026 Performance Baseline',
            'results': self.results,
            'requirements': {},
            'compliance': {},
            'recommendations': []
        }

        # Check compliance against requirements
        for req in self.REQUIREMENTS:
            if not req.enabled:
                continue

            result = self.results.get(req.name, {})
            compliant = False

            if 'p99_ms' in result:
                compliant = result['p99_ms'] <= req.max_p99_ms
            elif 'violation_rate_percent' in result:
                compliant = result['violation_rate_percent'] <= req.max_violation_rate_percent

            report['requirements'][req.name] = {
                'description': req.description,
                'max_p99_ms': req.max_p99_ms,
                'max_violation_rate_percent': req.max_violation_rate_percent,
                'result': result,
                'compliant': compliant
            }

        # Overall compliance
        compliant_requirements = sum(1 for r in report['requirements'].values() if r['compliant'])
        total_requirements = len(report['requirements'])

        report['compliance'] = {
            'compliant_requirements': compliant_requirements,
            'total_requirements': total_requirements,
            'compliance_rate_percent': (compliant_requirements / total_requirements) * 100 if total_requirements > 0 else 0,
            'overall_pass': compliant_requirements == total_requirements
        }

        # Generate recommendations
        if not report['compliance']['overall_pass']:
            report['recommendations'].append("Performance requirements not met - investigate bottlenecks")

        # Check for significant improvements
        if 'binary_protocol' in self.results and 'json_comparison' in self.results['binary_protocol']:
            improvement = self.results['binary_protocol']['json_comparison']['improvement_factor']
            if improvement > 3.0:
                report['recommendations'].append(".1f")

        return report

    def save_report(self, filename: str = None) -> str:
        """Save performance report to file."""
        if not filename:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f"performance_baseline_report_{timestamp}.json"

        report = self.generate_performance_report()

        with open(filename, 'w') as f:
            json.dump(report, f, indent=2, default=str)

        return filename


# Pytest fixtures and test functions
@pytest.fixture
def performance_tester():
    """Fixture for performance baseline tester."""
    return PerformanceBaselineTester()


def test_binary_protocol_performance(performance_tester):
    """Test binary protocol meets performance requirements."""
    result = performance_tester.test_binary_protocol_baseline()

    assert result['p99_ms'] < 1.0, f"Binary protocol too slow: {result['p99_ms']:.3f}ms p99"
    assert result['json_comparison']['improvement_factor'] > 2.0, "Insufficient improvement over JSON"


def test_sensor_timestamp_performance(performance_tester):
    """Test sensor timestamp provider meets performance requirements."""
    result = performance_tester.test_sensor_timestamp_baseline()

    assert result['p99_ms'] < 1.0, f"Timestamp provider too slow: {result['p99_ms']:.3f}ms p99"


def test_ipc_bridge_performance(performance_tester):
    """Test IPC bridge meets performance requirements."""
    result = performance_tester.test_ipc_bridge_baseline()

    assert result['p99_ms'] < 5.0, f"IPC bridge too slow: {result['p99_ms']:.3f}ms p99"


def test_motion_control_deadlines(performance_tester):
    """Test motion control meets deadline requirements."""
    result = performance_tester.test_motion_control_baseline()

    assert result['violation_rate_percent'] < 1.0, f"Too many deadline violations: {result['violation_rate_percent']:.1f}%"


def test_end_to_end_pipeline(performance_tester):
    """Test complete sensor pipeline meets performance requirements."""
    result = performance_tester.test_end_to_end_pipeline()

    assert result['p99_ms'] < 5.0, f"End-to-end pipeline too slow: {result['p99_ms']:.3f}ms p99"


def test_stress_concurrent_operations(performance_tester):
    """Test concurrent operations under stress."""
    stress_result = performance_tester._stress_test_ipc_bridge_concurrent()

    assert stress_result['success'], "Concurrent operations failed"
    assert stress_result['p99_latency_ms'] < 10.0, f"Concurrent ops too slow: {stress_result['p99_latency_ms']:.3f}ms"


def test_stress_memory_usage(performance_tester):
    """Test memory usage under stress."""
    memory_result = performance_tester._stress_test_memory_usage()

    assert not memory_result['memory_leak_detected'], f"Memory leak detected: +{memory_result['memory_increase_mb']:.1f}MB"


def test_stress_high_frequency(performance_tester):
    """Test high-frequency operations under stress."""
    hf_result = performance_tester._stress_test_high_frequency()

    assert hf_result['success'], "High-frequency operations failed"
    assert hf_result['completion_rate_percent'] > 90.0, f"Low completion rate: {hf_result['completion_rate_percent']:.1f}%"


if __name__ == "__main__":
    # Run comprehensive performance baseline
    tester = PerformanceBaselineTester()
    report = tester.run_all_baselines()

    # Save detailed report
    filename = tester.save_report()

    # Print summary
    compliance = report['compliance']
    print("\n🎯 PERFORMANCE BASELINE REPORT")
    print(f"Compliance Rate: {compliance['compliance_rate_percent']:.1f}%")
    print(f"Passed: {compliance['compliant_requirements']}/{compliance['total_requirements']}")

    if compliance['overall_pass']:
        print("✅ ALL REQUIREMENTS MET - ARCHITECTURE READY")
    else:
        print("❌ REQUIREMENTS NOT MET - INVESTIGATION NEEDED")

    print(f"\n📄 Detailed report saved to: {filename}")
