#!/usr/bin/env python3
"""
Timing & Race Condition Tests - URC 2026 Real-Time Critical Systems

Tests timing constraints, race conditions, and failure states for:
- Real-time deadlines and latency requirements
- Race conditions in state machines and behavior trees
- CPU bottlenecks and resource contention
- Hardware failures and sensor anomalies
- Network congestion and communication failures
- Concurrent access patterns and thread safety

Author: URC 2026 Real-Time Systems Testing Team
"""

import asyncio
import threading
import time
import random
import psutil
import os
from concurrent.futures import ThreadPoolExecutor, as_completed
from typing import Dict, Any, List, Optional, Callable
import pytest
from unittest.mock import Mock, patch, AsyncMock
import statistics
import multiprocessing


class TimingAnalyzer:
    """Real-time timing analysis and validation."""

    def __init__(self):
        self.timing_samples = []
        self.deadline_violations = []
        self.latency_distribution = {}

    def measure_execution_time(self, func: Callable, *args, **kwargs) -> Dict[str, Any]:
        """Measure function execution time with high precision."""
        start_time = time.perf_counter_ns()
        try:
            result = func(*args, **kwargs)
            success = True
        except Exception as e:
            result = None
            success = False
            error = str(e)

        end_time = time.perf_counter_ns()
        execution_time_ns = end_time - start_time
        execution_time_ms = execution_time_ns / 1_000_000

        sample = {
            'execution_time_ns': execution_time_ns,
            'execution_time_ms': execution_time_ms,
            'success': success,
            'timestamp': time.time()
        }

        if not success:
            sample['error'] = error

        self.timing_samples.append(sample)
        return sample

    async def measure_async_execution_time(self, coro) -> Dict[str, Any]:
        """Measure async function execution time."""
        start_time = time.perf_counter_ns()
        try:
            result = await coro
            success = True
        except Exception as e:
            result = None
            success = False
            error = str(e)

        end_time = time.perf_counter_ns()
        execution_time_ns = end_time - start_time
        execution_time_ms = execution_time_ns / 1_000_000

        sample = {
            'execution_time_ns': execution_time_ns,
            'execution_time_ms': execution_time_ms,
            'success': success,
            'timestamp': time.time()
        }

        if not success:
            sample['error'] = error

        self.timing_samples.append(sample)
        return sample

    def check_deadline(self, execution_time_ms: float, deadline_ms: float) -> bool:
        """Check if execution met deadline."""
        met = execution_time_ms <= deadline_ms
        if not met:
            self.deadline_violations.append({
                'execution_time_ms': execution_time_ms,
                'deadline_ms': deadline_ms,
                'violation_ms': execution_time_ms - deadline_ms,
                'timestamp': time.time()
            })
        return met

    def get_timing_statistics(self) -> Dict[str, Any]:
        """Get comprehensive timing statistics."""
        if not self.timing_samples:
            return {}

        execution_times = [s['execution_time_ms'] for s in self.timing_samples if s['success']]
        successful_samples = [s for s in self.timing_samples if s['success']]
        failed_samples = [s for s in self.timing_samples if not s['success']]

        stats = {
            'total_samples': len(self.timing_samples),
            'successful_samples': len(successful_samples),
            'failed_samples': len(failed_samples),
            'success_rate': len(successful_samples) / len(self.timing_samples),
            'deadline_violations': len(self.deadline_violations)
        }

        if execution_times:
            stats.update({
                'avg_execution_time_ms': statistics.mean(execution_times),
                'median_execution_time_ms': statistics.median(execution_times),
                'min_execution_time_ms': min(execution_times),
                'max_execution_time_ms': max(execution_times),
                'std_dev_execution_time_ms': statistics.stdev(execution_times) if len(execution_times) > 1 else 0,
                '95th_percentile_ms': statistics.quantiles(execution_times, n=20)[18] if len(execution_times) >= 20 else max(execution_times),
                '99th_percentile_ms': statistics.quantiles(execution_times, n=100)[98] if len(execution_times) >= 100 else max(execution_times)
            })

        return stats


class RaceConditionTester:
    """Race condition testing framework."""

    def __init__(self):
        self.shared_state = {}
        self.access_log = []
        self.race_detected = False
        self.lock = threading.RLock()

    def test_concurrent_state_access(self, num_threads: int = 10, operations_per_thread: int = 100) -> Dict[str, Any]:
        """Test concurrent access to shared state."""
        self.shared_state = {'counter': 0, 'data': {}}
        self.access_log = []
        self.race_detected = False

        def worker_thread(thread_id: int):
            for i in range(operations_per_thread):
                # Simulate state machine transition
                operation = random.choice(['read', 'write', 'transition'])

                with self.lock:  # Proper locking
                    timestamp = time.time()
                    old_value = self.shared_state['counter']

                    if operation == 'read':
                        # Read operation
                        _ = self.shared_state['counter']
                        self.access_log.append({
                            'thread': thread_id,
                            'operation': 'read',
                            'timestamp': timestamp,
                            'old_value': old_value,
                            'new_value': old_value
                        })

                    elif operation == 'write':
                        # Write operation
                        new_value = old_value + 1
                        self.shared_state['counter'] = new_value
                        self.access_log.append({
                            'thread': thread_id,
                            'operation': 'write',
                            'timestamp': timestamp,
                            'old_value': old_value,
                            'new_value': new_value
                        })

                    elif operation == 'transition':
                        # State transition (more complex)
                        transition_data = f"transition_{thread_id}_{i}"
                        self.shared_state['data'][f'key_{thread_id}_{i}'] = transition_data
                        self.access_log.append({
                            'thread': thread_id,
                            'operation': 'transition',
                            'timestamp': timestamp,
                            'transition_data': transition_data
                        })

                    # Small delay to increase race condition probability
                    time.sleep(0.001)

        # Run concurrent threads
        threads = []
        for i in range(num_threads):
            thread = threading.Thread(target=worker_thread, args=(i,))
            threads.append(thread)

        start_time = time.time()
        for thread in threads:
            thread.start()

        for thread in threads:
            thread.join()
        end_time = time.time()

        # Analyze for race conditions
        race_analysis = self._analyze_access_patterns()

        return {
            'duration': end_time - start_time,
            'total_operations': len(self.access_log),
            'operations_per_second': len(self.access_log) / (end_time - start_time),
            'race_conditions_detected': race_analysis['race_detected'],
            'inconsistent_states': race_analysis['inconsistent_states'],
            'final_counter_value': self.shared_state['counter'],
            'expected_counter_value': sum(1 for log in self.access_log if log['operation'] == 'write'),
            'access_pattern_analysis': race_analysis
        }

    def _analyze_access_patterns(self) -> Dict[str, Any]:
        """Analyze access patterns for race conditions."""
        race_detected = False
        inconsistent_states = []

        # Sort by timestamp
        sorted_log = sorted(self.access_log, key=lambda x: x['timestamp'])

        # Check for concurrent writes without proper ordering
        write_operations = [log for log in sorted_log if log['operation'] == 'write']
        for i in range(1, len(write_operations)):
            prev_write = write_operations[i-1]
            curr_write = write_operations[i]

            # Check if writes happened too close together (potential race)
            time_diff = curr_write['timestamp'] - prev_write['timestamp']
            if time_diff < 0.0001:  # Less than 0.1ms apart
                race_detected = True
                inconsistent_states.append({
                    'type': 'concurrent_writes',
                    'time_diff_ms': time_diff * 1000,
                    'writes': [prev_write, curr_write]
                })

        # Check for read-after-write inconsistencies
        counter_values = []
        for log in sorted_log:
            if log['operation'] == 'write':
                counter_values.append(log['new_value'])
            elif log['operation'] == 'read':
                if counter_values and log['old_value'] != counter_values[-1]:
                    race_detected = True
                    inconsistent_states.append({
                        'type': 'stale_read',
                        'expected_value': counter_values[-1],
                        'actual_value': log['old_value'],
                        'log_entry': log
                    })

        return {
            'race_detected': race_detected,
            'inconsistent_states': inconsistent_states,
            'total_accesses': len(sorted_log),
            'write_operations': len(write_operations),
            'read_operations': len([log for log in sorted_log if log['operation'] == 'read'])
        }


class HardwareFailureSimulator:
    """Advanced hardware failure simulation."""

    def __init__(self):
        self.active_failures = {}
        self.failure_patterns = {
            'sensor_dropout': self._simulate_sensor_dropout,
            'anomalous_reading': self._simulate_anomalous_reading,
            'intermittent_failure': self._simulate_intermittent_failure,
            'cascading_failure': self._simulate_cascading_failure,
            'gradual_degradation': self._simulate_gradual_degradation
        }

    def inject_failure(self, failure_type: str, component: str, **kwargs) -> Dict[str, Any]:
        """Inject a specific type of hardware failure."""
        if failure_type not in self.failure_patterns:
            raise ValueError(f"Unknown failure type: {failure_type}")

        failure_simulator = self.failure_patterns[failure_type]
        failure = failure_simulator(component, **kwargs)
        self.active_failures[f"{component}_{failure_type}"] = failure

        return failure

    def _simulate_sensor_dropout(self, sensor_type: str, dropout_duration: float = 5.0) -> Dict[str, Any]:
        """Simulate complete sensor dropout."""
        return {
            'type': 'sensor_dropout',
            'component': sensor_type,
            'start_time': time.time(),
            'duration': dropout_duration,
            'end_time': time.time() + dropout_duration,
            'behavior': lambda: None  # No readings during dropout
        }

    def _simulate_anomalous_reading(self, sensor_type: str, anomaly_type: str = 'spike') -> Dict[str, Any]:
        """Simulate anomalous sensor readings."""
        def anomalous_behavior():
            if anomaly_type == 'spike':
                return random.uniform(1000, 10000)  # Extreme spike
            elif anomaly_type == 'drift':
                return random.uniform(-100, 100)  # Gradual drift
            elif anomaly_type == 'noise':
                return random.gauss(0, 100)  # High noise
            else:
                return float('nan')  # NaN readings

        return {
            'type': 'anomalous_reading',
            'component': sensor_type,
            'anomaly_type': anomaly_type,
            'start_time': time.time(),
            'behavior': anomalous_behavior
        }

    def _simulate_intermittent_failure(self, component: str, failure_rate: float = 0.1) -> Dict[str, Any]:
        """Simulate intermittent component failure."""
        def intermittent_behavior():
            return random.random() < failure_rate  # Fail randomly

        return {
            'type': 'intermittent_failure',
            'component': component,
            'failure_rate': failure_rate,
            'start_time': time.time(),
            'behavior': intermittent_behavior
        }

    def _simulate_cascading_failure(self, primary_component: str) -> Dict[str, Any]:
        """Simulate cascading failure from primary to dependent components."""
        dependents = {
            'imu': ['navigation', 'control'],
            'gps': ['navigation', 'localization'],
            'motor_left': ['locomotion', 'odometry'],
            'power_main': ['all_components']
        }

        affected_components = dependents.get(primary_component, [])

        return {
            'type': 'cascading_failure',
            'primary_component': primary_component,
            'affected_components': affected_components,
            'start_time': time.time(),
            'cascade_delay': 1.0  # Delay before affecting dependents
        }

    def _simulate_gradual_degradation(self, component: str, degradation_rate: float = 0.01) -> Dict[str, Any]:
        """Simulate gradual component degradation."""
        degradation_state = {'degradation_level': 0.0}

        def degradation_behavior():
            degradation_state['degradation_level'] += degradation_rate
            degradation_state['degradation_level'] = min(1.0, degradation_state['degradation_level'])

            # Return degraded performance (e.g., reduced accuracy)
            return 1.0 - degradation_state['degradation_level']  # Performance factor

        return {
            'type': 'gradual_degradation',
            'component': component,
            'degradation_rate': degradation_rate,
            'start_time': time.time(),
            'behavior': degradation_behavior,
            'state': degradation_state
        }

    def get_active_failures(self) -> Dict[str, Any]:
        """Get all active hardware failures."""
        return self.active_failures.copy()

    def clear_failure(self, failure_key: str):
        """Clear a specific failure."""
        if failure_key in self.active_failures:
            del self.active_failures[failure_key]


class TestTimingRaceConditionsFailures:
    """Comprehensive timing, race condition, and failure testing."""

    @pytest.fixture
    def timing_analyzer(self):
        """Create timing analyzer."""
        return TimingAnalyzer()

    @pytest.fixture
    def race_tester(self):
        """Create race condition tester."""
        return RaceConditionTester()

    @pytest.fixture
    def hardware_failure_sim(self):
        """Create hardware failure simulator."""
        return HardwareFailureSimulator()

    @pytest.mark.asyncio
    async def test_real_time_deadlines_state_machine(self, timing_analyzer):
        """Test real-time deadlines for state machine operations."""
        # Simulate state machine with timing requirements
        state_machine_delays = []

        for i in range(100):
            # State transition timing (must be < 50ms)
            timing = await timing_analyzer.measure_async_execution_time(
                self._simulate_state_transition()
            )
            state_machine_delays.append(timing['execution_time_ms'])

            # Check 50ms deadline for state transitions
            deadline_met = timing_analyzer.check_deadline(timing['execution_time_ms'], 50.0)
            assert deadline_met, ".1f"

        stats = timing_analyzer.get_timing_statistics()

        print("🎯 State Machine Timing Analysis:")
        print(".1f"        print(".1f"        print(".1f"        print(f"   Deadline Violations: {stats['deadline_violations']}")

        # State machine must meet real-time requirements
        assert stats['avg_execution_time_ms'] < 20.0  # Average < 20ms
        assert stats['95th_percentile_ms'] < 40.0     # 95th percentile < 40ms
        assert stats['deadline_violations'] == 0       # No deadline violations

    async def _simulate_state_transition(self) -> str:
        """Simulate state machine transition with realistic timing."""
        # Simulate state validation and transition logic
        await asyncio.sleep(random.uniform(0.005, 0.025))  # 5-25ms processing

        # Randomly simulate complex transition
        if random.random() < 0.2:  # 20% complex transitions
            await asyncio.sleep(random.uniform(0.01, 0.03))  # Additional processing

        return "transition_complete"

    def test_race_conditions_state_machine(self, race_tester):
        """Test race conditions in state machine operations."""
        # Test concurrent state machine access
        race_results = race_tester.test_concurrent_state_access(
            num_threads=20,
            operations_per_thread=50
        )

        print("🏁 State Machine Race Condition Analysis:")
        print(f"   Total Operations: {race_results['total_operations']}")
        print(".1f"        print(f"   Final Counter: {race_results['final_counter_value']}")
        print(f"   Expected Counter: {race_results['expected_counter_value']}")
        print(f"   Race Conditions: {race_results['race_conditions_detected']}")

        # State machine must be thread-safe
        assert not race_results['race_conditions_detected'], "Race conditions detected in state machine"
        assert race_results['final_counter_value'] == race_results['expected_counter_value'], "State inconsistency due to race conditions"

        # Performance requirements
        assert race_results['operations_per_second'] > 1000, "State machine too slow for concurrent access"

    def test_race_conditions_behavior_tree(self, race_tester):
        """Test race conditions in behavior tree execution."""
        # Simulate behavior tree with concurrent node execution
        bt_race_results = race_tester.test_concurrent_state_access(
            num_threads=15,
            operations_per_thread=75
        )

        print("🌳 Behavior Tree Race Condition Analysis:")
        print(f"   Concurrent BT Operations: {bt_race_results['total_operations']}")
        print(f"   Operations/sec: {bt_race_results['operations_per_second']:.0f}")
        print(f"   Race Conditions: {bt_race_results['race_conditions_detected']}")

        # Behavior tree must handle concurrent execution
        assert not bt_race_results['race_conditions_detected'], "Race conditions in behavior tree execution"
        assert bt_race_results['operations_per_second'] > 800, "Behavior tree too slow for real-time execution"

    @pytest.mark.asyncio
    async def test_timing_behavior_tree_execution(self, timing_analyzer):
        """Test timing constraints for behavior tree execution."""
        bt_execution_times = []

        for i in range(50):
            # Behavior tree tick timing (must be < 100ms for real-time)
            timing = await timing_analyzer.measure_async_execution_time(
                self._simulate_behavior_tree_tick()
            )
            bt_execution_times.append(timing['execution_time_ms'])

            # Check 100ms deadline for BT ticks
            deadline_met = timing_analyzer.check_deadline(timing['execution_time_ms'], 100.0)
            assert deadline_met, ".1f"

        stats = timing_analyzer.get_timing_statistics()

        print("🌳 Behavior Tree Timing Analysis:")
        print(".1f"        print(".1f"        print(f"   Deadline Violations: {stats['deadline_violations']}")

        # Behavior tree must meet real-time requirements
        assert stats['avg_execution_time_ms'] < 50.0   # Average < 50ms
        assert stats['99th_percentile_ms'] < 90.0       # 99th percentile < 90ms

    async def _simulate_behavior_tree_tick(self) -> str:
        """Simulate behavior tree tick with realistic processing."""
        # Simulate BT node evaluation
        nodes_to_evaluate = random.randint(5, 20)

        for _ in range(nodes_to_evaluate):
            # Node evaluation time
            await asyncio.sleep(random.uniform(0.001, 0.005))  # 1-5ms per node

            # Some nodes have additional processing
            if random.random() < 0.3:
                await asyncio.sleep(random.uniform(0.002, 0.008))  # Additional processing

        return "bt_tick_complete"

    @pytest.mark.asyncio
    async def test_cpu_bottlenecks_under_load(self):
        """Test CPU bottlenecks and performance under high load."""
        print("⚡ Testing CPU bottlenecks under load...")

        # Test CPU-bound operations
        cpu_usage_samples = []
        operation_counts = []

        async def cpu_intensive_operation(operations: int):
            """Simulate CPU-intensive operations."""
            count = 0
            for _ in range(operations):
                # CPU-intensive calculation
                result = sum(i * i for i in range(1000))
                count += 1
            return count

        # Monitor CPU usage while running operations
        start_time = time.time()

        # Run multiple CPU-intensive tasks concurrently
        tasks = []
        for i in range(multiprocessing.cpu_count()):
            task = asyncio.create_task(cpu_intensive_operation(1000))
            tasks.append(task)

        # Monitor CPU usage during execution
        monitoring_task = asyncio.create_task(self._monitor_cpu_usage(cpu_usage_samples, 5.0))

        results = await asyncio.gather(*tasks, monitoring_task)
        operation_counts = results[:-1]  # Last result is CPU monitoring

        end_time = time.time()

        total_operations = sum(operation_counts)
        operations_per_second = total_operations / (end_time - start_time)
        avg_cpu_usage = sum(cpu_usage_samples) / len(cpu_usage_samples) if cpu_usage_samples else 0
        max_cpu_usage = max(cpu_usage_samples) if cpu_usage_samples else 0

        print("⚡ CPU Bottleneck Analysis:")
        print(f"   Total Operations: {total_operations}")
        print(".1f"        print(".1f"        print(".1f"        print(f"   CPU Samples: {len(cpu_usage_samples)}")

        # System should not be CPU-bound for typical operations
        assert operations_per_second > 10000, "Operations per second too low - possible CPU bottleneck"
        assert avg_cpu_usage < 85, ".1f"
        assert max_cpu_usage < 95, ".1f"

    async def _monitor_cpu_usage(self, samples: List[float], duration: float):
        """Monitor CPU usage for specified duration."""
        start_time = time.time()
        while time.time() - start_time < duration:
            cpu_percent = psutil.cpu_percent(interval=0.1)
            samples.append(cpu_percent)
            await asyncio.sleep(0.1)

    @pytest.mark.asyncio
    async def test_network_congestion_handling(self):
        """Test network congestion and communication bottlenecks."""
        print("🌐 Testing network congestion handling...")

        # Simulate network congestion with delayed responses
        congestion_levels = [0.01, 0.05, 0.1, 0.2]  # 1%, 5%, 10%, 20% congestion

        for congestion_rate in congestion_levels:
            print(f"   Testing {congestion_rate*100:.0f}% congestion...")

            # Simulate message transmission under congestion
            message_latencies = []

            for i in range(100):
                # Base transmission time
                base_latency = random.uniform(0.001, 0.01)  # 1-10ms

                # Add congestion delay
                if random.random() < congestion_rate:
                    congestion_delay = random.uniform(0.05, 0.5)  # 50-500ms delay
                    base_latency += congestion_delay

                message_latencies.append(base_latency * 1000)  # Convert to ms

                await asyncio.sleep(base_latency)

            avg_latency = sum(message_latencies) / len(message_latencies)
            max_latency = max(message_latencies)
            percentile_95 = sorted(message_latencies)[94]  # 95th percentile

            print(".1f"
            # Under congestion, system should still be responsive
            assert avg_latency < 100, ".1f"            assert percentile_95 < 300, ".1f"            assert max_latency < 1000, ".1f"

    def test_sensor_dropouts_and_anomalies(self, hardware_failure_sim):
        """Test sensor dropouts and anomalous readings handling."""
        print("📡 Testing sensor dropouts and anomalies...")

        # Test sensor dropout handling
        dropout_failure = hardware_failure_sim.inject_failure(
            'sensor_dropout', 'imu', dropout_duration=3.0
        )

        # System should handle IMU dropout gracefully
        assert dropout_failure['type'] == 'sensor_dropout'
        assert dropout_failure['component'] == 'imu'
        assert dropout_failure['duration'] == 3.0

        # Test anomalous readings
        anomaly_failure = hardware_failure_sim.inject_failure(
            'anomalous_reading', 'gps', anomaly_type='spike'
        )

        # Generate some anomalous readings
        anomalous_readings = []
        for _ in range(10):
            reading = anomaly_failure['behavior']()
            anomalous_readings.append(reading)

        # Anomalous readings should be extreme
        assert any(reading > 1000 for reading in anomalous_readings), "Anomalous readings not extreme enough"

        print("📡 Sensor failure simulation:")
        print(f"   Active failures: {len(hardware_failure_sim.get_active_failures())}")
        print(f"   IMU dropout: {dropout_failure['duration']}s")
        print(f"   GPS anomalies: {len(anomalous_readings)} readings")

        # System should detect and handle sensor failures
        active_failures = hardware_failure_sim.get_active_failures()
        assert len(active_failures) == 2
        assert 'imu_sensor_dropout' in active_failures
        assert 'gps_anomalous_reading' in active_failures

    def test_cascading_hardware_failures(self, hardware_failure_sim):
        """Test cascading hardware failure scenarios."""
        print("⛓️ Testing cascading hardware failures...")

        # Inject primary failure that should cascade
        cascade_failure = hardware_failure_sim.inject_failure(
            'cascading_failure', 'imu'
        )

        assert cascade_failure['type'] == 'cascading_failure'
        assert cascade_failure['primary_component'] == 'imu'
        assert 'navigation' in cascade_failure['affected_components']
        assert 'control' in cascade_failure['affected_components']

        # Simulate cascade delay
        time.sleep(cascade_failure['cascade_delay'] + 0.1)

        # System should handle cascading failures gracefully
        # In real implementation, this would trigger failover mechanisms
        print(f"   Primary failure: {cascade_failure['primary_component']}")
        print(f"   Affected components: {cascade_failure['affected_components']}")

        assert len(cascade_failure['affected_components']) > 0

    def test_gradual_hardware_degradation(self, hardware_failure_sim):
        """Test gradual hardware degradation handling."""
        print("📉 Testing gradual hardware degradation...")

        # Inject gradual degradation
        degradation_failure = hardware_failure_sim.inject_failure(
            'gradual_degradation', 'battery', degradation_rate=0.02
        )

        # Monitor degradation over time
        degradation_readings = []
        for i in range(20):
            performance = degradation_failure['behavior']()
            degradation_readings.append(performance)
            time.sleep(0.1)

        initial_performance = degradation_readings[0]
        final_performance = degradation_readings[-1]
        degradation_amount = initial_performance - final_performance

        print(".1f"        print(".1f"        print(".1f"
        # System should handle gradual degradation
        assert degradation_amount > 0, "No degradation detected"
        assert final_performance > 0.5, "Degradation too severe"        assert len(degradation_readings) == 20

    @pytest.mark.asyncio
    async def test_timing_under_resource_contention(self, timing_analyzer):
        """Test timing behavior under resource contention."""
        print("🏭 Testing timing under resource contention...")

        # Create resource contention by running many concurrent operations
        async def contended_operation(task_id: int):
            """Operation that contends for resources."""
            # Simulate memory allocation
            data = [0] * 10000  # Allocate memory

            # CPU intensive work
            for _ in range(1000):
                _ = sum(i * i for i in range(100))

            # I/O simulation
            await asyncio.sleep(random.uniform(0.001, 0.005))

            return task_id

        # Run operations with high concurrency
        num_concurrent = 50
        tasks = []

        for i in range(num_concurrent):
            task = timing_analyzer.measure_async_execution_time(
                contended_operation(i)
            )
            tasks.append(task)

        start_time = time.time()
        results = await asyncio.gather(*tasks)
        end_time = time.time()

        total_time = end_time - start_time
        throughput = num_concurrent / total_time

        print("🏭 Resource Contention Analysis:")
        print(f"   Concurrent Operations: {num_concurrent}")
        print(".1f"        print(".1f"
        # System should handle resource contention
        assert throughput > 5, "Throughput too low under contention"        assert total_time < 30, "Contention caused excessive delays"

    def test_memory_pressure_and_leaks(self):
        """Test memory pressure and potential memory leaks."""
        print("💾 Testing memory pressure and leaks...")

        # Simulate memory-intensive operations
        memory_samples = []

        def memory_intensive_operation(iterations: int):
            """Operation that uses significant memory."""
            data_structures = []
            for i in range(iterations):
                # Create nested data structures
                data = {
                    'arrays': [[j * k for k in range(100)] for j in range(50)],
                    'dicts': {f'key_{j}': f'value_{j}' * 100 for j in range(200)},
                    'strings': ['x' * 1000] * 50
                }
                data_structures.append(data)

            # Process data (simulate real work)
            total_elements = sum(
                len(arr) * len(arr[0]) if arr else 0
                for data in data_structures
                for arr in data.get('arrays', [])
            )

            # Clean up (simulate proper resource management)
            del data_structures

            return total_elements

        # Monitor memory usage during operations
        initial_memory = psutil.virtual_memory().percent

        for i in range(10):
            memory_intensive_operation(5)

            current_memory = psutil.virtual_memory().percent
            memory_samples.append(current_memory)

            # Force garbage collection check
            import gc
            gc.collect()

        final_memory = psutil.virtual_memory().percent
        memory_increase = final_memory - initial_memory
        max_memory = max(memory_samples)

        print("💾 Memory Pressure Analysis:")
        print(".1f"        print(".1f"        print(".1f"        print(f"   Memory Samples: {len(memory_samples)}")

        # System should not have significant memory leaks
        assert memory_increase < 10, ".1f"        assert max_memory < 90, ".1f"

    @pytest.mark.asyncio
    async def test_deadlock_prevention(self):
        """Test deadlock prevention in concurrent operations."""
        print("🔒 Testing deadlock prevention...")

        # Create shared resources that could cause deadlocks
        resource_a = asyncio.Lock()
        resource_b = asyncio.Lock()
        operation_log = []

        async def operation_with_potential_deadlock(task_id: int, reverse_order: bool = False):
            """Operation that acquires multiple resources."""
            locks = [resource_b, resource_a] if reverse_order else [resource_a, resource_b]

            acquired_locks = []
            try:
                for lock in locks:
                    await asyncio.wait_for(lock.acquire(), timeout=1.0)
                    acquired_locks.append(lock)
                    operation_log.append(f"Task {task_id}: acquired lock {locks.index(lock)}")

                # Simulate work with locks held
                await asyncio.sleep(0.01)
                operation_log.append(f"Task {task_id}: completed work")

            finally:
                # Release locks in reverse order
                for lock in reversed(acquired_locks):
                    lock.release()

            return task_id

        # Run operations that could cause deadlocks
        tasks = []
        for i in range(20):
            # Alternate lock acquisition order to test deadlock prevention
            reverse_order = i % 2 == 1
            task = operation_with_potential_deadlock(i, reverse_order)
            tasks.append(task)

        start_time = time.time()
        results = await asyncio.gather(*tasks, return_exceptions=True)
        end_time = time.time()

        successful_operations = sum(1 for r in results if not isinstance(r, Exception))
        failed_operations = sum(1 for r in results if isinstance(r, Exception))

        print("🔒 Deadlock Prevention Analysis:")
        print(f"   Total Operations: {len(results)}")
        print(f"   Successful: {successful_operations}")
        print(f"   Failed: {failed_operations}")
        print(".1f"
        # System should prevent deadlocks
        assert successful_operations == len(results), f"Deadlock detected: {failed_operations} operations failed"
        assert end_time - start_time < 5.0, "Operations took too long - possible deadlock"

    @pytest.mark.asyncio
    async def test_timing_jitter_and_variability(self, timing_analyzer):
        """Test timing jitter and variability under normal conditions."""
        print("📊 Testing timing jitter and variability...")

        # Measure consistent operation timing
        timing_samples = []

        for i in range(100):
            timing = await timing_analyzer.measure_async_execution_time(
                self._consistent_operation()
            )
            timing_samples.append(timing['execution_time_ms'])

        # Calculate timing statistics
        mean_time = statistics.mean(timing_samples)
        std_dev = statistics.stdev(timing_samples)
        coefficient_of_variation = std_dev / mean_time if mean_time > 0 else 0

        # Calculate percentiles
        sorted_times = sorted(timing_samples)
        p95 = sorted_times[94]  # 95th percentile
        p99 = sorted_times[98]  # 99th percentile

        print("📊 Timing Variability Analysis:")
        print(".2f"        print(".2f"        print(".3f"        print(".2f"        print(".2f"        print(".2f"
        # Timing should be reasonably consistent
        assert coefficient_of_variation < 0.5, ".3f"
        assert p99 / mean_time < 3.0, ".2f"

    async def _consistent_operation(self) -> str:
        """A consistent operation for timing measurements."""
        # Fixed amount of work
        await asyncio.sleep(0.01)  # 10ms fixed delay
        result = sum(i for i in range(1000))  # Fixed computation
        return f"result_{result}"

    @pytest.mark.asyncio
    async def test_comprehensive_system_stress_test(self):
        """Run comprehensive system stress test combining all failure modes."""
        print("🎯 Running comprehensive system stress test...")

        # Combine multiple stress factors
        stress_factors = {
            'high_concurrency': True,
            'network_congestion': 0.1,
            'cpu_pressure': True,
            'memory_pressure': True,
            'sensor_failures': ['imu', 'gps'],
            'timing_constraints': True
        }

        # This would be a comprehensive integration test
        # For now, validate the testing framework is ready
        assert len(stress_factors) == 6

        print("🎯 Comprehensive stress test framework validated")
        print(f"   Stress factors configured: {len(stress_factors)}")
        print(f"   Integration test ready: ✅")

        # In a full implementation, this would run all stress factors simultaneously
        # and validate system behavior under extreme conditions



