#!/usr/bin/env python3
"""
Integration Test: IPC Motion Control Bridge

Tests the IPC motion control bridge for deterministic <20ms latency
and reliable command/state communication.
"""

import time
import threading
import pytest
from multiprocessing import Process, Queue
from src.motion.ipc_motion_bridge import (
    IpcMotionBridge,
    VelocityCommand,
    MotionControlStatus,
    MotionControlState,
    create_motion_bridge_server,
    create_motion_bridge_client,
)
from src.testing.performance_profiling import PerformanceProfiler


class TestIpcMotionBridgeIntegration:
    """Test IPC motion control bridge integration."""

    def setup_method(self):
        """Setup test fixtures."""
        self.performance_profiler = PerformanceProfiler()
        self.shared_memory_name = f"test_bridge_{int(time.time() * 1000)}"

    def teardown_method(self):
        """Cleanup test fixtures."""
        pass

    def test_basic_bridge_communication(self):
        """Test basic IPC bridge command and state communication."""
        # Create server (motion control side)
        server_bridge = create_motion_bridge_server(self.shared_memory_name)

        # Create client (autonomy side)
        client_bridge = create_motion_bridge_client(self.shared_memory_name)

        try:
            # Test velocity command
            command = VelocityCommand(
                linear_x=1.0,
                angular_z=0.5,
                acceleration_limit=1.0,
                deceleration_limit=2.0,
            )

            # Send command from client
            success = client_bridge.send_velocity_command(command)
            assert success

            # Receive command on server
            received_command, received_velocity = server_bridge.read_pending_command()
            assert received_command is not None
            assert received_velocity is not None
            assert abs(received_velocity.linear_x - 1.0) < 1e-6
            assert abs(received_velocity.angular_z - 0.5) < 1e-6

            # Test state update
            state = MotionControlState(
                timestamp_ns=time.time_ns(),
                sequence_number=1,
                status=MotionControlStatus.OK,
                current_velocity=command,
                target_velocity=command,
                emergency_stop_active=False,
                hardware_ok=True,
                temperature_c=35.0,
                battery_voltage=12.5,
                motor_currents=(1.2, 1.1),
            )

            # Send state from server
            success = server_bridge.update_motion_state(state)
            assert success

            # Receive state on client
            received_state = client_bridge.read_motion_state()
            assert received_state is not None
            assert received_state.status == MotionControlStatus.OK
            assert abs(received_state.temperature_c - 35.0) < 1e-6
            assert abs(received_state.battery_voltage - 12.5) < 1e-6

        finally:
            server_bridge.cleanup()
            client_bridge.cleanup()

    def test_emergency_stop_integration(self):
        """Test emergency stop functionality through IPC bridge."""
        server_bridge = create_motion_bridge_server(f"{self.shared_memory_name}_estop")
        client_bridge = create_motion_bridge_client(f"{self.shared_memory_name}_estop")

        try:
            # Send emergency stop
            success = client_bridge.send_emergency_stop()
            assert success

            # Server should receive emergency stop command
            command, velocity = server_bridge.read_pending_command()
            assert command is not None
            # Emergency stop should be detectable (implementation dependent)

            # Test state with emergency stop active
            state = MotionControlState(
                timestamp_ns=time.time_ns(),
                sequence_number=2,
                status=MotionControlStatus.EMERGENCY_STOP_ACTIVE,
                current_velocity=VelocityCommand(),  # Zero velocity
                target_velocity=VelocityCommand(),
                emergency_stop_active=True,
                hardware_ok=True,
                temperature_c=40.0,
                battery_voltage=11.8,
                motor_currents=(0.0, 0.0),  # Motors stopped
            )

            server_bridge.update_motion_state(state)

            received_state = client_bridge.read_motion_state()
            assert received_state is not None
            assert received_state.emergency_stop_active == True
            assert received_state.status == MotionControlStatus.EMERGENCY_STOP_ACTIVE
            assert abs(received_state.motor_currents[0]) < 0.1  # Near zero current

        finally:
            server_bridge.cleanup()
            client_bridge.cleanup()

    def test_bridge_performance_latency(self):
        """Test IPC bridge latency performance."""
        server_bridge = create_motion_bridge_server(f"{self.shared_memory_name}_perf")
        client_bridge = create_motion_bridge_client(f"{self.shared_memory_name}_perf")

        try:
            iterations = 1000
            latencies = []

            for i in range(iterations):
                # Measure command send/receive latency
                command = VelocityCommand(
                    linear_x=0.5 + i * 0.001,  # Vary command slightly
                    angular_z=0.1 + i * 0.0005,
                )

                start_time = time.perf_counter()
                client_bridge.send_velocity_command(command)
                server_bridge.read_pending_command()  # Receive on server
                end_time = time.perf_counter()

                latency_ms = (end_time - start_time) * 1000
                latencies.append(latency_ms)

                # Record in profiler
                self.performance_profiler.record_measurement(
                    "ipc_command_latency", latency_ms
                )

            # Analyze results
            latencies.sort()
            p50_latency = latencies[int(len(latencies) * 0.5)]
            p95_latency = latencies[int(len(latencies) * 0.95)]
            p99_latency = latencies[int(len(latencies) * 0.99)]

            print(f"\nIPC Bridge Command Latency (n={iterations}):")
            print(".3f")
            print(".3f")
            print(".3f")
            print(".3f")

            # Assert performance requirements
            assert p99_latency < 5.0, f"p99 latency too high: {p99_latency:.3f}ms"
            assert p95_latency < 2.0, f"p95 latency too high: {p95_latency:.3f}ms"

        finally:
            server_bridge.cleanup()
            client_bridge.cleanup()

    def test_bridge_state_update_performance(self):
        """Test IPC bridge state update performance."""
        server_bridge = create_motion_bridge_server(
            f"{self.shared_memory_name}_state_perf"
        )
        client_bridge = create_motion_bridge_client(
            f"{self.shared_memory_name}_state_perf"
        )

        try:
            iterations = 1000
            latencies = []

            base_state = MotionControlState(
                timestamp_ns=0,
                sequence_number=0,
                status=MotionControlStatus.OK,
                current_velocity=VelocityCommand(linear_x=1.0, angular_z=0.5),
                target_velocity=VelocityCommand(linear_x=1.0, angular_z=0.5),
                emergency_stop_active=False,
                hardware_ok=True,
                temperature_c=35.0,
                battery_voltage=12.5,
                motor_currents=(1.2, 1.1),
            )

            for i in range(iterations):
                # Update state with new timestamp and sequence
                state = base_state
                state.timestamp_ns = time.time_ns()
                state.sequence_number = i

                start_time = time.perf_counter()
                server_bridge.update_motion_state(state)
                client_bridge.read_motion_state()  # Receive on client
                end_time = time.perf_counter()

                latency_ms = (end_time - start_time) * 1000
                latencies.append(latency_ms)

                self.performance_profiler.record_measurement(
                    "ipc_state_latency", latency_ms
                )

            # Analyze results
            latencies.sort()
            p50_latency = latencies[int(len(latencies) * 0.5)]
            p95_latency = latencies[int(len(latencies) * 0.95)]
            p99_latency = latencies[int(len(latencies) * 0.99)]

            print(f"\nIPC Bridge State Update Latency (n={iterations}):")
            print(".3f")
            print(".3f")
            print(".3f")
            print(".3f")

            # Assert performance requirements
            assert p99_latency < 5.0, f"p99 state latency too high: {p99_latency:.3f}ms"

        finally:
            server_bridge.cleanup()
            client_bridge.cleanup()

    def test_bridge_error_handling(self):
        """Test IPC bridge error handling and recovery."""
        # Test with invalid shared memory name (should fail gracefully)
        with pytest.raises(RuntimeError):
            create_motion_bridge_client("nonexistent_bridge")

        # Test cleanup after errors
        bridge = create_motion_bridge_server(f"{self.shared_memory_name}_error_test")
        bridge.cleanup()  # Should not crash

        # Test double cleanup
        bridge.cleanup()  # Should be safe

    def test_concurrent_bridge_access(self):
        """Test concurrent access to IPC bridge."""
        server_bridge = create_motion_bridge_server(
            f"{self.shared_memory_name}_concurrent"
        )
        client_bridge = create_motion_bridge_client(
            f"{self.shared_memory_name}_concurrent"
        )

        try:
            results = []
            errors = []

            def client_worker(worker_id):
                """Client worker thread."""
                try:
                    for i in range(100):
                        command = VelocityCommand(
                            linear_x=0.1 * worker_id, angular_z=0.05 * worker_id
                        )
                        success = client_bridge.send_velocity_command(command)
                        if not success:
                            errors.append(f"Client {worker_id} failed at iteration {i}")
                        time.sleep(0.001)  # Small delay
                    results.append(f"Client {worker_id} completed successfully")
                except Exception as e:
                    errors.append(f"Client {worker_id} error: {e}")

            def server_worker():
                """Server worker thread."""
                try:
                    received_count = 0
                    for i in range(200):  # Expect commands from 2 clients
                        command, velocity = server_bridge.read_pending_command()
                        if command is not None:
                            received_count += 1
                        time.sleep(0.0005)  # Faster polling
                    results.append(f"Server received {received_count} commands")
                except Exception as e:
                    errors.append(f"Server error: {e}")

            # Start workers
            threads = []
            threads.append(threading.Thread(target=server_worker))
            for i in range(2):  # 2 client threads
                threads.append(threading.Thread(target=client_worker, args=(i + 1,)))

            for thread in threads:
                thread.start()

            for thread in threads:
                thread.join(timeout=10.0)

            # Verify results
            assert len(errors) == 0, f"Concurrent access errors: {errors}"
            assert (
                len(results) == 3
            ), f"Expected 3 successful completions, got {len(results)}"

        finally:
            server_bridge.cleanup()
            client_bridge.cleanup()

    def test_bridge_health_monitoring(self):
        """Test IPC bridge health monitoring."""
        server_bridge = create_motion_bridge_server(f"{self.shared_memory_name}_health")
        client_bridge = create_motion_bridge_client(f"{self.shared_memory_name}_health")

        try:
            # Initially healthy
            health = client_bridge.check_bridge_health()
            assert health["overall_healthy"] == True
            assert health["shared_memory_ok"] == True

            # Send some data
            command = VelocityCommand(linear_x=1.0, angular_z=0.5)
            client_bridge.send_velocity_command(command)

            state = MotionControlState(
                timestamp_ns=time.time_ns(),
                sequence_number=1,
                status=MotionControlStatus.OK,
                current_velocity=command,
                target_velocity=command,
                emergency_stop_active=False,
                hardware_ok=True,
                temperature_c=35.0,
                battery_voltage=12.5,
                motor_currents=(1.2, 1.1),
            )
            server_bridge.update_motion_state(state)

            # Check stats
            stats = client_bridge.get_bridge_stats()
            assert stats["commands_sent"] >= 1
            assert stats["buffer_size"] > 0

            health = client_bridge.check_bridge_health()
            assert health["overall_healthy"] == True

        finally:
            server_bridge.cleanup()
            client_bridge.cleanup()

    def test_motion_control_deadline_compliance(self):
        """Test that IPC bridge meets motion control deadline requirements."""
        server_bridge = create_motion_bridge_server(
            f"{self.shared_memory_name}_deadline"
        )
        client_bridge = create_motion_bridge_client(
            f"{self.shared_memory_name}_deadline"
        )

        try:
            # Simulate motion control loop at 50Hz (20ms period)
            target_period_ms = 20.0
            iterations = 500  # 10 seconds of operation

            deadline_misses = 0
            total_latencies = []

            start_time = time.time()

            for i in range(iterations):
                loop_start = time.perf_counter()

                # Simulate motion control iteration
                # 1. Read sensor data (simulated)
                # 2. Send command via IPC
                command = VelocityCommand(
                    linear_x=0.5 + 0.01 * (i % 50),  # Vary command
                    angular_z=0.1 + 0.005 * (i % 50),
                )
                client_bridge.send_velocity_command(command)

                # 3. Receive state feedback (if available)
                state = client_bridge.read_motion_state()

                # 4. Simulate processing (compute control law)
                for _ in range(500):  # Simulate computation
                    pass

                # Check deadline
                loop_end = time.perf_counter()
                loop_time_ms = (loop_end - loop_start) * 1000
                total_latencies.append(loop_time_ms)

                if loop_time_ms > target_period_ms:
                    deadline_misses += 1

                # Maintain 50Hz timing
                elapsed = time.time() - start_time
                expected_elapsed = (i + 1) * (target_period_ms / 1000)
                if elapsed < expected_elapsed:
                    time.sleep(expected_elapsed - elapsed)

            # Analyze results
            total_latencies.sort()
            p99_latency = total_latencies[int(len(total_latencies) * 0.99)]
            deadline_miss_rate = (deadline_misses / iterations) * 100

            print(
                f"\nMotion Control Deadline Analysis (50Hz, {iterations} iterations):"
            )
            print(".3f")
            print(".1f")
            print(".1f")

            # Assert requirements
            assert (
                p99_latency < target_period_ms
            ), f"p99 latency {p99_latency:.3f}ms exceeds deadline {target_period_ms}ms"
            assert (
                deadline_miss_rate < 1.0
            ), f"Deadline miss rate {deadline_miss_rate:.1f}% too high (target <1%)"

        finally:
            server_bridge.cleanup()
            client_bridge.cleanup()

    def test_performance_profiling_integration(self):
        """Test IPC bridge integration with performance profiling."""
        server_bridge = create_motion_bridge_server(
            f"{self.shared_memory_name}_profile"
        )
        client_bridge = create_motion_bridge_client(
            f"{self.shared_memory_name}_profile"
        )

        try:
            # Profile IPC operations
            def profiled_command():
                command = VelocityCommand(linear_x=1.0, angular_z=0.5)
                client_bridge.send_velocity_command(command)
                server_bridge.read_pending_command()

            def profiled_state():
                state = MotionControlState(
                    timestamp_ns=time.time_ns(),
                    sequence_number=1,
                    status=MotionControlStatus.OK,
                    current_velocity=VelocityCommand(),
                    target_velocity=VelocityCommand(),
                    emergency_stop_active=False,
                    hardware_ok=True,
                    temperature_c=35.0,
                    battery_voltage=12.5,
                    motor_currents=(1.2, 1.1),
                )
                server_bridge.update_motion_state(state)
                client_bridge.read_motion_state()

            # Measure operations
            self.performance_profiler.measure_latency(
                "ipc_command_roundtrip", profiled_command
            )
            self.performance_profiler.measure_latency(
                "ipc_state_roundtrip", profiled_state
            )

            # Check profiling results
            report = self.performance_profiler.get_performance_report()

            assert "ipc_command_roundtrip" in report["profiles"]
            assert "ipc_state_roundtrip" in report["profiles"]

            cmd_stats = report["profiles"]["ipc_command_roundtrip"]
            state_stats = report["profiles"]["ipc_state_roundtrip"]

            assert cmd_stats["count"] >= 1
            assert state_stats["count"] >= 1
            assert cmd_stats["mean_ms"] > 0
            assert state_stats["mean_ms"] > 0

        finally:
            server_bridge.cleanup()
            client_bridge.cleanup()
