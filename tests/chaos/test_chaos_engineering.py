#!/usr/bin/env python3
"""
Chaos Engineering Tests - URC 2026 Rover

Deliberately injects failures and abnormal conditions to test system resilience.
Critical for ensuring the rover can handle real-world failure scenarios.

Chaos tests include:
- Network partitions and latency
- Service crashes and restarts
- Resource exhaustion
- Time synchronization issues
- Data corruption scenarios
"""

import asyncio
import queue
import random
import threading
import time
import unittest
from unittest.mock import AsyncMock, Mock, patch


class TestNetworkChaos(unittest.TestCase):
    """Test network-related chaos scenarios."""

    def test_network_partition_recovery(self):
        """Test recovery from complete network partition."""
        network_simulator = self.create_mock_network_simulator()

        # Establish normal communication
        client = network_simulator.create_client()
        self.assertTrue(client.connect())

        # Send normal messages
        for i in range(5):
            self.assertTrue(client.send_message(f"normal_{i}"))

        # Simulate network partition
        network_simulator.partition_network()

        # Messages should fail during partition
        for i in range(3):
            self.assertFalse(client.send_message(f"partitioned_{i}"))

        # Verify messages were buffered
        self.assertEqual(len(network_simulator.message_buffer), 3)

        # Restore network
        network_simulator.restore_network()

        # Messages should be sent from buffer
        client.flush_buffer()
        self.assertEqual(len(network_simulator.message_buffer), 0)

    def test_high_latency_handling(self):
        """Test system behavior under high network latency."""
        latency_simulator = self.create_mock_latency_simulator()

        # Set high latency (5 seconds)
        latency_simulator.set_latency(5.0)

        start_time = time.time()

        # Send message with high latency
        result = latency_simulator.send_with_latency("test_message")

        elapsed = time.time() - start_time

        # Should take at least the latency time
        self.assertGreaterEqual(elapsed, 5.0)
        self.assertTrue(result["success"])

        # Test timeout handling
        latency_simulator.set_timeout(2.0)  # 2 second timeout

        # This should timeout
        result_timeout = latency_simulator.send_with_latency("timeout_test")
        self.assertFalse(result_timeout["success"])
        self.assertIn("timeout", result_timeout["error"].lower())

    def test_packet_loss_simulation(self):
        """Test packet loss and retransmission."""
        packet_simulator = self.create_mock_packet_simulator()

        # Set 30% packet loss
        packet_simulator.set_loss_rate(0.3)

        successful_sends = 0
        total_attempts = 100

        for i in range(total_attempts):
            if packet_simulator.send_packet(f"packet_{i}"):
                successful_sends += 1

        # Should have roughly 70% success rate (with some variance)
        success_rate = successful_sends / total_attempts
        self.assertGreater(success_rate, 0.5)  # At least 50% success
        self.assertLess(success_rate, 0.9)  # Less than 90% success

        # Verify retransmission worked
        self.assertGreater(packet_simulator.retransmit_count, 0)

    def create_mock_network_simulator(self):
        """Create mock network simulator for chaos testing."""

        class MockNetworkSimulator:
            def __init__(self):
                self.network_up = True
                self.message_buffer = []

            def partition_network(self):
                self.network_up = False

            def restore_network(self):
                self.network_up = True

            def create_client(self):
                return MockNetworkClient(self)

        class MockNetworkClient:
            def __init__(self, simulator):
                self.simulator = simulator

            def connect(self):
                return self.simulator.network_up

            def send_message(self, message):
                if self.simulator.network_up:
                    return True
                else:
                    self.simulator.message_buffer.append(message)
                    return False

            def flush_buffer(self):
                if self.simulator.network_up:
                    self.simulator.message_buffer.clear()
                    return True
                return False

        return MockNetworkSimulator()

    def create_mock_latency_simulator(self):
        """Create mock latency simulator."""

        class MockLatencySimulator:
            def __init__(self):
                self.latency = 0.1
                self.timeout = 10.0

            def set_latency(self, latency):
                self.latency = latency

            def set_timeout(self, timeout):
                self.timeout = timeout

            def send_with_latency(self, message):
                start_time = time.time()

                # Simulate network delay
                time.sleep(min(self.latency, self.timeout))

                elapsed = time.time() - start_time

                if elapsed >= self.timeout:
                    return {"success": False, "error": f"Timeout after {elapsed:.2f}s"}

                return {"success": True, "elapsed": elapsed, "message": message}

        return MockLatencySimulator()

    def create_mock_packet_simulator(self):
        """Create mock packet loss simulator."""

        class MockPacketSimulator:
            def __init__(self):
                self.loss_rate = 0.0
                self.retransmit_count = 0

            def set_loss_rate(self, rate):
                self.loss_rate = rate

            def send_packet(self, packet, retransmit=False):
                if random.random() < self.loss_rate:
                    if not retransmit:
                        # Try retransmission
                        self.retransmit_count += 1
                        return self.send_packet(packet, retransmit=True)
                    else:
                        return False  # Give up after retransmission

                return True

        return MockPacketSimulator()


class TestServiceChaos(unittest.TestCase):
    """Test service failure and recovery scenarios."""

    def test_service_crash_recovery(self):
        """Test automatic recovery from service crashes."""
        service_manager = self.create_mock_service_manager()

        # Start services
        mission_service = service_manager.start_service("mission_executor")
        nav_service = service_manager.start_service("navigation")

        self.assertTrue(mission_service.is_alive())
        self.assertTrue(nav_service.is_alive())

        # Simulate mission service crash
        mission_service.crash()

        # Should be automatically restarted
        recovered_service = service_manager.get_service("mission_executor")
        self.assertTrue(recovered_service.is_alive())
        self.assertEqual(recovered_service.restart_count, 1)

    def test_cascading_failure_prevention(self):
        """Test prevention of cascading failures."""
        system_monitor = self.create_mock_system_monitor()

        # Normal operation
        system_monitor.add_service("mission", health=100)
        system_monitor.add_service("navigation", health=100)
        system_monitor.add_service("safety", health=100)

        # One service fails
        system_monitor.fail_service("navigation")

        # Other services should remain healthy (no cascading failure)
        self.assertEqual(system_monitor.get_service_health("mission"), 100)
        self.assertEqual(system_monitor.get_service_health("safety"), 100)

        # Circuit breaker should activate for failed service
        self.assertTrue(system_monitor.is_circuit_breaker_active("navigation"))

    def test_resource_exhaustion_handling(self):
        """Test handling of resource exhaustion."""
        resource_manager = self.create_mock_resource_manager()

        # Normal resource usage
        for i in range(10):
            self.assertTrue(resource_manager.allocate_memory(100))  # 100MB each

        # Approach memory limit
        self.assertTrue(resource_manager.allocate_memory(500))  # Near limit

        # Hit memory limit
        self.assertFalse(resource_manager.allocate_memory(600))  # Over limit

        # Automatic cleanup should occur
        resource_manager.trigger_cleanup()
        self.assertTrue(
            resource_manager.allocate_memory(200)
        )  # Should work after cleanup

    def test_time_synchronization_issues(self):
        """Test handling of time synchronization problems."""
        time_manager = self.create_mock_time_manager()

        # Normal time synchronization
        self.assertTrue(time_manager.is_synchronized())

        # Simulate NTP server failure
        time_manager.disconnect_ntp()

        # Time should drift
        time.sleep(0.1)  # Simulate time passage
        self.assertFalse(time_manager.is_synchronized())

        # System should use local time with drift compensation
        timestamp1 = time_manager.get_timestamp()
        time.sleep(0.05)
        timestamp2 = time_manager.get_timestamp()

        # Timestamps should still be monotonic
        self.assertGreater(timestamp2, timestamp1)

    def create_mock_service_manager(self):
        """Create mock service manager."""

        class MockServiceManager:
            def __init__(self):
                self.services = {}

            def start_service(self, name):
                service = Mock()
                service.name = name
                service.is_alive = Mock(return_value=True)
                service.crash = Mock(
                    side_effect=lambda: setattr(
                        service, "is_alive", Mock(return_value=False)
                    )
                )
                service.restart_count = 0
                self.services[name] = service
                return service

            def get_service(self, name):
                service = self.services.get(name)
                if service and not service.is_alive():
                    # Auto-restart
                    service.restart_count += 1
                    service.is_alive = Mock(return_value=True)
                return service

        return MockServiceManager()

    def create_mock_system_monitor(self):
        """Create mock system monitor."""

        class MockSystemMonitor:
            def __init__(self):
                self.services = {}
                self.circuit_breakers = {}

            def add_service(self, name, health=100):
                self.services[name] = health

            def fail_service(self, name):
                self.services[name] = 0
                self.circuit_breakers[name] = True

            def get_service_health(self, name):
                return self.services.get(name, 0)

            def is_circuit_breaker_active(self, name):
                return self.circuit_breakers.get(name, False)

        return MockSystemMonitor()

    def create_mock_resource_manager(self):
        """Create mock resource manager."""

        class MockResourceManager:
            def __init__(self):
                self.memory_limit = 1024  # 1GB
                self.allocated = 0

            def allocate_memory(self, mb):
                if self.allocated + mb > self.memory_limit:
                    return False
                self.allocated += mb
                return True

            def trigger_cleanup(self):
                # Free 50% of memory
                self.allocated = int(self.allocated * 0.5)

        return MockResourceManager()

    def create_mock_time_manager(self):
        """Create mock time manager."""

        class MockTimeManager:
            def __init__(self):
                self.ntp_connected = True
                self.drift_compensation = 0
                self.start_time = time.time()

            def disconnect_ntp(self):
                self.ntp_connected = False

            def is_synchronized(self):
                if not self.ntp_connected:
                    # Allow 1 second of drift before declaring unsynchronized
                    return (time.time() - self.start_time) < 1.0
                return True

            def get_timestamp(self):
                base_time = time.time()
                if not self.ntp_connected:
                    # Add drift compensation
                    return base_time + self.drift_compensation
                return base_time

        return MockTimeManager()


class TestDataCorruptionChaos(unittest.TestCase):
    """Test data corruption scenarios and recovery."""

    def test_corrupted_message_handling(self):
        """Test handling of corrupted network messages."""
        message_validator = self.create_mock_message_validator()

        # Valid messages
        valid_messages = [
            '{"type": "mission_command", "data": {"action": "start"}}',
            '{"sensor": "imu", "reading": {"x": 1.2, "y": 0.3}}',
        ]

        for msg in valid_messages:
            self.assertTrue(message_validator.validate(msg))

        # Corrupted messages
        corrupted_messages = [
            '{"type": "mission_command", "data": {"action": ',  # Incomplete JSON
            '{"sensor": "imu", "reading": null}',  # Null data
            "invalid json string",  # Not JSON at all
        ]

        for msg in corrupted_messages:
            self.assertFalse(message_validator.validate(msg))

        # System should continue processing after corruption
        self.assertTrue(message_validator.validate(valid_messages[0]))

    def test_database_corruption_recovery(self):
        """Test recovery from database corruption."""
        db_manager = self.create_mock_database_manager()

        # Store valid data
        test_data = {
            "mission_id": "test_001",
            "status": "active",
            "waypoints": [1, 2, 3],
        }
        db_manager.store("mission_state", test_data)

        # Simulate corruption
        db_manager.corrupt_database()

        # Attempt to load corrupted data (should fail gracefully)
        loaded_data = db_manager.load("mission_state")
        self.assertIsNone(loaded_data)  # Should return None for corrupted data

        # Recovery mechanism should activate
        recovery_success = db_manager.recover_from_backup()
        self.assertTrue(recovery_success)

        # Data should be restored
        recovered_data = db_manager.load("mission_state")
        self.assertEqual(recovered_data, test_data)

    def create_mock_message_validator(self):
        """Create mock message validator."""

        class MockMessageValidator:
            def validate(self, message):
                try:
                    import json

                    data = json.loads(message)
                    # Basic validation
                    if not isinstance(data, dict):
                        return False
                    if "type" not in data and "sensor" not in data:
                        return False
                    return True
                except (json.JSONDecodeError, TypeError):
                    return False

        return MockMessageValidator()

    def create_mock_database_manager(self):
        """Create mock database manager."""

        class MockDatabaseManager:
            def __init__(self):
                self.data = {}
                self.backup = {}

            def store(self, key, value):
                self.data[key] = value.copy()
                self.backup[key] = value.copy()

            def load(self, key):
                return self.data.get(key)

            def corrupt_database(self):
                # Simulate corruption by clearing data
                self.data.clear()

            def recover_from_backup(self):
                self.data = self.backup.copy()
                return True

        return MockDatabaseManager()


if __name__ == "__main__":
    unittest.main()
