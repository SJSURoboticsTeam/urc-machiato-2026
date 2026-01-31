#!/usr/bin/env python3
"""
Network Resilience Data Flow Test

Test data flow under network degradation conditions including
packet loss, high latency, and network partitions.

Author: URC 2026 Network Systems Team
"""

import asyncio
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Any, Callable
from unittest.mock import Mock, patch, AsyncMock
import pytest
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String, Header
from geometry_msgs.msg import Twist, PoseStamped


# Import network components
try:
    from src.bridges.websocket_manager import WebSocketManager
    from src.bridges.telemetry_manager import TelemetryManager
    from src.core.network_resilience import NetworkResilienceManager
except ImportError as e:
    pytest.skip(f"Skipping network resilience tests due to import error: {e}", allow_module_level=True)


@dataclass
class NetworkCondition:
    """Network condition for testing"""
    name: str
    packet_loss_rate: float = 0.0
    latency_ms: float = 0.0
    bandwidth_limit_mbps: float = 1000.0
    jitter_ms: float = 0.0
    connection_stability: float = 1.0  # 0.0 = unstable, 1.0 = stable
    
    @property
    def latency_seconds(self) -> float:
        return self.latency_ms / 1000.0
        
    @property
    def jitter_seconds(self) -> float:
        return self.jitter_ms / 1000.0


@dataclass
class NetworkMetrics:
    """Network performance metrics"""
    packets_sent: int = 0
    packets_received: int = 0
    bytes_sent: int = 0
    bytes_received: int = 0
    latencies: List[float] = field(default_factory=list)
    dropped_packets: int = 0
    
    @property
    def packet_loss_rate(self) -> float:
        if self.packets_sent == 0:
            return 0.0
        return self.dropped_packets / self.packets_sent
        
    @property
    def average_latency(self) -> float:
        if not self.latencies:
            return 0.0
        return np.mean(self.latencies)
        
    @property
    def throughput_mbps(self) -> float:
        if not self.latencies:
            return 0.0
        time_window = max(self.latencies) - min(self.latencies) + 0.001
        return (self.bytes_received * 8) / (time_window * 1e6)


class NetworkEmulator:
    """Emulate various network conditions for testing"""
    
    def __init__(self):
        self.current_condition: NetworkCondition = NetworkCondition("normal")
        self.packet_queue: List[Dict] = []
        self.metrics = NetworkMetrics()
        self.enabled = True
        
    def set_condition(self, condition: NetworkCondition):
        """Set network condition"""
        self.current_condition = condition
        print(f"Network condition set to: {condition.name}")
        
    def send_packet(self, packet_data: Dict, destination: str) -> bool:
        """Send packet with network condition applied"""
        if not self.enabled:
            return False
            
        self.metrics.packets_sent += 1
        self.metrics.bytes_sent += len(str(packet_data).encode())
        
        # Apply packet loss
        if np.random.random() < self.current_condition.packet_loss_rate:
            self.metrics.dropped_packets += 1
            return False
            
        # Apply latency
        delivery_time = time.time() + self.current_condition.latency_seconds
        
        # Apply jitter
        if self.current_condition.jitter_seconds > 0:
            jitter = np.random.normal(0, self.current_condition.jitter_seconds)
            delivery_time += max(0, jitter)
            
        # Queue packet for delivery
        self.packet_queue.append({
            'data': packet_data,
            'destination': destination,
            'delivery_time': delivery_time,
            'sent_time': time.time()
        })
        
        return True
        
    def process_deliveries(self) -> List[Dict]:
        """Process and return packets ready for delivery"""
        current_time = time.time()
        delivered_packets = []
        remaining_packets = []
        
        for packet in self.packet_queue:
            if packet['delivery_time'] <= current_time:
                # Calculate latency
                latency = current_time - packet['sent_time']
                self.metrics.latencies.append(latency)
                
                delivered_packets.append(packet)
                self.metrics.packets_received += 1
                self.metrics.bytes_received += len(str(packet['data']).encode())
            else:
                remaining_packets.append(packet)
                
        self.packet_queue = remaining_packets
        return delivered_packets
        
    def reset_metrics(self):
        """Reset network metrics"""
        self.metrics = NetworkMetrics()


class MockWebSocketManager(Node):
    """Mock WebSocket manager with network emulation"""
    
    def __init__(self, network_emulator: NetworkEmulator):
        super().__init__('mock_websocket_manager')
        self.network_emulator = network_emulator
        self.connected_clients: Dict[str, bool] = {}
        self.message_handlers: Dict[str, Callable] = {}
        self.received_messages: List[Dict] = []
        
        # Publishers for ROS2 integration
        self.status_publisher = self.create_publisher(String, '/websocket/status', 10)
        
    def connect_client(self, client_id: str) -> bool:
        """Connect client with network reliability"""
        if np.random.random() < self.network_emulator.current_condition.connection_stability:
            self.connected_clients[client_id] = True
            return True
        return False
        
    def disconnect_client(self, client_id: str):
        """Disconnect client"""
        self.connected_clients.pop(client_id, None)
        
    def send_message(self, client_id: str, message: Dict) -> bool:
        """Send message through network"""
        if client_id not in self.connected_clients:
            return False
            
        packet_data = {
            'type': 'message',
            'client_id': client_id,
            'message': message,
            'timestamp': time.time()
        }
        
        return self.network_emulator.send_packet(packet_data, client_id)
        
    def receive_messages(self) -> List[Dict]:
        """Process incoming messages"""
        delivered_packets = self.network_emulator.process_deliveries()
        
        for packet in delivered_packets:
            self.received_messages.append(packet['message'])
            
            # Call registered handler if exists
            message_type = packet['message'].get('type', 'unknown')
            if message_type in self.message_handlers:
                self.message_handlers[message_type](packet['message'])
                
        return [packet['message'] for packet in delivered_packets]
        
    def register_handler(self, message_type: str, handler: Callable):
        """Register message handler"""
        self.message_handlers[message_type] = handler


class MockTelemetryManager(Node):
    """Mock telemetry manager for data flow testing"""
    
    def __init__(self, websocket_manager: MockWebSocketManager):
        super().__init__('mock_telemetry_manager')
        self.websocket_manager = websocket_manager
        self.telemetry_data: Dict[str, Any] = {}
        self.subscribers = []
        
        # Setup message handler
        self.websocket_manager.register_handler('telemetry', self.handle_telemetry)
        
    def handle_telemetry(self, message: Dict):
        """Handle incoming telemetry data"""
        self.telemetry_data.update(message.get('data', {}))
        
    def send_telemetry(self, data: Dict, client_id: str = "dashboard") -> bool:
        """Send telemetry data"""
        message = {
            'type': 'telemetry',
            'data': data,
            'timestamp': time.time()
        }
        return self.websocket_manager.send_message(client_id, message)
        
    def get_latest_telemetry(self) -> Dict[str, Any]:
        """Get latest telemetry data"""
        return self.telemetry_data.copy()


class MockNetworkResilienceManager:
    """Mock network resilience manager"""
    
    def __init__(self, network_emulator: NetworkEmulator):
        self.network_emulator = network_emulator
        self.adaptive_strategies: Dict[str, bool] = {
            'retransmission': True,
            'compression': False,
            'prioritization': True,
            'redundancy': False
        }
        self.performance_history: List[Dict] = []
        
    def monitor_network_performance(self) -> Dict[str, float]:
        """Monitor current network performance"""
        metrics = self.network_emulator.metrics
        return {
            'packet_loss': metrics.packet_loss_rate,
            'latency_ms': metrics.average_latency * 1000,
            'throughput_mbps': metrics.throughput_mbps,
            'connection_stability': self.network_emulator.current_condition.connection_stability
        }
        
    def adapt_to_conditions(self):
        """Adapt strategies based on network conditions"""
        performance = self.monitor_network_performance()
        
        # Enable compression for low bandwidth
        if performance['throughput_mbps'] < 1.0:
            self.adaptive_strategies['compression'] = True
        else:
            self.adaptive_strategies['compression'] = False
            
        # Enable redundancy for high packet loss
        if performance['packet_loss'] > 0.1:
            self.adaptive_strategies['redundancy'] = True
        else:
            self.adaptive_strategies['redundancy'] = False
            
        # Update performance history
        self.performance_history.append({
            'timestamp': time.time(),
            'performance': performance,
            'strategies': self.adaptive_strategies.copy()
        })
        
    def get_current_strategies(self) -> Dict[str, bool]:
        """Get current adaptive strategies"""
        return self.adaptive_strategies.copy()


class NetworkResilienceMonitor:
    """Monitor network resilience and data flow integrity"""
    
    def __init__(self):
        self.test_results: List[Dict] = []
        self.data_integrity_checks: List[Dict] = []
        self.recovery_events: List[Dict] = []
        
    def record_test_result(self, condition: NetworkCondition, success: bool, 
                         metrics: Dict, data_integrity: Dict):
        """Record test result"""
        result = {
            'condition': condition.name,
            'success': success,
            'metrics': metrics,
            'data_integrity': data_integrity,
            'timestamp': time.time()
        }
        self.test_results.append(result)
        
    def record_recovery_event(self, condition: str, recovery_time: float, 
                           strategies_used: List[str]):
        """Record recovery event"""
        self.recovery_events.append({
            'condition': condition,
            'recovery_time': recovery_time,
            'strategies': strategies_used,
            'timestamp': time.time()
        })
        
    def get_test_summary(self) -> Dict[str, Any]:
        """Get summary of all test results"""
        if not self.test_results:
            return {}
            
        success_count = sum(1 for r in self.test_results if r['success'])
        total_tests = len(self.test_results)
        
        return {
            'total_tests': total_tests,
            'successful_tests': success_count,
            'success_rate': success_count / total_tests if total_tests > 0 else 0.0,
            'conditions_tested': list(set(r['condition'] for r in self.test_results)),
            'recovery_events': len(self.recovery_events)
        }


@pytest.fixture
def ros_context():
    """Provide ROS2 context for tests"""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def network_emulator():
    """Provide network emulator"""
    return NetworkEmulator()


@pytest.fixture
def websocket_manager(ros_context, network_emulator):
    """Provide mock WebSocket manager"""
    return MockWebSocketManager(network_emulator)


@pytest.fixture
def telemetry_manager(websocket_manager):
    """Provide mock telemetry manager"""
    return MockTelemetryManager(websocket_manager)


@pytest.fixture
def resilience_manager(network_emulator):
    """Provide mock network resilience manager"""
    return NetworkResilienceManager(network_emulator)


@pytest.fixture
def network_monitor():
    """Provide network resilience monitor"""
    return NetworkResilienceMonitor()


class TestNetworkResilienceDataflow:
    """Test network resilience and data flow integrity"""
    
    def test_high_latency_data_flow(
        self, ros_context, network_emulator, websocket_manager, telemetry_manager,
        resilience_manager, network_monitor
    ):
        """Test data flow under high network latency"""
        
        # Set high latency condition
        high_latency_condition = NetworkCondition(
            name="high_latency",
            packet_loss_rate=0.0,
            latency_ms=500.0,  # 500ms latency
            jitter_ms=50.0,
            bandwidth_limit_mbps=10.0
        )
        network_emulator.set_condition(high_latency_condition)
        
        # Connect client
        client_id = "test_dashboard"
        connected = websocket_manager.connect_client(client_id)
        assert connected, "Client should connect despite high latency"
        
        # Send test telemetry data
        test_data = {
            'position': {'x': 1.0, 'y': 2.0, 'z': 0.0},
            'battery_level': 0.75,
            'timestamp': time.time()
        }
        
        send_success = telemetry_manager.send_telemetry(test_data, client_id)
        assert send_success, "Telemetry send should succeed with high latency"
        
        # Wait for high latency delivery
        time.sleep(high_latency_condition.latency_seconds + 0.1)
        
        # Process deliveries
        received_messages = websocket_manager.receive_messages()
        
        # Verify data integrity despite latency
        assert len(received_messages) > 0, "Should receive messages despite high latency"
        
        received_data = received_messages[-1].get('data', {})
        assert received_data['position'] == test_data['position'], "Position data corrupted"
        assert received_data['battery_level'] == test_data['battery_level'], "Battery data corrupted"
        
        # Record test result
        performance = resilience_manager.monitor_network_performance()
        data_integrity = {
            'position_integrity': received_data.get('position') == test_data['position'],
            'battery_integrity': received_data.get('battery_level') == test_data['battery_level']
        }
        
        network_monitor.record_test_result(
            high_latency_condition, 
            True, 
            performance, 
            data_integrity
        )
        
        print(f"✅ High latency data flow: {performance['latency_ms']:.1f}ms")
        
    def test_packet_loss_data_recovery(
        self, ros_context, network_emulator, websocket_manager, telemetry_manager,
        resilience_manager, network_monitor
    ):
        """Test data recovery from packet loss"""
        
        # Set packet loss condition
        packet_loss_condition = NetworkCondition(
            name="packet_loss",
            packet_loss_rate=0.3,  # 30% packet loss
            latency_ms=50.0,
            bandwidth_limit_mbps=10.0
        )
        network_emulator.set_condition(packet_loss_condition)
        
        # Connect client
        client_id = "test_dashboard"
        websocket_manager.connect_client(client_id)
        
        # Send multiple telemetry messages
        test_messages = []
        for i in range(10):
            data = {
                'message_id': i,
                'position': {'x': i * 0.1, 'y': i * 0.2, 'z': 0.0},
                'battery_level': 0.8 - i * 0.01,
                'timestamp': time.time()
            }
            test_messages.append(data)
            telemetry_manager.send_telemetry(data, client_id)
            
        # Wait for all potential deliveries
        time.sleep(0.5)
        
        # Process deliveries
        received_messages = websocket_manager.receive_messages()
        
        # Verify some messages got through
        received_count = len(received_messages)
        expected_received = int(10 * (1 - packet_loss_condition.packet_loss_rate))
        
        assert received_count >= expected_received - 2, f"Too few messages received: {received_count} vs expected {expected_received}"
        
        # Verify data integrity of received messages
        received_ids = set()
        for msg in received_messages:
            msg_id = msg['data'].get('message_id')
            received_ids.add(msg_id)
            
            # Verify data integrity
            original_data = test_messages[msg_id]
            assert msg['data'] == original_data, f"Message {msg_id} data corrupted"
            
        print(f"✅ Packet loss recovery: {received_count}/10 messages received")
        
        # Record test result
        performance = resilience_manager.monitor_network_performance()
        data_integrity = {
            'messages_received': received_count,
            'data_corrupted': 0,
            'recovery_rate': received_count / 10.0
        }
        
        network_monitor.record_test_result(
            packet_loss_condition, 
            received_count > 5, 
            performance, 
            data_integrity
        )
        
    def test_network_partition_recovery(
        self, ros_context, network_emulator, websocket_manager, telemetry_manager,
        resilience_manager, network_monitor
    ):
        """Test system recovery from network partitions"""
        
        # Start with normal network
        normal_condition = NetworkCondition("normal")
        network_emulator.set_condition(normal_condition)
        
        # Connect client
        client_id = "test_dashboard"
        websocket_manager.connect_client(client_id)
        
        # Send initial data
        initial_data = {'status': 'connected', 'timestamp': time.time()}
        telemetry_manager.send_telemetry(initial_data, client_id)
        
        time.sleep(0.1)
        websocket_manager.receive_messages()
        
        # Simulate network partition
        partition_condition = NetworkCondition(
            name="partition",
            packet_loss_rate=1.0,  # Complete packet loss
            connection_stability=0.0  # Unstable connection
        )
        network_emulator.set_condition(partition_condition)
        
        # Try to send data during partition
        partition_data = {'status': 'partitioned', 'timestamp': time.time()}
        send_result = telemetry_manager.send_telemetry(partition_data, client_id)
        
        # Data should fail to send
        assert not send_result, "Send should fail during partition"
        
        # Wait and verify no messages received
        time.sleep(0.2)
        received_messages = websocket_manager.receive_messages()
        assert len(received_messages) == 0, "Should receive no messages during partition"
        
        # Recover from partition
        recovery_start = time.time()
        network_emulator.set_condition(normal_condition)
        
        # Reconnect client
        websocket_manager.connect_client(client_id)
        
        # Send recovery data
        recovery_data = {'status': 'recovered', 'timestamp': time.time()}
        telemetry_manager.send_telemetry(recovery_data, client_id)
        
        # Process recovery
        time.sleep(0.1)
        received_messages = websocket_manager.receive_messages()
        
        recovery_time = time.time() - recovery_start
        
        # Verify recovery
        assert len(received_messages) > 0, "Should receive messages after recovery"
        assert recovery_time < 2.0, f"Recovery too slow: {recovery_time:.1f}s"
        
        recovery_status = received_messages[-1]['data'].get('status')
        assert recovery_status == 'recovered', "Recovery data corrupted"
        
        # Record recovery event
        network_monitor.record_recovery_event(
            "partition", 
            recovery_time, 
            ["reconnection", "message_retransmission"]
        )
        
        print(f"✅ Network partition recovery: {recovery_time:.2f}s")
        
    def test_adaptive_strategies(
        self, ros_context, network_emulator, websocket_manager, telemetry_manager,
        resilience_manager, network_monitor
    ):
        """Test adaptive network strategies"""
        
        # Test conditions and expected strategies
        test_conditions = [
            (NetworkCondition("good_network", bandwidth_limit_mbps=100.0), ["retransmission"]),
            (NetworkCondition("slow_network", bandwidth_limit_mbps=0.5), ["retransmission", "compression"]),
            (NetworkCondition("lossy_network", packet_loss_rate=0.2), ["retransmission", "redundancy"]),
            (NetworkCondition("poor_network", packet_loss_rate=0.3, bandwidth_limit_mbps=0.5), 
             ["retransmission", "compression", "redundancy"])
        ]
        
        for condition, expected_strategies in test_conditions:
            print(f"Testing adaptive strategies for: {condition.name}")
            
            network_emulator.set_condition(condition)
            network_emulator.reset_metrics()
            
            # Adapt to conditions
            resilience_manager.adapt_to_conditions()
            
            # Get current strategies
            current_strategies = resilience_manager.get_current_strategies()
            
            # Verify expected strategies are enabled
            for strategy in expected_strategies:
                assert current_strategies.get(strategy, False), f"Strategy {strategy} should be enabled for {condition.name}"
                
            # Test data flow with adaptive strategies
            client_id = "test_dashboard"
            websocket_manager.connect_client(client_id)
            
            test_data = {'test_id': condition.name, 'timestamp': time.time()}
            send_success = telemetry_manager.send_telemetry(test_data, client_id)
            
            time.sleep(0.2)
            received_messages = websocket_manager.receive_messages()
            
            # Verify data got through
            if condition.packet_loss_rate < 1.0:
                assert len(received_messages) > 0, f"Data should get through with strategies for {condition.name}"
                
        print("✅ Adaptive strategies verified")
        
    def test_bandwidth_limit_handling(
        self, ros_context, network_emulator, websocket_manager, telemetry_manager,
        resilience_manager, network_monitor
    ):
        """Test handling of bandwidth limitations"""
        
        # Test different bandwidth limits
        bandwidth_tests = [
            (1.0, "1_mbps"),   # Very limited
            (5.0, "5_mbps"),   # Limited
            (100.0, "100_mbps") # Unlimited
        ]
        
        test_data_size = 1000  # 1KB test data
        
        for bandwidth_mbps, test_name in bandwidth_tests:
            print(f"Testing bandwidth limit: {test_name}")
            
            condition = NetworkCondition(
                name=f"bandwidth_{test_name}",
                bandwidth_limit_mbps=bandwidth_mbps
            )
            network_emulator.set_condition(condition)
            network_emulator.reset_metrics()
            
            # Send large amount of data
            client_id = "test_dashboard"
            websocket_manager.connect_client(client_id)
            
            messages_sent = 0
            start_time = time.time()
            
            # Send data for 2 seconds
            while time.time() - start_time < 2.0:
                test_data = {
                    'message_id': messages_sent,
                    'payload': 'x' * test_data_size,
                    'timestamp': time.time()
                }
                
                if telemetry_manager.send_telemetry(test_data, client_id):
                    messages_sent += 1
                    
                time.sleep(0.01)  # 100Hz send rate
                
            # Process deliveries
            time.sleep(0.5)
            received_messages = websocket_manager.receive_messages()
            
            # Calculate metrics
            actual_throughput = network_emulator.metrics.throughput_mbps
            messages_received = len(received_messages)
            
            # Verify bandwidth limitation is respected
            if bandwidth_mbps < 100:
                assert actual_throughput <= bandwidth_mbps * 1.2, f"Throughput {actual_throughput:.2f} exceeds limit {bandwidth_mbps}"
                
            # Verify some data got through
            assert messages_received > 0, f"No messages received for {test_name}"
            
            print(f"  Throughput: {actual_throughput:.2f} Mbps, Messages: {messages_received}/{messages_sent}")
            
        print("✅ Bandwidth limit handling verified")
        
    def test_concurrent_connections_under_stress(
        self, ros_context, network_emulator, websocket_manager, telemetry_manager,
        resilience_manager, network_monitor
    ):
        """Test concurrent connections under network stress"""
        
        # Set stressful network conditions
        stress_condition = NetworkCondition(
            name="stress",
            packet_loss_rate=0.1,
            latency_ms=200.0,
            jitter_ms=100.0,
            connection_stability=0.8
        )
        network_emulator.set_condition(stress_condition)
        
        # Connect multiple clients
        num_clients = 10
        clients = []
        
        for i in range(num_clients):
            client_id = f"client_{i}"
            if websocket_manager.connect_client(client_id):
                clients.append(client_id)
                
        assert len(clients) >= num_clients * 0.7, f"Too few clients connected: {len(clients)}/{num_clients}"
        
        # Send concurrent data from all clients
        messages_per_client = 5
        total_messages_sent = 0
        
        for client_id in clients:
            for i in range(messages_per_client):
                test_data = {
                    'client_id': client_id,
                    'message_id': i,
                    'timestamp': time.time()
                }
                
                if telemetry_manager.send_telemetry(test_data, client_id):
                    total_messages_sent += 1
                    
        # Process all deliveries
        time.sleep(1.0)
        received_messages = websocket_manager.receive_messages()
        
        # Verify data integrity
        received_by_client = {}
        for msg in received_messages:
            client_id = msg['data']['client_id']
            if client_id not in received_by_client:
                received_by_client[client_id] = 0
            received_by_client[client_id] += 1
            
        # Verify reasonable delivery rate
        total_received = len(received_messages)
        delivery_rate = total_received / total_messages_sent if total_messages_sent > 0 else 0
        
        assert delivery_rate > 0.5, f"Delivery rate too low: {delivery_rate:.2f}"
        assert len(received_by_client) >= len(clients) * 0.7, "Too few clients received messages"
        
        print(f"✅ Concurrent connections: {total_received}/{total_messages_sent} messages, {delivery_rate:.2f} rate")
        
        # Record test result
        performance = resilience_manager.monitor_network_performance()
        data_integrity = {
            'delivery_rate': delivery_rate,
            'active_clients': len(received_by_client),
            'total_clients': len(clients)
        }
        
        network_monitor.record_test_result(
            stress_condition, 
            delivery_rate > 0.5, 
            performance, 
            data_integrity
        )


if __name__ == "__main__":
    # Run tests manually for debugging
    pytest.main([__file__, "-v"])