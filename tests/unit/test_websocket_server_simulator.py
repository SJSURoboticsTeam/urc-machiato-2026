#!/usr/bin/env python3
"""
Unit Tests for WebSocket Server Simulator

Tests all functionality of the WebSocket/Socket.IO server simulator
including connection management, event handling, network simulation,
and message validation.

Author: URC 2026 Testing Team
"""

import pytest
import asyncio
import time
import json
import sys
from pathlib import Path
from typing import Dict, Any

# Add simulation to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from simulation.network.websocket_server_simulator import (
    WebSocketServerSimulator,
    ConnectionState,
    NetworkConditions,
    MessageRecord,
    create_websocket_simulator
)


class TestWebSocketServerSimulator:
    """Test suite for WebSocket server simulator."""
    
    @pytest.fixture
    def simulator(self):
        """Create basic simulator for testing."""
        sim = WebSocketServerSimulator({
            'network': {
                'latency_ms': 10.0,
                'packet_loss_rate': 0.0,
                'enabled': True
            },
            'validate_messages': True,
            'record_messages': True
        })
        yield sim
    
    @pytest.fixture
    def perfect_simulator(self):
        """Create simulator with no network delays."""
        sim = WebSocketServerSimulator({
            'network': {
                'latency_ms': 0.0,
                'jitter_ms': 0.0,
                'packet_loss_rate': 0.0,
                'enabled': False
            }
        })
        yield sim
    
    @pytest.fixture
    def lossy_simulator(self):
        """Create simulator with high packet loss."""
        sim = WebSocketServerSimulator({
            'network': {
                'latency_ms': 5.0,
                'packet_loss_rate': 0.5,  # 50% loss
                'enabled': True
            }
        })
        yield sim
    
    # Connection Management Tests
    
    async def test_client_connection(self, simulator):
        """Test client can connect successfully."""
        client_id = await simulator.connect()
        
        assert client_id is not None
        assert len(client_id) > 0
        assert client_id in simulator.connected_clients
        assert simulator.connection_state == ConnectionState.CONNECTED
    
    async def test_multiple_client_connections(self, simulator):
        """Test multiple clients can connect simultaneously."""
        client_ids = []
        for i in range(5):
            client_id = await simulator.connect()
            client_ids.append(client_id)
        
        assert len(simulator.connected_clients) == 5
        for client_id in client_ids:
            assert client_id in simulator.connected_clients
    
    async def test_client_disconnection(self, simulator):
        """Test client disconnection works correctly."""
        client_id = await simulator.connect()
        assert client_id in simulator.connected_clients
        
        await simulator.disconnect(client_id)
        assert client_id not in simulator.connected_clients
    
    async def test_connection_statistics(self, simulator):
        """Test connection statistics are tracked correctly."""
        # Connect 3 clients
        clients = []
        for i in range(3):
            client_id = await simulator.connect()
            clients.append(client_id)
        
        stats = simulator.get_statistics()
        assert stats['stats']['connections'] == 3
        assert stats['connected_clients'] == 3
        
        # Disconnect 1 client
        await simulator.disconnect(clients[0])
        stats = simulator.get_statistics()
        assert stats['connected_clients'] == 2
    
    # Event Handling Tests
    
    async def test_event_registration(self, simulator):
        """Test event handlers can be registered."""
        called = []
        
        async def handler(data):
            called.append(data)
        
        simulator.on('testEvent', handler)
        assert 'testEvent' in simulator.event_handlers
        assert len(simulator.event_handlers['testEvent']) == 1
    
    async def test_event_handler_execution(self, perfect_simulator):
        """Test event handlers are called with correct data."""
        simulator = perfect_simulator
        received_data = []
        
        async def handler(data):
            received_data.append(data)
        
        simulator.on('driveCommands', handler)
        
        # Emit event
        test_data = {'linear': 0.5, 'angular': 0.2}
        client_id = await simulator.connect()
        await simulator.receive('driveCommands', test_data, client_id)
        
        # Wait for async execution
        await asyncio.sleep(0.01)
        
        assert len(received_data) == 1
        assert received_data[0]['linear'] == 0.5
        assert received_data[0]['angular'] == 0.2
    
    async def test_multiple_handlers_same_event(self, perfect_simulator):
        """Test multiple handlers can be registered for same event."""
        simulator = perfect_simulator
        call_count = [0]
        
        async def handler1(data):
            call_count[0] += 1
        
        async def handler2(data):
            call_count[0] += 10
        
        simulator.on('testEvent', handler1)
        simulator.on('testEvent', handler2)
        
        client_id = await simulator.connect()
        await simulator.receive('testEvent', {}, client_id)
        await asyncio.sleep(0.01)
        
        assert call_count[0] == 11  # Both handlers called
    
    async def test_default_handler(self, perfect_simulator):
        """Test default handler catches unregistered events."""
        simulator = perfect_simulator
        caught_events = []
        
        async def default_handler(event_name, data):
            caught_events.append((event_name, data))
        
        simulator.set_default_handler(default_handler)
        
        client_id = await simulator.connect()
        await simulator.receive('unknownEvent', {'test': 'data'}, client_id)
        await asyncio.sleep(0.01)
        
        assert len(caught_events) == 1
        assert caught_events[0][0] == 'unknownEvent'
    
    # Message Validation Tests
    
    async def test_message_validation_valid(self, simulator):
        """Test valid messages pass validation."""
        test_data = {
            'linear': 0.5,
            'angular': 0.2,
            'timestamp': time.time()
        }
        
        validated = simulator._validate_message('driveCommands', test_data)
        assert validated is not None
        assert 'linear' in validated
        assert 'angular' in validated
    
    async def test_message_validation_adds_metadata(self, simulator):
        """Test validation adds required metadata."""
        test_data = {'linear': 0.5}
        
        validated = simulator._validate_message('driveCommands', test_data)
        assert 'timestamp' in validated
        assert 'message_id' in validated
    
    async def test_emit_to_client(self, perfect_simulator):
        """Test emitting message to specific client."""
        simulator = perfect_simulator
        client_id = await simulator.connect()
        
        test_data = {'status': 'ok', 'battery': 95.0}
        await simulator.emit('systemStatus', test_data, client_id)
        
        stats = simulator.get_statistics()
        assert stats['stats']['messages_sent'] == 1
    
    async def test_emit_to_all_clients(self, perfect_simulator):
        """Test broadcasting to all clients."""
        simulator = perfect_simulator
        
        # Connect multiple clients
        for i in range(3):
            await simulator.connect()
        
        # Broadcast message
        test_data = {'alert': 'test'}
        await simulator.emit('broadcast', test_data)
        
        # Should record 3 messages (one per client)
        stats = simulator.get_statistics()
        assert stats['stats']['messages_sent'] == 3
    
    # Network Simulation Tests
    
    async def test_network_delay_accuracy(self, simulator):
        """Test network delay simulation is approximately correct."""
        simulator.set_network_conditions({
            'latency_ms': 50.0,
            'jitter_ms': 0.0,
            'enabled': True
        })
        
        start = time.time()
        await simulator._apply_network_delay()
        duration_ms = (time.time() - start) * 1000
        
        # Allow 20% tolerance for async overhead
        assert 40 < duration_ms < 60, f"Delay {duration_ms:.1f}ms not near 50ms"
    
    async def test_network_jitter(self, simulator):
        """Test jitter adds variance to latency."""
        simulator.set_network_conditions({
            'latency_ms': 50.0,
            'jitter_ms': 20.0,
            'enabled': True
        })
        
        delays = []
        for i in range(10):
            start = time.time()
            await simulator._apply_network_delay()
            delays.append((time.time() - start) * 1000)
        
        # Check delays vary
        min_delay = min(delays)
        max_delay = max(delays)
        assert max_delay - min_delay > 10, "Jitter not adding variance"
    
    async def test_packet_loss_simulation(self, lossy_simulator):
        """Test packet loss causes some messages to be dropped."""
        simulator = lossy_simulator
        client_id = await simulator.connect()
        
        sent_count = 0
        received_count = 0
        
        # Send many messages
        for i in range(100):
            try:
                await simulator.emit('test', {'id': i}, client_id)
                sent_count += 1
            except Exception:
                pass
        
        # Check statistics
        stats = simulator.get_statistics()
        received_count = stats['stats']['messages_sent']
        
        # With 50% loss, expect roughly 40-60 messages received
        assert 30 < received_count < 70, f"Expected ~50/100, got {received_count}/100"
    
    async def test_disable_network_simulation(self, simulator):
        """Test network simulation can be disabled."""
        simulator.set_network_conditions({'enabled': False})
        
        start = time.time()
        await simulator._apply_network_delay()
        duration_ms = (time.time() - start) * 1000
        
        # Should be nearly instant
        assert duration_ms < 5
    
    # Statistics and Recording Tests
    
    async def test_message_recording(self, simulator):
        """Test messages are recorded in history."""
        client_id = await simulator.connect()
        
        test_data = {'linear': 0.5, 'angular': 0.0}
        await simulator.receive('driveCommands', test_data, client_id)
        
        history = simulator.get_message_history()
        assert len(history) > 0
        assert history[0]['event_name'] == 'driveCommands'
    
    async def test_message_history_limit(self, simulator):
        """Test message history respects max limit."""
        simulator.max_history = 10
        client_id = await simulator.connect()
        
        # Send more than limit
        for i in range(20):
            await simulator.receive('test', {'id': i}, client_id)
        
        history = simulator.get_message_history()
        assert len(history) <= 10
    
    async def test_clear_history(self, simulator):
        """Test clearing message history."""
        client_id = await simulator.connect()
        
        for i in range(5):
            await simulator.receive('test', {'id': i}, client_id)
        
        assert len(simulator.message_history) > 0
        
        simulator.clear_history()
        assert len(simulator.message_history) == 0
    
    async def test_statistics_accuracy(self, perfect_simulator):
        """Test statistics are accurately tracked."""
        simulator = perfect_simulator
        client_id = await simulator.connect()
        
        # Send and receive messages
        for i in range(10):
            await simulator.receive('driveCommands', {'linear': 0.5}, client_id)
            await simulator.emit('status', {'ok': True}, client_id)
        
        stats = simulator.get_statistics()
        assert stats['stats']['messages_received'] == 10
        assert stats['stats']['messages_sent'] == 10
    
    # Rover State Management Tests
    
    async def test_rover_state_update(self, simulator):
        """Test rover state can be updated."""
        test_state = {
            'position': {'x': 10.0, 'y': 20.0, 'z': 0.0},
            'battery': 85.0,
            'mode': 'autonomous'
        }
        
        simulator.update_rover_state(test_state)
        state = simulator.get_rover_state()
        
        assert state['position']['x'] == 10.0
        assert state['battery'] == 85.0
        assert state['mode'] == 'autonomous'
    
    async def test_rover_state_partial_update(self, simulator):
        """Test partial rover state updates."""
        simulator.update_rover_state({'battery': 100.0})
        simulator.update_rover_state({'mode': 'manual'})
        
        state = simulator.get_rover_state()
        assert state['battery'] == 100.0
        assert state['mode'] == 'manual'
    
    # History Export Tests
    
    async def test_export_history(self, simulator, tmp_path):
        """Test exporting message history to file."""
        client_id = await simulator.connect()
        
        for i in range(5):
            await simulator.receive('test', {'id': i}, client_id)
        
        export_file = tmp_path / "history.json"
        simulator.export_history(str(export_file))
        
        assert export_file.exists()
        
        # Verify file content
        with open(export_file, 'r') as f:
            data = json.load(f)
        
        assert 'message_history' in data
        assert len(data['message_history']) == 5
    
    # Factory Function Tests
    
    def test_create_default_simulator(self):
        """Test factory creates default simulator."""
        sim = create_websocket_simulator('default')
        assert sim is not None
        assert sim.network_conditions.latency_ms == 50.0
    
    def test_create_perfect_simulator(self):
        """Test factory creates perfect simulator."""
        sim = create_websocket_simulator('perfect')
        assert sim is not None
        assert sim.network_conditions.latency_ms == 0.0
        assert sim.network_conditions.packet_loss_rate == 0.0
    
    def test_create_stressed_simulator(self):
        """Test factory creates stressed simulator."""
        sim = create_websocket_simulator('stressed')
        assert sim is not None
        assert sim.network_conditions.latency_ms >= 200.0
        assert sim.network_conditions.packet_loss_rate >= 0.05
    
    # Error Handling Tests
    
    async def test_disconnect_nonexistent_client(self, simulator):
        """Test disconnecting non-existent client doesn't crash."""
        # Should not raise exception
        await simulator.disconnect("nonexistent_client_id")
    
    async def test_emit_to_disconnected_client(self, simulator):
        """Test emitting to disconnected client fails gracefully."""
        client_id = await simulator.connect()
        await simulator.disconnect(client_id)
        
        # Should not crash
        await simulator.emit('test', {}, client_id)
        
        # Message should not be sent
        stats = simulator.get_statistics()
        # Sent count might be 0 or handle gracefully
    
    async def test_invalid_event_data(self, simulator):
        """Test handling of invalid event data."""
        client_id = await simulator.connect()
        
        # None data should be handled
        validated = simulator._validate_message('test', None)
        assert validated is not None
        assert isinstance(validated, dict)
    
    # Concurrent Access Tests
    
    async def test_concurrent_message_sending(self, perfect_simulator):
        """Test multiple concurrent message sends."""
        simulator = perfect_simulator
        client_id = await simulator.connect()
        
        # Send multiple messages concurrently
        tasks = []
        for i in range(10):
            task = simulator.emit('test', {'id': i}, client_id)
            tasks.append(task)
        
        await asyncio.gather(*tasks)
        
        stats = simulator.get_statistics()
        assert stats['stats']['messages_sent'] == 10
    
    async def test_concurrent_client_connections(self, perfect_simulator):
        """Test multiple clients connecting concurrently."""
        simulator = perfect_simulator
        
        # Connect multiple clients concurrently
        tasks = [simulator.connect() for _ in range(10)]
        client_ids = await asyncio.gather(*tasks)
        
        assert len(simulator.connected_clients) == 10
        assert len(set(client_ids)) == 10  # All unique


# Performance benchmark tests
@pytest.mark.benchmark
class TestWebSocketSimulatorPerformance:
    """Performance tests for WebSocket simulator."""
    
    async def test_message_throughput(self, perfect_simulator):
        """Test message throughput under load."""
        simulator = perfect_simulator
        client_id = await simulator.connect()
        
        start = time.time()
        count = 1000
        
        for i in range(count):
            await simulator.receive('test', {'id': i}, client_id)
        
        duration = time.time() - start
        throughput = count / duration
        
        # Should handle at least 500 msg/s
        assert throughput > 500, f"Throughput too low: {throughput:.0f} msg/s"
    
    async def test_memory_usage_stable(self, perfect_simulator):
        """Test memory usage doesn't grow unbounded."""
        simulator = perfect_simulator
        simulator.max_history = 100
        client_id = await simulator.connect()
        
        # Send many messages
        for i in range(1000):
            await simulator.receive('test', {'id': i}, client_id)
        
        # History should be limited
        assert len(simulator.message_history) <= 100


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
