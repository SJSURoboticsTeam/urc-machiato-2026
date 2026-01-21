#!/usr/bin/env python3
"""
WebSocket/Socket.IO Server Simulator for Teleoperation Testing

Simulates the teleoperation server for testing frontend/backend integration
without requiring the actual py_server.py or hardware.

Features:
- Socket.IO event handling (driveCommands, emergencyStop, etc.)
- Realistic network delays and disconnections
- Message validation and error injection
- Connection state management
- Message recording for replay/analysis

Author: URC 2026 Simulation Team
"""

import asyncio
import logging
import time
import random
import json
from typing import Dict, Any, List, Optional, Callable, Set
from dataclasses import dataclass, field
from enum import Enum
import uuid

logger = logging.getLogger(__name__)


class ConnectionState(Enum):
    """WebSocket connection states."""
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    RECONNECTING = "reconnecting"
    ERROR = "error"


@dataclass
class NetworkConditions:
    """Network condition parameters for simulation."""
    latency_ms: float = 50.0  # Average latency
    jitter_ms: float = 10.0   # Latency variance
    packet_loss_rate: float = 0.01  # 1% packet loss
    disconnect_rate: float = 0.001  # 0.1% chance of disconnect per message
    bandwidth_mbps: float = 100.0  # Available bandwidth
    enabled: bool = True


@dataclass
class MessageRecord:
    """Record of a Socket.IO message for analysis."""
    message_id: str
    timestamp: float
    event_name: str
    data: Dict[str, Any]
    direction: str  # 'inbound' or 'outbound'
    latency_ms: float = 0.0
    dropped: bool = False


class WebSocketServerSimulator:
    """
    Simulates teleoperation WebSocket server for testing.
    
    Provides complete Socket.IO event simulation with realistic
    network conditions, validation, and recording.
    """
    
    def __init__(self, config: Optional[Dict[str, Any]] = None):
        """Initialize WebSocket server simulator.
        
        Args:
            config: Configuration dictionary with network conditions,
                   validation settings, and recording options
        """
        self.logger = logging.getLogger(f"{__name__}.WebSocketServerSimulator")
        
        # Configuration
        config = config or {}
        self.network_conditions = NetworkConditions(**config.get('network', {}))
        self.validate_messages = config.get('validate_messages', True)
        self.record_messages = config.get('record_messages', True)
        
        # Connection state
        self.connection_state = ConnectionState.DISCONNECTED
        self.connected_clients: Set[str] = set()
        self.client_metadata: Dict[str, Dict[str, Any]] = {}
        
        # Event handlers
        self.event_handlers: Dict[str, List[Callable]] = {}
        self.default_handler: Optional[Callable] = None
        
        # Message recording
        self.message_history: List[MessageRecord] = []
        self.max_history = config.get('max_history', 10000)
        
        # Statistics
        self.stats = {
            'messages_sent': 0,
            'messages_received': 0,
            'messages_dropped': 0,
            'connections': 0,
            'disconnections': 0,
            'errors': 0
        }
        
        # State for simulation
        self.rover_state = {
            'linear_velocity': 0.0,
            'angular_velocity': 0.0,
            'battery_voltage': 24.0,
            'battery_percentage': 100.0,
            'motor_currents': [0.0] * 6,
            'temperatures': [25.0] * 6,
            'emergency_stop': False,
            'homing_complete': False,
            'last_update': time.time()
        }
        
        self.logger.info("WebSocket server simulator initialized")
    
    def on(self, event_name: str, handler: Callable):
        """Register event handler.
        
        Args:
            event_name: Name of Socket.IO event (e.g., 'driveCommands')
            handler: Async function to handle the event
        """
        if event_name not in self.event_handlers:
            self.event_handlers[event_name] = []
        self.event_handlers[event_name].append(handler)
        self.logger.debug(f"Registered handler for event: {event_name}")
    
    def set_default_handler(self, handler: Callable):
        """Set default handler for unregistered events."""
        self.default_handler = handler
    
    async def connect(self, client_id: Optional[str] = None) -> str:
        """Simulate client connection.
        
        Args:
            client_id: Optional client identifier
            
        Returns:
            str: Client ID for the connection
        """
        client_id = client_id or str(uuid.uuid4())
        
        # Simulate connection delay
        await self._apply_network_delay()
        
        self.connection_state = ConnectionState.CONNECTED
        self.connected_clients.add(client_id)
        self.client_metadata[client_id] = {
            'connected_at': time.time(),
            'messages_sent': 0,
            'messages_received': 0
        }
        self.stats['connections'] += 1
        
        self.logger.info(f"Client connected: {client_id}")
        
        # Emit connection event
        await self._emit_to_handlers('connect', {'client_id': client_id})
        
        return client_id
    
    async def disconnect(self, client_id: str):
        """Simulate client disconnection.
        
        Args:
            client_id: Client identifier to disconnect
        """
        if client_id in self.connected_clients:
            self.connected_clients.remove(client_id)
            if client_id in self.client_metadata:
                del self.client_metadata[client_id]
            
            self.stats['disconnections'] += 1
            self.logger.info(f"Client disconnected: {client_id}")
            
            # Emit disconnection event
            await self._emit_to_handlers('disconnect', {'client_id': client_id})
            
            if len(self.connected_clients) == 0:
                self.connection_state = ConnectionState.DISCONNECTED
    
    async def emit(self, event_name: str, data: Dict[str, Any], 
                   client_id: Optional[str] = None) -> bool:
        """Emit event to client(s).
        
        Args:
            event_name: Name of the event
            data: Event data payload
            client_id: Optional specific client to emit to
            
        Returns:
            bool: True if message sent successfully
        """
        # Check if clients connected
        if not self.connected_clients:
            self.logger.warning(f"Cannot emit '{event_name}': No clients connected")
            return False
        
        # Apply network conditions
        if self.network_conditions.enabled:
            # Simulate packet loss
            if random.random() < self.network_conditions.packet_loss_rate:
                self.stats['messages_dropped'] += 1
                self._record_message(event_name, data, 'outbound', dropped=True)
                self.logger.debug(f"Message dropped (simulated packet loss): {event_name}")
                return False
            
            # Simulate network delay
            await self._apply_network_delay()
        
        # Validate message
        if self.validate_messages:
            validation_result = self._validate_message(event_name, data)
            if not validation_result['valid']:
                self.logger.error(f"Invalid message: {validation_result['error']}")
                self.stats['errors'] += 1
                return False
        
        # Record message
        latency = await self._apply_network_delay()
        self._record_message(event_name, data, 'outbound', latency_ms=latency)
        
        self.stats['messages_sent'] += 1
        
        # Update client metadata
        if client_id and client_id in self.client_metadata:
            self.client_metadata[client_id]['messages_sent'] += 1
        
        self.logger.debug(f"Emitted event '{event_name}' to {len(self.connected_clients)} client(s)")
        return True
    
    async def receive(self, event_name: str, data: Dict[str, Any], 
                     client_id: Optional[str] = None) -> bool:
        """Receive event from client.
        
        Args:
            event_name: Name of the event
            data: Event data payload
            client_id: Client sending the event
            
        Returns:
            bool: True if message processed successfully
        """
        # Apply network conditions
        if self.network_conditions.enabled:
            # Simulate packet loss
            if random.random() < self.network_conditions.packet_loss_rate:
                self.stats['messages_dropped'] += 1
                self._record_message(event_name, data, 'inbound', dropped=True)
                return False
            
            # Simulate network delay
            await self._apply_network_delay()
            
            # Simulate random disconnection
            if random.random() < self.network_conditions.disconnect_rate:
                if client_id:
                    await self.disconnect(client_id)
                return False
        
        # Validate message
        if self.validate_messages:
            validation_result = self._validate_message(event_name, data)
            if not validation_result['valid']:
                self.logger.error(f"Invalid message: {validation_result['error']}")
                self.stats['errors'] += 1
                return False
        
        # Record message
        latency = self.network_conditions.latency_ms if self.network_conditions.enabled else 0.0
        self._record_message(event_name, data, 'inbound', latency_ms=latency)
        
        self.stats['messages_received'] += 1
        
        # Update client metadata
        if client_id and client_id in self.client_metadata:
            self.client_metadata[client_id]['messages_received'] += 1
        
        # Process the event
        await self._emit_to_handlers(event_name, data)
        
        return True
    
    async def _emit_to_handlers(self, event_name: str, data: Dict[str, Any]):
        """Emit event to registered handlers.
        
        Args:
            event_name: Name of the event
            data: Event data
        """
        handlers = self.event_handlers.get(event_name, [])
        
        if not handlers and self.default_handler:
            handlers = [self.default_handler]
        
        for handler in handlers:
            try:
                if asyncio.iscoroutinefunction(handler):
                    await handler(data)
                else:
                    handler(data)
            except Exception as e:
                self.logger.error(f"Error in handler for '{event_name}': {e}")
                self.stats['errors'] += 1
    
    async def _apply_network_delay(self) -> float:
        """Apply simulated network delay.
        
        Returns:
            float: Applied delay in milliseconds
        """
        if not self.network_conditions.enabled:
            return 0.0
        
        # Calculate delay with jitter
        delay_ms = random.gauss(
            self.network_conditions.latency_ms,
            self.network_conditions.jitter_ms
        )
        delay_ms = max(0.0, delay_ms)  # No negative delays
        
        # Apply delay
        await asyncio.sleep(delay_ms / 1000.0)
        
        return delay_ms
    
    def _validate_message(self, event_name: str, data: Dict[str, Any]) -> Dict[str, Any]:
        """Validate message format.
        
        Args:
            event_name: Name of the event
            data: Event data to validate
            
        Returns:
            Dict with 'valid' bool and optional 'error' message
        """
        # Define expected schemas for each event type
        schemas = {
            'driveCommands': {
                'required_fields': ['linear', 'angular'],
                'optional_fields': ['timestamp'],
                'field_types': {'linear': (int, float), 'angular': (int, float)}
            },
            'emergencyStop': {
                'required_fields': [],
                'optional_fields': ['reason'],
                'field_types': {}
            },
            'homingRequest': {
                'required_fields': [],
                'optional_fields': [],
                'field_types': {}
            },
            'systemStatus': {
                'required_fields': [],
                'optional_fields': ['battery', 'motors', 'sensors'],
                'field_types': {}
            }
        }
        
        # Check if we have a schema for this event
        if event_name not in schemas:
            return {'valid': True, 'note': 'No validation schema for this event'}
        
        schema = schemas[event_name]
        
        # Check required fields
        for field in schema['required_fields']:
            if field not in data:
                return {'valid': False, 'error': f"Missing required field: {field}"}
        
        # Check field types
        for field, expected_types in schema['field_types'].items():
            if field in data and not isinstance(data[field], expected_types):
                return {
                    'valid': False,
                    'error': f"Invalid type for {field}: expected {expected_types}, got {type(data[field])}"
                }
        
        return {'valid': True}
    
    def _record_message(self, event_name: str, data: Dict[str, Any], 
                       direction: str, latency_ms: float = 0.0, dropped: bool = False):
        """Record message for analysis.
        
        Args:
            event_name: Name of the event
            data: Event data
            direction: 'inbound' or 'outbound'
            latency_ms: Applied latency
            dropped: Whether message was dropped
        """
        if not self.record_messages:
            return
        
        record = MessageRecord(
            message_id=str(uuid.uuid4()),
            timestamp=time.time(),
            event_name=event_name,
            data=data.copy(),
            direction=direction,
            latency_ms=latency_ms,
            dropped=dropped
        )
        
        self.message_history.append(record)
        
        # Trim history if too long
        if len(self.message_history) > self.max_history:
            self.message_history = self.message_history[-self.max_history:]
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get simulator statistics.
        
        Returns:
            Dict with statistics and current state
        """
        return {
            'stats': self.stats.copy(),
            'connection_state': self.connection_state.value,
            'connected_clients': len(self.connected_clients),
            'message_history_size': len(self.message_history),
            'network_conditions': {
                'latency_ms': self.network_conditions.latency_ms,
                'packet_loss_rate': self.network_conditions.packet_loss_rate,
                'enabled': self.network_conditions.enabled
            }
        }
    
    def get_message_history(self, count: Optional[int] = None, 
                           event_name: Optional[str] = None) -> List[MessageRecord]:
        """Get message history.
        
        Args:
            count: Number of recent messages to return
            event_name: Filter by event name
            
        Returns:
            List of message records
        """
        history = self.message_history
        
        if event_name:
            history = [msg for msg in history if msg.event_name == event_name]
        
        if count:
            history = history[-count:]
        
        return history
    
    def clear_history(self):
        """Clear message history."""
        self.message_history.clear()
        self.logger.info("Message history cleared")
    
    def set_network_conditions(self, conditions: Dict[str, Any]):
        """Update network conditions.
        
        Args:
            conditions: Dictionary with network parameters
        """
        if 'latency_ms' in conditions:
            self.network_conditions.latency_ms = conditions['latency_ms']
        if 'jitter_ms' in conditions:
            self.network_conditions.jitter_ms = conditions['jitter_ms']
        if 'packet_loss_rate' in conditions:
            self.network_conditions.packet_loss_rate = conditions['packet_loss_rate']
        if 'disconnect_rate' in conditions:
            self.network_conditions.disconnect_rate = conditions['disconnect_rate']
        if 'enabled' in conditions:
            self.network_conditions.enabled = conditions['enabled']
        
        self.logger.info(f"Network conditions updated: {conditions}")
    
    def update_rover_state(self, state: Dict[str, Any]):
        """Update simulated rover state.
        
        Args:
            state: Dictionary with rover state updates
        """
        self.rover_state.update(state)
        self.rover_state['last_update'] = time.time()
    
    def get_rover_state(self) -> Dict[str, Any]:
        """Get current simulated rover state.
        
        Returns:
            Dict with rover state
        """
        return self.rover_state.copy()
    
    def export_history(self, filename: str):
        """Export message history to file.
        
        Args:
            filename: Path to output file
        """
        try:
            history_data = [
                {
                    'message_id': msg.message_id,
                    'timestamp': msg.timestamp,
                    'event_name': msg.event_name,
                    'data': msg.data,
                    'direction': msg.direction,
                    'latency_ms': msg.latency_ms,
                    'dropped': msg.dropped
                }
                for msg in self.message_history
            ]
            
            with open(filename, 'w') as f:
                json.dump({
                    'export_time': time.time(),
                    'statistics': self.get_statistics(),
                    'messages': history_data
                }, f, indent=2)
            
            self.logger.info(f"Message history exported to {filename}")
        except Exception as e:
            self.logger.error(f"Failed to export history: {e}")


# Convenience function for creating simulator with common configurations
def create_websocket_simulator(profile: str = 'default') -> WebSocketServerSimulator:
    """Create WebSocket simulator with predefined configuration.
    
    Args:
        profile: Configuration profile ('default', 'perfect', 'poor', 'extreme')
        
    Returns:
        Configured WebSocketServerSimulator instance
    """
    profiles = {
        'default': {
            'network': {
                'latency_ms': 50.0,
                'jitter_ms': 10.0,
                'packet_loss_rate': 0.01,
                'disconnect_rate': 0.001
            }
        },
        'perfect': {
            'network': {
                'latency_ms': 0.0,
                'jitter_ms': 0.0,
                'packet_loss_rate': 0.0,
                'disconnect_rate': 0.0,
                'enabled': False
            }
        },
        'poor': {
            'network': {
                'latency_ms': 200.0,
                'jitter_ms': 50.0,
                'packet_loss_rate': 0.05,
                'disconnect_rate': 0.01
            }
        },
        'extreme': {
            'network': {
                'latency_ms': 500.0,
                'jitter_ms': 100.0,
                'packet_loss_rate': 0.15,
                'disconnect_rate': 0.05
            }
        }
    }
    
    config = profiles.get(profile, profiles['default'])
    return WebSocketServerSimulator(config)
