#!/usr/bin/env python3
"""
Secondary WebSocket Bridge

Provides secondary WebSocket endpoint for telemetry streaming with
reduced functionality compared to primary bridge.

Author: URC 2026 Autonomy Team
"""

import asyncio
import json
import os

# Add src to path for imports
import sys
import threading
import time
from typing import Any, Dict, Optional

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from bridges.websocket_redundancy_manager import (
    EndpointPriority,
    WebSocketEndpoint,
    get_redundancy_manager,
)
from core.dynamic_config_manager import get_dynamic_config_manager
from core.state_synchronization_manager import get_state_manager


class SecondaryWebSocketBridge:
    """
    Secondary WebSocket bridge with limited telemetry scope.
    Provides backup WebSocket connectivity with essential telemetry only.
    """

    def __init__(self):
        self.port = 8081  # Secondary port
        self.max_clients = 25  # Fewer clients than primary

        # Reduced telemetry scope - essential data only
        self.telemetry_scope = [
            'timestamp', 'battery_level', 'system_health',
            'position', 'velocity', 'emergency_stop'
        ]

        # WebSocket server
        self.websocket_server = None
        self.clients = set()
        self.running = False

        # Managers
        self.redundancy_manager = get_redundancy_manager()
        self.state_manager = get_state_manager("secondary_bridge")
        self.config_manager = get_dynamic_config_manager()

        # Telemetry data (simulated)
        self.telemetry_data = {
            'timestamp': time.time(),
            'battery_level': 85.0,
            'system_health': 'nominal',
            'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'emergency_stop': False
        }

        # Configuration
        self.update_rate_hz = 2.0  # Slower than primary

    def start(self):
        """Start the secondary WebSocket bridge."""
        self.get_logger().info("[UPDATE] Starting Secondary WebSocket Bridge...")
        # Register with managers
        self._register_with_managers()

        # Start WebSocket server
        self.running = True
        self.websocket_thread = threading.Thread(target=self._run_websocket_server, daemon=True)
        self.websocket_thread.start()

        # Start telemetry updates
        self.telemetry_thread = threading.Thread(target=self._telemetry_loop, daemon=True)
        self.telemetry_thread.start()
        self.get_logger().info(f"[SUCCESS] Secondary WebSocket Bridge started on port {self.port}")
    def stop(self):
        """Stop the secondary WebSocket bridge."""
        self.get_logger().info("[STOP] Stopping Secondary WebSocket Bridge...")
        self.running = False

        if self.websocket_server:
            self.websocket_server.close()

        if hasattr(self, 'websocket_thread') and self.websocket_thread.is_alive():
            self.websocket_thread.join(timeout=2.0)

        if hasattr(self, 'telemetry_thread') and self.telemetry_thread.is_alive():
            self.telemetry_thread.join(timeout=2.0)
        self.get_logger().info("[SUCCESS] Secondary WebSocket Bridge stopped")
    def _register_with_managers(self):
        """Register with various system managers."""
        # WebSocket redundancy
        endpoint = WebSocketEndpoint(
            name="secondary_bridge",
            port=self.port,
            priority=EndpointPriority.SECONDARY,
            telemetry_scope=self.telemetry_scope,
            max_clients=self.max_clients
        )
        self.redundancy_manager.add_endpoint(endpoint)

        # State synchronization
        self.state_manager.start()
        self.state_manager.add_state_callback(self._on_state_change)

        # Dynamic configuration
        self.config_manager.register_node("secondary_bridge", {
            'update_rate_hz': self.update_rate_hz,
            'max_clients': self.max_clients
        })
        self.config_manager.add_update_callback(self._on_config_update)

    def _run_websocket_server(self):
        """Run the WebSocket server."""
        try:
            import websockets
        except ImportError:
            self.get_logger().info("websockets library not available, simulating WebSocket server")
            while self.running:
                time.sleep(1)
            return

        async def handle_client(websocket, path):
            """Handle individual WebSocket client connections."""
            self.clients.add(websocket)
            client_id = f"secondary_{len(self.clients)}"
        self.get_logger().info(f"[SATELLITE] Secondary client connected: {client_id}")
        try:
                # Send initial telemetry
                await websocket.send(json.dumps({
                    'type': 'initial',
                    'data': self.telemetry_data
                }))

                # Keep connection alive
                async for message in websocket:
                    # Handle any client messages if needed
                    pass

            except websockets.exceptions.ConnectionClosed:
                self.get_logger().info(f"[SATELLITE] Secondary client disconnected: {client_id}")
            finally:
                self.clients.discard(websocket)

        # Start WebSocket server
        start_server = websockets.serve(handle_client, "localhost", self.port)
        asyncio.get_event_loop().run_until_complete(start_server)
        asyncio.get_event_loop().run_forever()

    def _telemetry_loop(self):
        """Main telemetry update loop."""
        while self.running:
            try:
                # Update telemetry data
                self.telemetry_data['timestamp'] = time.time()

                # Simulate some telemetry changes
                import random
                self.telemetry_data['battery_level'] = max(10, min(100,
                    self.telemetry_data['battery_level'] + random.uniform(-2, 1)))

                # Send to connected clients
                self._broadcast_telemetry()

                # Sync state
                self.state_manager.update_state('secondary_battery_level', self.telemetry_data['battery_level'])

                time.sleep(1.0 / self.update_rate_hz)

            except Exception as e:
        self.get_logger().info(f"Secondary telemetry error: {e}")
                time.sleep(1)

    def _broadcast_telemetry(self):
        """Broadcast telemetry to all connected clients."""
        if not self.clients:
            return

        message = json.dumps({
            'type': 'telemetry',
            'data': self.telemetry_data
        })

        # In real implementation, would use asyncio to send to all clients
        self.get_logger().info(f"[SATELLITE] Secondary broadcasting telemetry to {len(self.clients)} clients")
    def _on_state_change(self, key: str, entry):
        """Handle state synchronization updates."""
        if key == 'system_health':
            self.telemetry_data['system_health'] = entry.value
        elif key == 'emergency_stop':
            self.telemetry_data['emergency_stop'] = entry.value

    def _on_config_update(self, snapshot):
        """Handle dynamic configuration updates."""
        for change in snapshot.changes:
            if change.node_name == "secondary_bridge":
                if change.parameter_name == 'update_rate_hz':
                    self.update_rate_hz = change.new_value
        self.get_logger().info(f"Secondary update rate changed to {change.new_value}Hz")
def main():
    """Main function for secondary bridge."""
    import signal
    import sys

    bridge = SecondaryWebSocketBridge()

    def signal_handler(sig, frame):
        bridge.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        bridge.start()
        # Keep running
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.stop()


if __name__ == "__main__":
    main()
