#!/usr/bin/env python3
"""
Tertiary WebSocket Bridge

Provides tertiary/emergency WebSocket endpoint for minimal telemetry
streaming during critical situations.

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


class TertiaryWebSocketBridge:
    """
    Tertiary WebSocket bridge with minimal telemetry scope.
    Provides emergency connectivity with critical telemetry only.
    """

    def __init__(self):
        self.port = 8082  # Tertiary port
        self.max_clients = 10  # Minimal clients for emergency use

        # Minimal telemetry scope - critical data only
        self.telemetry_scope = [
            'timestamp', 'battery_level', 'emergency_stop',
            'system_health', 'critical_errors'
        ]

        # WebSocket server
        self.websocket_server = None
        self.clients = set()
        self.running = False

        # Managers
        self.redundancy_manager = get_redundancy_manager()
        self.state_manager = get_state_manager("tertiary_bridge")
        self.config_manager = get_dynamic_config_manager()

        # Minimal telemetry data (simulated)
        self.telemetry_data = {
            'timestamp': time.time(),
            'battery_level': 80.0,
            'emergency_stop': False,
            'system_health': 'nominal',
            'critical_errors': []
        }

        # Configuration
        self.update_rate_hz = 1.0  # Very slow updates for emergency

    def start(self):
        """Start the tertiary WebSocket bridge."""
        self.get_logger().info("[ALERT] Starting Tertiary WebSocket Bridge...")
        # Register with managers
        self._register_with_managers()

        # Start WebSocket server
        self.running = True
        self.websocket_thread = threading.Thread(target=self._run_websocket_server, daemon=True)
        self.websocket_thread.start()

        # Start telemetry updates
        self.telemetry_thread = threading.Thread(target=self._telemetry_loop, daemon=True)
        self.telemetry_thread.start()
        self.get_logger().info(f"[SUCCESS] Tertiary WebSocket Bridge started on port {self.port}")
    def stop(self):
        """Stop the tertiary WebSocket bridge."""
        self.get_logger().info("[STOP] Stopping Tertiary WebSocket Bridge...")
        self.running = False

        if self.websocket_server:
            self.websocket_server.close()

        if hasattr(self, 'websocket_thread') and self.websocket_thread.is_alive():
            self.websocket_thread.join(timeout=2.0)

        if hasattr(self, 'telemetry_thread') and self.telemetry_thread.is_alive():
            self.telemetry_thread.join(timeout=2.0)
        self.get_logger().info("[SUCCESS] Tertiary WebSocket Bridge stopped")
    def _register_with_managers(self):
        """Register with various system managers."""
        # WebSocket redundancy
        endpoint = WebSocketEndpoint(
            name="tertiary_bridge",
            port=self.port,
            priority=EndpointPriority.TERTIARY,
            telemetry_scope=self.telemetry_scope,
            max_clients=self.max_clients
        )
        self.redundancy_manager.add_endpoint(endpoint)

        # State synchronization
        self.state_manager.start()
        self.state_manager.add_state_callback(self._on_state_change)

        # Dynamic configuration
        self.config_manager.register_node("tertiary_bridge", {
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
            client_id = f"tertiary_{len(self.clients)}"
        self.get_logger().info(f"[ALERT] Tertiary client connected: {client_id}")
        try:
                # Send initial critical telemetry
                await websocket.send(json.dumps({
                    'type': 'emergency_initial',
                    'data': self.telemetry_data
                }))

                # Keep connection alive with minimal heartbeat
                while self.running:
                    await asyncio.sleep(5)  # Slow heartbeat for emergency mode
                    await websocket.send(json.dumps({
                        'type': 'emergency_heartbeat',
                        'timestamp': time.time()
                    }))

            except websockets.exceptions.ConnectionClosed:
                self.get_logger().info(f"[ALERT] Tertiary client disconnected: {client_id}")
            finally:
                self.clients.discard(websocket)

        # Start WebSocket server
        start_server = websockets.serve(handle_client, "localhost", self.port)
        asyncio.get_event_loop().run_until_complete(start_server)
        asyncio.get_event_loop().run_forever()

    def _telemetry_loop(self):
        """Main telemetry update loop - minimal and infrequent."""
        while self.running:
            try:
                # Update critical telemetry data
                self.telemetry_data['timestamp'] = time.time()

                # Minimal telemetry simulation
                import random
                self.telemetry_data['battery_level'] = max(5, min(100,
                    self.telemetry_data['battery_level'] + random.uniform(-1, 0.5)))

                # Send to connected clients (emergency broadcast)
                self._broadcast_emergency_telemetry()

                # Sync critical state
                self.state_manager.update_state('tertiary_battery_level', self.telemetry_data['battery_level'])
                self.state_manager.update_state('emergency_stop', self.telemetry_data['emergency_stop'])

                time.sleep(1.0 / self.update_rate_hz)

            except Exception as e:
        self.get_logger().info(f"Tertiary telemetry error: {e}")
                time.sleep(1)

    def _broadcast_emergency_telemetry(self):
        """Broadcast critical telemetry to emergency clients."""
        if not self.clients:
            return

        message = json.dumps({
            'type': 'emergency_telemetry',
            'data': self.telemetry_data,
            'priority': 'critical'
        })

        # In real implementation, would use asyncio to send to all clients
        self.get_logger().info(f"[ALERT] Tertiary broadcasting emergency telemetry to {len(self.clients)} clients")
    def _on_state_change(self, key: str, entry):
        """Handle state synchronization updates."""
        if key == 'emergency_stop':
            self.telemetry_data['emergency_stop'] = entry.value
            if entry.value:
        self.get_logger().info("[ALERT] EMERGENCY STOP ACTIVATED - Tertiary bridge")
        elif key == 'system_health':
            self.telemetry_data['system_health'] = entry.value
        elif key.startswith('critical_error'):
            self.telemetry_data['critical_errors'].append(entry.value)

    def _on_config_update(self, snapshot):
        """Handle dynamic configuration updates."""
        for change in snapshot.changes:
            if change.node_name == "tertiary_bridge":
                if change.parameter_name == 'update_rate_hz':
                    self.update_rate_hz = change.new_value
        self.get_logger().info(f"Tertiary update rate changed to {change.new_value}Hz")
def main():
    """Main function for tertiary bridge."""
    import signal
    import sys

    bridge = TertiaryWebSocketBridge()

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
