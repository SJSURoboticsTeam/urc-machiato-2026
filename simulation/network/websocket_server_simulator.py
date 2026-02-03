"""
WebSocket Server Simulator - Stub for testing and simulation.

Provides WebSocket/Socket.IO server simulation for tests and HIL.
When full implementation is needed, extend this module or replace.
"""

import asyncio
import logging
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Dict, List, Optional

logger = logging.getLogger(__name__)


class ConnectionState(Enum):
    """Connection state for simulator."""

    DISCONNECTED = "disconnected"
    CONNECTED = "connected"
    RECONNECTING = "reconnecting"


@dataclass
class NetworkConditions:
    """Network conditions (latency, loss, etc.)."""

    latency_ms: float = 0.0
    jitter_ms: float = 0.0
    packet_loss_rate: float = 0.0
    enabled: bool = False


@dataclass
class MessageRecord:
    """Record of a message for validation/replay."""

    event: str
    data: Any
    client_id: str
    timestamp_ns: int = 0


def create_websocket_simulator(
    config: Optional[Dict[str, Any]] = None
) -> "WebSocketServerSimulator":
    """Factory: create WebSocket server simulator with optional config."""
    return WebSocketServerSimulator(config or {})


class WebSocketServerSimulator:
    """
    Minimal WebSocket/Socket.IO server simulator for tests and HIL.

    Supports connect, disconnect, event handlers, and message recording.
    """

    def __init__(self, config: Optional[Dict[str, Any]] = None) -> None:
        self.config = config or {}
        self.connected_clients: List[str] = []
        self.connection_state = ConnectionState.DISCONNECTED
        self.event_handlers: Dict[str, List[Callable]] = {}
        self._message_records: List[MessageRecord] = []
        self._client_counter = 0
        self._validate_messages = self.config.get("validate_messages", False)
        self._record_messages = self.config.get("record_messages", False)
        network = self.config.get("network", {})
        self._network = NetworkConditions(
            latency_ms=network.get("latency_ms", 0.0),
            jitter_ms=network.get("jitter_ms", 0.0),
            packet_loss_rate=network.get("packet_loss_rate", 0.0),
            enabled=network.get("enabled", False),
        )

    async def connect(self) -> str:
        """Simulate client connect; returns client_id."""
        self._client_counter += 1
        client_id = f"client_{self._client_counter}"
        self.connected_clients.append(client_id)
        self.connection_state = ConnectionState.CONNECTED
        return client_id

    async def disconnect(self, client_id: str) -> None:
        """Simulate client disconnect."""
        if client_id in self.connected_clients:
            self.connected_clients.remove(client_id)
        if not self.connected_clients:
            self.connection_state = ConnectionState.DISCONNECTED

    def get_statistics(self) -> Dict[str, Any]:
        """Return connection and message statistics."""
        return {
            "stats": {"connections": len(self.connected_clients)},
            "connected_clients": len(self.connected_clients),
        }

    def on(self, event: str, handler: Callable) -> None:
        """Register event handler."""
        if event not in self.event_handlers:
            self.event_handlers[event] = []
        self.event_handlers[event].append(handler)

    async def receive(self, event: str, data: Any, client_id: str) -> None:
        """Simulate receiving a message from client; invoke handlers."""
        import time

        if self._record_messages:
            self._message_records.append(
                MessageRecord(
                    event=event,
                    data=data,
                    client_id=client_id,
                    timestamp_ns=int(time.time() * 1e9),
                )
            )
        for h in self.event_handlers.get(event, []):
            try:
                if asyncio.iscoroutinefunction(h):
                    await h(data)
                else:
                    h(data)
            except Exception as e:
                logger.warning("WebSocket simulator handler error: %s", e)
