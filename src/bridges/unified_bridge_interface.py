#!/usr/bin/env python3
"""
Unified Bridge Interface - Common Interface for All Bridge Types

Provides a consistent interface for CAN, WebSocket, HTTP, and emergency bridges.
Eliminates code duplication and enables centralized bridge management.

Author: URC 2026 Unified Bridge Team
"""

import asyncio
from abc import ABC, abstractmethod
from typing import Dict, Any, Optional, Callable, List
from dataclasses import dataclass
from enum import Enum
import time


class BridgeType(Enum):
    """Supported bridge types."""
    CAN = "can"
    WEBSOCKET = "websocket"
    HTTP = "http"
    EMERGENCY = "emergency"


class MessagePriority(Enum):
    """Message priority levels."""
    LOW = 1
    NORMAL = 2
    HIGH = 3
    CRITICAL = 4


@dataclass
class BridgeMessage:
    """Unified message format for all bridges."""
    message_type: str
    data: Dict[str, Any]
    priority: MessagePriority = MessagePriority.NORMAL
    correlation_id: Optional[str] = None
    source_bridge: Optional[BridgeType] = None
    timestamp: Optional[float] = None
    requires_ack: bool = False

    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = time.time()
        if self.correlation_id is None:
            self.correlation_id = f"{self.message_type}_{int(self.timestamp * 1000000)}"


@dataclass
class BridgeStatus:
    """Standardized bridge status information."""
    bridge_type: BridgeType
    is_connected: bool
    last_message_time: Optional[float]
    messages_sent: int = 0
    messages_received: int = 0
    errors: int = 0
    uptime: float = 0.0
    additional_info: Dict[str, Any] = None

    def __post_init__(self):
        if self.additional_info is None:
            self.additional_info = {}


class BridgeInterface(ABC):
    """
    Unified interface for all bridge implementations.

    All bridge types (CAN, WebSocket, HTTP, Emergency) must implement this interface.
    """

    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self.bridge_type: BridgeType
        self.is_connected = False
        self.message_handlers: Dict[str, List[Callable]] = {}
        self.start_time = time.time()
        self.stats = BridgeStatus(
            bridge_type=self.bridge_type,
            is_connected=False,
            last_message_time=None
        )

    @abstractmethod
    async def connect(self) -> bool:
        """
        Establish connection to the bridge.

        Returns:
            bool: True if connection successful, False otherwise
        """
        pass

    @abstractmethod
    async def disconnect(self) -> None:
        """Cleanly disconnect from the bridge."""
        pass

    @abstractmethod
    async def send_message(self, message: BridgeMessage) -> bool:
        """
        Send a message through the bridge.

        Args:
            message: The message to send

        Returns:
            bool: True if message sent successfully, False otherwise
        """
        pass

    @abstractmethod
    def get_status(self) -> BridgeStatus:
        """
        Get current bridge status.

        Returns:
            BridgeStatus: Current status information
        """
        # Update common stats
        self.stats.uptime = time.time() - self.start_time
        self.stats.is_connected = self.is_connected
        return self.stats

    async def register_handler(self, message_type: str, handler: Callable[[BridgeMessage], None]) -> None:
        """
        Register a message handler for a specific message type.

        Args:
            message_type: Type of message to handle ("*" for all messages)
            handler: Async function to call when message received
        """
        if message_type not in self.message_handlers:
            self.message_handlers[message_type] = []
        self.message_handlers[message_type].append(handler)

    async def unregister_handler(self, message_type: str, handler: Callable) -> None:
        """
        Unregister a message handler.

        Args:
            message_type: Message type to stop handling
            handler: Handler function to remove
        """
        if message_type in self.message_handlers:
            try:
                self.message_handlers[message_type].remove(handler)
                if not self.message_handlers[message_type]:
                    del self.message_handlers[message_type]
            except ValueError:
                pass  # Handler not found

    async def _route_message(self, message: BridgeMessage) -> None:
        """
        Route incoming message to registered handlers.

        Args:
            message: Received message to route
        """
        # Update stats
        self.stats.messages_received += 1
        self.stats.last_message_time = message.timestamp

        # Route to specific handlers
        if message.message_type in self.message_handlers:
            for handler in self.message_handlers[message.message_type]:
                try:
                    if asyncio.iscoroutinefunction(handler):
                        await handler(message)
                    else:
                        handler(message)
                except Exception as e:
                    print(f"Bridge handler error for {message.message_type}: {e}")
                    self.stats.errors += 1

        # Route to wildcard handlers
        if "*" in self.message_handlers:
            for handler in self.message_handlers["*"]:
                try:
                    if asyncio.iscoroutinefunction(handler):
                        await handler(message)
                    else:
                        handler(message)
                except Exception as e:
                    print(f"Bridge wildcard handler error: {e}")
                    self.stats.errors += 1

    async def _send_acknowledgment(self, original_message: BridgeMessage) -> None:
        """Send acknowledgment for messages that require it."""
        if not original_message.requires_ack:
            return

        ack_message = BridgeMessage(
            message_type="ack",
            data={
                "correlation_id": original_message.correlation_id,
                "status": "received",
                "timestamp": time.time()
            },
            priority=MessagePriority.NORMAL,
            correlation_id=f"ack_{original_message.correlation_id}"
        )

        await self.send_message(ack_message)

    def _should_retry_message(self, message: BridgeMessage, attempt: int) -> bool:
        """
        Determine if a message should be retried.

        Args:
            message: Message that failed to send
            attempt: Current attempt number (0-based)

        Returns:
            bool: True if should retry
        """
        max_retries = self.config.get('max_retries', 3)
        retry_priorities = self.config.get('retry_priorities', [MessagePriority.HIGH, MessagePriority.CRITICAL])

        if attempt >= max_retries:
            return False

        return message.priority in retry_priorities

    async def _retry_send_message(self, message: BridgeMessage) -> bool:
        """Retry sending a message with exponential backoff."""
        max_retries = self.config.get('max_retries', 3)
        base_delay = self.config.get('retry_delay', 0.1)

        for attempt in range(max_retries):
            if await self.send_message(message):
                return True

            if not self._should_retry_message(message, attempt):
                break

            delay = base_delay * (2 ** attempt)  # Exponential backoff
            await asyncio.sleep(delay)

        return False


# Bridge factory function type
BridgeFactory = Callable[[Dict[str, Any]], BridgeInterface]

# Registry of bridge factories
_bridge_factories: Dict[BridgeType, BridgeFactory] = {}


def register_bridge_factory(bridge_type: BridgeType, factory: BridgeFactory) -> None:
    """Register a bridge factory function."""
    _bridge_factories[bridge_type] = factory


def create_bridge(bridge_type: BridgeType, config: Dict[str, Any]) -> BridgeInterface:
    """
    Create a bridge instance using registered factory.

    Args:
        bridge_type: Type of bridge to create
        config: Bridge-specific configuration

    Returns:
        BridgeInterface: Configured bridge instance

    Raises:
        ValueError: If bridge type not registered
    """
    if bridge_type not in _bridge_factories:
        raise ValueError(f"No factory registered for bridge type: {bridge_type}")

    return _bridge_factories[bridge_type](config)


# Export key components
__all__ = [
    'BridgeType',
    'MessagePriority',
    'BridgeMessage',
    'BridgeStatus',
    'BridgeInterface',
    'BridgeFactory',
    'register_bridge_factory',
    'create_bridge'
]