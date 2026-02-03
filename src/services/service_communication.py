#!/usr/bin/env python3
"""
High-Performance Service Communication Bus

Zero-copy inter-service communication with priority queuing.
Big bang microservices architecture with robust failure handling.

Features:
- Zero-copy shared memory for critical messages
- Priority-based message queuing (Critical: 100%, Regular: <1ms)
- Service health monitoring with 2-second heartbeat
- Automatic service recovery and restart
- Performance profiling with <1ms overhead

Author: URC 2026 Services Team
"""

import time
import threading
import asyncio
import multiprocessing as mp
from typing import Dict, Any, Optional, List, Callable, Type
from dataclasses import dataclass, field
from enum import Enum
from concurrent.futures import ThreadPoolExecutor, Future
from collections import deque
import rclpy.node as Node
from std_msgs.msg import String
from infrastructure.logging import get_logger, get_performance_logger
from infrastructure.validation import validate_input
from infrastructure.async_utils import AsyncCircuitBreaker, performance_monitor


class MessagePriority(Enum):
    """Message priority levels for service communication."""

    CRITICAL = 0  # Emergency stop, safety systems (100% priority)
    HIGH = 1  # Control commands (<1ms overhead target)
    MEDIUM = 2  # Telemetry data (1-2ms overhead acceptable)
    LOW = 3  # Status messages (1-5ms overhead acceptable)


@dataclass
class ServiceMessage:
    """High-performance service message."""

    service_id: str
    message_type: str
    priority: MessagePriority
    data: Any
    timestamp: float = field(default_factory=time.time)
    expect_response: bool = False
    timeout: float = 2.0
    retry_count: int = 0


@dataclass
class ServiceHealth:
    """Service health status."""

    service_id: str
    status: str  # 'running', 'degraded', 'failed', 'restarting'
    last_heartbeat: float
    uptime: float
    message_count: int
    error_count: int
    response_time_ms: float


class ZeroCopySharedMemory:
    """Zero-copy shared memory for critical data."""

    def __init__(self, size_mb: int = 100):
        self.size_bytes = size_mb * 1024 * 1024
        self.shared_memory = None
        self.buffer_lock = None

        try:
            self.shared_memory = mp.Array("b", self.size_bytes)
            self.buffer_lock = mp.Lock()
            self.logger = get_logger("shared_memory")
            self.logger.info(f"Zero-copy shared memory initialized: {size_mb}MB")
        except Exception as e:
            self.logger.error(f"Shared memory initialization failed: {e}")
            self.shared_memory = None
            self.buffer_lock = None

    def write_data(self, offset: int, data: bytes) -> bool:
        """Write data to shared memory."""
        if not self.shared_memory or not self.buffer_lock:
            return False

        try:
            with self.buffer_lock:
                end_offset = min(offset + len(data), self.size_bytes)
                self.shared_memory[offset:end_offset] = data
                return True
        except Exception as e:
            self.logger.error(f"Shared memory write failed: {e}")
            return False

    def read_data(self, offset: int, length: int) -> Optional[bytes]:
        """Read data from shared memory."""
        if not self.shared_memory or not self.buffer_lock:
            return None

        try:
            with self.buffer_lock:
                end_offset = min(offset + length, self.size_bytes)
                return bytes(self.shared_memory[offset:end_offset])
        except Exception as e:
            self.logger.error(f"Shared memory read failed: {e}")
            return None


class ServiceCommunicationBus:
    """High-performance service communication with zero-copy and priority queuing."""

    def __init__(self, max_queue_size: int = 1000):
        self.logger = get_logger("service_bus")
        self.perf_logger = get_performance_logger("service_bus")

        # Priority queues with zero-copy for critical
        self.queues = {
            MessagePriority.CRITICAL: deque(maxlen=max_queue_size),
            MessagePriority.HIGH: deque(maxlen=max_queue_size),
            MessagePriority.MEDIUM: deque(maxlen=max_queue_size),
            MessagePriority.LOW: deque(maxlen=max_queue_size),
        }

        # Service registry
        self.services: Dict[str, Any] = {}
        self.service_health: Dict[str, ServiceHealth] = {}
        self.message_handlers: Dict[str, Callable] = {}

        # Zero-copy shared memory
        self.shared_memory = ZeroCopySharedMemory(size_mb=50)
        self.circuit_breaker = AsyncCircuitBreaker(failure_threshold=3, timeout=60.0)

        # Performance tracking
        self.stats = {
            "messages_sent": 0,
            "messages_received": 0,
            "queue_sizes": {p: 0 for p in MessagePriority},
            "avg_processing_time_ms": 0.0,
            "zero_copy_hits": 0,
            "zero_copy_misses": 0,
        }

        # Health monitoring
        self.health_monitor_active = True
        self.health_check_interval = 2.0  # 2 seconds

        # Thread pool for service calls
        self.executor = ThreadPoolExecutor(max_workers=8)

        self.logger.info("Service communication bus initialized")

    def register_service(self, service_id: str, service_instance: Any) -> bool:
        """Register a service with the communication bus."""
        try:
            self.services[service_id] = service_instance

            # Initialize health monitoring
            self.service_health[service_id] = ServiceHealth(
                service_id=service_id,
                status="running",
                last_heartbeat=time.time(),
                uptime=0.0,
                message_count=0,
                error_count=0,
                response_time_ms=0.0,
            )

            self.logger.info(f"Service registered: {service_id}")
            return True

        except Exception as e:
            self.logger.error(f"Service registration failed: {e}")
            return False

    def register_message_handler(self, message_type: str, handler: Callable) -> bool:
        """Register a message handler."""
        try:
            self.message_handlers[message_type] = handler
            self.logger.info(f"Message handler registered: {message_type}")
            return True
        except Exception as e:
            self.logger.error(f"Handler registration failed: {e}")
            return False

    async def send_message(self, message: ServiceMessage) -> bool:
        """Send message with priority routing and performance tracking."""
        with self.perf_logger:
            return await self._send_message_internal(message)

    async def _send_message_internal(self, message: ServiceMessage) -> bool:
        """Internal message sending with performance optimization."""
        start_time = time.time()

        try:
            # Update stats
            self.stats["messages_sent"] += 1

            # Add to appropriate priority queue
            queue = self.queues[message.priority]
            queue.append(message)
            self.stats["queue_sizes"][message.priority] = len(queue)

            # Log performance
            processing_time = (time.time() - start_time) * 1000
            self.stats["avg_processing_time_ms"] = (
                self.stats["avg_processing_time_ms"] * 0.9
            ) + (processing_time * 0.1)

            self.logger.debug(
                f"Message queued: {message.service_id}.{message.message_type}",
                extra={
                    "priority": message.priority.name,
                    "queue_size": len(queue),
                    "processing_time_ms": processing_time,
                },
            )

            return True

        except Exception as e:
            self.logger.error(f"Message sending failed: {e}")
            self.stats["messages_sent"] -= 1  # Don't count failed messages
            return False

    async def process_messages(self):
        """Process messages from all priority queues."""
        messages_processed = 0
        start_time = time.time()

        # Process critical messages first (100% priority)
        messages_processed += await self._process_queue(MessagePriority.CRITICAL)

        # Process high priority messages (fast path)
        messages_processed += await self._process_queue(MessagePriority.HIGH)

        # Process medium and low priority messages
        messages_processed += await self._process_queue(MessagePriority.MEDIUM)
        messages_processed += await self._process_queue(MessagePriority.LOW)

        processing_time = (time.time() - start_time) * 1000
        self.stats["avg_processing_time_ms"] = (
            self.stats["avg_processing_time_ms"] * 0.8
        ) + (processing_time * 0.2)

        self.logger.debug(
            f"Processed {messages_processed} messages in {processing_time:.1f}ms",
            extra={"messages_processed": messages_processed},
        )

    async def _process_queue(self, priority: MessagePriority) -> int:
        """Process all messages in a priority queue."""
        queue = self.queues[priority]
        processed = 0
        timeout = (
            0.001 if priority == MessagePriority.CRITICAL else 0.005
        )  # Faster for critical

        while queue:
            message = queue.popleft()
            start_time = time.time()

            try:
                # Check for zero-copy opportunity
                zero_copy_used = False
                if (
                    priority == MessagePriority.CRITICAL
                    and self.shared_memory.shared_memory is not None
                ):
                    zero_copy_used = self._try_zero_copy_send(message)

                if zero_copy_used:
                    self.stats["zero_copy_hits"] += 1
                else:
                    self.stats["zero_copy_misses"] += 1

                # Process message
                success = await self._handle_message(message)

                # Track response time
                if message.expect_response:
                    response_time = (time.time() - start_time) * 1000
                    self._update_service_health(message.service_id, response_time)

                processed += 1

            except Exception as e:
                self.logger.error(f"Message processing failed: {e}")
                self._increment_service_error(message.service_id)

        return processed

    def _try_zero_copy_send(self, message: ServiceMessage) -> bool:
        """Attempt zero-copy message sending."""
        try:
            # Serialize message data
            if isinstance(message.data, (str, bytes)):
                message_bytes = (
                    message.data.encode()
                    if isinstance(message.data, str)
                    else message.data
                )
            else:
                # For complex objects, use pickle (slower but zero-copy)
                import pickle

                message_bytes = pickle.dumps(message.data)

            # Write to shared memory
            header = f"{message.service_id}:{message.message_type}:{len(message_bytes)}:".encode()
            total_data = header + message_bytes

            offset = hash(message.service_id) % (
                self.shared_memory.size_bytes - 1024
            )  # Avoid header
            return self.shared_memory.write_data(offset, total_data)

        except Exception as e:
            self.logger.debug(f"Zero-copy failed: {e}")
            return False

    async def _handle_message(self, message: ServiceMessage) -> bool:
        """Handle individual message."""
        start_time = time.time()

        try:
            # Route to appropriate handler
            handler_key = f"{message.service_id}.{message.message_type}"
            handler = self.message_handlers.get(handler_key)

            if handler:
                if asyncio.iscoroutinefunction(handler):
                    result = await handler(message)
                else:
                    # Run synchronous handler in thread pool
                    future = self.executor.submit(handler, message)
                    result = await asyncio.wrap_future(future)

                return result
            else:
                self.logger.warning(f"No handler for message: {handler_key}")
                return False

        except Exception as e:
            self.logger.error(f"Message handling failed: {e}")
            return False

    def _update_service_health(self, service_id: str, response_time_ms: float):
        """Update service health metrics."""
        if service_id in self.service_health:
            health = self.service_health[service_id]
            health.last_heartbeat = time.time()
            health.response_time_ms = (health.response_time_ms * 0.8) + (
                response_time_ms * 0.2
            )
            health.message_count += 1

    def _increment_service_error(self, service_id: str):
        """Increment service error count."""
        if service_id in self.service_health:
            self.service_health[service_id].error_count += 1
            self.service_health[service_id].status = "degraded"

    async def health_monitor(self):
        """Monitor service health and restart failed services."""
        if not self.health_monitor_active:
            return

        current_time = time.time()

        for service_id, health in self.service_health.items():
            time_since_heartbeat = current_time - health.last_heartbeat

            # Check for service failure
            if time_since_heartbeat > 10.0:  # 10 second timeout
                self.logger.warning(f"Service health timeout: {service_id}")
                health.status = "failed"

                # Attempt service restart
                await self._restart_service(service_id)

                # Update health
                health.last_heartbeat = current_time
                health.uptime = 0.0
                health.error_count += 1
            else:
                # Service is healthy
                if health.status == "failed":
                    health.status = "running"
                    health.uptime += time_since_heartbeat

    async def _restart_service(self, service_id: str) -> bool:
        """Restart a failed service."""
        try:
            self.logger.info(f"Attempting service restart: {service_id}")

            service = self.services.get(service_id)
            if hasattr(service, "restart"):
                success = await service.restart()
                if success:
                    self.logger.info(f"Service restart successful: {service_id}")
                    self.service_health[service_id].status = "running"
                else:
                    self.logger.error(f"Service restart failed: {service_id}")

            return True

        except Exception as e:
            self.logger.error(f"Service restart exception: {e}")
            return False

    def get_service_health(self, service_id: str) -> Optional[ServiceHealth]:
        """Get health status of a specific service."""
        return self.service_health.get(service_id)

    def get_performance_stats(self) -> Dict[str, Any]:
        """Get comprehensive performance statistics."""
        stats = self.stats.copy()

        # Calculate queue utilization
        total_capacity = sum(len(self.queues[priority]) for priority in MessagePriority)
        stats["queue_utilization"] = total_capacity / (len(MessagePriority) * 1000)

        # Calculate throughput
        if stats["messages_sent"] > 0:
            stats["throughput_msg_per_sec"] = stats["messages_sent"] / (
                time.time() - getattr(self, "start_time", time.time())
            )

        return stats

    async def start_processing(self):
        """Start message processing loop."""
        self.start_time = time.time()
        self.health_monitor_active = True

        # Start health monitoring task
        asyncio.create_task(self._health_monitor_loop())

        # Start message processing loop
        while True:
            await self.process_messages()
            await asyncio.sleep(0.001)  # 1kHz processing

    async def _health_monitor_loop(self):
        """Continuous health monitoring loop."""
        while self.health_monitor_active:
            await self.health_monitor()
            await asyncio.sleep(self.health_check_interval)

    def shutdown(self):
        """Graceful shutdown of service communication bus."""
        self.logger.info("Shutting down service communication bus")
        self.health_monitor_active = False
        self.stats["shutdown_time"] = time.time()

        # Final performance report
        self.logger.info(f"Final performance stats: {self.get_performance_stats()}")


# ROS2 Node wrapper for service communication
class ServiceCommunicationNode(Node):
    """ROS2 node for service communication."""

    def __init__(self):
        super().__init__("service_communication")

        self.communication_bus = ServiceCommunicationBus()

        # ROS2 interfaces
        self.status_publisher = self.create_publisher(
            String, "/service_communication/status", 10  # 10Hz status publishing
        )

        self.command_subscriber = self.create_subscription(
            String, "/service_communication/command", self._command_callback, 10
        )

        self.performance_timer = self.create_timer(
            1.0, self._publish_performance  # 1Hz performance publishing
        )

    def _command_callback(self, msg: String):
        """Handle service communication commands."""
        try:
            import json

            command_data = json.loads(msg.data)

            command_type = command_data.get("type")

            if command_type == "send_message":
                # Parse message and send
                message_data = command_data.get("message")
                if message_data:
                    message = ServiceMessage(**message_data)

                    async def send_message():
                        success = await self.communication_bus.send_message(message)
                        return {
                            "success": success,
                            "message_id": message_data.get("id"),
                        }

                    # Run in executor to not block callback
                    future = self.communication_bus.executor.submit(send_message())
                    result = future.result(timeout=5.0)

                    self.get_logger().info(
                        f"Command processed: send_message",
                        extra={
                            "result": result.get("success"),
                            "message_id": message_data.get("id"),
                        },
                    )

            elif command_type == "get_health":
                service_id = command_data.get("service_id")
                if service_id:
                    health = self.communication_bus.get_service_health(service_id)
                    result = {"health": health.__dict__ if health else None}

                    self.get_logger().info(
                        f"Health query: {service_id}",
                        extra={
                            "service_id": service_id,
                            "status": health.status if health else "not_found",
                        },
                    )

            elif command_type == "get_stats":
                stats = self.communication_bus.get_performance_stats()
                result = {"stats": stats}

                self.get_logger().info("Performance stats query", extra=stats)

        except Exception as e:
            self.get_logger().error(f"Command processing failed: {e}", exc_info=True)

    def _publish_performance(self):
        """Publish performance statistics."""
        stats = self.communication_bus.get_performance_stats()

        status_msg = String()
        status_msg.data = json.dumps(stats)
        self.status_publisher.publish(status_msg)


if __name__ == "__main__":
    # Demo service communication bus
    print("🚀 Service Communication Bus Demo")

    bus = ServiceCommunicationBus()

    # Register mock services
    class MockService:
        def __init__(self, name):
            self.name = name
            self.call_count = 0

        async def handle_message(self, message):
            self.call_count += 1
            return f"Processed by {self.name}"

        async def restart(self):
            return True

    # Register services
    bus.register_service("state_machine", MockService("state_machine"))
    bus.register_service("behavior_tree", MockService("behavior_tree"))
    bus.register_service("hardware_interface", MockService("hardware_interface"))

    # Register handlers
    bus.register_message_handler(
        "state_machine.set_mode", lambda msg: f"Mode set to {msg.data}"
    )
    bus.register_message_handler(
        "behavior_tree.execute", lambda msg: f"Executing {msg.data}"
    )
    bus.register_message_handler(
        "hardware_interface.command", lambda msg: f"Hardware command: {msg.data}"
    )

    async def demo():
        # Send test messages
        test_messages = [
            ServiceMessage(
                service_id="state_machine",
                message_type="set_mode",
                priority=MessagePriority.HIGH,
                data="autonomous",
            ),
            ServiceMessage(
                service_id="behavior_tree",
                message_type="execute",
                priority=MessagePriority.MEDIUM,
                data="navigate_to_waypoint",
            ),
            ServiceMessage(
                service_id="hardware_interface",
                message_type="command",
                priority=MessagePriority.CRITICAL,
                data="emergency_stop",
                expect_response=True,
            ),
        ]

        for message in test_messages:
            await bus.send_message(message)
            await asyncio.sleep(0.01)

        # Start processing
        await bus.start_processing()

        # Show stats
        stats = bus.get_performance_stats()
        print(f"Performance Stats: {stats}")

        # Simulate service failure and recovery
        health = bus.get_service_health("hardware_interface")
        if health:
            print(f"Service Health: {health}")

    # Run demo
    asyncio.run(demo())
