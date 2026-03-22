#!/usr/bin/env python3
"""
URC 2026 Network Resilience Module

Provides robust network communication with automatic retry, circuit breaking,
and fault tolerance using industry-standard libraries:
- Tenacity for retry patterns and backoff strategies
- aiohttp for async HTTP communication
- Circuit breaker pattern implementation
- Connection pooling and timeout management

Author: URC 2026 Team
"""

import asyncio
import time
import threading
from typing import Any, Callable, Dict, List, Optional, Tuple, Union
from dataclasses import dataclass, field
from enum import Enum
import logging
import random
import json

# Optional dependencies - graceful degradation if not available
try:
    import aiohttp
    import aiohttp.web

    AIOHTTP_AVAILABLE = True
except ImportError:
    AIOHTTP_AVAILABLE = False

    # Mock aiohttp classes
    class aiohttp:
        class ClientSession:
            pass

        class ClientTimeout:
            pass

        class web:
            class Application:
                pass

            class Request:
                pass

            class Response:
                pass


try:
    from tenacity import (
        retry,
        stop_after_attempt,
        wait_exponential,
        retry_if_exception_type,
        RetryError,
    )

    TENACITY_AVAILABLE = True
except ImportError:
    TENACITY_AVAILABLE = False

    # Mock tenacity decorators
    def retry(*args, **kwargs):
        def decorator(func):
            return func

        return decorator

    def stop_after_attempt(*args):
        pass

    def wait_exponential(*args):
        pass

    def retry_if_exception_type(*args):
        pass

    class RetryError(Exception):
        pass


class CircuitBreakerState(Enum):
    """Circuit breaker states."""

    CLOSED = "closed"  # Normal operation
    OPEN = "open"  # Failing, requests rejected
    HALF_OPEN = "half_open"  # Testing if service recovered


class NetworkError(Exception):
    """Base network error."""

    pass


class ConnectionError(NetworkError):
    """Connection failed."""

    pass


class TimeoutError(NetworkError):
    """Request timed out."""

    pass


class CircuitBreakerOpenError(NetworkError):
    """Circuit breaker is open."""

    pass


@dataclass
class CircuitBreakerConfig:
    """Configuration for circuit breaker."""

    failure_threshold: int = 5  # Failures before opening
    recovery_timeout: float = 60.0  # Seconds to wait before half-open
    success_threshold: int = 3  # Successes needed to close
    timeout: float = 30.0  # Request timeout


@dataclass
class RetryConfig:
    """Configuration for retry logic."""

    max_attempts: int = 3
    backoff_factor: float = 1.0
    max_backoff: float = 30.0
    jitter: bool = True


@dataclass
class ConnectionPoolConfig:
    """Configuration for connection pooling."""

    max_connections: int = 20
    max_keepalive_connections: int = 10
    keepalive_timeout: float = 60.0
    timeout: aiohttp.ClientTimeout = None

    def __post_init__(self):
        if self.timeout is None:
            self.timeout = aiohttp.ClientTimeout(total=30.0)


class CircuitBreaker:
    """
    Circuit breaker implementation for fault tolerance.

    Prevents cascading failures by temporarily stopping requests to failing services.
    """

    def __init__(self, name: str, config: CircuitBreakerConfig = None):
        self.name = name
        self.config = config or CircuitBreakerConfig()
        self.state = CircuitBreakerState.CLOSED
        self.failure_count = 0
        self.success_count = 0
        self.last_failure_time = 0
        self._lock = threading.Lock()

    def _should_attempt_reset(self) -> bool:
        """Check if circuit breaker should attempt reset."""
        if self.state != CircuitBreakerState.OPEN:
            return False
        return time.time() - self.last_failure_time >= self.config.recovery_timeout

    def _record_success(self):
        """Record a successful request."""
        with self._lock:
            if self.state == CircuitBreakerState.HALF_OPEN:
                self.success_count += 1
                if self.success_count >= self.config.success_threshold:
                    self.state = CircuitBreakerState.CLOSED
                    self.failure_count = 0
                    self.success_count = 0
                    print(
                        f"🔄 Circuit breaker '{self.name}' closed (service recovered)"
                    )

    def _record_failure(self):
        """Record a failed request."""
        with self._lock:
            self.failure_count += 1
            self.last_failure_time = time.time()

            if self.state == CircuitBreakerState.HALF_OPEN:
                self.state = CircuitBreakerState.OPEN
                self.success_count = 0
                print(
                    f"🚫 Circuit breaker '{self.name}' opened (service still failing)"
                )
            elif (
                self.state == CircuitBreakerState.CLOSED
                and self.failure_count >= self.config.failure_threshold
            ):
                self.state = CircuitBreakerState.OPEN
                print(
                    f"🚫 Circuit breaker '{self.name}' opened (failure threshold exceeded)"
                )

    def call(self, func: Callable, *args, **kwargs):
        """Execute function through circuit breaker."""
        with self._lock:
            if self.state == CircuitBreakerState.OPEN:
                if self._should_attempt_reset():
                    self.state = CircuitBreakerState.HALF_OPEN
                    print(
                        f"🔄 Circuit breaker '{self.name}' half-open (testing recovery)"
                    )
                else:
                    raise CircuitBreakerOpenError(
                        f"Circuit breaker '{self.name}' is open"
                    )

        try:
            result = func(*args, **kwargs)
            self._record_success()
            return result
        except Exception as e:
            self._record_failure()
            raise

    async def call_async(self, func: Callable, *args, **kwargs):
        """Execute async function through circuit breaker."""
        with self._lock:
            if self.state == CircuitBreakerState.OPEN:
                if self._should_attempt_reset():
                    self.state = CircuitBreakerState.HALF_OPEN
                    print(
                        f"🔄 Circuit breaker '{self.name}' half-open (testing recovery)"
                    )
                else:
                    raise CircuitBreakerOpenError(
                        f"Circuit breaker '{self.name}' is open"
                    )

        try:
            result = await func(*args, **kwargs)
            self._record_success()
            return result
        except Exception as e:
            self._record_failure()
            raise

    def get_state(self) -> Dict[str, Any]:
        """Get circuit breaker state information."""
        return {
            "name": self.name,
            "state": self.state.value,
            "failure_count": self.failure_count,
            "success_count": self.success_count,
            "last_failure_time": self.last_failure_time,
            "time_since_last_failure": time.time() - self.last_failure_time,
        }


class BridgeOutboundQueue:
    """
    Bounded outbound queue for bridge messages with retry and exponential backoff.

    Failed sends are queued and retried by calling process_one() periodically
    (e.g. from a bridge timer). Reuses the pattern from WebhookResilienceManager.
    """

    def __init__(
        self,
        max_size: int = 100,
        initial_backoff: float = 1.0,
        max_backoff: float = 30.0,
    ):
        self.max_size = max_size
        self.initial_backoff = initial_backoff
        self.max_backoff = max_backoff
        self._queue: List[Tuple[Any, Callable]] = []
        self._consecutive_failures = 0
        self._lock = threading.Lock()

    def put(self, payload: Any, sender_fn: Callable) -> None:
        """Queue a payload and its sender (async or sync callable). Drops oldest if over max_size."""
        with self._lock:
            if len(self._queue) >= self.max_size:
                self._queue.pop(0)
            self._queue.append((payload, sender_fn))

    def __len__(self) -> int:
        with self._lock:
            return len(self._queue)

    async def process_one(self) -> bool:
        """
        Try to send one queued item. Returns True if an item was processed (success or dropped).
        On send failure, re-queues and applies backoff sleep.
        """
        with self._lock:
            if not self._queue:
                return False
            item = self._queue.pop(0)
        payload, sender_fn = item
        backoff = min(
            self.initial_backoff * (2**self._consecutive_failures), self.max_backoff
        )
        try:
            result = sender_fn(payload)
            if asyncio.iscoroutine(result):
                result = await result
            self._consecutive_failures = 0
            return True
        except Exception:
            self._consecutive_failures += 1
            with self._lock:
                if len(self._queue) < self.max_size:
                    self._queue.append((payload, sender_fn))
            await asyncio.sleep(backoff)
            return True


class ResilientHTTPClient:
    """
    HTTP client with automatic retry, circuit breaking, and fault tolerance.
    """

    def __init__(
        self,
        base_url: str = "",
        circuit_breaker_config: CircuitBreakerConfig = None,
        retry_config: RetryConfig = None,
        pool_config: ConnectionPoolConfig = None,
    ):
        self.base_url = base_url.rstrip("/")
        self.circuit_breaker_config = circuit_breaker_config or CircuitBreakerConfig()
        self.retry_config = retry_config or RetryConfig()
        self.pool_config = pool_config or ConnectionPoolConfig()

        # Circuit breakers per endpoint
        self.circuit_breakers: Dict[str, CircuitBreaker] = {}

        # Session management
        self._session: Optional[aiohttp.ClientSession] = None
        self._session_lock = asyncio.Lock()

        # Setup retry decorator if tenacity is available
        if TENACITY_AVAILABLE:
            self._retry_decorator = retry(
                stop=stop_after_attempt(self.retry_config.max_attempts),
                wait=wait_exponential(
                    multiplier=self.retry_config.backoff_factor,
                    max=self.retry_config.max_backoff,
                ),
                retry=retry_if_exception_type(
                    (ConnectionError, TimeoutError, aiohttp.ClientError)
                ),
            )
        else:
            self._retry_decorator = lambda func: func

    async def _get_session(self) -> aiohttp.ClientSession:
        """Get or create HTTP session."""
        async with self._session_lock:
            if self._session is None or self._session.closed:
                connector = aiohttp.TCPConnector(
                    limit=self.pool_config.max_connections,
                    limit_per_host=self.pool_config.max_keepalive_connections,
                    keepalive_timeout=self.pool_config.keepalive_timeout,
                    ttl_dns_cache=300,
                )
                self._session = aiohttp.ClientSession(
                    connector=connector, timeout=self.pool_config.timeout
                )
        return self._session

    def _get_circuit_breaker(self, endpoint: str) -> CircuitBreaker:
        """Get circuit breaker for endpoint."""
        if endpoint not in self.circuit_breakers:
            self.circuit_breakers[endpoint] = CircuitBreaker(
                f"http_{endpoint}", self.circuit_breaker_config
            )
        return self.circuit_breakers[endpoint]

    async def request(self, method: str, endpoint: str, **kwargs) -> Dict[str, Any]:
        """
        Make HTTP request with resilience patterns.

        Args:
            method: HTTP method (GET, POST, etc.)
            endpoint: API endpoint (relative to base_url)
            **kwargs: Additional request parameters

        Returns:
            Dict with response data
        """
        url = f"{self.base_url}/{endpoint.lstrip('/')}" if self.base_url else endpoint

        circuit_breaker = self._get_circuit_breaker(endpoint)

        @self._retry_decorator
        async def _make_request():
            session = await self._get_session()

            try:
                async with session.request(method, url, **kwargs) as response:
                    response_data = await response.json()
                    return {
                        "status": response.status,
                        "headers": dict(response.headers),
                        "data": response_data,
                        "url": str(response.url),
                    }
            except aiohttp.ClientConnectorError as e:
                raise ConnectionError(f"Connection failed: {e}")
            except asyncio.TimeoutError as e:
                raise TimeoutError(f"Request timeout: {e}")
            except aiohttp.ClientError as e:
                raise NetworkError(f"HTTP client error: {e}")

        try:
            return await circuit_breaker.call_async(_make_request)
        except CircuitBreakerOpenError:
            raise CircuitBreakerOpenError(
                f"Service {endpoint} is currently unavailable"
            )

    async def get(self, endpoint: str, **kwargs) -> Dict[str, Any]:
        """GET request."""
        return await self.request("GET", endpoint, **kwargs)

    async def post(self, endpoint: str, data: Any = None, **kwargs) -> Dict[str, Any]:
        """POST request."""
        if data is not None:
            if isinstance(data, dict):
                kwargs["json"] = data
            else:
                kwargs["data"] = data
        return await self.request("POST", endpoint, **kwargs)

    async def put(self, endpoint: str, data: Any = None, **kwargs) -> Dict[str, Any]:
        """PUT request."""
        if data is not None:
            if isinstance(data, dict):
                kwargs["json"] = data
            else:
                kwargs["data"] = data
        return await self.request("PUT", endpoint, **kwargs)

    async def delete(self, endpoint: str, **kwargs) -> Dict[str, Any]:
        """DELETE request."""
        return await self.request("DELETE", endpoint, **kwargs)

    def get_circuit_breaker_states(self) -> Dict[str, Dict[str, Any]]:
        """Get states of all circuit breakers."""
        return {name: cb.get_state() for name, cb in self.circuit_breakers.items()}

    async def close(self):
        """Close HTTP client and cleanup resources."""
        async with self._session_lock:
            if self._session and not self._session.closed:
                await self._session.close()


class WebhookResilienceManager:
    """
    Manages webhook delivery with resilience patterns.
    """

    def __init__(self, webhook_url: str, client: ResilientHTTPClient = None):
        self.webhook_url = webhook_url
        self.client = client or ResilientHTTPClient()
        self.delivery_queue: List[Dict[str, Any]] = []
        self._running = False
        self._task: Optional[asyncio.Task] = None

    async def start(self):
        """Start the webhook delivery manager."""
        self._running = True
        self._task = asyncio.create_task(self._process_queue())

    async def stop(self):
        """Stop the webhook delivery manager."""
        self._running = False
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass

    async def send_webhook(self, payload: Dict[str, Any], urgent: bool = False) -> bool:
        """
        Send webhook with resilience.

        Args:
            payload: Webhook payload
            urgent: If True, attempt immediate delivery

        Returns:
            True if delivered successfully (or queued)
        """
        if urgent:
            try:
                await self.client.post("", json=payload)
                return True
            except Exception as e:
                print(f"⚠️ Urgent webhook failed, queuing: {e}")
                self.delivery_queue.append(payload)
                return False
        else:
            self.delivery_queue.append(payload)
            return True

    async def _process_queue(self):
        """Process queued webhooks."""
        while self._running:
            if self.delivery_queue:
                payload = self.delivery_queue.pop(0)
                try:
                    await self.client.post("", json=payload)
                    print(f"✅ Webhook delivered: {payload.get('event', 'unknown')}")
                except Exception as e:
                    print(f"❌ Webhook delivery failed, re-queuing: {e}")
                    self.delivery_queue.insert(0, payload)  # Re-queue at front

            await asyncio.sleep(1.0)  # Process every second


class NetworkHealthMonitor:
    """
    Monitors network connectivity and service health.
    """

    def __init__(self, check_interval: float = 30.0):
        self.check_interval = check_interval
        self.services: Dict[str, Dict[str, Any]] = {}
        self._running = False
        self._task: Optional[asyncio.Task] = None

    def add_service_check(
        self, name: str, check_func: Callable[[], bool], failure_threshold: int = 3
    ):
        """Add a service health check."""
        self.services[name] = {
            "check_func": check_func,
            "failure_threshold": failure_threshold,
            "failure_count": 0,
            "last_check": 0,
            "healthy": True,
        }

    async def start_monitoring(self):
        """Start network health monitoring."""
        self._running = True
        self._task = asyncio.create_task(self._monitor_loop())

    async def stop_monitoring(self):
        """Stop network health monitoring."""
        self._running = False
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass

    async def _monitor_loop(self):
        """Main monitoring loop."""
        while self._running:
            await self._check_all_services()
            await asyncio.sleep(self.check_interval)

    async def _check_all_services(self):
        """Check health of all registered services."""
        for name, service_info in self.services.items():
            try:
                healthy = await asyncio.get_event_loop().run_in_executor(
                    None, service_info["check_func"]
                )

                if healthy:
                    service_info["failure_count"] = 0
                    if not service_info["healthy"]:
                        service_info["healthy"] = True
                        print(f"✅ Service '{name}' recovered")
                else:
                    service_info["failure_count"] += 1
                    if (
                        service_info["failure_count"]
                        >= service_info["failure_threshold"]
                        and service_info["healthy"]
                    ):
                        service_info["healthy"] = False
                        print(f"❌ Service '{name}' failed (threshold exceeded)")

            except Exception as e:
                service_info["failure_count"] += 1
                print(f"⚠️ Service check '{name}' error: {e}")

            service_info["last_check"] = time.time()

    def get_health_status(self) -> Dict[str, Dict[str, Any]]:
        """Get health status of all services."""
        return {
            name: {
                "healthy": info["healthy"],
                "failure_count": info["failure_count"],
                "last_check": info["last_check"],
                "time_since_check": time.time() - info["last_check"],
            }
            for name, info in self.services.items()
        }


# Global instances
_http_client_instance = None
_monitor_instance = None
_client_lock = threading.Lock()
_monitor_lock = threading.Lock()


def get_resilient_http_client(base_url: str = "") -> ResilientHTTPClient:
    """Get or create global resilient HTTP client."""
    global _http_client_instance

    if _http_client_instance is None:
        with _client_lock:
            if _http_client_instance is None:
                _http_client_instance = ResilientHTTPClient(base_url)

    return _http_client_instance


def get_network_health_monitor() -> NetworkHealthMonitor:
    """Get or create global network health monitor."""
    global _monitor_instance

    if _monitor_instance is None:
        with _monitor_lock:
            if _monitor_instance is None:
                _monitor_instance = NetworkHealthMonitor()

    return _monitor_instance


def create_circuit_breaker(
    name: str, config: CircuitBreakerConfig = None
) -> CircuitBreaker:
    """Create a new circuit breaker instance."""
    return CircuitBreaker(name, config)


class NetworkResilienceManager:
    """
    High-level network resilience manager for robotics applications.

    Manages wireless communication resilience including frequency hopping,
    power control, and adaptive networking strategies.
    """

    def __init__(self):
        self.frequency_hopping_active = False
        self.current_frequency = 2400  # MHz
        self.transmit_power = 1.0  # 0.0 to 1.0
        self.adaptive_mode = True
        self._hop_task: Optional[asyncio.Task] = None

        # Initialize components
        self.http_client = ResilientHTTPClient()
        self.health_monitor = NetworkHealthMonitor()
        self.webhook_manager = WebhookResilienceManager("")

    async def start_frequency_hopping(self, channels: List[int] = None):
        """
        Start frequency hopping for interference avoidance.

        Args:
            channels: List of channels/frequencies to hop between
        """
        if self.frequency_hopping_active:
            return

        self.frequency_hopping_active = True
        channels = channels or [2400, 2420, 2440, 2460]  # Default WiFi channels

        print(f"📡 Starting frequency hopping: {channels}")

        async def hop_loop():
            while self.frequency_hopping_active:
                for freq in channels:
                    if not self.frequency_hopping_active:
                        break
                    self.current_frequency = freq
                    print(f"📡 Hopped to frequency: {freq} MHz")
                    await asyncio.sleep(0.1)  # Hop every 100ms

        self._hop_task = asyncio.create_task(hop_loop())

    def stop_frequency_hopping(self):
        """Stop frequency hopping."""
        self.frequency_hopping_active = False
        if self._hop_task:
            self._hop_task.cancel()
        print("📡 Stopped frequency hopping")

    def adjust_transmit_power(self, power_level: float):
        """
        Adjust transmit power level.

        Args:
            power_level: Power level between 0.0 and 1.0
        """
        self.transmit_power = max(0.0, min(1.0, power_level))
        print(f"📡 Transmit power adjusted to: {self.transmit_power:.1%}")

    def enable_adaptive_networking(self):
        """Enable adaptive networking strategies."""
        self.adaptive_mode = True
        print("📡 Adaptive networking enabled")

    def disable_adaptive_networking(self):
        """Disable adaptive networking strategies."""
        self.adaptive_mode = False
        print("📡 Adaptive networking disabled")

    async def test_connectivity(self, target_url: str) -> bool:
        """
        Test network connectivity to target.

        Args:
            target_url: URL to test connectivity to

        Returns:
            True if connectivity is good
        """
        try:
            response = await self.http_client.get(target_url)
            return response.get("status") == 200
        except Exception:
            return False

    def get_network_status(self) -> Dict[str, Any]:
        """Get current network status."""
        return {
            "frequency_hopping_active": self.frequency_hopping_active,
            "current_frequency": self.current_frequency,
            "transmit_power": self.transmit_power,
            "adaptive_mode": self.adaptive_mode,
            "circuit_breakers": self.http_client.get_circuit_breaker_states(),
            "health_status": self.health_monitor.get_health_status(),
        }

    async def shutdown(self):
        """Shutdown network resilience manager."""
        self.stop_frequency_hopping()
        await self.http_client.close()
        await self.health_monitor.stop_monitoring()
        await self.webhook_manager.stop()
        print("📡 Network resilience manager shutdown")


async def resilient_get(url: str, **kwargs) -> Dict[str, Any]:
    """Convenience function for resilient GET requests."""
    client = get_resilient_http_client()
    return await client.get(url, **kwargs)


async def resilient_post(url: str, data: Any = None, **kwargs) -> Dict[str, Any]:
    """Convenience function for resilient POST requests."""
    client = get_resilient_http_client()
    return await client.post(url, data, **kwargs)
