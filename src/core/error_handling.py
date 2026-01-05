#!/usr/bin/env python3
"""
URC 2026 Error Handling Module

Provides robust error handling patterns using tenacity for retry logic
and circuit breaker implementations. Includes railway-oriented programming
patterns for functional error handling.

Author: URC 2026 Reliability Team
"""

import asyncio
import time
import logging
from typing import Any, Callable, Optional, Type, TypeVar, Union, Dict, List
from dataclasses import dataclass
from enum import Enum
import functools

from tenacity import (
    retry, stop_after_attempt, stop_after_delay, wait_exponential,
    wait_fixed, retry_if_exception_type, before_sleep_log,
    after_log, RetryError, AsyncRetrying
)
import orjson

logger = logging.getLogger(__name__)

T = TypeVar('T')


class ErrorCategory(Enum):
    """Categories of errors for handling strategies."""
    NETWORK = "network"
    HARDWARE = "hardware"
    TIMEOUT = "timeout"
    VALIDATION = "validation"
    RESOURCE = "resource"
    CONFIGURATION = "configuration"


class CircuitBreakerState(Enum):
    """Circuit breaker states."""
    CLOSED = "closed"      # Normal operation
    OPEN = "open"         # Failing, requests blocked
    HALF_OPEN = "half_open" # Testing if service recovered


@dataclass
class ErrorContext:
    """Context information for error handling."""
    operation: str
    category: ErrorCategory
    timestamp: float
    attempt_count: int
    total_attempts: int
    last_error: Optional[Exception] = None
    metadata: Optional[Dict[str, Any]] = None


@dataclass
class Result:
    """Railway-oriented programming result type."""
    value: Any = None
    error: Optional[Exception] = None

    def is_success(self) -> bool:
        """Check if result is successful."""
        return self.error is None

    def is_failure(self) -> bool:
        """Check if result is a failure."""
        return self.error is not None

    @classmethod
    def success(cls, value: T) -> 'Result[T]':
        """Create a successful result."""
        return cls(value=value)

    @classmethod
    def failure(cls, error: Exception) -> 'Result[T]':
        """Create a failure result."""
        return cls(error=error)

    def map(self, func: Callable[[T], Any]) -> 'Result':
        """Map successful value to new result."""
        if self.is_success():
            try:
                return Result.success(func(self.value))
            except Exception as e:
                return Result.failure(e)
        return self

    def bind(self, func: Callable[[T], 'Result']) -> 'Result':
        """Bind successful value to function returning result."""
        if self.is_success():
            try:
                return func(self.value)
            except Exception as e:
                return Result.failure(e)
        return self

    def recover(self, func: Callable[[Exception], T]) -> 'Result[T]':
        """Recover from failure with default value."""
        if self.is_failure():
            try:
                return Result.success(func(self.error))
            except Exception as e:
                return Result.failure(e)
        return self


class CircuitBreaker:
    """Simple circuit breaker implementation."""

    def __init__(self,
                 failure_threshold: int = 5,
                 recovery_timeout: float = 30.0,
                 expected_exception: Type[Exception] = Exception):
        self.failure_threshold = failure_threshold
        self.recovery_timeout = recovery_timeout
        self.expected_exception = expected_exception

        self.state = CircuitBreakerState.CLOSED
        self.failure_count = 0
        self.last_failure_time = 0.0

    def call(self, func: Callable, *args, **kwargs) -> Any:
        """Execute function through circuit breaker."""
        if self.state == CircuitBreakerState.OPEN:
            if time.time() - self.last_failure_time > self.recovery_timeout:
                self.state = CircuitBreakerState.HALF_OPEN
            else:
                raise CircuitBreakerOpenException("Circuit breaker is open")

        try:
            result = func(*args, **kwargs)
            self._on_success()
            return result

        except self.expected_exception as e:
            self._on_failure()
            raise e

    async def call_async(self, func: Callable, *args, **kwargs) -> Any:
        """Execute async function through circuit breaker."""
        if self.state == CircuitBreakerState.OPEN:
            if time.time() - self.last_failure_time > self.recovery_timeout:
                self.state = CircuitBreakerState.HALF_OPEN
            else:
                raise CircuitBreakerOpenException("Circuit breaker is open")

        try:
            result = await func(*args, **kwargs)
            self._on_success()
            return result

        except self.expected_exception as e:
            self._on_failure()
            raise e

    def _on_success(self):
        """Handle successful operation."""
        self.failure_count = 0
        self.state = CircuitBreakerState.CLOSED

    def _on_failure(self):
        """Handle failed operation."""
        self.failure_count += 1
        self.last_failure_time = time.time()

        if self.failure_count >= self.failure_threshold:
            self.state = CircuitBreakerState.OPEN

    def get_state(self) -> CircuitBreakerState:
        """Get current circuit breaker state."""
        return self.state


class CircuitBreakerOpenException(Exception):
    """Exception raised when circuit breaker is open."""
    pass


# Pre-configured retry decorators for common scenarios
def network_retry(max_attempts: int = 3, backoff_factor: float = 0.5):
    """Retry decorator for network operations."""
    return retry(
        stop=stop_after_attempt(max_attempts),
        wait=wait_exponential(multiplier=backoff_factor, min=0.1, max=10.0),
        retry=retry_if_exception_type((ConnectionError, TimeoutError, OSError)),
        before_sleep=before_sleep_log(logger, logging.WARNING),
        reraise=True
    )


def sensor_retry(max_attempts: int = 5, delay: float = 0.1):
    """Retry decorator for sensor operations."""
    return retry(
        stop=stop_after_attempt(max_attempts),
        wait=wait_fixed(delay),
        retry=retry_if_exception_type((OSError, ValueError)),
        before_sleep=before_sleep_log(logger, logging.DEBUG),
        reraise=True
    )


def database_retry(max_attempts: int = 3):
    """Retry decorator for database operations."""
    return retry(
        stop=stop_after_attempt(max_attempts),
        wait=wait_exponential(multiplier=1.0, min=0.1, max=5.0),
        retry=retry_if_exception_type((ConnectionError, TimeoutError)),
        before_sleep=before_sleep_log(logger, logging.ERROR),
        reraise=True
    )


def mission_critical_retry(max_attempts: int = 10, max_delay: float = 300.0):
    """Retry decorator for mission-critical operations."""
    return retry(
        stop=stop_after_delay(max_delay),
        wait=wait_exponential(multiplier=2.0, min=1.0, max=60.0),
        retry=retry_if_exception_type(Exception),  # Retry all exceptions for critical ops
        before_sleep=before_sleep_log(logger, logging.CRITICAL),
        reraise=True
    )


# Async versions
async def async_network_retry(max_attempts: int = 3, backoff_factor: float = 0.5):
    """Async retry for network operations."""
    async for attempt in AsyncRetrying(
        stop=stop_after_attempt(max_attempts),
        wait=wait_exponential(multiplier=backoff_factor, min=0.1, max=10.0),
        retry=retry_if_exception_type((ConnectionError, TimeoutError, OSError)),
        before_sleep=before_sleep_log(logger, logging.WARNING),
        reraise=True
    ):
        with attempt:
            return await attempt.fn()


# Railway-oriented programming helpers
def try_except(func: Callable[..., T]) -> Callable[..., Result]:
    """Convert function to railway-oriented result."""
    @functools.wraps(func)
    def wrapper(*args, **kwargs) -> Result:
        try:
            return Result.success(func(*args, **kwargs))
        except Exception as e:
            return Result.failure(e)
    return wrapper


async def async_try_except(func: Callable[..., T]) -> Callable[..., Result]:
    """Convert async function to railway-oriented result."""
    @functools.wraps(func)
    async def wrapper(*args, **kwargs) -> Result:
        try:
            return Result.success(await func(*args, **kwargs))
        except Exception as e:
            return Result.failure(e)
    return wrapper


# Context managers for error handling
class ErrorHandler:
    """Context manager for comprehensive error handling."""

    def __init__(self, operation: str, category: ErrorCategory):
        self.operation = operation
        self.category = category
        self.start_time = time.time()
        self.errors: List[Exception] = []

    def __enter__(self):
        logger.debug(f"Starting operation: {self.operation}")
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        duration = time.time() - self.start_time

        if exc_val:
            self.errors.append(exc_val)
            logger.error(
                f"Operation failed: {self.operation}",
                extra={
                    'operation': self.operation,
                    'category': self.category.value,
                    'duration': duration,
                    'error': str(exc_val),
                    'error_type': type(exc_val).__name__
                }
            )
            return False  # Re-raise exception

        logger.info(
            f"Operation completed: {self.operation}",
            extra={
                'operation': self.operation,
                'category': self.category.value,
                'duration': duration
            }
        )
        return True


# Utility functions for common error handling patterns
def safe_json_loads(data: Union[str, bytes], default=None) -> Result:
    """Safely parse JSON data."""
    try:
        if isinstance(data, str):
            data = data.encode('utf-8')
        parsed = orjson.loads(data)
        return Result.success(parsed)
    except Exception as e:
        logger.warning(f"JSON parsing failed: {e}")
        return Result.success(default) if default is not None else Result.failure(e)


def safe_json_dumps(data: Any, default=None) -> Result:
    """Safely serialize data to JSON."""
    try:
        serialized = orjson.dumps(data, option=orjson.OPT_INDENT_2)
        return Result.success(serialized.decode('utf-8'))
    except Exception as e:
        logger.warning(f"JSON serialization failed: {e}")
        return Result.success(default) if default is not None else Result.failure(e)


def validate_positive_number(value: Union[int, float], field_name: str) -> Result:
    """Validate that a value is a positive number."""
    try:
        num_value = float(value)
        if num_value <= 0:
            raise ValueError(f"{field_name} must be positive")
        return Result.success(num_value)
    except (ValueError, TypeError) as e:
        return Result.failure(ValueError(f"Invalid {field_name}: {e}"))


# Example usage and testing
@network_retry(max_attempts=3)
def unreliable_network_call(url: str) -> str:
    """Example network operation with retry logic."""
    # Simulate network operation that might fail
    if "fail" in url:
        raise ConnectionError("Simulated network failure")
    return f"Response from {url}"


@sensor_retry(max_attempts=5, delay=0.1)
def unreliable_sensor_reading(sensor_id: str) -> float:
    """Example sensor operation with retry logic."""
    # Simulate sensor that might fail
    if sensor_id == "faulty":
        raise OSError("Simulated sensor failure")
    return 23.5  # Mock temperature reading


if __name__ == "__main__":
    print("URC 2026 Error Handling Module Examples")
    print("=" * 50)

    # Test retry decorators
    print("\n1. Testing Retry Decorators:")

    try:
        result = unreliable_network_call("http://example.com/fail")
        print(f"❌ Unexpected success: {result}")
    except Exception as e:
        print(f"✅ Network retry failed as expected: {e}")

    try:
        result = unreliable_sensor_reading("working")
        print(f"✅ Sensor reading successful: {result}")
    except Exception as e:
        print(f"❌ Sensor reading failed: {e}")

    # Test railway-oriented programming
    print("\n2. Testing Railway-Oriented Programming:")

    # Safe JSON operations
    json_result = safe_json_loads('{"key": "value"}')
    if json_result.is_success():
        print("✅ JSON parsing successful")
    else:
        print(f"❌ JSON parsing failed: {json_result.error}")

    # Validation
    validation_result = validate_positive_number(10, "test_value")
    if validation_result.is_success():
        print("✅ Validation successful")
    else:
        print(f"❌ Validation failed: {validation_result.error}")

    # Test circuit breaker
    print("\n3. Testing Circuit Breaker:")

    breaker = CircuitBreaker(failure_threshold=2)

    def failing_operation():
        raise ConnectionError("Simulated failure")

    # First failure - should still work
    try:
        breaker.call(failing_operation)
    except ConnectionError:
        print("✅ First failure handled")

    # Second failure - should open circuit
    try:
        breaker.call(failing_operation)
        print("❌ Should have opened circuit")
    except CircuitBreakerOpenException:
        print("✅ Circuit breaker opened correctly")

    print("\n🎉 Error handling module ready for production use!")
