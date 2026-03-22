#!/usr/bin/env python3
"""
Async Utilities for Performance Optimization

Provides async alternatives to blocking operations like time.sleep.
Includes utilities for non-blocking delays, timeouts, and concurrent operations.

Usage:
    # Replace time.sleep(1.0)
    await asyncio.sleep(1.0)
    
    # Replace blocking operations with async versions
    result = await async_timeout(operation, timeout=5.0)
    
    # Use async context managers for resources
    async with async_file_lock(filename) as f:
        content = await f.read()

Author: URC 2026 Performance Team
"""

import asyncio
import time
import threading
from typing import Any, Awaitable, TypeVar, Callable, Optional, Union
from concurrent.futures import ThreadPoolExecutor, Future
from contextlib import asynccontextmanager

T = TypeVar("T")


class AsyncLockManager:
    """Thread-safe async lock manager."""

    def __init__(self, max_workers: int = 4):
        self.executor = ThreadPoolExecutor(max_workers=max_workers)
        self.file_locks = {}
        self.global_lock = asyncio.Lock()

    async def acquire_file_lock(self, filename: str) -> asyncio.Lock:
        """Get async lock for file operations."""
        async with self.global_lock:
            if filename not in self.file_locks:
                self.file_locks[filename] = asyncio.Lock()
            return self.file_locks[filename]

    async def run_blocking(
        self, func: Callable[..., T], *args, timeout: Optional[float] = None
    ) -> T:
        """Run blocking function in thread pool."""
        loop = asyncio.get_event_loop()

        if timeout:
            return await asyncio.wait_for(
                loop.run_in_executor(self.executor, func, *args), timeout=timeout
            )
        else:
            return await loop.run_in_executor(self.executor, func, *args)


class AsyncTimeoutError(Exception):
    """Custom timeout exception for async operations."""

    pass


class AsyncPerformanceMonitor:
    """Monitor performance of async operations."""

    def __init__(self):
        self.operation_times = {}
        self.operation_counts = {}

    async def monitor_operation(
        self, operation_name: str, operation: Awaitable[T]
    ) -> T:
        """Monitor and time an async operation."""
        start_time = time.time()
        try:
            result = await operation
            duration = time.time() - start_time

            # Track metrics
            if operation_name not in self.operation_times:
                self.operation_times[operation_name] = []
                self.operation_counts[operation_name] = 0

            self.operation_times[operation_name].append(duration)
            self.operation_counts[operation_name] += 1

            # Log performance warnings
            if duration > 1.0:  # Operations taking >1 second
                import logging

                logger = logging.getLogger(__name__)
                logger.warning(
                    f"Slow operation detected: {operation_name} took {duration:.2f}s"
                )

            return result
        except Exception as e:
            duration = time.time() - start_time
            import logging

            logger = logging.getLogger(__name__)
            logger.error(
                f"Operation {operation_name} failed after {duration:.2f}s: {e}"
            )
            raise

    def get_performance_stats(self) -> dict:
        """Get performance statistics for all tracked operations."""
        stats = {}
        for op_name, times in self.operation_times.items():
            if times:
                count = self.operation_counts[op_name]
                avg_time = sum(times) / len(times)
                max_time = max(times)
                min_time = min(times)

                stats[op_name] = {
                    "count": count,
                    "avg_time": avg_time,
                    "max_time": max_time,
                    "min_time": min_time,
                    "total_time": sum(times),
                }
        return stats


# Global instances
_async_manager = AsyncLockManager()
_perf_monitor = AsyncPerformanceMonitor()


def get_async_manager() -> AsyncLockManager:
    """Get global async lock manager."""
    return _async_manager


def get_perf_monitor() -> AsyncPerformanceMonitor:
    """Get global performance monitor."""
    return _perf_monitor


async def async_timeout(operation: Awaitable[T], timeout: float) -> T:
    """Add timeout to any async operation."""
    try:
        return await asyncio.wait_for(operation, timeout=timeout)
    except asyncio.TimeoutError:
        raise AsyncTimeoutError(f"Operation timed out after {timeout} seconds")


async def async_delay(seconds: float) -> None:
    """Non-blocking delay - replacement for time.sleep()."""
    await asyncio.sleep(seconds)


async def async_retry(
    operation: Callable[..., Awaitable[T]],
    max_attempts: int = 3,
    delay: float = 1.0,
    backoff_factor: float = 2.0,
) -> T:
    """Async retry with exponential backoff."""
    last_exception = None

    for attempt in range(max_attempts):
        try:
            return await operation()
        except Exception as e:
            last_exception = e
            if attempt < max_attempts - 1:
                wait_time = delay * (backoff_factor**attempt)
                import logging

                logger = logging.getLogger(__name__)
                logger.warning(
                    f"Operation failed (attempt {attempt + 1}/{max_attempts}), retrying in {wait_time:.1f}s: {e}"
                )
                await asyncio.sleep(wait_time)

    raise last_exception


@asynccontextmanager
async def async_file_lock(filename: str):
    """Async context manager for file locking."""
    lock = await _async_manager.acquire_file_lock(filename)
    await lock.acquire()
    try:
        yield lock
    finally:
        lock.release()


class AsyncBatchProcessor:
    """Process items in batches asynchronously."""

    def __init__(self, batch_size: int = 100, max_concurrent: int = 10):
        self.batch_size = batch_size
        self.semaphore = asyncio.Semaphore(max_concurrent)

    async def process_items(
        self, items: list[T], processor: Callable[[T], Awaitable[Any]]
    ) -> list[Any]:
        """Process items in batches with concurrency control."""
        results = []

        # Process in batches
        for i in range(0, len(items), self.batch_size):
            batch = items[i : i + self.batch_size]
            batch_results = await asyncio.gather(
                *[self._process_single_item(processor, item) for item in batch]
            )
            results.extend(batch_results)

        return results

    async def _process_single_item(
        self, processor: Callable[[T], Awaitable[Any]], item: T
    ) -> Any:
        """Process single item with semaphore control."""
        async with self.semaphore:
            return await processor(item)


async def async_gather_with_errors(
    *operations: Awaitable[T],
) -> tuple[list[T], list[Exception]]:
    """Gather operations, returning results and errors separately."""
    results = []
    errors = []

    for operation in asyncio.as_completed(operations):
        try:
            result = await operation
            results.append(result)
        except Exception as e:
            errors.append(e)

    return results, errors


class AsyncCircuitBreaker:
    """Async circuit breaker for fault tolerance."""

    def __init__(
        self,
        failure_threshold: int = 5,
        timeout: float = 60.0,
        expected_exception: type = Exception,
    ):
        self.failure_threshold = failure_threshold
        self.timeout = timeout
        self.expected_exception = expected_exception
        self.failure_count = 0
        self.last_failure_time = None
        self.state = "CLOSED"  # CLOSED, OPEN, HALF_OPEN

    async def call(self, operation: Callable[..., Awaitable[T]], *args, **kwargs) -> T:
        """Execute operation with circuit breaker protection."""
        if self.state == "OPEN":
            if time.time() - self.last_failure_time > self.timeout:
                self.state = "HALF_OPEN"
            else:
                raise AsyncTimeoutError("Circuit breaker is OPEN")

        try:
            result = await operation(*args, **kwargs)
            self._on_success()
            return result
        except self.expected_exception as e:
            self._on_failure()
            raise

    def _on_success(self):
        """Handle successful operation."""
        self.failure_count = 0
        self.state = "CLOSED"

    def _on_failure(self):
        """Handle failed operation."""
        self.failure_count += 1
        self.last_failure_time = time.time()

        if self.failure_count >= self.failure_threshold:
            self.state = "OPEN"


# Utility functions for common async patterns
async def async_rate_limit(calls_per_second: float):
    """Rate limiter for async operations."""
    min_interval = 1.0 / calls_per_second
    last_call_time = 0

    while True:
        current_time = time.time()
        elapsed = current_time - last_call_time

        if elapsed < min_interval:
            await asyncio.sleep(min_interval - elapsed)

        last_call_time = time.time()
        yield


def sync_to_async(func: Callable[..., T]) -> Callable[..., Awaitable[T]]:
    """Convert synchronous function to async."""

    async def wrapper(*args, **kwargs):
        loop = asyncio.get_event_loop()
        return await loop.run_in_executor(
            _async_manager.executor, func, *args, **kwargs
        )

    return wrapper


def performance_monitor(operation_name: str):
    """Decorator to monitor async operation performance."""

    def decorator(func: Callable[..., Awaitable[T]]) -> Callable[..., Awaitable[T]]:
        async def wrapper(*args, **kwargs):
            return await _perf_monitor.monitor_operation(
                operation_name, func(*args, **kwargs)
            )

        return wrapper

    return decorator


# Example usage and testing
if __name__ == "__main__":

    async def demo_async_utilities():
        """Demonstrate async utilities."""
        print("🚀 Async Utilities Demo")

        # Demo 1: Non-blocking delay
        print("1. Non-blocking delay...")
        start = time.time()
        await async_delay(1.0)
        print(f"   Delay completed in {time.time() - start:.2f}s")

        # Demo 2: Timeout protection
        print("2. Timeout protection...")
        try:
            await async_timeout(asyncio.sleep(2.0), 1.0)
        except AsyncTimeoutError:
            print("   ✅ Timeout protection working")

        # Demo 3: Retry with backoff
        print("3. Retry with backoff...")
        attempt = 0

        async def failing_operation():
            nonlocal attempt
            attempt += 1
            if attempt < 3:
                raise ValueError("Simulated failure")
            return "success"

        try:
            result = await async_retry(failing_operation, max_attempts=3, delay=0.1)
            print(f"   ✅ Retry successful: {result}")
        except Exception as e:
            print(f"   ❌ Retry failed: {e}")

        # Demo 4: Performance monitoring
        print("4. Performance monitoring...")

        @performance_monitor("demo_operation")
        async def demo_operation():
            await asyncio.sleep(0.5)
            return "completed"

        await demo_operation()

        stats = get_perf_monitor().get_performance_stats()
        if "demo_operation" in stats:
            op_stats = stats["demo_operation"]
            print(f"   📊 Performance: {op_stats['avg_time']:.3f}s average")

    # Run demo
    asyncio.run(demo_async_utilities())
