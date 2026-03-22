#!/usr/bin/env python3
"""
Unified Blackboard Client - Python interface to BT.CPP blackboard via ROS2 services.

Provides a unified interface for Python code to access the BT.CPP blackboard,
which is the single source of truth for system state.

Author: URC 2026 Autonomy Team
"""

import time
import threading
import logging
import os
import json
from typing import Any, Dict, Optional, Union, Callable, List
from collections import defaultdict
import rclpy
from rclpy.node import Node
try:
    from autonomy_interfaces.srv import GetBlackboardValue, SetBlackboardValue
except ImportError:
    GetBlackboardValue = None
    SetBlackboardValue = None

# Import blackboard key constants
try:
    from core.blackboard_keys import BlackboardKeys
except ImportError:
    # Fallback if not available
    BlackboardKeys = None

logger = logging.getLogger(__name__)

# Metrics for monitoring blackboard access patterns
_ACCESS_COUNT = defaultdict(int)
_ACCESS_LATENCY = defaultdict(list)
_ERROR_COUNT = defaultdict(int)


class UnifiedBlackboardClient:
    """
    Client for unified BT.CPP blackboard via ROS2 services.

    This client provides a unified interface to the BT.CPP blackboard,
    which is the single source of truth for system state. All Python code
    should use this client instead of maintaining separate blackboards.

    Features:
    - Caching for performance (100ms TTL)
    - Type-safe get/set operations
    - Automatic retry on service failures
    - Fallback to defaults on errors
    """

    def __init__(
        self,
        node: Node,
        cache_ttl: float = 0.1,
        get_timeout_sec: float = 0.1,
    ):
        """
        Initialize unified blackboard client.

        Args:
            node: ROS2 node for service calls
            cache_ttl: Cache time-to-live in seconds (default: 100ms)
            get_timeout_sec: Timeout for get() service calls in seconds (default: 0.1).
                Use a longer value (e.g. 2.0) in tests or when the server may be slow.
        """
        self.node = node
        self.cache_ttl = cache_ttl
        self.get_timeout_sec = get_timeout_sec

        # Service clients
        self.get_client = node.create_client(
            GetBlackboardValue, "/blackboard/get_value"
        )
        self.set_client = node.create_client(
            SetBlackboardValue, "/blackboard/set_value"
        )

        # Cache for performance (key -> (value, timestamp))
        self.cache: Dict[str, tuple[Any, float]] = {}

        # Callback management for change notifications
        self._callback_lock = threading.RLock()
        self._exact_callbacks: Dict[str, List[Callable[[str, Any], None]]] = defaultdict(list)
        self._prefix_callbacks: Dict[str, List[Callable[[str, Any], None]]] = defaultdict(list)

        # Schema validation: key -> {type: str, validator: Callable[[Any], bool], error_msg: str}
        self._schema: Dict[str, Dict[str, Any]] = {}

        # Auditing
        self._audit_lock = threading.RLock()
        self._audit_log: List[Dict[str, Any]] = []
        self._auditing_enabled = False  # Disabled by default to avoid performance impact

        # DON'T wait for services - creates clients and proceeds
        # If services aren't available, writes will fail silently
        # This allows the node to start immediately instead of blocking for 10 seconds
        logger.info("Unified blackboard client initialized")

        # Metrics for monitoring blackboard access patterns
        global _ACCESS_COUNT, _ACCESS_LATENCY, _ERROR_COUNT
        _ACCESS_COUNT = defaultdict(int)
        _ACCESS_LATENCY = defaultdict(list)
        _ERROR_COUNT = defaultdict(int)
        
        # Performance optimization: batch operation tracking for sets
        self._batch_operations_enabled = False
        self._pending_sets: Dict[str, Any] = {}   # key -> value
        self._batch_lock = threading.RLock()

    def get(self, key: str, default: Any = None, value_type: str = "") -> Any:
        """
        Get value from unified blackboard.

        Args:
            key: Blackboard key to retrieve
            default: Default value if key not found or error occurs
            value_type: Type hint ("bool", "int", "double", "string") - auto-detected if empty

        Returns:
            Value from blackboard, or default if not found/error
        """
        start_time = time.time()
        success = False
        error_msg = None
        value = default
        
        try:
            # Check cache first
            if key in self.cache:
                cached_value, timestamp = self.cache[key]
                if time.time() - timestamp < self.cache_ttl:
                    _ACCESS_COUNT[key] += 1
                    _ACCESS_LATENCY[key].append(time.time() - start_time)
                    # Keep only last 100 latency measurements
                    if len(_ACCESS_LATENCY[key]) > 100:
                        _ACCESS_LATENCY[key] = _ACCESS_LATENCY[key][-100:]
                    success = True
                    value = cached_value
                    return value

            # Call service
            request = GetBlackboardValue.Request()
            request.key = key
            request.value_type = value_type

            try:
                future = self.get_client.call_async(request)
                rclpy.spin_until_future_complete(
                    self.node, future, timeout_sec=self.get_timeout_sec
                )

                if future.done():
                    response = future.result()
                    if response and response.success:
                        # Parse value based on type
                        value = self._parse_value(response.value, response.value_type)
                        # Update cache
                        self.cache[key] = (value, time.time())
                        _ACCESS_COUNT[key] += 1
                        _ACCESS_LATENCY[key].append(time.time() - start_time)
                        # Keep only last 100 latency measurements
                        if len(_ACCESS_LATENCY[key]) > 100:
                            _ACCESS_LATENCY[key] = _ACCESS_LATENCY[key][-100:]
                        success = True
                        return value
                    else:
                        error_msg = response.error_message if response else "No response"
                        logger.debug(f"Blackboard get failed for '{key}': {error_msg}")
                        _ERROR_COUNT[key] += 1
                else:
                    logger.warning(f"Blackboard get service call timed out for '{key}'")
                    _ERROR_COUNT[key] += 1
            except Exception as e:
                logger.error(f"Blackboard get error for '{key}': {e}")
                _ERROR_COUNT[key] += 1
        except Exception as e:
            logger.error(f"Unexpected error in blackboard get for '{key}': {e}")
            _ERROR_COUNT[key] += 1
        finally:
            # Record latency even on error
            if key in _ACCESS_LATENCY and len(_ACCESS_LATENCY[key]) == 0:
                _ACCESS_LATENCY[key].append(time.time() - start_time)
            # Audit the operation
            self._audit_operation('get', key, value, success, error_msg)
        
        return value

    def set(self, key: str, value: Any) -> bool:
        """
        Set value in unified blackboard. If batch operations are enabled, the set is queued for later flush.
        Otherwise, it is sent immediately (non-blocking fire-and-forget).

        Args:
            key: Blackboard key to set
            value: Value to set (bool, int, float, str)

        Returns:
            True if the operation was queued (if batching) or sent immediately (if not batching), False only on validation error.
        """
        # Validate against schema if defined
        if not self.validate_key(key, value):
            logger.error(f"Blackboard set rejected for key '{key}' due to validation failure")
            return False
            
        # Update cache optimistically and notify callbacks (same as before)
        old_value = self.cache.get(key, (None, None))[0]
        self.cache[key] = (value, time.time())
        if old_value != value:
            self._notify_callbacks(key, value)

        # If batch operations are enabled, queue the set for later flush
        if self._batch_operations_enabled:
            with self._batch_lock:
                self._pending_sets[key] = value
            logger.debug(f"Queued blackboard set for key '{key}' (batch mode)")
            return True
        else:
            # Send immediately (non-blocking)
            return self._send_set_immediately(key, value)

    def _send_set_immediately(self, key: str, value: Any) -> bool:
        """
        Send a set operation immediately (non-blocking) without batching or cache update.
        Assumes that the value has already been validated and the cache updated.
        
        Args:
            key: Blackboard key to set
            value: Value to set (bool, int, float, str)
            
        Returns:
            True if the send was successful, False otherwise.
        """
        request = SetBlackboardValue.Request()
        request.key = key

        # Determine type and serialize value
        if isinstance(value, bool):
            request.value_type = "bool"
            request.value = "true" if value else "false"
        elif isinstance(value, int):
            request.value_type = "int"
            request.value = str(value)
        elif isinstance(value, float):
            request.value_type = "double"
            request.value = str(value)
        elif isinstance(value, str):
            request.value_type = "string"
            request.value = value
        else:
            logger.error(f"Unsupported value type for blackboard set: {type(value)}")
            return False

        try:
            future = self.set_client.call_async(request)
            # DON'T BLOCK - just send and forget
            logger.debug(f"Sent blackboard set for key '{key}' immediately (non-batch mode)")
            return True
        except Exception as e:
            logger.error(f"Blackboard set error for '{key}': {e}")
            return False
            
        request = SetBlackboardValue.Request()
        request.key = key

        # Determine type and serialize value
        if isinstance(value, bool):
            request.value_type = "bool"
            request.value = "true" if value else "false"
        elif isinstance(value, int):
            request.value_type = "int"
            request.value = str(value)
        elif isinstance(value, float):
            request.value_type = "double"
            request.value = str(value)
        elif isinstance(value, str):
            request.value_type = "string"
            request.value = value
        else:
            logger.error(f"Unsupported value type for blackboard set: {type(value)}")
            self._audit_operation('set', key, value, False, f"Unsupported type: {type(value)}")
            return False

        try:
            future = self.set_client.call_async(request)
            # DON'T BLOCK - just send and forget. Don't use spin_until_future_complete
            # The service will respond asynchronously without blocking the telemetry loop
            # This is fire-and-forget - we don't wait for response
            # Update cache optimistically and notify callbacks
            old_value = self.cache.get(key, (None, None))[0]
            self.cache[key] = (value, time.time())
            # Notify callbacks if the value actually changed (or if we don't have an old value)
            if old_value != value:
                self._notify_callbacks(key, value)
            self._audit_operation('set', key, value, True, None)
            return True
        except Exception as e:
            logger.error(f"Blackboard set error for '{key}': {e}")
            # In case of error, remove from cache to avoid stale optimistic value
            if key in self.cache:
                del self.cache[key]
            self._audit_operation('set', key, value, False, str(e))
            return False

    def _parse_value(self, value_str: str, value_type: str) -> Any:
        """Parse string value to appropriate type."""
        if value_type == "bool":
            return value_str.lower() in ("true", "1")
        elif value_type == "int":
            return int(float(value_str))  # Handle "1.0" -> 1
        elif value_type == "double":
            return float(value_str)
        elif value_type == "string":
            return value_str
        else:
            # Try to auto-detect
            try:
                if value_str.lower() in ("true", "false"):
                    return value_str.lower() == "true"
                elif "." in value_str:
                    return float(value_str)
                else:
                    return int(value_str)
            except ValueError:
                return value_str

    def clear_cache(self):
        """Clear the cache (useful for testing or forced refresh)."""
        self.cache.clear()

    def get_bool(self, key: str, default: bool = False) -> bool:
        """Convenience method for getting boolean values."""
        return self.get(key, default, "bool")

    def get_int(self, key: str, default: int = 0) -> int:
        """Convenience method for getting integer values."""
        return self.get(key, default, "int")

    def get_double(self, key: str, default: float = 0.0) -> float:
        """Convenience method for getting double values."""
        return self.get(key, default, "double")

    def get_string(self, key: str, default: str = "") -> str:
        """Convenience method for getting string values."""
        return self.get(key, default, "string")

    # Performance optimization: batch operations
    def enable_batch_operations(self, enabled: bool = True) -> None:
        """
        Enable or disable batching of set operations for improved performance.
        When enabled, set operations are queued and flushed periodically.
        
        Args:
            enabled: Whether to enable batch operations (default: True)
        """
        self._batch_operations_enabled = enabled
        if not enabled:
            self.flush_batch()
        logger.info(f"Blackboard batch operations {'enabled' if enabled else 'disabled'}")

    def flush_batch(self) -> int:
        """
        Flush all pending batched set operations to the blackboard.
        
        Returns:
            Number of operations that were flushed.
        """
        if not self._pending_sets:
            return 0
            
        with self._batch_lock:
            pending = dict(self._pending_sets)
            self._pending_sets.clear()
        
        # Apply all pending sets
        count = 0
        for key, value in pending.items():
            if self.set(key, value):  # This will still validate, schema check, etc.
                count += 1
                
        logger.debug(f"Flushed {count} batched blackboard set operations")
        return count

    def get_metrics(self) -> Dict[str, Any]:
        """
        Get monitoring metrics for blackboard access.
        
        Returns:
            Dictionary containing access counts, latency statistics, and error counts
        """
        metrics = {
            "access_count": dict(_ACCESS_COUNT),
            "error_count": dict(_ERROR_COUNT),
            "latency_stats": {},
            "batch_operations": {
                "enabled": self._batch_operations_enabled,
                "pending_sets": len(self._pending_sets)
            }
        }
        
        # Calculate latency statistics for each key
        for key, latencies in _ACCESS_LATENCY.items():
            if latencies:
                metrics["latency_stats"][key] = {
                    "count": len(latencies),
                    "mean": sum(latencies) / len(latencies),
                    "min": min(latencies),
                    "max": max(latencies),
                    "p50": sorted(latencies)[len(latencies) // 2] if latencies else 0,
                    "p95": sorted(latencies)[int(len(latencies) * 0.95)] if len(latencies) >= 20 else max(latencies) if latencies else 0,
                }
        
        return metrics

    def validate_key(self, key: str, value: Any) -> bool:
        """
        Validate a value against the schema for a key (if schema is defined).
        
        Args:
            key: The blackboard key
            value: The value to validate
            
        Returns:
            True if validation passes or no schema is defined, False otherwise
        """
        if key not in self._schema:
            # No schema defined for this key, validation passes by default
            return True
            
        schema = self._schema[key]
        expected_type = schema.get("type")
        validator = schema.get("validator")
        
        # Type check
        type_ok = False
        if expected_type == "bool":
            type_ok = isinstance(value, bool)
        elif expected_type == "int":
            type_ok = isinstance(value, int)
        elif expected_type == "double":
            type_ok = isinstance(value, (int, float))
        elif expected_type == "string":
            type_ok = isinstance(value, str)
        else:
            # Unknown type, skip type checking
            type_ok = True
            
        if not type_ok:
            logger.warning(f"Blackboard key '{key}' failed type check: expected {expected_type}, got {type(value)}")
            return False
            
        # Custom validator check
        if validator and not validator(value):
            logger.warning(f"Blackboard key '{key}' failed validation: {schema.get('error_msg', 'Validation failed')}")
            return False
            
        return True

    def define_schema(self, key: str, type_: str, validator: Optional[Callable[[Any], bool]] = None, error_msg: str = "") -> None:
        """
        Define a validation schema for a blackboard key.
        
        Args:
            key: The blackboard key to define schema for
            type_: Expected type ("bool", "int", "double", "string")
            validator: Optional custom validator function that takes value and returns bool
            error_msg: Error message to log if validation fails
        """
        self._schema[key] = {
            "type": type_,
            "validator": validator,
            "error_msg": error_msg
        }
        logger.debug(f"Defined schema for blackboard key '{key}': type={type_}")




    def _notify_callbacks(self, key: str, value: Any) -> None:
        """
        Notify all registered callbacks for a key change.
        
        Args:
            key: The blackboard key that changed
            value: The new value
        """
        with self._callback_lock:
            # Notify exact match callbacks
            if key in self._exact_callbacks:
                for callback in self._exact_callbacks[key][:]:  # Copy to avoid modification during iteration
                    try:
                        callback(key, value)
                    except Exception as e:
                        logger.error(f"Error in blackboard callback for key '{key}': {e}")
            
            # Notify prefix match callbacks
            for prefix, callbacks in self._prefix_callbacks.items():
                if key.startswith(prefix):
                    for callback in callbacks[:]:  # Copy to avoid modification during iteration
                        try:
                            callback(key, value)
                        except Exception as e:
                            logger.error(f"Error in blackboard prefix callback for key '{key}': {e}")

    def subscribe(self, key: str, callback: Callable[[str, Any], None]) -> Callable[[], None]:
        """
        Subscribe to changes for a specific blackboard key.
        
        Args:
            key: The blackboard key to subscribe to
            callback: Function to call when the key changes (receives key and new value)
            
        Returns:
            Unsubscribe function that can be called to remove the subscription
        """
        with self._callback_lock:
            self._exact_callbacks[key].append(callback)
            
        def unsubscribe():
            with self._callback_lock:
                if key in self._exact_callbacks and callback in self._exact_callbacks[key]:
                    self._exact_callbacks[key].remove(callback)
                    
        return unsubscribe

    def subscribe_prefix(self, prefix: str, callback: Callable[[str, Any], None]) -> Callable[[], None]:
        """
        Subscribe to changes for all keys matching a prefix.
        
        Args:
            prefix: The prefix to match (e.g., "auto." or "sensor.")
            callback: Function to call when any matching key changes (receives key and new value)
            
        Returns:
            Unsubscribe function that can be called to remove the subscription
        """
        with self._callback_lock:
            self._prefix_callbacks[prefix].append(callback)
            
        def unsubscribe():
            with self._callback_lock:
                if prefix in self._prefix_callbacks and callback in self._prefix_callbacks[prefix]:
                    self._prefix_callbacks[prefix].remove(callback)
                    
        return unsubscribe
