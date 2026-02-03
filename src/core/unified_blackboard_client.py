#!/usr/bin/env python3
"""
Unified Blackboard Client - Python interface to BT.CPP blackboard via ROS2 services.

Provides a unified interface for Python code to access the BT.CPP blackboard,
which is the single source of truth for system state.

Author: URC 2026 Autonomy Team
"""

import time
import logging
from typing import Any, Dict, Optional, Union
import rclpy
from rclpy.node import Node
from autonomy_interfaces.srv import GetBlackboardValue, SetBlackboardValue

# Import blackboard key constants
try:
    from core.blackboard_keys import BlackboardKeys
except ImportError:
    # Fallback if not available
    BlackboardKeys = None

logger = logging.getLogger(__name__)


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

        # DON'T wait for services - creates clients and proceeds
        # If services aren't available, writes will fail silently
        # This allows the node to start immediately instead of blocking for 10 seconds
        logger.info("Unified blackboard client initialized")

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
        # Check cache first
        if key in self.cache:
            cached_value, timestamp = self.cache[key]
            if time.time() - timestamp < self.cache_ttl:
                return cached_value

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
                    return value
                else:
                    error_msg = response.error_message if response else "No response"
                    logger.debug(f"Blackboard get failed for '{key}': {error_msg}")
            else:
                logger.warning(f"Blackboard get service call timed out for '{key}'")
        except Exception as e:
            logger.error(f"Blackboard get error for '{key}': {e}")
        return default

    def set(self, key: str, value: Any) -> bool:
        """
        Set value in unified blackboard. Non-blocking (fire-and-forget).
        Does not wait for service response; safe to call from high-rate telemetry loops.

        Args:
            key: Blackboard key to set
            value: Value to set (bool, int, float, str)

        Returns:
            True if request was sent, False only on serialization error
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
            # DON'T BLOCK - just send and forget. Don't use spin_until_future_complete
            # The service will respond asynchronously without blocking the telemetry loop
            # This is fire-and-forget - we don't wait for response
            # Cache is invalidated optimistically
            if key in self.cache:
                del self.cache[key]
            return True
        except Exception as e:
            logger.error(f"Blackboard set error for '{key}': {e}")

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
