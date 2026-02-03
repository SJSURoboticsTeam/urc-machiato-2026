#!/usr/bin/env python3
"""
URC 2026 Infrastructure Bridges

Unified communication bridges for CAN, WebSocket, and other protocols.
Includes circuit breaker patterns for fault tolerance.

Author: URC 2026 Infrastructure Team
"""

# Simplified circuit breaker only; stubs if import fails
try:
    from .simplified_circuit_breaker import (
        AdaptiveCircuitBreaker,
        CircuitBreakerState,
        CircuitBreakerOpenException,
        CircuitBreakerTimeoutException,
        AvailabilityRequirement,
        get_adaptive_circuit_breaker,
        get_circuit_breaker_stats,
        get_motion_control_breaker,
        get_sensor_fusion_breaker,
        get_navigation_breaker,
        get_telemetry_breaker,
        get_camera_breaker,
        get_gps_breaker,
    )
except ImportError:
    # Stubs for compatibility when simplified_circuit_breaker unavailable
    class CircuitBreakerState:
        CLOSED = "closed"
        OPEN = "open"
        HALF_OPEN = "half_open"

    class CircuitBreakerOpenException(Exception):
        pass

    class CircuitBreakerTimeoutException(Exception):
        pass

    class AvailabilityRequirement:
        HIGH = "high"
        MEDIUM = "medium"
        LOW = "low"

    class AdaptiveCircuitBreaker:
        def __init__(self, name="breaker"):
            self.name = name
            self.state = CircuitBreakerState.CLOSED

    def get_adaptive_circuit_breaker(name):
        return AdaptiveCircuitBreaker(name)

    def get_circuit_breaker_stats(name):
        return {}

    def get_motion_control_breaker():
        return AdaptiveCircuitBreaker("motion_control")

    def get_sensor_fusion_breaker():
        return AdaptiveCircuitBreaker("sensor_fusion")

    def get_navigation_breaker():
        return AdaptiveCircuitBreaker("navigation")

    def get_telemetry_breaker():
        return AdaptiveCircuitBreaker("telemetry")

    def get_camera_breaker():
        return AdaptiveCircuitBreaker("camera")

    def get_gps_breaker():
        return AdaptiveCircuitBreaker("gps")


# Try to import CAN bridge
try:
    from .can_bridge import CANBridge, BridgeMessage, BridgeStatus
except ImportError:
    # Stubs for CAN bridge
    class BridgeMessage:
        pass

    class BridgeStatus:
        CONNECTED = "connected"
        DISCONNECTED = "disconnected"

    class CANBridge:
        def __init__(self, channel="vcan0"):
            self.channel = channel
            self.status = BridgeStatus.DISCONNECTED


__all__ = [
    # CAN Bridge
    "CANBridge",
    "BridgeMessage",
    "BridgeStatus",
    # Circuit Breaker
    "AdaptiveCircuitBreaker",
    "CircuitBreakerState",
    "CircuitBreakerOpenException",
    "CircuitBreakerTimeoutException",
    "AvailabilityRequirement",
    "get_adaptive_circuit_breaker",
    "get_circuit_breaker_stats",
    "get_motion_control_breaker",
    "get_sensor_fusion_breaker",
    "get_navigation_breaker",
    "get_telemetry_breaker",
    "get_camera_breaker",
    "get_gps_breaker",
]
