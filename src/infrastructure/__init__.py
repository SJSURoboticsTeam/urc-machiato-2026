#!/usr/bin/env python3
"""
URC 2026 Infrastructure Package

Provides unified infrastructure components:
- config: Unified configuration system
- bridges: Communication bridges (CAN, WebSocket, etc.)
- monitoring: System health and performance monitoring

Author: URC 2026 Infrastructure Team
"""

from .config import (
    get_urc_config,
    get_config,
    get_system_config,
    reload_config,
    RoverConfig,
)

from .bridges import (
    CANBridge,
    BridgeMessage,
    BridgeStatus,
    AdaptiveCircuitBreaker,
    get_adaptive_circuit_breaker,
)

__all__ = [
    # Config
    "get_urc_config",
    "get_config",
    "get_system_config",
    "reload_config",
    "RoverConfig",
    # Bridges
    "CANBridge",
    "BridgeMessage",
    "BridgeStatus",
    "AdaptiveCircuitBreaker",
    "get_adaptive_circuit_breaker",
]
