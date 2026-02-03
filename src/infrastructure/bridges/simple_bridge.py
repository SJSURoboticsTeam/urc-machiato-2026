"""
Stub for backward compatibility - use CANBridge directly.

SimpleBridge is an alias for CANBridge. New code should use CANBridge.
"""

from .can_bridge import CANBridge, BridgeStatus, BridgeMessage

SimpleBridge = CANBridge
_singleton: CANBridge | None = None


def get_simple_bridge(config: dict | None = None) -> CANBridge:
    """Return a CAN bridge instance (singleton with default config)."""
    global _singleton
    if _singleton is None:
        _singleton = CANBridge(config or {"device": "/dev/ttyACM0", "baudrate": 115200})
    return _singleton


__all__ = [
    "SimpleBridge",
    "get_simple_bridge",
    "CANBridge",
    "BridgeStatus",
    "BridgeMessage",
]
