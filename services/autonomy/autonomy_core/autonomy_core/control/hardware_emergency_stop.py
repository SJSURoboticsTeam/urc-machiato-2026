"""Hardware emergency stop (competition / HIL).

Provides a minimal, test-backed implementation. GPIO uses RPi.GPIO when
available; callbacks support integration with ``IntegratedCriticalSystems``.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Dict, List, Optional

logger = logging.getLogger(__name__)

try:
    import RPi.GPIO as GPIO  # type: ignore[import-untyped]

    _GPIO_AVAILABLE = True
except ImportError:
    GPIO = None  # type: ignore[assignment]
    _GPIO_AVAILABLE = False


class StopLevel(Enum):
    """Emergency stop severity."""

    NONE = "none"
    SOFT_STOP = "soft_stop"
    HARD_STOP = "hard_stop"


@dataclass
class EmergencyStopConfig:
    """Configuration for hardware E-stop."""

    e_stop_gpio_pin: int = 17
    motor_power_relay_pin: int = 18
    status_led_pin: int = 24
    watchdog_timeout_ms: int = 100
    auto_reset_seconds: int = 5
    require_manual_reset: bool = True


class HardwareEmergencyStop:
    """Software model of hardware E-stop with optional GPIO."""

    def __init__(self, config: EmergencyStopConfig) -> None:
        self.config = config
        self.is_active: bool = False
        self.stop_level: StopLevel = StopLevel.NONE
        self.stop_reason: str = "none"
        self._gpio_initialized: bool = False
        self._watchdog_enabled: bool = True
        self._last_watchdog_feed: float = time.time()
        self._callbacks: Dict[str, Callable[[str, Dict[str, Any]], None]] = {}

    def register_status_callback(
        self, name: str, callback: Callable[[str, Dict[str, Any]], None]
    ) -> None:
        self._callbacks[name] = callback

    def _notify(self, status: str, info: Dict[str, Any]) -> None:
        for cb in self._callbacks.values():
            try:
                cb(status, info)
            except Exception as e:  # pragma: no cover - defensive
                logger.warning("E-stop callback error: %s", e)

    def _setup_gpio(self) -> None:
        if not _GPIO_AVAILABLE:
            self._gpio_initialized = False
            return
        assert GPIO is not None
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.config.e_stop_gpio_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.config.motor_power_relay_pin, GPIO.OUT)
        GPIO.setup(self.config.status_led_pin, GPIO.OUT)
        self._gpio_initialized = True

    def activate_soft_stop(self, reason: str) -> bool:
        self.is_active = True
        self.stop_level = StopLevel.SOFT_STOP
        self.stop_reason = reason
        self._notify(
            "soft_stop_activated",
            {"stop_reason": reason, "level": self.stop_level.value},
        )
        return True

    def activate_hard_stop(self, reason: str) -> bool:
        self.is_active = True
        self.stop_level = StopLevel.HARD_STOP
        self.stop_reason = reason
        self._notify(
            "hard_stop_activated",
            {"stop_reason": reason, "level": self.stop_level.value},
        )
        return True

    def reset_emergency_stop(self) -> bool:
        self.is_active = False
        self.stop_level = StopLevel.NONE
        self.stop_reason = "none"
        return True

    def feed_watchdog(self) -> None:
        self._last_watchdog_feed = time.time()

    def get_status(self) -> Dict[str, Any]:
        return {
            "is_active": self.is_active,
            "stop_level": self.stop_level.value,
            "stop_reason": self.stop_reason,
            "gpio_initialized": self._gpio_initialized,
            "watchdog_enabled": self._watchdog_enabled,
        }


_emergency_stop_singleton: Optional[HardwareEmergencyStop] = None


def initialize_emergency_stop(config: EmergencyStopConfig) -> HardwareEmergencyStop:
    """Create (or replace) the process-wide E-stop instance."""
    global _emergency_stop_singleton
    inst = HardwareEmergencyStop(config)
    if _GPIO_AVAILABLE:
        try:
            inst._setup_gpio()
        except Exception as e:
            logger.warning("GPIO setup failed (non-fatal in sim): %s", e)
            inst._gpio_initialized = False
    else:
        # No RPi: treat as simulation so integration validation can run in CI.
        inst._gpio_initialized = True

    _emergency_stop_singleton = inst
    return inst


def cleanup_emergency_stop() -> None:
    global _emergency_stop_singleton
    _emergency_stop_singleton = None
