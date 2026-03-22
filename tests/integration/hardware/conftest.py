"""Hardware / HIL integration test fixtures."""

from __future__ import annotations

import os
from typing import Optional

import pytest


@pytest.fixture(scope="session")
def can_interface_name() -> Optional[str]:
    """Real CAN interface from env (unset in CI / Docker without hardware)."""
    return os.environ.get("URC_CAN_INTERFACE")


@pytest.fixture
def require_can_interface(can_interface_name: Optional[str]) -> str:
    """Skip unless URC_CAN_INTERFACE is set (lab / HIL)."""
    if not can_interface_name:
        pytest.skip("Set URC_CAN_INTERFACE to run CAN HIL tests")
    return can_interface_name


@pytest.fixture
def stm32_serial_port() -> Optional[str]:
    """STM32 serial device path from env."""
    return os.environ.get("URC_STM32_SERIAL")


@pytest.fixture
def require_stm32_serial(stm32_serial_port: Optional[str]) -> str:
    """Skip unless URC_STM32_SERIAL is set."""
    if not stm32_serial_port:
        pytest.skip("Set URC_STM32_SERIAL to run serial HIL tests")
    return stm32_serial_port


@pytest.fixture
def hil_timeout_s() -> float:
    """Default timeout for hardware operations (seconds)."""
    return float(os.environ.get("URC_HIL_TIMEOUT_S", "30"))


@pytest.fixture
def ros_domain_id() -> Optional[int]:
    """Optional ROS_DOMAIN_ID override for isolated HIL runs."""
    raw = os.environ.get("ROS_DOMAIN_ID")
    if raw is None or raw == "":
        return None
    return int(raw)
