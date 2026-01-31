======================
Hardware Interfaces
======================

Hardware abstraction and interfaces for the URC 2026 rover.

Overview
========

- **Hardware interface**: ``src/autonomy/control/hardware_interface/`` — Motor drivers, CAN/serial
- **CAN bridge**: ``src/infrastructure/bridges/can_bridge.py`` — SLCAN protocol to chassis
- **Mock mode**: Set ``config.hardware.use_mock`` (from ``get_urc_config()``) for simulation and testing

Configuration
=============

::

    from src.infrastructure.config import get_urc_config
    config = get_urc_config()
    config.hardware.use_mock
    config.network.can_interface
    config.network.can_bitrate

See :doc:`../architecture/unified_infrastructure` and :doc:`../onboarding/PILLAR_3_MOTION_CONTROL`.
