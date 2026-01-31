=======================
Unified Infrastructure
=======================

The unified infrastructure package (``src/infrastructure/``) provides configuration, bridges, and monitoring in a single place.

Configuration
=============

Single source of truth::

    from src.infrastructure.config import get_urc_config

    config = get_urc_config()
    config.navigation.max_linear_velocity_ms
    config.safety.emergency_stop_enabled
    config.simulation.enabled
    config.network.websocket_port

- **Location**: ``src/infrastructure/config/``
- **Components**: ``settings.py`` (Dynaconf), ``schemas.py`` (Pydantic), ``validators.py`` (URC-specific validation)
- **Environment**: ``URC_ENV`` (development, simulation, competition) controls which overrides load

Bridges
=======

- **CAN bridge**: ``src/infrastructure/bridges/can_bridge.py`` — SLCAN protocol, velocity commands, heartbeat
- **Circuit breaker**: ``src/infrastructure/bridges/circuit_breaker.py`` — AdaptiveCircuitBreaker for fault tolerance
- **Legacy**: ``src/bridges/`` and ``src/comms/`` content has been consolidated into ``src/infrastructure/bridges/``

Circuit Breaker
===============

Use for unreliable or external calls::

    from src.infrastructure.bridges import get_adaptive_circuit_breaker

    breaker = get_adaptive_circuit_breaker("motion_control")
    result = breaker.call(my_function, arg1, arg2)

Profiles: motion_control (99.9%), sensor_fusion (99.5%), navigation (99%), telemetry (95%).

Monitoring
==========

- **Location**: ``src/infrastructure/monitoring/``
- Message loss and network partition detection utilities

See onboarding :doc:`../onboarding/PILLAR_4_COMMUNICATION` for communication details.
