======================
Hardware Integration
======================

Hardware abstraction and integration preparation for URC 2026.

Overview
========

The hardware integration system provides:

- **Hardware abstraction**: Unified interface for mock/hardware switching
- **Configuration**: Runtime validation and hardware profile (see :doc:`architecture/unified_infrastructure`)
- **Calibration**: See :doc:`hardware/calibration` and :doc:`calibration/camera_calibration`

Configuration
=============

Use unified config for hardware settings::

    from src.infrastructure.config import get_urc_config
    config = get_urc_config()
    config.hardware.use_mock
    config.network.can_interface
    config.safety.emergency_stop_enabled

See also :doc:`hardware/interfaces` and :doc:`onboarding/PILLAR_3_MOTION_CONTROL`.
