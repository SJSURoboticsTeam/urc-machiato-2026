================
Hardware Calibration
================

Calibration procedures for cameras, sensors, and arm systems.

Camera Calibration
==================

See the dedicated camera calibration guide: :doc:`../calibration/camera_calibration`.

Summary:

- **Intrinsic calibration**: Lens distortion and focal length
- **Extrinsic calibration**: Camera position relative to rover
- **Hand-eye calibration**: Coordinate system alignment for arm and vision

Configuration
=============

Unified config (``src/infrastructure/config``) provides calibration-related settings. Use ``get_urc_config()`` for runtime configuration.

Arm and Typing
==============

Arm calibration and keyboard alignment: see :doc:`../arm_guide`.
