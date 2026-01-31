================
Troubleshooting
================

Common issues and resolutions for URC 2026.

Build Issues
============

**Colcon / Python conflicts**

- Use the unified build: ``./scripts/build.sh dev``
- Ensure system Python is used for ROS2; avoid activating a venv that overrides ROS2 Python.

**Import errors**

- Install package in development mode: ``pip install -e .``
- Ensure ``PYTHONPATH`` includes project root when running from ``scripts/`` or ``tests/``.

**Configuration not loading**

- Single source of truth: ``from src.infrastructure.config import get_urc_config``
- Check ``config/rover.yaml`` and ``URC_ENV`` (development/simulation/competition).

ROS2 Issues
===========

**Nodes not found**

- Build and source: ``./scripts/build.sh dev`` then ``source install/setup.bash``
- Check package name: ``autonomy_core`` for consolidated nodes.

**Topics not publishing**

- Verify launch mode matches deployment (simulation vs hardware).
- Check diagnostics: ``ros2 topic echo /diagnostics``.

Testing
=======

**Tests fail with config errors**

- Unified config uses fallback when Dynaconf is not installed; ensure ``src.infrastructure.config`` is importable.

**Hardware tests**

- Use ``python scripts/run_tests.py hardware`` when hardware is available; otherwise run unit and integration tests only.

See :doc:`build_system` and :doc:`deployment`.
