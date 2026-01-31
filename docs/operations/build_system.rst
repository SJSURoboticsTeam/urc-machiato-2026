================
Build System
================

Unified build system for URC 2026. Single entry point for development, production, and competition builds.

Unified Build Commands
======================

Development build (default, fast)::

    ./scripts/build.sh dev

Production build (optimized)::

    ./scripts/build.sh prod

Competition build (safety-first)::

    ./scripts/build.sh comp

With tests::

    ./scripts/build.sh dev --test
    ./scripts/build.sh prod --test --verbose

Clean build artifacts::

    ./scripts/build.sh clean

Options
=======

- ``--test`` — Run tests after build
- ``--deploy`` — Deploy after successful build
- ``--no-ros`` — Skip ROS2 packages (Python only)
- ``--verbose`` / ``-v`` — Verbose output

What Gets Built
===============

- **Python**: ``pip install -e .`` (development mode)
- **ROS2**: ``colcon build`` with mode-specific args
- **Frontend** (prod/comp only): ``npm run build`` in ``src/dashboard``

Environment
===========

- **dev**: ``URC_ENV=development``, ``CMAKE_BUILD_TYPE=Debug``, symlink install
- **prod**: ``URC_ENV=production``, ``CMAKE_BUILD_TYPE=Release``
- **comp**: ``URC_ENV=competition``, ``URC_SAFETY_LEVEL=HIGH``

See also :doc:`deployment` and :doc:`troubleshooting`.
