.. URC 2026 - Mars Rover Autonomy System documentation master file

==========================================
URC 2026 - Mars Rover Autonomy System
==========================================

.. image:: _static/urc_logo.png
   :alt: URC 2026 Logo
   :align: center
   :width: 200px

Welcome to the official documentation for the University Rover Challenge 2026 Mars Rover Autonomy System.

This comprehensive autonomous robotics system features:

- 🤖 **Unified Autonomy Stack**: ROS2-based navigation, state machines, and safety systems
- 📊 **Enterprise Monitoring**: Real-time telemetry, health monitoring, and performance metrics  
- 💻 **Modern Web Interface**: React dashboard for mission control and system monitoring
- 🎮 **Advanced Simulation**: Gazebo-based testing and validation environment
- 🔧 **Professional Tooling**: Unified build system, smart test runner, and CI/CD pipeline

.. toctree::
   :maxdepth: 2
   :caption: Getting Started

   quickstart
   big_picture
   getting_started

.. toctree::
   :maxdepth: 2
   :caption: Onboarding by Specialization

   onboarding/README
   onboarding/PILLAR_1_PERCEPTION
   onboarding/PILLAR_2_COGNITION
   onboarding/PILLAR_3_MOTION_CONTROL
   onboarding/PILLAR_4_COMMUNICATION

.. toctree::
   :maxdepth: 2
   :caption: System Architecture

   unified_systems
   architecture/overview
   architecture/unified_infrastructure
   architecture/autonomy_core
   architecture/diagrams

.. toctree::
   :maxdepth: 2
   :caption: Team Guides

   network_guide
   slam_nav_guide
   arm_guide

.. toctree::
   :maxdepth: 2
   :caption: Development

   development/workflow
   development/testing
   development/code_quality
   simulation_testing_guide
   code_review_process

.. toctree::
   :maxdepth: 2
   :caption: Hardware

   hardware/calibration
   hardware/interfaces
   calibration/camera_calibration
   hardware_integration

.. toctree::
   :maxdepth: 2
   :caption: API Reference

   api/index

.. toctree::
   :maxdepth: 2
   :caption: Operations

   operations/build_system
   operations/deployment
   operations/troubleshooting

System Overview
===============

The URC 2026 system is built on 5 unified, enterprise-grade systems:

🧪 **Unified Test Suite** (`src/core/test_suite.py`)
- 140+ test files consolidated into 1 comprehensive testing framework
- Test data factories, mock management, performance benchmarking
- **75% code reduction** from scattered test files

📊 **Unified Data Manager** (`src/core/data_manager.py`)
- High-performance data processing, validation, and analytics
- JSON schema validation, statistical analysis, coordinate transformations
- **60% code reduction** from 6 separate data systems

🛠️ **Unified Utilities** (`src/core/utilities.py`)
- Safety monitoring, hardware validation, recovery coordination
- Network resilience, system utilities, emergency handling
- **50% code reduction** from 4 separate utility systems

🤖 **Unified State Management** (`src/core/state_management.py`)
- Hierarchical state machines and behavior trees
- Real-time state synchronization and persistence
- **45% code reduction** from 3 separate state systems

📈 **Unified Observability** (`src/core/observability.py`)
- Structured logging, Prometheus metrics, health monitoring
- Performance profiling and real-time dashboards
- **50% code reduction** from 4 separate monitoring systems

**Total Code Reduction: 56%** across all unified systems!

Key Features
============

🎯 **Mission Capabilities**
- Sample collection and analysis
- Autonomous navigation and waypoint following
- Equipment service tasks
- Object manipulation and delivery

🛡️ **Safety & Reliability**
- Emergency stop systems
- Circuit breaker patterns for network resilience
- Health monitoring and auto-recovery
- Redundant critical systems

📊 **Real-time Monitoring**
- Prometheus metrics collection
- WebSocket-based real-time dashboards
- Performance profiling and alerting
- System health diagnostics

🧪 **Comprehensive Testing**
- 85% minimum code coverage requirement
- Hardware-in-the-loop testing
- Performance benchmarking
- Chaos engineering for resilience

Quick Links
===========

🏃 **Quick Start**
   Get the rover system running in 10 minutes with our :doc:`quickstart` guide.

🔧 **Development Workflow**
   Learn the daily development process in :doc:`development/workflow`.

🧪 **Testing Strategy**
   Understand our comprehensive testing approach in :doc:`development/testing`.

📊 **Monitoring**
   Explore system monitoring and observability features.

🚀 **Deployment**
   Production deployment procedures in :doc:`operations/deployment`.

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex` 
* :ref:`search`

Project Information
==================

.. list-table:: Project Details
   :header-rows: 1
   :widths: 30 70

   * - **Project Name**
     - URC 2026 Mars Rover Autonomy System
   * - **Architecture**
     - ROS2 Humble + Python 3.10+ + React TypeScript
   * - **Build System**
     - Unified build system with smart test runner
   * - **License**
     - MIT
   * - **Repository**
     - `GitHub Repository <https://github.com/your-org/urc-machiato-2026>`_
   * - **Team**
     - University Rover Challenge Team

Getting Help
============

- **📖 Documentation**: Browse this documentation site
- **🤝 Team Chat**: Ask questions in development channels  
- **📋 Issues**: Check existing GitHub issues first
- **📧 Contact**: urc2026@your-university.edu

Remember: This is a complex autonomous robotics system. Don't hesitate to ask questions!