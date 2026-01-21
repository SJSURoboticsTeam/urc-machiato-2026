.. _unified_systems:

===================
Unified Systems
===================

The URC 2026 system has been architected with a unified, enterprise-grade approach to system design. This section documents the five core unified systems that provide the foundation for all rover functionality.

.. image:: _static/unified_systems_architecture.png
   :alt: Unified Systems Architecture
   :align: center
   :width: 100%

Overview
========

The unified systems architecture eliminates code duplication, improves maintainability, and provides consistent APIs across all components. Each system is designed to be modular, testable, and extensible.

Core Principles
---------------

**Single Responsibility**
    Each unified system has one clear purpose and provides a single, consistent API.

**Unified APIs**
    All systems follow consistent patterns for configuration, initialization, and usage.

**Modular Design**
    Systems can be used independently or together, with clear dependency management.

**Enterprise Quality**
    Professional-grade error handling, logging, monitoring, and testing.

**Performance Optimized**
    Designed for embedded systems with minimal memory footprint and fast startup times.

The Five Unified Systems
========================

.. _observability_system:

1. Unified Observability System
=================================

**Location**: ``src/core/observability.py``

**Purpose**: Comprehensive monitoring, logging, and health checking for the entire system.

**Consolidated From**:
- ``src/core/monitoring_system.py`` (Prometheus metrics)
- ``src/core/logging/structured_logger.py`` (Structured logging)
- ``src/autonomy/core/node_utils.py`` (NodeHealthMonitor)

Features
--------

**Structured Logging**
    Correlation IDs, multiple log levels, component tagging, and performance monitoring.

.. code-block:: python

    from src.core.observability import get_observability_manager

    obs = get_observability_manager()
    correlation_id = obs.info("User login", user_id="123", component="auth")
    obs.error("Database connection failed", correlation_id=correlation_id)

**Metrics Collection**
    Prometheus-compatible metrics with automatic labeling and aggregation.

.. code-block:: python

    obs.record_metric("response_time", 0.145, {"endpoint": "/api/users"})
    obs.increment_counter("requests_total", labels={"method": "GET"})

**Health Monitoring**
    Real-time health checks with automatic emergency response.

.. code-block:: python

    obs.add_health_check("database", lambda: check_db_health())
    status = obs.get_system_status()

**Performance Profiling**
    Built-in performance benchmarking for operations.

.. code-block:: python

    obs.start_performance_profile("api_call", "web_service")
    # ... execute operation ...
    metrics = obs.end_performance_profile("api_call")

API Reference
-------------

.. autoclass:: src.core.observability.ObservabilityManager
   :members:
   :undoc-members:
   :show-inheritance:

.. _state_management_system:

2. Unified State Management System
====================================

**Location**: ``src/core/state_management.py``

**Purpose**: Hierarchical state machines and behavior trees for complex system coordination.

**Consolidated From**:
- ``src/core/state_machine.py`` (State machines)
- ``src/core/behavior_tree.py`` (Behavior trees)
- ``src/core/state_synchronization_manager.py`` (State sync)

Features
--------

**Hierarchical State Machines**
    Complex state management with parent/child relationships and automatic transitions.

.. code-block:: python

    from src.core.state_management import get_state_manager

    state_mgr = get_state_manager()
    sm = state_mgr.create_state_machine("robot_control", "idle")
    sm.add_transition("start_mission", "idle", "executing")
    sm.trigger_event("start_mission")

**Behavior Trees**
    Modular decision-making trees with success/failure/pending states.

.. code-block:: python

    bt = state_mgr.create_behavior_tree("mission_logic")
    # Add behaviors to tree
    status = bt.tick()  # Returns "SUCCESS", "FAILURE", or "RUNNING"

**State Synchronization**
    Real-time state sharing across distributed components.

.. code-block:: python

    state_mgr.update_state("system_mode", "AUTONOMOUS")
    mode = state_mgr.get_state("system_mode")

**Persistence**
    Automatic state saving and recovery across system restarts.

.. code-block:: python

    state_mgr.enable_persistence("state_backup.json")

API Reference
-------------

.. autoclass:: src.core.state_management.StateManager
   :members:
   :undoc-members:
   :show-inheritance:

.. _data_management_system:

3. Unified Data Manager
========================

**Location**: ``src/core/data_manager.py``

**Purpose**: High-performance data processing, validation, and analytics.

**Consolidated From**:
- ``src/core/data_processor.py`` (Telemetry processing)
- ``src/core/statistics_processor.py`` (Statistical analysis)
- ``src/core/json_processor.py`` (JSON handling)
- ``src/core/transforms.py`` (Coordinate transformations)
- ``src/core/data_structures.py`` (Efficient data structures)

Features
--------

**Schema Validation**
    JSON schema-based validation with automatic error reporting.

.. code-block:: python

    from src.core.data_manager import get_data_manager, validate_data

    data_mgr = get_data_manager()
    is_valid, errors = validate_data(sensor_data, 'telemetry')

**Telemetry Processing**
    High-performance processing of sensor data with quality assessment.

.. code-block:: python

    processed_data, quality_metrics = data_mgr.process_telemetry(raw_sensor_data)

**Statistical Analysis**
    Comprehensive statistical analysis with outlier detection.

.. code-block:: python

    stats = data_mgr.analyze_statistics(values, 'distribution')
    # Returns: mean, std, outliers, normality tests, etc.

**High-Performance JSON**
    Optimized JSON serialization/deserialization using orjson.

.. code-block:: python

    json_str = data_mgr.json_dumps(large_dataset)
    parsed = data_mgr.json_loads(json_string)

**Coordinate Transformations**
    3D coordinate system transformations for robotics.

.. code-block:: python

    transformed = data_mgr.transform_coordinates(pose_data, 'world', 'robot')

API Reference
-------------

.. autoclass:: src.core.data_manager.DataManager
   :members:
   :undoc-members:
   :show-inheritance:

.. _utilities_system:

4. Unified Utilities
=====================

**Location**: ``src/core/utilities.py``

**Purpose**: Common utility functions for safety, hardware, and system management.

**Consolidated From**:
- ``src/core/hardware_validator.py`` (Hardware validation)
- ``src/core/safety_system.py`` (Safety monitoring)
- ``src/core/recovery_coordinator.py`` (Recovery mechanisms)
- ``src/core/network_resilience.py`` (Network fault tolerance)

Features
--------

**Safety Management**
    Comprehensive safety checking with emergency response.

.. code-block:: python

    from src.core.utilities import get_safety_manager

    safety_mgr = get_safety_manager()
    check = safety_mgr.perform_safety_check('motor_temp', 'thermal', check_temperature)

**Hardware Validation**
    Component compatibility checking and health monitoring.

.. code-block:: python

    hw_validator = get_hardware_validator()
    is_valid, errors = hw_validator.validate_hardware_config('imu', config)

**Recovery Coordination**
    Automated failure recovery with configurable strategies.

.. code-block:: python

    recovery_coord = get_recovery_coordinator()
    recovery_coord.initiate_recovery('communication_failure', context_data)

**Network Resilience**
    Adaptive timeouts and retry logic for unreliable networks.

.. code-block:: python

    network_mgr = get_network_resilience_manager()
    timeout = network_mgr.get_adaptive_timeout(5.0)  # Adapts to conditions

**System Utilities**
    Common functions for formatting, validation, and system information.

.. code-block:: python

    from src.core.utilities import SystemUtilities

    mem_mb = SystemUtilities.format_bytes(1048576)  # "1.0MB"
    sys_info = SystemUtilities.get_system_info()

API Reference
-------------

.. autoclass:: src.core.utilities.SafetyManager
   :members:
   :undoc-members:
   :show-inheritance:

.. _test_suite_system:

5. Unified Test Suite
======================

**Location**: ``src/core/test_suite.py``

**Purpose**: Comprehensive testing framework with data factories and mock management.

**Consolidated From**:
- ``tests/unit/`` (140+ individual test files)
- ``tests/integration/``, ``tests/system/``, ``tests/property/``
- ``tests/monitoring/``, ``tests/security/``, ``tests/competition/``
- ``tests/mocks/``, ``tests/demo/``, ``tests/ros2_integration/``

Features
--------

**Organized Test Suites**
    Hierarchical test organization with priorities and tags.

.. code-block:: python

    from src.core.test_suite import get_test_suite, create_test_suite, TestType

    test_suite = get_test_suite()
    unit_suite = create_test_suite('unit', 'Unit tests', TestType.UNIT)

**Test Data Factories**
    Realistic test data generation for all system components.

.. code-block:: python

    mission_data = test_suite.get_test_data('mission')
    sensor_data = test_suite.get_test_data('sensor', sensor_type='imu')

**Mock Management**
    Centralized mock objects for isolated testing.

.. code-block:: python

    mock_sensor = test_suite.create_mock('sensor', sensor_type='lidar')
    mock_can = test_suite.create_mock('can_bus')

**Performance Benchmarking**
    Built-in performance measurement for tests.

.. code-block:: python

    test_suite.start_performance_test('algorithm')
    # ... run test ...
    metrics = test_suite.end_performance_test('algorithm')

**Comprehensive Reporting**
    JSON and text reports with coverage and performance metrics.

.. code-block:: python

    report = test_suite.run_test_suite('unit_tests')
    print(f"Results: {report.passed_tests}/{report.total_tests} passed")

API Reference
-------------

.. autoclass:: src.core.test_suite.UnifiedTestSuite
   :members:
   :undoc-members:
   :show-inheritance:

System Integration
==================

The unified systems are designed to work seamlessly together, providing a cohesive architecture.

Integration Patterns
--------------------

**Configuration-Driven**
    All systems use the unified configuration manager for settings.

.. code-block:: python

    from src.core.configuration import get_config_manager
    from src.core.observability import get_observability_manager

    config = get_config_manager().load_config('development')
    obs = get_observability_manager()
    obs.info("System initialized", config=config.environment)

**Observer Pattern**
    Systems communicate through events and shared state.

.. code-block:: python

    # State changes trigger observability events
    state_mgr = get_state_manager()
    obs = get_observability_manager()

    def on_state_change(component, old_state, new_state):
        obs.info(f"State changed: {component} {old_state} -> {new_state}")

    state_mgr.subscribe_to_state('robot_mode', on_state_change)

**Data Flow**
    Data moves seamlessly between systems with automatic validation.

.. code-block:: python

    # Data flows: Sensor -> Validation -> Processing -> Storage -> Analytics
    data_mgr = get_data_manager()
    obs = get_observability_manager()

    # Validate incoming data
    if data_mgr.validate_data(sensor_reading, 'telemetry')[0]:
        processed, quality = data_mgr.process_telemetry([sensor_reading])
        obs.record_metric('data_quality', quality.validity)
        # Store processed data...

Performance Characteristics
===========================

The unified systems are optimized for embedded deployment with minimal resource usage.

Memory Footprint
----------------

- **Observability**: < 5MB baseline, scales with log retention
- **State Management**: < 2MB for typical state machines
- **Data Manager**: < 10MB with caching, depends on data volume
- **Utilities**: < 1MB core functionality
- **Test Suite**: < 3MB, only loaded during testing

Startup Time
------------

- **Cold Start**: < 500ms for all systems combined
- **Lazy Loading**: Heavy components loaded on-demand
- **Caching**: Configuration and schemas cached for fast access

Embedded Optimization
---------------------

All systems include optimizations for resource-constrained environments:

- Minimal imports and lazy loading
- Efficient data structures (circular buffers, LRU caches)
- Configurable logging levels and monitoring frequency
- Memory monitoring and cleanup
- Compressed serialization formats

Migration Guide
===============

Migrating from legacy systems to unified systems.

From Legacy Monitoring
----------------------

.. code-block:: python

    # Old
    from src.core.monitoring_system import get_monitoring_system
    monitoring = get_monitoring_system()
    monitoring.record_metric('cpu', 85)

    # New
    from src.core.observability import get_observability_manager
    obs = get_observability_manager()
    obs.record_metric('cpu_usage', 85.0, {'component': 'system'})

From Legacy Data Processing
---------------------------

.. code-block:: python

    # Old
    from src.core.data_processor import TelemetryDataProcessor
    processor = TelemetryDataProcessor()
    processed = processor.process_data(raw_data)

    # New
    from src.core.data_manager import get_data_manager
    data_mgr = get_data_manager()
    processed, quality = data_mgr.process_telemetry(raw_data)

From Legacy State Management
-----------------------------

.. code-block:: python

    # Old
    from src.core.state_machine import URCStateMachine
    sm = URCStateMachine()

    # New
    from src.core.state_management import get_state_manager
    state_mgr = get_state_manager()
    sm = state_mgr.create_state_machine('urc_states', 'idle')

Best Practices
==============

**System Initialization**
    Initialize systems in the correct order: Configuration → Observability → Others.

.. code-block:: python

    # Recommended initialization order
    config_mgr = get_config_manager()
    config = config_mgr.load_config()

    obs = get_observability_manager()
    obs.info("System starting", config=config.environment)

    # Initialize other systems...

**Error Handling**
    All systems provide comprehensive error handling with structured logging.

.. code-block:: python

    try:
        result = data_mgr.process_telemetry(sensor_data)
    except Exception as e:
        obs.error("Data processing failed", error=str(e), sensor_count=len(sensor_data))
        raise

**Performance Monitoring**
    Use built-in performance profiling for optimization.

.. code-block:: python

    obs.start_performance_profile("navigation_update", "autonomy")
    # ... navigation calculations ...
    metrics = obs.end_performance_profile("navigation_update")

    if metrics['duration'] > 0.1:  # 100ms threshold
        obs.warning("Slow navigation update", duration=metrics['duration'])

**Testing**
    Use the unified test suite for all testing needs.

.. code-block:: python

    def test_data_processing():
        test_suite = get_test_suite()
        sensor_data = test_suite.get_test_data('sensor')

        # Test with realistic data
        processed, quality = data_mgr.process_telemetry([sensor_data])
        assert quality.validity > 0.8

Troubleshooting
===============

Common issues and solutions.

High Memory Usage
-----------------

**Symptoms**: System using more memory than expected.

**Solutions**:
- Enable lazy loading in lightweight core
- Reduce log retention in observability system
- Clear data manager cache periodically
- Use circular buffers for streaming data

Slow Startup
------------

**Symptoms**: System taking too long to initialize.

**Solutions**:
- Use lazy loading for heavy components
- Pre-compile schemas in data manager
- Cache configuration files
- Profile and optimize imports

Integration Issues
------------------

**Symptoms**: Systems not communicating properly.

**Solutions**:
- Verify system initialization order
- Check configuration consistency
- Enable debug logging temporarily
- Test individual systems in isolation

.. code-block:: bash

    # Run integration diagnostics
    python -c "
    from src.core.observability import get_observability_manager
    obs = get_observability_manager()
    status = obs.get_system_status()
    print('System Status:', status)
    "




