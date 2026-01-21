==================================
Simulation Testing Guide
==================================

Complete guide to testing rover interfaces before hardware availability using the simulation framework.

.. contents:: Table of Contents
   :depth: 3
   :local:

Introduction
============

Purpose
-------

The simulation framework enables comprehensive testing of all rover communication interfaces before hardware is available. This accelerates development, reduces hardware dependency, and provides a safe environment for testing fault conditions.

What Can Be Tested
------------------

* **Communication Paths**: Frontend → WebSocket → ROS2 → SLCAN → Firmware
* **Protocol Correctness**: Message encoding/decoding accuracy
* **Fault Handling**: Network failures, firmware faults, emergency stops
* **Performance**: Throughput, latency, resource usage
* **Integration**: Component interactions and data flow
* **Safety Features**: Emergency stop timing, fault propagation

What Cannot Be Tested
----------------------

* **Physical Dynamics**: Actual wheel slippage, terrain interaction
* **Real Hardware Timing**: Exact hardware latencies and delays
* **Environmental Factors**: Temperature, vibration, dust effects
* **Mechanical Wear**: Long-term degradation patterns

When to Use Simulation
-----------------------

**Use simulation for:**

* Early development and prototyping
* Interface contract validation
* Fault scenario testing (dangerous on hardware)
* Automated regression testing
* Performance benchmarking
* Team training and onboarding

**Use hardware for:**

* Final validation and calibration
* Performance tuning with real dynamics
* Environmental testing
* Competition preparation
* Operator training with real feedback

Architecture Overview
=====================

System Architecture
-------------------

The simulation framework mirrors the actual rover architecture::

    ┌─────────────────────────┐
    │  Frontend / Operator    │ (React UI)
    └───────────┬─────────────┘
                │ WebSocket
    ┌───────────▼─────────────┐
    │  Teleoperation Server   │ (Socket.IO)
    └───────────┬─────────────┘
                │ Events
    ┌───────────▼─────────────┐
    │  ROS2 Bridge Layer      │ (Twist messages)
    └───────────┬─────────────┘
                │ Topics
    ┌───────────▼─────────────┐
    │  Protocol Adapter       │ (ROS2 → SLCAN)
    └───────────┬─────────────┘
                │ Serial/CAN
    ┌───────────▼─────────────┐
    │  STM32 Firmware         │ (Motor Control)
    └─────────────────────────┘

Simulation Components
---------------------

The framework provides simulators for each layer:

**WebSocket Server Simulator**
   * Location: ``simulation/network/websocket_server_simulator.py``
   * Purpose: Simulates Socket.IO server and frontend connections
   * Features: Network delays, packet loss, connection management

**SLCAN Protocol Simulator**
   * Location: ``simulation/can/slcan_protocol_simulator.py``
   * Purpose: Simulates CAN bus serial protocol encoding/decoding
   * Features: Velocity scaling, message routing, error injection

**STM32 Firmware Simulator**
   * Location: ``simulation/firmware/stm32_firmware_simulator.py``
   * Purpose: Simulates motor control firmware behavior
   * Features: Control loop, encoder feedback, fault injection

**Full Stack Simulator**
   * Location: ``simulation/integration/full_stack_simulator.py``
   * Purpose: Orchestrates complete end-to-end communication
   * Features: Test scenarios, metrics collection, automated testing

Component Communication
-----------------------

Components communicate through well-defined interfaces::

    # Frontend sends drive command
    websocket.emit('driveCommands', {linear: 0.5, angular: 0.2})
    
    # Converted to ROS2 Twist message
    twist.linear.x = 0.5
    twist.angular.z = 0.2
    
    # Encoded to SLCAN frame
    frame = "t00C600080000..." 
    
    # Parsed by firmware
    firmware.set_chassis_velocities(0.5, 0.0, 0.2)

Quick Start Guide
=================

Installation
------------

No additional installation required. The simulation framework is included in the main repository::

    cd /path/to/urc-machiato-2026
    # Simulation modules are in simulation/

Running Your First Test
-----------------------

**1. Simple Component Test**

.. code-block:: python

    from simulation.can.slcan_protocol_simulator import create_slcan_simulator
    
    # Create simulator
    sim = create_slcan_simulator('default')
    
    # Encode velocity command
    frame = sim.encode_velocity_command(0.5, 0.0, 0.2)
    print(f"Encoded: {frame}")
    
    # Decode it back
    cmd = sim.decode_velocity_command(frame)
    print(f"Linear: {cmd.linear_x:.3f} m/s")
    print(f"Angular: {cmd.angular_z:.3f} rad/s")

**2. Full Stack Test**

.. code-block:: python

    from simulation.integration.full_stack_simulator import (
        create_full_stack_simulator, ScenarioType
    )
    import asyncio
    
    async def test():
        # Create simulator
        sim = create_full_stack_simulator('perfect')
        
        # Run test scenario
        result = await sim.run_scenario(ScenarioType.BASIC_VELOCITY)
        
        print(f"Success: {result.success}")
        print(f"Duration: {result.duration_s:.3f}s")
        
        sim.shutdown()
    
    asyncio.run(test())

**3. Running Unit Tests**

.. code-block:: bash

    # Run all simulator unit tests
    pytest tests/unit/test_*_simulator*.py -v
    
    # Run specific simulator tests
    pytest tests/unit/test_slcan_protocol_simulator.py -v
    
    # Run integration tests
    pytest tests/integration/test_complete_communication_stack.py -v

**4. Running Full Test Suite**

.. code-block:: python

    from simulation.integration.full_stack_simulator import create_full_stack_simulator
    import asyncio
    
    async def run_all_tests():
        sim = create_full_stack_simulator('default')
        summary = await sim.run_test_suite()
        
        print(f"Total: {summary['total_scenarios']}")
        print(f"Passed: {summary['passed']}")
        print(f"Failed: {summary['failed']}")
        print(f"Pass Rate: {summary['pass_rate']*100:.1f}%")
        
        sim.shutdown()
    
    asyncio.run(run_all_tests())

Component Reference
===================

WebSocket Server Simulator
---------------------------

**Purpose**: Simulates the teleoperation server for frontend testing.

**Key Features**:

* Socket.IO event handling
* Network condition simulation (latency, packet loss)
* Multi-client support
* Message validation and recording

**Basic Usage**:

.. code-block:: python

    from simulation.network.websocket_server_simulator import (
        WebSocketServerSimulator
    )
    
    sim = WebSocketServerSimulator({
        'network': {
            'latency_ms': 50.0,
            'packet_loss_rate': 0.01
        }
    })
    
    # Register event handler
    async def handle_command(data):
        print(f"Command: {data}")
    
    sim.on('driveCommands', handle_command)
    
    # Simulate client
    client_id = await sim.connect()
    await sim.receive('driveCommands', {'linear': 0.5}, client_id)

**Configuration Options**:

* ``latency_ms``: Network latency (default: 50ms)
* ``jitter_ms``: Latency variation (default: 10ms)
* ``packet_loss_rate``: Packet loss probability (default: 0.01)
* ``validate_messages``: Enable message validation (default: True)
* ``record_messages``: Enable message history (default: True)

**See Also**: ``simulation/network/README.md``

SLCAN Protocol Simulator
-------------------------

**Purpose**: Simulates Serial Line CAN protocol for hardware interface testing.

**Key Features**:

* Complete SLCAN encoding/decoding
* Accurate velocity scaling (×4096 linear, ×64 angular)
* All message IDs (0x00C, 0x00E, 0x110, 0x1FF, etc.)
* Error injection and buffer management

**Basic Usage**:

.. code-block:: python

    from simulation.can.slcan_protocol_simulator import (
        SLCANProtocolSimulator
    )
    
    sim = SLCANProtocolSimulator()
    
    # Encode velocity command
    frame = sim.encode_velocity_command(
        linear_x=0.5,    # m/s
        linear_y=0.0,    # m/s (swerve)
        angular_z=0.2    # rad/s
    )
    
    # Decode back
    cmd = sim.decode_velocity_command(frame)
    print(f"Linear X: {cmd.linear_x:.3f}")

**Message IDs**:

* ``0x00C``: SET_CHASSIS_VELOCITIES (ROS2 → Firmware)
* ``0x00D``: FEEDBACK_CHASSIS_VELOCITIES (Firmware → ROS2)
* ``0x00E``: HEARTBEAT_REQUEST
* ``0x00F``: HEARTBEAT_RESPONSE
* ``0x110``: HOMING_REQUEST
* ``0x111``: HOMING_RESPONSE
* ``0x1FF``: EMERGENCY_STOP

**See Also**: ``simulation/can/README.md``

STM32 Firmware Simulator
-------------------------

**Purpose**: Simulates motor control firmware behavior.

**Key Features**:

* Real-time control loop (100Hz)
* Motor velocity control with ramping
* Encoder feedback simulation
* Thermal and current modeling
* Fault injection (overcurrent, overtemp, encoder failure)
* Emergency stop and homing sequences

**Basic Usage**:

.. code-block:: python

    from simulation.firmware.stm32_firmware_simulator import (
        STM32FirmwareSimulator, MotorFaultType
    )
    
    sim = STM32FirmwareSimulator({'num_motors': 6})
    sim.start()  # Start control loop
    
    # Set motor velocity
    sim.set_velocity_command(motor_id=0, velocity=5.0)
    
    time.sleep(0.5)
    
    # Check status
    status = sim.get_motor_status(0)
    print(f"Actual velocity: {status['velocity_actual']:.2f} rad/s")
    
    # Emergency stop
    sim.emergency_stop()
    
    sim.stop()

**Fault Injection**:

.. code-block:: python

    # Inject overcurrent fault
    sim.inject_fault(motor_id=0, fault_type=MotorFaultType.OVERCURRENT)
    
    # Check motor status
    status = sim.get_motor_status(0)
    assert status['fault'] == 'overcurrent'
    
    # Clear fault
    sim.clear_fault(motor_id=0)

**See Also**: ``simulation/firmware/README.md``

Full Stack Simulator
--------------------

**Purpose**: Orchestrates complete end-to-end communication testing.

**Key Features**:

* Pre-built test scenarios
* Complete pipeline simulation
* Metrics collection across all layers
* Automated test suite execution

**Basic Usage**:

.. code-block:: python

    from simulation.integration.full_stack_simulator import (
        create_full_stack_simulator, ScenarioType
    )
    
    sim = create_full_stack_simulator('default')
    
    # Run scenario
    result = await sim.run_scenario(ScenarioType.BASIC_VELOCITY)
    
    # Run all scenarios
    summary = await sim.run_test_suite()
    
    sim.shutdown()

**Available Scenarios**:

* ``BASIC_VELOCITY``: Basic command propagation
* ``EMERGENCY_STOP``: E-stop timing validation (<100ms)
* ``NETWORK_FAILURE``: High latency and packet loss
* ``FIRMWARE_FAULT``: Fault detection and handling
* ``HIGH_LOAD``: Stress test with high message rate
* ``RECOVERY``: Disconnection and reconnection

**See Also**: ``simulation/integration/README.md``

Writing Custom Tests
====================

Unit Tests
----------

Unit tests verify individual component behavior.

**Test Structure**:

.. code-block:: python

    import pytest
    from simulation.can.slcan_protocol_simulator import SLCANProtocolSimulator
    
    class TestSLCANProtocol:
        @pytest.fixture
        def simulator(self):
            sim = SLCANProtocolSimulator({'simulate_errors': False})
            yield sim
        
        def test_velocity_encoding(self, simulator):
            # Encode
            frame = simulator.encode_velocity_command(0.5, 0.0, 0.2)
            
            # Decode
            cmd = simulator.decode_velocity_command(frame)
            
            # Verify
            assert abs(cmd.linear_x - 0.5) < 0.001
            assert abs(cmd.angular_z - 0.2) < 0.001

**Best Practices**:

* Use fixtures for simulator instances
* Test edge cases (zero, max, negative values)
* Verify error handling
* Check statistics tracking
* Test both success and failure paths

Integration Tests
-----------------

Integration tests verify component interactions.

**Test Structure**:

.. code-block:: python

    import pytest
    import asyncio
    from simulation.integration.full_stack_simulator import (
        create_full_stack_simulator
    )
    
    class TestFullStack:
        @pytest.fixture
        def simulator(self):
            sim = create_full_stack_simulator('perfect')
            yield sim
            sim.shutdown()
        
        async def test_command_propagation(self, simulator):
            # Send command
            client_id = await simulator.websocket_sim.connect()
            await simulator.websocket_sim.receive('driveCommands', {
                'linear': 0.5,
                'angular': 0.2
            }, client_id)
            
            # Wait for propagation
            await asyncio.sleep(0.1)
            
            # Verify at each layer
            assert simulator.ros2_state['cmd_vel_teleop'] is not None
            motor_status = simulator.firmware_sim.get_motor_status(0)
            assert motor_status['velocity_actual'] > 0

**Best Practices**:

* Use async/await for async operations
* Add appropriate delays for propagation
* Verify state at each layer
* Test failure scenarios
* Clean up resources in fixtures

Performance Tests
-----------------

Performance tests validate timing and throughput.

**Test Structure**:

.. code-block:: python

    import time
    
    def test_encoding_performance():
        sim = SLCANProtocolSimulator()
        
        start = time.time()
        count = 10000
        
        for i in range(count):
            sim.encode_velocity_command(0.5, 0.0, 0.1)
        
        duration = time.time() - start
        rate = count / duration
        
        print(f"Encoding rate: {rate:.0f} frames/sec")
        assert rate > 10000  # Should be fast

**Best Practices**:

* Run sufficient iterations for accuracy
* Use time.perf_counter() for precision
* Test under different loads
* Establish baseline metrics
* Monitor resource usage

Custom Scenarios
----------------

Create custom test scenarios for specific use cases.

**Scenario Structure**:

.. code-block:: python

    async def custom_recovery_scenario(sim, params):
        """Test custom recovery logic."""
        success = True
        errors = []
        
        try:
            # 1. Establish baseline
            client_id = await sim.websocket_sim.connect()
            
            # 2. Inject fault
            sim.firmware_sim.inject_fault(0, MotorFaultType.ENCODER_FAILURE)
            
            # 3. Verify fault detected
            status = sim.firmware_sim.get_motor_status(0)
            if status['fault'] != 'encoder_failure':
                errors.append("Fault not detected")
                success = False
            
            # 4. Attempt recovery
            sim.firmware_sim.clear_fault(0)
            
            # 5. Verify recovery
            await asyncio.sleep(0.5)
            status = sim.firmware_sim.get_motor_status(0)
            if status['fault'] != 'none':
                errors.append("Recovery failed")
                success = False
            
        except Exception as e:
            errors.append(str(e))
            success = False
        
        return success, errors
    
    # Run custom scenario
    success, errors = await custom_recovery_scenario(sim, {})
    print(f"Success: {success}")
    if errors:
        print(f"Errors: {errors}")

Troubleshooting
===============

Common Issues
-------------

**Issue**: Tests fail with "connection refused" or timeout

**Cause**: Simulator components not started or configured incorrectly

**Solution**:

.. code-block:: python

    # Verify firmware started
    assert sim.firmware_sim.running == True
    
    # Check no emergency stop
    assert sim.firmware_sim.emergency_stop_active == False
    
    # Verify client connected
    assert client_id in sim.websocket_sim.connected_clients

**Issue**: Velocity encoding/decoding inaccurate

**Cause**: Quantization error from integer scaling

**Solution**:

.. code-block:: python

    # Allow tolerance for quantization
    tolerance = 1.0 / 4096  # ~0.00024 m/s
    assert abs(decoded.linear_x - original) < tolerance

**Issue**: Tests pass individually but fail when run together

**Cause**: Shared state between tests

**Solution**:

.. code-block:: python

    # Reset simulator between tests
    @pytest.fixture
    def simulator(self):
        sim = create_full_stack_simulator('default')
        yield sim
        sim.reset()  # Clear state
        sim.shutdown()  # Clean up

**Issue**: Performance tests are flaky

**Cause**: System load variability

**Solution**:

.. code-block:: python

    # Run multiple iterations and use statistics
    durations = []
    for _ in range(10):
        start = time.time()
        # ... test code ...
        durations.append(time.time() - start)
    
    avg_duration = sum(durations) / len(durations)
    # Use average instead of single measurement

Debugging Techniques
--------------------

**Enable Verbose Logging**:

.. code-block:: python

    import logging
    logging.basicConfig(level=logging.DEBUG)

**Check Component Statistics**:

.. code-block:: python

    # WebSocket stats
    print(sim.websocket_sim.get_statistics())
    
    # SLCAN stats
    print(sim.slcan_sim.get_statistics())
    
    # Firmware status
    print(sim.firmware_sim.get_system_status())

**Inspect Event Queue**:

.. code-block:: python

    # View recent events
    for event in sim.event_queue[-10:]:
        print(f"{event['timestamp']:.3f}: {event['type']}")

**Export Message History**:

.. code-block:: python

    # Export for analysis
    sim.websocket_sim.export_history('output/messages.json')

Performance Considerations
==========================

Optimization Tips
-----------------

**Use Perfect Profile for Speed**:

.. code-block:: python

    # No delays or error injection
    sim = create_full_stack_simulator('perfect')

**Disable Unnecessary Features**:

.. code-block:: python

    config = {
        'websocket': {
            'validate_messages': False,  # Skip validation
            'record_messages': False,    # Skip recording
            'network': {'enabled': False}  # No delays
        }
    }

**Reduce Control Loop Frequency**:

.. code-block:: python

    config = {
        'firmware': {
            'control_loop_hz': 50  # Lower frequency
        }
    }

**Limit History Size**:

.. code-block:: python

    config = {
        'websocket': {'max_history': 100},
        'slcan': {'max_history': 100}
    }

Performance Targets
-------------------

**Encoding/Decoding**: >10,000 frames/second
**Message Throughput**: >500 messages/second
**E2E Latency (perfect mode)**: <50ms
**Test Suite Duration**: <10 seconds

Resource Usage
--------------

Typical resource usage:

* **CPU**: 10-30% (single core, with control loop)
* **Memory**: 50-100 MB (with message history)
* **Disk**: Minimal (<1 MB for logs)

Hardware Validation Workflow
=============================

Simulation to Hardware Transition
----------------------------------

**Phase 1: Pure Simulation** (Weeks 1-2)

1. Develop interfaces in simulation
2. Validate protocol encoding/decoding
3. Test fault scenarios
4. Establish performance baselines

**Phase 2: Component Validation** (Weeks 3-4)

1. Test SLCAN protocol with real CAN adapter
2. Verify message timing with oscilloscope
3. Compare simulation vs. hardware latencies
4. Calibrate simulation parameters

**Phase 3: Subsystem Integration** (Weeks 5-6)

1. Test firmware on actual STM32
2. Verify motor control loop behavior
3. Test emergency stop timing
4. Validate encoder feedback

**Phase 4: Full System** (Weeks 7-8)

1. Complete end-to-end testing
2. Performance tuning
3. Operator training
4. Competition preparation

Validation Checklist
---------------------

Before Hardware Testing:

□ All unit tests passing (>80% coverage)
□ Integration tests passing
□ Performance baselines established
□ Protocol documentation complete
□ Test scenarios documented

During Hardware Testing:

□ SLCAN frame format validated
□ Velocity scaling verified
□ E-stop timing measured (<100ms)
□ Motor response characterized
□ Encoder accuracy checked

After Hardware Testing:

□ Simulation parameters calibrated
□ Discrepancies documented
□ Performance comparison report
□ Lessons learned captured

Comparison Scripts
------------------

Compare simulation vs. hardware results:

.. code-block:: python

    # Run in simulation
    sim_result = await sim.run_scenario(ScenarioType.BASIC_VELOCITY)
    
    # Run on hardware (use same test)
    hw_result = await hardware_test.run_scenario(ScenarioType.BASIC_VELOCITY)
    
    # Compare
    print(f"Sim duration: {sim_result.duration_s:.3f}s")
    print(f"HW duration: {hw_result.duration_s:.3f}s")
    print(f"Difference: {abs(sim_result.duration_s - hw_result.duration_s):.3f}s")

See ``scripts/compare_sim_vs_hardware.py`` for complete comparison framework.

Appendix
========

File Locations
--------------

**Simulation Components**:

* WebSocket: ``simulation/network/websocket_server_simulator.py``
* SLCAN: ``simulation/can/slcan_protocol_simulator.py``
* Firmware: ``simulation/firmware/stm32_firmware_simulator.py``
* Full Stack: ``simulation/integration/full_stack_simulator.py``

**Tests**:

* Unit Tests: ``tests/unit/test_*_simulator*.py``
* Integration Tests: ``tests/integration/test_complete_communication_stack.py``
* Performance Tests: ``tests/performance/test_simulation_*.py``

**Documentation**:

* Component READMEs: ``simulation/*/README.md``
* This Guide: ``docs/simulation_testing_guide.rst``
* Examples: ``simulation/examples/*.py``

**Configuration**:

* Profiles: ``simulation/config/*.yaml``
* Test Configs: ``tests/fixtures/*.yaml``

References
----------

* `ROS2 Testing Guide <https://docs.ros.org/en/humble/Tutorials/Testing.html>`_
* `pytest Documentation <https://docs.pytest.org/>`_
* `SLCAN Protocol Specification <http://www.fischl.de/usbtin/>`_
* `Socket.IO Documentation <https://socket.io/docs/>`_

Glossary
--------

**SLCAN**
   Serial Line CAN - Protocol for sending CAN messages over serial connection

**Twist**
   ROS2 message type for velocity commands (linear and angular)

**E2E**
   End-to-end, referring to complete communication path

**HIL**
   Hardware-in-the-loop testing with mixed simulation and real components

**Scenario**
   Pre-defined test case with specific conditions and expected outcomes

Contact
-------

For questions about the simulation framework:

* Check component READMEs in ``simulation/*/README.md``
* Review test examples in ``tests/``
* Ask team members during standup
* Create issue on project repository

Contributing
------------

When adding new simulation features:

1. Write unit tests first (TDD)
2. Update component README
3. Add usage example to ``simulation/examples/``
4. Update this guide if adding major features
5. Ensure all tests pass before committing

---

*Last Updated: 2026-01-20*
*Version: 1.0*
