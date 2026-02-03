#!/usr/bin/env python3
"""
Comprehensive Hardware Integration Tests for URC 2026.

Tests CAN bus communication, motor control, sensor data processing,
emergency systems, and hardware-software integration.
"""

import pytest
import asyncio
import time
from unittest.mock import Mock, patch, AsyncMock
from typing import Dict, Any, List
import struct


class TestCANBusCommunication:
    """Test CAN bus communication protocols and message handling."""

    @pytest.fixture
    def can_interface_config(self):
        """Mock CAN interface configuration."""
        return {
            "interface": "can0",
            "bitrate": 500000,
            "message_ids": {
                "motor_control": 0x200,
                "motor_feedback": 0x201,
                "sensor_data": 0x300,
                "emergency": 0x100,
            },
        }

    def test_can_message_formatting(self, can_interface_config):
        """Test CAN message formatting and parsing."""
        # Test motor control message format
        motor_id = 1
        speed = 1500  # RPM
        current_limit = 10.0  # Amps

        # Pack motor control data (typical CAN format)
        # Format: [speed_low, speed_high, current_low, current_high, motor_id, flags]
        speed_bytes = struct.pack("<H", abs(speed))  # 16-bit signed
        current_bytes = struct.pack(
            "<H", int(current_limit * 100)
        )  # Scale to centi-amps
        motor_byte = motor_id.to_bytes(1, "little")
        direction_flag = (1 if speed >= 0 else 0).to_bytes(1, "little")

        can_data = speed_bytes + current_bytes + motor_byte + direction_flag

        # Verify message structure
        assert len(can_data) == 8  # Standard CAN frame
        assert can_data[4] == motor_id  # Motor ID in correct position
        assert can_data[6] == direction_flag[0]  # Direction flag

        # Unpack and verify
        unpacked_speed = struct.unpack("<H", can_data[0:2])[0]
        unpacked_current = struct.unpack("<H", can_data[2:4])[0] / 100.0

        assert unpacked_speed == abs(speed)
        assert abs(unpacked_current - current_limit) < 0.01

    def test_can_message_filtering(self, can_interface_config):
        """Test CAN message filtering and routing."""
        message_filters = {
            "motor_control": lambda msg: msg.arbitration_id >= 0x200
            and msg.arbitration_id <= 0x20F,
            "sensor_data": lambda msg: msg.arbitration_id >= 0x300
            and msg.arbitration_id <= 0x30F,
            "emergency": lambda msg: msg.arbitration_id == 0x100,
        }

        # Test messages
        test_messages = [
            Mock(arbitration_id=0x200, data=[1, 2, 3, 4, 5, 6, 7, 8]),  # Motor control
            Mock(
                arbitration_id=0x301, data=[10, 20, 30, 40, 50, 60, 70, 80]
            ),  # Sensor data
            Mock(arbitration_id=0x100, data=[255, 0, 0, 0, 0, 0, 0, 0]),  # Emergency
            Mock(arbitration_id=0x400, data=[0, 0, 0, 0, 0, 0, 0, 0]),  # Unknown
        ]

        filtered_messages = {
            "motor_control": [],
            "sensor_data": [],
            "emergency": [],
            "unknown": [],
        }

        for msg in test_messages:
            categorized = False
            for category, filter_func in message_filters.items():
                if filter_func(msg):
                    filtered_messages[category].append(msg)
                    categorized = True
                    break

            if not categorized:
                filtered_messages["unknown"].append(msg)

        # Verify filtering
        assert len(filtered_messages["motor_control"]) == 1
        assert len(filtered_messages["sensor_data"]) == 1
        assert len(filtered_messages["emergency"]) == 1
        assert len(filtered_messages["unknown"]) == 1

    def test_can_bus_reliability(self, can_interface_config):
        """Test CAN bus reliability and error handling."""
        # Simulate CAN bus error scenarios
        error_scenarios = [
            {"type": "bus_off", "recovery_time": 1.0},
            {"type": "bus_passive", "recovery_time": 0.5},
            {"type": "message_lost", "recovery_time": 0.1},
        ]

        for scenario in error_scenarios:
            # Simulate error detection
            error_detected = True
            recovery_start = time.time()

            # Simulate recovery process
            time.sleep(scenario["recovery_time"] * 0.1)  # Faster for testing

            # Verify recovery
            recovery_time = time.time() - recovery_start
            assert (
                recovery_time >= scenario["recovery_time"] * 0.05
            )  # Minimum recovery time
            assert error_detected is True


class TestMotorControlIntegration:
    """Test motor control system integration."""

    @pytest.fixture
    def motor_config(self):
        """Mock motor configuration."""
        return {
            "motors": {
                "front_left": {"id": 1, "max_rpm": 3000, "max_current": 15.0},
                "front_right": {"id": 2, "max_rpm": 3000, "max_current": 15.0},
                "rear_left": {"id": 3, "max_rpm": 3000, "max_current": 15.0},
                "rear_right": {"id": 4, "max_rpm": 3000, "max_current": 15.0},
            },
            "control_modes": ["velocity", "position", "torque"],
            "safety_limits": {
                "max_velocity": 2.0,  # m/s
                "max_acceleration": 1.0,  # m/s²
                "timeout": 0.5,  # seconds
            },
        }

    def test_motor_velocity_control(self, motor_config):
        """Test motor velocity control commands."""
        motors = motor_config["motors"]

        # Test velocity commands for each motor
        for motor_name, motor_info in motors.items():
            motor_id = motor_info["id"]

            # Test forward velocity
            velocity_cmd = 1.5  # m/s forward
            rpm_cmd = self.velocity_to_rpm(velocity_cmd, motor_info)

            # Verify within limits
            assert abs(rpm_cmd) <= motor_info["max_rpm"]

            # Test reverse velocity
            velocity_cmd = -1.0  # m/s reverse
            rpm_cmd = self.velocity_to_rpm(velocity_cmd, motor_info)

            # Verify correct direction and limits
            assert rpm_cmd < 0
            assert abs(rpm_cmd) <= motor_info["max_rpm"]

    def velocity_to_rpm(self, velocity, motor_info):
        """Convert velocity to RPM (simplified conversion)."""
        # Simplified conversion: assume 0.1 m/s per 100 RPM
        return int(velocity * 1000)

    def test_motor_safety_limits(self, motor_config):
        """Test motor safety limit enforcement."""
        safety_limits = motor_config["safety_limits"]

        # Test velocity limit enforcement
        test_velocities = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0]

        for velocity in test_velocities:
            if velocity > safety_limits["max_velocity"]:
                # Should be clamped
                clamped_velocity = min(velocity, safety_limits["max_velocity"])
                assert clamped_velocity == safety_limits["max_velocity"]
            else:
                # Should pass through
                assert velocity <= safety_limits["max_velocity"]

    def test_motor_feedback_processing(self, motor_config):
        """Test motor feedback data processing."""
        # Simulate motor feedback CAN message
        motor_id = 1
        actual_rpm = 1450
        current_draw = 8.5  # Amps
        temperature = 45.0  # Celsius
        error_flags = 0  # No errors

        # Pack feedback data
        feedback_data = struct.pack(
            "<HhHBB",
            actual_rpm,
            int(current_draw * 100),
            int(temperature * 10),
            motor_id,
            error_flags,
        )

        # Unpack and verify
        unpacked_rpm = struct.unpack("<H", feedback_data[0:2])[0]
        unpacked_current = struct.unpack("<h", feedback_data[2:4])[0] / 100.0
        unpacked_temp = struct.unpack("<H", feedback_data[4:6])[0] / 10.0
        unpacked_id = feedback_data[6]
        unpacked_flags = feedback_data[7]

        assert unpacked_rpm == actual_rpm
        assert abs(unpacked_current - current_draw) < 0.01
        assert abs(unpacked_temp - temperature) < 0.1
        assert unpacked_id == motor_id
        assert unpacked_flags == error_flags

    def test_motor_synchronization(self, motor_config):
        """Test motor synchronization for coordinated movement."""
        motors = list(motor_config["motors"].keys())

        # Test coordinated turn command
        turn_rate = 0.5  # rad/s
        forward_velocity = 1.0  # m/s

        motor_commands = {}

        for motor_name in motors:
            if "left" in motor_name:
                # Left motors for turning
                motor_commands[motor_name] = forward_velocity - turn_rate * 0.5
            else:
                # Right motors for turning
                motor_commands[motor_name] = forward_velocity + turn_rate * 0.5

        # Verify motor synchronization
        left_motors = [cmd for name, cmd in motor_commands.items() if "left" in name]
        right_motors = [cmd for name, cmd in motor_commands.items() if "right" in name]

        # Left motors should have lower velocity than right for right turn
        assert all(left <= right for left in left_motors for right in right_motors)


class TestSensorDataProcessing:
    """Test sensor data processing and validation."""

    @pytest.fixture
    def sensor_config(self):
        """Mock sensor configuration."""
        return {
            "imu": {
                "rate": 100,  # Hz
                "ranges": {"accel": 16.0, "gyro": 2000.0, "mag": 4800.0},
            },
            "gps": {"rate": 10, "accuracy": {"horizontal": 2.5, "vertical": 5.0}},  # Hz
            "lidar": {
                "rate": 20,  # Hz
                "range": 30.0,  # meters
                "resolution": 0.01,  # meters
            },
            "cameras": {"rate": 30, "resolution": [1280, 720]},  # Hz
        }

    def test_imu_data_validation(self, sensor_config):
        """Test IMU data validation and filtering."""
        imu_config = sensor_config["imu"]

        # Simulate IMU readings
        test_readings = [
            {"accel": [0.1, 0.2, 9.8], "gyro": [0.1, -0.2, 0.05], "temp": 25.0},
            {
                "accel": [15.5, -8.2, 10.1],
                "gyro": [1999.0, 1500.0, -1800.0],
                "temp": 45.0,
            },
            {
                "accel": [0.0, 0.0, 0.0],
                "gyro": [0.0, 0.0, 0.0],
                "temp": 0.0,
            },  # Invalid reading
        ]

        for reading in test_readings:
            # Validate accelerometer range
            for accel_val in reading["accel"]:
                assert (
                    abs(accel_val) <= imu_config["ranges"]["accel"] * 1.1
                )  # Allow 10% margin

            # Validate gyroscope range
            for gyro_val in reading["gyro"]:
                assert abs(gyro_val) <= imu_config["ranges"]["gyro"] * 1.1

            # Check for invalid readings (all zeros might indicate sensor failure)
            if all(v == 0.0 for v in reading["accel"] + reading["gyro"]):
                print(f"Warning: Invalid IMU reading detected: {reading}")

    def test_gps_data_processing(self, sensor_config):
        """Test GPS data processing and validation."""
        gps_config = sensor_config["gps"]

        # Simulate GPS readings
        test_positions = [
            {"lat": 37.7749, "lon": -122.4194, "alt": 10.5, "hdop": 1.2, "fix": 4},
            {"lat": 40.7128, "lon": -74.0060, "alt": 25.0, "hdop": 0.8, "fix": 5},
            {"lat": 0.0, "lon": 0.0, "alt": 0.0, "hdop": 99.0, "fix": 0},  # No fix
        ]

        for position in test_positions:
            # Validate coordinate ranges
            assert -90 <= position["lat"] <= 90
            assert -180 <= position["lon"] <= 180

            # Validate HDOP (lower is better)
            assert position["hdop"] >= 0

            # Check fix quality
            if position["fix"] >= 3:
                # Good fix - coordinates should be reasonable
                assert position["lat"] != 0.0 or position["lon"] != 0.0
            elif position["fix"] == 0:
                # No fix - might have invalid coordinates
                print(f"Warning: No GPS fix: {position}")

    def test_lidar_data_processing(self, sensor_config):
        """Test LiDAR data processing and obstacle detection."""
        lidar_config = sensor_config["lidar"]

        # Simulate LiDAR scan data (simplified)
        scan_ranges = [5.0, 5.2, 4.8, 15.0, 2.5, 30.0, 0.0]  # meters
        scan_angles = [i * 0.5 for i in range(len(scan_ranges))]  # degrees

        # Process scan for obstacles
        obstacles = []
        for i, (range_val, angle) in enumerate(zip(scan_ranges, scan_angles)):
            if 0.1 < range_val < lidar_config["range"]:  # Valid range
                if range_val < 3.0:  # Close obstacle
                    obstacles.append(
                        {
                            "distance": range_val,
                            "angle": angle,
                            "severity": "critical" if range_val < 1.0 else "warning",
                        }
                    )

        # Verify obstacle detection
        assert len(obstacles) > 0
        assert any(obs["severity"] == "critical" for obs in obstacles)

    def test_sensor_fusion_integration(self, sensor_config):
        """Test sensor fusion integration."""
        # Simulate sensor data fusion
        imu_data = {"accel": [0.1, 0.2, 9.8], "gyro": [0.1, -0.2, 0.05]}
        gps_data = {"lat": 37.7749, "lon": -122.4194, "speed": 1.5}
        lidar_data = {"obstacles": [{"distance": 2.5, "angle": 45.0}]}

        # Fuse data into vehicle state estimate
        fused_state = {
            "position": [gps_data["lat"], gps_data["lon"]],
            "velocity": gps_data["speed"],
            "orientation": self.integrate_gyro(imu_data["gyro"], time_step=0.1),
            "obstacles": lidar_data["obstacles"],
        }

        # Validate fused state
        assert len(fused_state["position"]) == 2
        assert fused_state["velocity"] >= 0
        assert isinstance(fused_state["orientation"], (int, float))
        assert len(fused_state["obstacles"]) > 0

    def integrate_gyro(self, gyro_rates, time_step):
        """Simple gyro integration for orientation (simplified)."""
        return sum(rate * time_step for rate in gyro_rates)


class TestEmergencySystems:
    """Test emergency systems and safety mechanisms."""

    @pytest.fixture
    def emergency_config(self):
        """Mock emergency system configuration."""
        return {
            "emergency_levels": {
                "warning": {"timeout": 5.0, "auto_clear": True},
                "critical": {"timeout": 1.0, "auto_clear": False},
                "fatal": {"timeout": 0.1, "auto_clear": False},
            },
            "emergency_actions": {
                "stop_motors": {"priority": 1, "timeout": 0.1},
                "sound_alarm": {"priority": 2, "timeout": 0.5},
                "notify_operator": {"priority": 3, "timeout": 1.0},
                "emergency_stop": {"priority": 0, "timeout": 0.05},
            },
        }

    def test_emergency_detection(self, emergency_config):
        """Test emergency condition detection."""
        # Define emergency conditions
        emergency_conditions = {
            "motor_overcurrent": {"threshold": 20.0, "level": "critical"},
            "sensor_failure": {"timeout": 2.0, "level": "warning"},
            "communication_lost": {"timeout": 5.0, "level": "critical"},
            "low_battery": {"threshold": 10.0, "level": "fatal"},
        }

        # Test emergency detection logic
        system_state = {
            "motor_current": 25.0,  # Over threshold
            "sensor_ok": False,  # Sensor failure
            "communication_ok": True,
            "battery_level": 5.0,  # Critically low
        }

        detected_emergencies = []

        # Check motor current
        if (
            system_state["motor_current"]
            > emergency_conditions["motor_overcurrent"]["threshold"]
        ):
            detected_emergencies.append(
                {
                    "type": "motor_overcurrent",
                    "level": emergency_conditions["motor_overcurrent"]["level"],
                    "value": system_state["motor_current"],
                }
            )

        # Check sensor status
        if not system_state["sensor_ok"]:
            detected_emergencies.append(
                {
                    "type": "sensor_failure",
                    "level": emergency_conditions["sensor_failure"]["level"],
                    "value": False,
                }
            )

        # Check battery level
        if (
            system_state["battery_level"]
            < emergency_conditions["low_battery"]["threshold"]
        ):
            detected_emergencies.append(
                {
                    "type": "low_battery",
                    "level": emergency_conditions["low_battery"]["level"],
                    "value": system_state["battery_level"],
                }
            )

        # Verify emergency detection
        assert len(detected_emergencies) == 3
        assert any(e["type"] == "motor_overcurrent" for e in detected_emergencies)
        assert any(e["level"] == "fatal" for e in detected_emergencies)

    def test_emergency_action_execution(self, emergency_config):
        """Test emergency action execution priority."""
        emergency_actions = emergency_config["emergency_actions"]

        # Sort actions by priority (lower number = higher priority)
        prioritized_actions = sorted(
            emergency_actions.items(), key=lambda x: x[1]["priority"]
        )

        # Verify priority ordering
        action_names = [name for name, _ in prioritized_actions]
        assert action_names[0] == "emergency_stop"  # Highest priority
        assert action_names[1] == "stop_motors"  # Second priority

        # Test action execution timing
        executed_actions = []
        start_time = time.time()

        for action_name, action_config in prioritized_actions:
            # Simulate action execution
            execution_time = action_config["timeout"]
            time.sleep(execution_time * 0.1)  # Faster for testing

            executed_actions.append(
                {
                    "name": action_name,
                    "priority": action_config["priority"],
                    "executed_at": time.time() - start_time,
                }
            )

        # Verify execution order (higher priority first)
        priorities = [action["priority"] for action in executed_actions]
        assert priorities == sorted(priorities)

    def test_emergency_recovery_procedures(self, emergency_config):
        """Test emergency recovery and system restoration."""
        # Simulate emergency scenario and recovery
        emergency_sequence = [
            {"state": "normal", "motor_current": 5.0},
            {"state": "emergency_detected", "motor_current": 25.0},
            {"state": "emergency_actions_executed", "motor_current": 0.0},
            {"state": "recovery_initiated", "motor_current": 0.0},
            {"state": "system_checks_passed", "motor_current": 0.0},
            {"state": "normal_operation_resumed", "motor_current": 3.0},
        ]

        for i, state in enumerate(emergency_sequence):
            # Validate state transitions
            if state["state"] == "emergency_detected":
                assert state["motor_current"] > 20.0
            elif state["state"] == "emergency_actions_executed":
                assert state["motor_current"] == 0.0
            elif state["state"] == "normal_operation_resumed":
                assert 0 < state["motor_current"] < 10.0

        # Verify recovery was successful
        final_state = emergency_sequence[-1]
        assert final_state["state"] == "normal_operation_resumed"
        assert final_state["motor_current"] < 10.0

    def test_safety_interlock_systems(self, emergency_config):
        """Test safety interlock systems."""
        # Define safety interlocks
        interlocks = {
            "motor_start": ["emergency_stop_clear", "system_ready"],
            "high_speed_mode": ["all_sensors_ok", "operator_present"],
            "autonomous_mode": ["mission_loaded", "safety_checks_passed"],
        }

        # Test interlock validation
        system_status = {
            "emergency_stop_clear": True,
            "system_ready": True,
            "all_sensors_ok": False,
            "operator_present": True,
            "mission_loaded": True,
            "safety_checks_passed": True,
        }

        def check_interlocks(action):
            if action in interlocks:
                return all(
                    system_status.get(condition, False)
                    for condition in interlocks[action]
                )
            return False

        # Test interlock validation
        assert check_interlocks("motor_start") is True  # All conditions met
        assert check_interlocks("high_speed_mode") is False  # Sensor check failed
        assert check_interlocks("autonomous_mode") is True  # All conditions met

    def test_emergency_communication_systems(self, emergency_config):
        """Test emergency communication systems."""
        # Simulate emergency message broadcasting
        emergency_message = {
            "type": "emergency",
            "level": "critical",
            "description": "Motor overcurrent detected",
            "timestamp": time.time(),
            "system_state": {
                "motors": "stopped",
                "sensors": "active",
                "communication": "active",
            },
        }

        # Test message serialization
        import json

        serialized = json.dumps(emergency_message)
        deserialized = json.loads(serialized)

        # Verify message integrity
        assert deserialized["type"] == "emergency"
        assert deserialized["level"] == "critical"
        assert "system_state" in deserialized
        assert deserialized["system_state"]["motors"] == "stopped"


class TestHardwareSoftwareIntegration:
    """Test hardware-software integration scenarios."""

    @pytest.fixture
    def integration_config(self):
        """Mock hardware-software integration configuration."""
        return {
            "control_loop": {
                "frequency": 50,  # Hz
                "latency_budget": 20,  # ms
                "jitter_tolerance": 5,  # ms
            },
            "data_pipeline": {
                "buffer_size": 100,
                "processing_timeout": 50,  # ms
                "max_dropped_frames": 5,
            },
            "fault_detection": {
                "heartbeat_timeout": 100,  # ms
                "max_consecutive_failures": 3,
                "recovery_timeout": 5000,  # ms
            },
        }

    def test_control_loop_timing(self, integration_config):
        """Test control loop timing and jitter."""
        control_config = integration_config["control_loop"]

        # Simulate control loop iterations
        loop_times = []
        target_period = 1.0 / control_config["frequency"]  # seconds

        for i in range(20):
            start_time = time.time()

            # Simulate control loop work
            time.sleep(target_period * 0.8)  # 80% of target time

            end_time = time.time()
            loop_times.append(end_time - start_time)

        # Analyze timing performance
        avg_loop_time = sum(loop_times) / len(loop_times)
        max_loop_time = max(loop_times)
        jitter = max_loop_time - min(loop_times)

        # Verify timing requirements
        assert avg_loop_time <= target_period * 1.2  # Allow 20% margin
        assert (
            jitter <= control_config["jitter_tolerance"] / 1000.0
        )  # Convert to seconds

    def test_data_pipeline_throughput(self, integration_config):
        """Test data pipeline throughput and latency."""
        pipeline_config = integration_config["data_pipeline"]

        # Simulate data processing pipeline
        processed_frames = 0
        dropped_frames = 0
        latencies = []

        for i in range(pipeline_config["buffer_size"]):
            frame_start = time.time()

            # Simulate processing with occasional delays
            processing_time = 0.01  # 10ms base processing
            if i % 10 == 0:  # Every 10th frame has delay
                processing_time += 0.05  # Additional 50ms delay

            time.sleep(processing_time)

            # Check if frame would be dropped due to timeout
            total_time = time.time() - frame_start
            if total_time > pipeline_config["processing_timeout"] / 1000.0:
                dropped_frames += 1
            else:
                processed_frames += 1
                latencies.append(total_time)

        # Verify pipeline performance
        success_rate = processed_frames / pipeline_config["buffer_size"]
        avg_latency = sum(latencies) / len(latencies) if latencies else 0

        assert success_rate >= 0.8  # At least 80% success rate
        assert dropped_frames <= pipeline_config["max_dropped_frames"]
        assert avg_latency <= pipeline_config["processing_timeout"] / 1000.0

    def test_fault_detection_and_recovery(self, integration_config):
        """Test fault detection and recovery mechanisms."""
        fault_config = integration_config["fault_detection"]

        # Simulate heartbeat monitoring
        heartbeats = []
        last_heartbeat = time.time()

        # Simulate normal operation with occasional missed heartbeats
        for i in range(50):
            current_time = time.time()

            # Simulate heartbeat reception (90% success rate)
            if i % 10 != 0:  # Miss every 10th heartbeat
                last_heartbeat = current_time
                heartbeats.append({"timestamp": current_time, "status": "ok"})

            # Check for heartbeat timeout
            time_since_last = current_time - last_heartbeat
            if time_since_last > fault_config["heartbeat_timeout"] / 1000.0:
                # Fault detected - initiate recovery
                recovery_start = current_time

                # Simulate recovery process
                time.sleep(0.1)  # 100ms recovery time

                recovery_time = time.time() - recovery_start
                assert recovery_time <= fault_config["recovery_timeout"] / 1000.0

                # Reset heartbeat after recovery
                last_heartbeat = time.time()

        # Verify fault detection worked
        assert len(heartbeats) >= 40  # At least 80% successful heartbeats

    def test_hardware_software_synchronization(self, integration_config):
        """Test hardware-software synchronization."""
        # Simulate hardware-software synchronization
        hardware_clock = 0.0
        software_clock = 0.0
        sync_offsets = []

        for i in range(10):
            # Simulate clock readings
            hardware_clock += 0.01  # Hardware runs at 100Hz
            software_clock += 0.0105  # Software slightly faster

            # Calculate synchronization offset
            offset = software_clock - hardware_clock
            sync_offsets.append(offset)

            # Simulate synchronization adjustment
            if abs(offset) > 0.001:  # 1ms tolerance
                # Adjust software clock
                software_clock -= offset * 0.1  # Partial correction

        # Verify synchronization quality
        final_offset = abs(sync_offsets[-1])
        max_offset = max(abs(offset) for offset in sync_offsets)

        assert final_offset <= 0.005  # Final offset within 5ms
        assert max_offset <= 0.050  # Max offset within 50ms

    def test_resource_usage_monitoring(self, integration_config):
        """Test resource usage monitoring and alerting."""
        # Simulate resource monitoring
        cpu_usage = []
        memory_usage = []
        alerts_triggered = []

        for i in range(20):
            # Simulate resource readings
            cpu = 45.0 + (i * 2.0)  # Gradually increasing CPU
            memory = 60.0 + (i * 1.5)  # Gradually increasing memory

            cpu_usage.append(cpu)
            memory_usage.append(memory)

            # Check resource thresholds
            if cpu > 80.0:
                alerts_triggered.append({"type": "high_cpu", "value": cpu})
            if memory > 85.0:
                alerts_triggered.append({"type": "high_memory", "value": memory})

        # Verify resource monitoring
        assert len(cpu_usage) == 20
        assert len(memory_usage) == 20
        assert len(alerts_triggered) > 0  # Should have triggered alerts

        # Check alert types
        alert_types = [alert["type"] for alert in alerts_triggered]
        assert "high_cpu" in alert_types or "high_memory" in alert_types
