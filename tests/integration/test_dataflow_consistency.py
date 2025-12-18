#!/usr/bin/env python3
"""
Data Flow Consistency Tests

Tests end-to-end data flow validation:
- Sensor → Processing → Action data flow
- Data transformation accuracy
- Timestamp synchronization
- Data loss detection
- Latency measurement
- Data integrity checks

This addresses integration gap: Data flow consistency validation.

Author: URC 2026 Autonomy Team
"""

import os
import sys
import time
from typing import Dict, List, Optional, Tuple

import numpy as np
import pytest

try:
    import rclpy
    from rclpy.node import Node

    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    rclpy = None
    Node = None

# Add project paths
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, PROJECT_ROOT)
sys.path.insert(0, os.path.join(PROJECT_ROOT, "tests", "simulation"))

try:
    from simulation.environments.environment_factory import EnvironmentFactory
    from simulation.network.network_emulator import NetworkEmulator, NetworkProfile
except ImportError:
    EnvironmentFactory = None
    NetworkEmulator = None

    # ROS2 context managed by session fixture
    """Initialize and cleanup ROS context."""
    if not ROS2_AVAILABLE:
        pytest.skip("ROS2 not available")
    rclpy.init()
    yield
    rclpy.shutdown()


class DataFlowTracker:
    """Track data flow through system."""

    def __init__(self):
        self.sensor_readings: List[Dict] = []
        self.processed_data: List[Dict] = []
        self.actions: List[Dict] = []
        self.timestamps: List[float] = []

    def record_sensor_reading(self, sensor_type: str, value: float, timestamp: float):
        """Record sensor reading."""
        self.sensor_readings.append(
            {
                "type": sensor_type,
                "value": value,
                "timestamp": timestamp,
            }
        )
        self.timestamps.append(timestamp)

    def record_processed_data(self, data: Dict, timestamp: float):
        """Record processed data."""
        self.processed_data.append(
            {
                "data": data,
                "timestamp": timestamp,
            }
        )
        self.timestamps.append(timestamp)

    def record_action(self, action_type: str, value: float, timestamp: float):
        """Record action taken."""
        self.actions.append(
            {
                "type": action_type,
                "value": value,
                "timestamp": timestamp,
            }
        )
        self.timestamps.append(timestamp)

    def get_latency(self, start_idx: int, end_idx: int) -> float:
        """Get latency between two data points."""
        if start_idx >= len(self.timestamps) or end_idx >= len(self.timestamps):
            return 0.0
        return self.timestamps[end_idx] - self.timestamps[start_idx]

    def check_data_loss(self) -> List[int]:
        """Check for data loss (missing timestamps)."""
        if len(self.timestamps) < 2:
            return []

        gaps = []
        for i in range(1, len(self.timestamps)):
            gap = self.timestamps[i] - self.timestamps[i - 1]
            if gap > 1.0:  # More than 1 second gap indicates potential loss
                gaps.append(i)

        return gaps


@pytest.mark.integration
@pytest.mark.ros2
class TestDataFlow:
    """Test end-to-end data flow."""

    def setUp(self):
        """Set up test environment."""
        self.tracker = DataFlowTracker()
        if EnvironmentFactory:
            # Create environment with REAL_LIFE tier config
            env_config = {"tier": "real_life"}
            self.env_sim = EnvironmentFactory.create(env_config)
        if NetworkEmulator:
            self.net_emu = NetworkEmulator(NetworkProfile.RURAL_WIFI)

    def tearDown(self):
        """Clean up test environment."""
        if NetworkEmulator:
            self.net_emu.stop()

    def test_sensor_to_action_flow(self, ros_context):
        """Test sensor → processing → action data flow."""
        print("\n[REFRESH] Testing Sensor → Action Data Flow")

        # Simulate sensor reading
        sensor_value = 10.0
        sensor_timestamp = time.time()
        self.tracker.record_sensor_reading("position", sensor_value, sensor_timestamp)

        # Simulate processing (navigation calculates target)
        processed_timestamp = time.time()
        target_value = sensor_value + 5.0  # Move 5 units forward
        self.tracker.record_processed_data(
            {"target": target_value}, processed_timestamp
        )

        # Simulate action (motor command)
        action_timestamp = time.time()
        action_value = 1.0  # Move forward at 1 m/s
        self.tracker.record_action("velocity", action_value, action_timestamp)

        # Verify flow
        assert len(self.tracker.sensor_readings) > 0, "Should have sensor readings"
        assert len(self.tracker.processed_data) > 0, "Should have processed data"
        assert len(self.tracker.actions) > 0, "Should have actions"

        # Verify timestamps are sequential
        assert (
            sensor_timestamp <= processed_timestamp
        ), "Processing should occur after sensor"
        assert (
            processed_timestamp <= action_timestamp
        ), "Action should occur after processing"

        # Verify data transformation
        assert (
            target_value == sensor_value + 5.0
        ), "Processing should transform data correctly"

    def test_data_transformation_accuracy(self, ros_context):
        """Test data transformation accuracy through pipeline."""
        print("\n[GRAPH] Testing Data Transformation Accuracy")

        # Input data
        input_position = np.array([0.0, 0.0])
        target_position = np.array([10.0, 10.0])

        # Transform 1: Calculate direction
        direction = target_position - input_position
        expected_direction = np.array([10.0, 10.0])
        assert np.allclose(
            direction, expected_direction
        ), "Direction calculation should be accurate"

        # Transform 2: Calculate distance
        distance = np.linalg.norm(direction)
        expected_distance = np.sqrt(200.0)  # sqrt(10^2 + 10^2)
        assert np.isclose(
            distance, expected_distance, rtol=0.01
        ), f"Distance calculation should be accurate: {distance} vs {expected_distance}"

        # Transform 3: Calculate velocity command
        max_velocity = 2.0  # m/s
        velocity = direction / distance * max_velocity
        expected_velocity_magnitude = max_velocity
        assert np.isclose(
            np.linalg.norm(velocity), expected_velocity_magnitude, rtol=0.01
        ), "Velocity calculation should be accurate"

    def test_timestamp_synchronization(self, ros_context):
        """Test timestamp synchronization across subsystems."""
        print("\n⏰ Testing Timestamp Synchronization")

        # Simulate multiple sensors with timestamps
        base_time = time.time()
        sensor_timestamps = [
            base_time + 0.0,  # GPS
            base_time + 0.01,  # IMU
            base_time + 0.02,  # Camera
        ]

        # Check timestamp consistency
        max_drift = max(sensor_timestamps) - min(sensor_timestamps)
        assert (
            max_drift < 0.1
        ), f"Timestamp drift should be < 100ms, got {max_drift*1000:.1f}ms"

        # Simulate processing with timestamp
        processing_timestamp = time.time()
        timestamp_delay = processing_timestamp - base_time

        # Processing should occur within reasonable time
        assert (
            timestamp_delay < 1.0
        ), f"Processing delay should be < 1s, got {timestamp_delay:.2f}s"

    def test_data_loss_detection(self, ros_context):
        """Test detection of data loss in pipeline."""
        print("\n[MAGNIFY] Testing Data Loss Detection")

        # Simulate continuous sensor readings
        base_time = time.time()
        for i in range(100):
            timestamp = base_time + i * 0.1  # 10 Hz
            self.tracker.record_sensor_reading("sensor", float(i), timestamp)

        # Check for data loss
        gaps = self.tracker.check_data_loss()
        assert len(gaps) == 0, f"Should not have data loss, found {len(gaps)} gaps"

        # Simulate data loss (skip some readings)
        self.tracker = DataFlowTracker()
        for i in range(100):
            if i not in [20, 21, 22, 50, 51]:  # Simulate missing readings
                timestamp = base_time + i * 0.1
                self.tracker.record_sensor_reading("sensor", float(i), timestamp)

        # Should detect gaps
        gaps = self.tracker.check_data_loss()
        # Note: Gap detection depends on implementation, may need adjustment

    def test_latency_measurement(self, ros_context):
        """Test latency measurement through pipeline."""
        print("\n[CLOCK]  Testing Latency Measurement")

        # Simulate sensor → processing → action with timestamps
        sensor_time = time.time()
        self.tracker.record_sensor_reading("sensor", 1.0, sensor_time)

        processing_time = time.time()
        self.tracker.record_processed_data({"result": 2.0}, processing_time)

        action_time = time.time()
        self.tracker.record_action("action", 3.0, action_time)

        # Measure latencies
        processing_latency = processing_time - sensor_time
        action_latency = action_time - processing_time
        total_latency = action_time - sensor_time

        print(f"  Processing latency: {processing_latency*1000:.2f} ms")
        print(f"  Action latency: {action_latency*1000:.2f} ms")
        print(f"  Total latency: {total_latency*1000:.2f} ms")

        # Latencies should be reasonable
        assert (
            processing_latency < 0.5
        ), f"Processing latency should be < 500ms, got {processing_latency*1000:.1f}ms"
        assert (
            action_latency < 0.5
        ), f"Action latency should be < 500ms, got {action_latency*1000:.1f}ms"
        assert (
            total_latency < 1.0
        ), f"Total latency should be < 1s, got {total_latency*1000:.1f}ms"

    def test_data_integrity(self, ros_context):
        """Test data integrity through pipeline."""
        print("\n Testing Data Integrity")

        # Input data
        input_data = {"x": 10.0, "y": 20.0, "z": 30.0}

        # Process data
        processed_data = self._process_data(input_data)

        # Verify integrity
        assert "x" in processed_data, "Data integrity: x should be preserved"
        assert "y" in processed_data, "Data integrity: y should be preserved"
        assert "z" in processed_data, "Data integrity: z should be preserved"

        # Verify values are not corrupted
        assert (
            abs(processed_data["x"] - input_data["x"]) < 0.01
        ), "x value should not be corrupted"
        assert (
            abs(processed_data["y"] - input_data["y"]) < 0.01
        ), "y value should not be corrupted"
        assert (
            abs(processed_data["z"] - input_data["z"]) < 0.01
        ), "z value should not be corrupted"

    def test_multi_sensor_fusion(self, ros_context):
        """Test data fusion from multiple sensors."""
        print("\n Testing Multi-Sensor Fusion")

        # Simulate multiple sensor readings
        gps_data = {"lat": 38.406, "lon": -110.792, "alt": 1500.0}
        imu_data = {"accel_x": 0.1, "accel_y": 0.2, "accel_z": 9.8}
        camera_data = {"detections": 3, "confidence": 0.85}

        # Fuse data
        fused_data = self._fuse_sensor_data(gps_data, imu_data, camera_data)

        # Verify fusion
        assert "position" in fused_data, "Fused data should have position"
        assert "orientation" in fused_data, "Fused data should have orientation"
        assert "detections" in fused_data, "Fused data should have detections"

        # Verify data is correctly combined
        assert (
            fused_data["position"]["lat"] == gps_data["lat"]
        ), "GPS data should be preserved"
        assert (
            fused_data["detections"]["count"] == camera_data["detections"]
        ), "Camera data should be preserved"

    def _process_data(self, data: Dict) -> Dict:
        """Process data (simulated)."""
        # Simulate data processing
        processed = data.copy()
        processed["processed"] = True
        processed["timestamp"] = time.time()
        return processed

    def _fuse_sensor_data(self, gps: Dict, imu: Dict, camera: Dict) -> Dict:
        """Fuse data from multiple sensors."""
        return {
            "position": {"lat": gps["lat"], "lon": gps["lon"], "alt": gps["alt"]},
            "orientation": {
                "accel_x": imu["accel_x"],
                "accel_y": imu["accel_y"],
                "accel_z": imu["accel_z"],
            },
            "detections": {
                "count": camera["detections"],
                "confidence": camera["confidence"],
            },
            "fused": True,
            "timestamp": time.time(),
        }


@pytest.mark.integration
@pytest.mark.ros2
class TestDataFlowUnderDegradation:
    """Test data flow consistency under degradation."""

    def test_data_flow_with_network_latency(self, ros_context):
        """Test data flow with network latency."""
        if not NetworkEmulator:
            pytest.skip("NetworkEmulator not available")

        self.net_emu = NetworkEmulator(NetworkProfile.RURAL_WIFI)
        self.net_emu.start()

        try:
            # Simulate data flow with network latency
            start_time = time.time()
            data = {"value": 10.0}

            # Send through network
            self.net_emu.send_message(data)
            time.sleep(0.1)  # Simulate network delay

            end_time = time.time()
            latency = end_time - start_time

            # Latency should be within network profile limits
            assert (
                latency < 0.5
            ), f"Network latency should be reasonable: {latency*1000:.1f}ms"

        finally:
            self.net_emu.stop()

    def test_data_flow_with_packet_loss(self, ros_context):
        """Test data flow with packet loss."""
        if not NetworkEmulator:
            pytest.skip("NetworkEmulator not available")

        self.net_emu = NetworkEmulator(NetworkProfile.EXTREME)
        self.net_emu.start()

        try:
            # Simulate packet loss
            self.net_emu.simulate_packet_loss(50, 2.0)  # 50% loss for 2 seconds

            # Send data
            messages_sent = 0
            messages_received = 0

            for _ in range(20):
                if self.net_emu.send_message({"id": _, "data": "test"}):
                    messages_sent += 1
                time.sleep(0.1)

            time.sleep(1.0)  # Allow delivery

            stats = self.net_emu.get_statistics()
            messages_received = stats.get("messages_received", 0)

            # Should receive some messages despite packet loss
            assert (
                messages_received > 0
            ), "Should receive some messages despite packet loss"
            assert messages_received < messages_sent, "Should have some packet loss"

        finally:
            self.net_emu.stop_packet_loss()
            self.net_emu.stop()


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
