#!/usr/bin/env python3
"""
Integration Test: Binary Protocol + Timestamp Provider

Tests the integration between binary sensor protocol and timestamp provider
to ensure accurate sensor data timing and efficient serialization.
"""

import time
import pytest

# Try to import, skip test if module not available
try:
    from src.infrastructure.monitoring.binary_sensor_protocol import (
        BinarySensorProtocol,
        IMUData,
        SensorMessageType,
    )

    HAS_BINARY_PROTOCOL = True
except ImportError:
    HAS_BINARY_PROTOCOL = False

try:
    from src.sensors.timestamp_provider import SensorTimestampProvider, SensorType

    HAS_TIMESTAMP_PROVIDER = True
except ImportError:
    HAS_TIMESTAMP_PROVIDER = False

try:
    from src.testing.performance_profiling import PerformanceProfiler

    HAS_PERFORMANCE_PROFILER = True
except ImportError:
    HAS_PERFORMANCE_PROFILER = False

pytestmark = pytest.mark.skipif(
    not (HAS_BINARY_PROTOCOL and HAS_TIMESTAMP_PROVIDER and HAS_PERFORMANCE_PROFILER),
    reason="Binary protocol, timestamp provider, or performance profiler not available",
)


class TestBinaryProtocolTimestampIntegration:
    """Test binary protocol and timestamp provider integration."""

    def setup_method(self):
        """Setup test fixtures."""
        self.timestamp_provider = SensorTimestampProvider()
        self.performance_profiler = PerformanceProfiler()

    def teardown_method(self):
        """Cleanup test fixtures."""
        pass

    def test_timestamp_provider_basic_functionality(self):
        """Test basic timestamp provider functionality."""
        # Create raw sensor data
        raw_imu_data = {
            "accel_x": 0.1,
            "accel_y": 0.2,
            "accel_z": 9.8,
            "gyro_x": 0.01,
            "gyro_y": 0.02,
            "gyro_z": 0.03,
        }

        # Tag with timestamps
        timestamped_data = self.timestamp_provider.tag_sensor_data(
            SensorType.IMU, raw_imu_data
        )

        # Verify timestamps exist and are reasonable
        assert timestamped_data.measurement_timestamp_ns > 0
        assert timestamped_data.reception_timestamp_ns > 0
        assert timestamped_data.latency_ms >= 0
        assert timestamped_data.quality_score > 0
        assert timestamped_data.sequence_number >= 0

        # Verify original data preserved
        assert timestamped_data.data["accel_x"] == 0.1
        assert timestamped_data.data["accel_z"] == 9.8

    def test_binary_protocol_imu_roundtrip(self):
        """Test binary protocol IMU data encoding/decoding roundtrip."""
        # Create IMU data with timestamps
        imu_data = IMUData(
            measurement_timestamp_ns=1234567890000000000,
            reception_timestamp_ns=1234567890100000000,
            accel_x=0.1,
            accel_y=0.2,
            accel_z=9.8,
            gyro_x=0.01,
            gyro_y=0.02,
            gyro_z=0.03,
            orientation_x=0.1,
            orientation_y=0.2,
            orientation_z=0.3,
            orientation_w=0.9,
        )

        # Encode to binary
        binary_data = BinarySensorProtocol.encode_imu(imu_data, sequence_number=42)

        # Verify message structure
        assert (
            len(binary_data)
            == BinarySensorProtocol.HEADER_SIZE + BinarySensorProtocol.IMU_SIZE
        )

        # Verify message type and sequence
        assert (
            BinarySensorProtocol.get_message_type(binary_data) == SensorMessageType.IMU
        )
        assert BinarySensorProtocol.get_sequence_number(binary_data) == 42

        # Decode back
        decoded_imu = BinarySensorProtocol.decode_imu(binary_data)

        # Verify data integrity
        assert decoded_imu is not None
        assert decoded_imu.measurement_timestamp_ns == imu_data.measurement_timestamp_ns
        assert decoded_imu.reception_timestamp_ns == imu_data.reception_timestamp_ns
        assert abs(decoded_imu.accel_x - imu_data.accel_x) < 1e-6
        assert abs(decoded_imu.gyro_z - imu_data.gyro_z) < 1e-6
        assert abs(decoded_imu.orientation_w - imu_data.orientation_w) < 1e-6

    def test_end_to_end_sensor_pipeline(self):
        """Test complete sensor data pipeline: raw → timestamped → binary → decoded."""
        # Step 1: Raw sensor data arrives
        raw_sensor_data = {
            "accel_x": 1.5,
            "accel_y": -0.3,
            "accel_z": 9.7,
            "gyro_x": 0.02,
            "gyro_y": -0.01,
            "gyro_z": 0.005,
        }

        # Step 2: Add accurate timestamps
        timestamped_data = self.timestamp_provider.tag_sensor_data(
            SensorType.IMU, raw_sensor_data
        )

        # Step 3: Convert to binary IMU data structure
        imu_data = IMUData(
            measurement_timestamp_ns=timestamped_data.measurement_timestamp_ns,
            reception_timestamp_ns=timestamped_data.reception_timestamp_ns,
            accel_x=raw_sensor_data["accel_x"],
            accel_y=raw_sensor_data["accel_y"],
            accel_z=raw_sensor_data["accel_z"],
            gyro_x=raw_sensor_data["gyro_x"],
            gyro_y=raw_sensor_data["gyro_y"],
            gyro_z=raw_sensor_data["gyro_z"],
        )

        # Step 4: Encode to binary protocol
        binary_message = BinarySensorProtocol.encode_imu(
            imu_data, timestamped_data.sequence_number
        )

        # Step 5: Decode (simulating receiver)
        decoded_imu = BinarySensorProtocol.decode_imu(binary_message)

        # Step 6: Verify end-to-end integrity
        assert decoded_imu is not None
        assert decoded_imu.measurement_timestamp_ns == imu_data.measurement_timestamp_ns
        assert abs(decoded_imu.accel_x - 1.5) < 1e-6
        assert abs(decoded_imu.gyro_y - (-0.01)) < 1e-6

        # Verify sequence number preserved
        assert (
            BinarySensorProtocol.get_sequence_number(binary_message)
            == timestamped_data.sequence_number
        )

    def test_performance_binary_vs_timestamped_pipeline(self):
        """Compare performance of binary protocol pipeline with timestamps."""
        iterations = 1000

        # Test data
        raw_data = {"accel_x": 0.1, "accel_y": 0.2, "accel_z": 9.8}

        # Measure full pipeline performance
        start_time = time.perf_counter()

        for i in range(iterations):
            # Timestamp
            timestamped = self.timestamp_provider.tag_sensor_data(
                SensorType.IMU, raw_data
            )

            # Convert to IMU data
            imu_data = IMUData(
                measurement_timestamp_ns=timestamped.measurement_timestamp_ns,
                reception_timestamp_ns=timestamped.reception_timestamp_ns,
                accel_x=raw_data["accel_x"],
                accel_y=raw_data["accel_y"],
                accel_z=raw_data["accel_z"],
                gyro_x=0.01,
                gyro_y=0.02,
                gyro_z=0.03,
            )

            # Encode/decode
            binary_msg = BinarySensorProtocol.encode_imu(
                imu_data, timestamped.sequence_number
            )
            decoded = BinarySensorProtocol.decode_imu(binary_msg)

            assert decoded is not None

        end_time = time.perf_counter()
        total_time = end_time - start_time

        # Calculate per-iteration metrics
        time_per_iteration_ms = (total_time / iterations) * 1000
        throughput_hz = iterations / total_time

        print(f"\nBinary Protocol + Timestamp Pipeline Performance:")
        print(".3f")
        print(".0f")
        print(".1f")

        # Assert reasonable performance (< 1ms per iteration for real-time use)
        assert (
            time_per_iteration_ms < 1.0
        ), f"Pipeline too slow: {time_per_iteration_ms:.3f}ms/iteration"

    def test_timestamp_provider_health_monitoring(self):
        """Test timestamp provider health monitoring capabilities."""
        # Simulate good sensor data
        for i in range(10):
            raw_data = {"accel_x": 0.1, "accel_y": 0.2, "accel_z": 9.8}
            timestamped = self.timestamp_provider.tag_sensor_data(
                SensorType.IMU, raw_data
            )
            time.sleep(0.001)  # Small delay between readings

        # Check health status
        health = self.timestamp_provider.get_sensor_health(SensorType.IMU)

        assert health["health_status"] == "good"
        assert health["quality_score"] > 0.8
        assert health["avg_latency_ms"] >= 0
        assert health["time_since_update_ms"] < 1000  # Less than 1 second ago

    def test_timestamp_provider_stale_sensor_detection(self):
        """Test detection of stale sensor data."""
        # Initially healthy
        raw_data = {"accel_x": 0.1, "accel_y": 0.2, "accel_z": 9.8}
        self.timestamp_provider.tag_sensor_data(SensorType.IMU, raw_data)

        # Should not be stale immediately
        stale_sensors = self.timestamp_provider.detect_stale_sensors(max_age_ms=100)
        assert SensorType.IMU not in stale_sensors

        # After 200ms, should be considered stale
        time.sleep(0.2)
        stale_sensors = self.timestamp_provider.detect_stale_sensors(max_age_ms=100)
        assert SensorType.IMU in stale_sensors

    def test_timestamp_synchronization_validation(self):
        """Test validation of synchronized sensor timestamps."""
        # Create multiple sensor readings with slight time differences
        sensor_readings = []

        for i in range(5):
            raw_data = {"accel_x": 0.1, "accel_y": 0.2, "accel_z": 9.8}
            timestamped = self.timestamp_provider.tag_sensor_data(
                SensorType.IMU, raw_data
            )
            sensor_readings.append(timestamped)
            time.sleep(0.001)  # 1ms between readings

        # Validate synchronization (should be within reasonable bounds)
        sync_result = self.timestamp_provider.validate_timestamp_synchronization(
            sensor_readings, max_time_diff_ms=10.0
        )

        assert sync_result["synchronized"] == True
        assert sync_result["max_diff_ms"] < 10.0

    def test_sensor_latency_calibration(self):
        """Test sensor latency calibration functionality."""
        # Initially use default latency
        initial_latency = self.timestamp_provider._latency_calibration[SensorType.IMU]

        # Calibrate with measured latency
        self.timestamp_provider.calibrate_sensor_latency(
            SensorType.IMU, 8.5
        )  # 8.5ms measured

        # Should be updated (exponential moving average)
        updated_latency = self.timestamp_provider._latency_calibration[SensorType.IMU]
        assert abs(updated_latency - 8.5) < 0.1  # Should be close to measured value

    def test_performance_profiling_integration(self):
        """Test integration with performance profiling system."""

        # Profile a timestamp + encode/decode operation
        def profiled_operation():
            raw_data = {"accel_x": 0.1, "accel_y": 0.2, "accel_z": 9.8}
            timestamped = self.timestamp_provider.tag_sensor_data(
                SensorType.IMU, raw_data
            )

            imu_data = IMUData(
                measurement_timestamp_ns=timestamped.measurement_timestamp_ns,
                reception_timestamp_ns=timestamped.reception_timestamp_ns,
                accel_x=raw_data["accel_x"],
                accel_y=raw_data["accel_y"],
                accel_z=raw_data["accel_z"],
                gyro_x=0.01,
                gyro_y=0.02,
                gyro_z=0.03,
            )

            binary_msg = BinarySensorProtocol.encode_imu(imu_data)
            decoded = BinarySensorProtocol.decode_imu(binary_msg)
            return decoded

        # Measure with profiler
        result = self.performance_profiler.measure_latency(
            "sensor_pipeline_complete", profiled_operation
        )

        assert result is not None
        assert result.measurement_timestamp_ns > 0

        # Check that profiling recorded the measurement
        report = self.performance_profiler.get_performance_report()
        assert "sensor_pipeline_complete" in report["profiles"]

        stats = report["profiles"]["sensor_pipeline_complete"]
        assert stats["count"] >= 1
        assert stats["mean_ms"] > 0
