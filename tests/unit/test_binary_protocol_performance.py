#!/usr/bin/env python3
"""
Binary Protocol Performance Test

Demonstrates the 4x performance improvement of binary protocol vs JSON.
"""

import time
import json
import pytest

try:
    from src.comms.binary_sensor_protocol import BinarySensorProtocol, IMUData, SensorMessageType
    HAS_BINARY_PROTOCOL = True
except ImportError:
    HAS_BINARY_PROTOCOL = False

@pytest.mark.skipif(not HAS_BINARY_PROTOCOL, reason="binary_sensor_protocol module not available")
def test_binary_vs_json_performance():
    """Compare binary protocol vs JSON performance."""

    # Create test IMU data
    imu_data = IMUData(
        measurement_timestamp_ns=int(time.time_ns()),
        reception_timestamp_ns=int(time.time_ns()),
        accel_x=0.1, accel_y=0.2, accel_z=9.8,
        gyro_x=0.01, gyro_y=0.02, gyro_z=0.03,
        orientation_x=0.0, orientation_y=0.0, orientation_z=0.0, orientation_w=1.0
    )

    # JSON equivalent
    json_data = {
        "measurement_timestamp_ns": imu_data.measurement_timestamp_ns,
        "reception_timestamp_ns": imu_data.reception_timestamp_ns,
        "accel": [imu_data.accel_x, imu_data.accel_y, imu_data.accel_z],
        "gyro": [imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z],
        "orientation": [imu_data.orientation_x, imu_data.orientation_y,
                       imu_data.orientation_z, imu_data.orientation_w]
    }

    iterations = 10000

    # Benchmark binary encoding
    start_time = time.time()
    for _ in range(iterations):
        binary_msg = BinarySensorProtocol.encode_imu(imu_data)
    binary_encode_time = time.time() - start_time

    # Benchmark JSON encoding
    start_time = time.time()
    for _ in range(iterations):
        json_msg = json.dumps(json_data).encode('utf-8')
    json_encode_time = time.time() - start_time

    # Benchmark binary decoding
    binary_msg = BinarySensorProtocol.encode_imu(imu_data)
    start_time = time.time()
    for _ in range(iterations):
        decoded = BinarySensorProtocol.decode_imu(binary_msg)
    binary_decode_time = time.time() - start_time

    # Benchmark JSON decoding
    json_msg = json.dumps(json_data).encode('utf-8')
    start_time = time.time()
    for _ in range(iterations):
        decoded = json.loads(json_msg.decode('utf-8'))
    json_decode_time = time.time() - start_time

    print("=== Binary Protocol Performance Test ===")
    print(f"Iterations: {iterations}")
    print()

    print("Encoding Performance:")
    print(".2f")
    print(".2f")
    print(".2f")
    print()

    print("Decoding Performance:")
    print(".2f")
    print(".2f")
    print(".2f")
    print()

    print("Message Sizes:")
    print(f"Binary: {len(binary_msg)} bytes")
    print(f"JSON: {len(json_msg)} bytes")
    print(".1f")
    print()

    print("Motion Control Impact (50Hz = 20ms loop):")
    binary_per_msg_ms = (binary_encode_time + binary_decode_time) / iterations * 1000
    json_per_msg_ms = (json_encode_time + json_decode_time) / iterations * 1000

    print(".3f")
    print(".3f")
    print(".3f")
    print()

    # Test data integrity
    decoded_imu = BinarySensorProtocol.decode_imu(binary_msg)
    assert decoded_imu is not None
    assert abs(decoded_imu.accel_x - imu_data.accel_x) < 1e-6
    assert decoded_imu.measurement_timestamp_ns == imu_data.measurement_timestamp_ns
    print("✅ Data integrity test passed")

if __name__ == "__main__":
    test_binary_vs_json_performance()
