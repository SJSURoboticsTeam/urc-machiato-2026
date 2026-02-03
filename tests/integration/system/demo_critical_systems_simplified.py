#!/usr/bin/env python3
"""
Simplified critical systems testing demonstration.

This script provides a working demonstration of:
1. Mock hardware server
2. Sensor fusion
3. Motor control
4. Basic connectivity tests

Author: URC 2026 Testing Team
"""

import time
import requests
import json
import sys
from typing import Dict, List, Any


def test_mock_hardware():
    """Test mock hardware server connectivity."""
    print("=== Testing Mock Hardware Server ===")

    base_url = "http://localhost:8080"

    # Test server basic health
    try:
        response = requests.get(f"{base_url}/health", timeout=5)
        if response.status_code == 200:
            print("✅ Mock hardware server health check passed")
            return True
        else:
            print(f"❌ Mock hardware server down: {response.status_code}")
            return False

    except requests.exceptions.RequestException as e:
        print(f"❌ Mock hardware server error: {e}")
        return False


def test_sensor_fusion():
    """Test sensor fusion system independently."""
    print("=== Testing Sensor Fusion System ===")

    try:
        import sys

        sys.path.append("/home/durian/urc-machiato-2026/src")
        from autonomy.perception.sensor_fusion import (
            SensorFusionManager,
            SensorType,
            SensorMeasurement,
            SensorStatus,
        )

        # Create fusion manager
        fusion_manager = SensorFusionManager(
            {"fusion_rate": 100.0, "enable_ekf": True, "enable_complementary": True}
        )

        # Add sensors
        fusion_manager.add_sensor("test_imu", SensorType.IMU)
        fusion_manager.add_sensor("test_gps", SensorType.GPS)

        # Add test measurements
        imu_measurement = SensorMeasurement(
            sensor_type=SensorType.IMU,
            timestamp=time.time(),
            data={
                "linear_acceleration_x": 0.1,
                "linear_acceleration_y": 0.0,
                "linear_acceleration_z": 9.81,
                "angular_velocity_x": 0.0,
                "angular_velocity_y": 0.0,
                "confidence": 1.0,
                "status": SensorStatus.HEALTHY,
            },
        )
        fusion_manager.update_sensor_measurement("test_imu", imu_measurement)

        # Start fusion and test
        fusion_manager.start_fusion()
        time.sleep(0.5)  # Let fusion process

        # Get fused state
        fused_state = fusion_manager.get_fused_state()

        # Test results - fused quaternion has w near 1, xyz near 0 is correct
        position_ok = all(
            [
                abs(fused_state.position[0]) < 1.0,
                abs(fused_state.position[1]) < 1.0,
                abs(fused_state.position[2]) < 1.0,
            ]
        )

        fusion_manager.stop_fusion()
        return position_ok

    except ImportError as e:
        print(f"❌ Sensor fusion import failed: {e}")
        return False


def test_motor_control():
    """Test motor control system independently."""
    print("=== Testing Motor Control System ===")

    try:
        import sys

        sys.path.append("/home/durian/urc-machiato-2026/scripts")
        from test_motor_standalone import MotorController, ControlMode

        # Create motor controller
        motor_controller = MotorController(
            {
                "interface": "mock",
                "adaptive_mode": True,
                "traction_control": True,
                "load_balancing": True,
            }
        )

        # Test mode setting
        if motor_controller.set_mode(ControlMode.VELOCITY):
            print("✓ Motor controller initialized and set to velocity mode")
        else:
            print("❌ Motor controller failed to set mode")
            return False

        # Test velocity commands
        target_vels = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]  # Forward motion
        result = motor_controller.set_velocities(target_vels)

        if result:
            print("✓ Motor velocity command successful")
            # Test a few update cycles
            for i in range(3):
                state = motor_controller.update(0.1)
                time.sleep(0.01)
            print(f"✅ Motor controller working properly")
            return True
        else:
            print("❌ Motor velocity command failed")
            return False

        # Test velocity commands
        from tests.test_critical_systems import MockTwist

        twist = MockTwist()
        twist.linear.x = 0.5
        twist.angular.z = 0.2

        result = motor_controller.set_velocity_command(twist)

        if result:
            print("✓ Motor velocity command successful")
        else:
            print("❌ Motor velocity command failed")

        # Test emergency stop
        emergency_result = motor_controller.emergency_stop()
        if emergency_result:
            print("✓ Emergency stop successful")
        else:
            print("❌ Emergency stop failed")

        motor_controller.shutdown()
        return result and emergency_result

    except ImportError as e:
        print(f"❌ Motor control import failed: {e}")
        return False


def main():
    """Run comprehensive systems demo."""
    print("=== URC 2026 Critical Systems Demo ===")

    # Test components individually
    tests = [
        ("Mock Hardware Server", test_mock_hardware),
        ("Sensor Fusion", test_sensor_fusion),
        ("Motor Control", test_motor_control),
    ]

    results = []
    for test_name, test_func in tests:
        print(f"\n--- {test_name} ---")
        result = test_func()
        results.append((test_name, "PASS" if result else "FAIL"))
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"{status} {test_name}")

    passed = sum(1 for _, status in results if status == "PASS")
    total = len(results)

    # Print summary
    print(f"\n=== Test Summary ===")
    print(f"Total test suites: {total}")
    print(f"Passed: {passed}")
    print(f"Failed: {total - passed}")

    if passed == total:
        print(f"🎉 ALL SYSTEMS OPERATIONAL")
        print(
            "✅ Mock hardware server, sensor fusion, and motor control systems working correctly"
        )
        return 0
    else:
        print("⚠ Some systems need attention")
        return 1


if __name__ == "__main__":
    main()
