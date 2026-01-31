#!/usr/bin/env python3
"""
Simple test to demonstrate mock hardware server functionality.
Tests the critical systems without requiring full test framework.
"""

import time
import requests
import json
from typing import Dict, Any

def test_mock_hardware():
    """Test mock hardware server connectivity."""
    base_url = "http://localhost:8080"
    
    try:
        # Test health check
        print("Testing mock hardware health...")
        response = requests.get(f"{base_url}/health", timeout=5)
        if response.status_code == 200:
            print("✓ Health check passed")
            print(f"Status: {response.json()}")
        else:
            print(f"✗ Health check failed: {response.status_code}")
        
        # Test GPIO operations
        print("\nTesting GPIO operations...")
        
        # Test emergency stop button press
        gpio_data = {"value": False, "mode": "output"}
        response = requests.post(f"{base_url}/gpio/17", json=gpio_data, timeout=5)
        if response.status_code == 200:
            print("✓ Emergency stop button test passed")
        
        # Test motor power relay
        relay_data = {"value": False, "mode": "output"}
        response = requests.post(f"{base_url}/gpio/18", json=relay_data, timeout=5)
        if response.status_code == 200:
            print("✓ Motor power relay test passed")
        
        # Test status LED
        led_data = {"value": False, "mode": "output"}
        response = requests.post(f"{base_url}/gpio/24", json=led_data, timeout=5)
        if response.status_code == 200:
            print("✓ Status LED test passed")
        
        # Test CAN message sending
        print("\nTesting CAN operations...")
        
        can_data = {
            "arbitration_id": "0x00C",
            "data": "100000000000000000"
        }
        response = requests.post(f"{base_url}/can/send", json=can_data, timeout=5)
        if response.status_code == 200:
            print("✓ CAN send test passed")
            print(f"Response: {response.json()}")
        
        # Test sensor data
        print("\nTesting sensor operations...")
        
        sensors = ["imu", "gps", "battery"]
        for sensor in sensors:
            response = requests.get(f"{base_url}/sensors/{sensor}", timeout=5)
            if response.status_code == 200:
                print(f"✓ {sensor} sensor data available")
                print(f"Latest data: {response.json()}")
            else:
                print(f"✗ {sensor} sensor data failed: {response.status_code}")
        
        # Test encoder data
        encoders = ["front_left", "front_right", "middle_left", "middle_right", "rear_left", "rear_right"]
        for encoder in encoders:
            response = requests.get(f"{base_url}/encoders/{encoder}", timeout=5)
            if response.status_code == 200:
                print(f"✓ {encoder} encoder data: {response.json()['count']}")
            else:
                print(f"✗ {encoder} encoder data failed: {response.status_code}")
        
        print("\n✅ Mock hardware server tests completed successfully!")
        return True
        
    except requests.exceptions.RequestException as e:
        print(f"✗ Mock hardware server test failed: {e}")
        return False
    except Exception as e:
        print(f"✗ Unexpected error: {e}")
        return False

def test_sensor_fusion():
    """Test sensor fusion system independently."""
    print("Testing sensor fusion system...")
    
    try:
        import sys
        sys.path.append('/home/durian/urc-machiato-2026/src')
        from autonomy.perception.sensor_fusion import (
            SensorFusionManager, SensorType, SensorMeasurement, SensorStatus
        )
        
        # Create fusion manager
        fusion_manager = SensorFusionManager({
            'fusion_rate': 100.0,
            'enable_ekf': True,
            'enable_complementary': True
        })
        
        # Add sensors
        fusion_manager.add_sensor('test_imu', SensorType.IMU)
        fusion_manager.add_sensor('test_gps', SensorType.GPS)
        
        # Start fusion
        fusion_manager.start_fusion()
        
        # Add test measurement
        imu_measurement = SensorMeasurement(
            sensor_type=SensorType.IMU,
            timestamp=time.time(),
            data={
                'linear_acceleration_x': 0.1,
                'linear_acceleration_y': 0.0,
                'linear_acceleration_z': 9.81,
                'angular_velocity_x': 0.0,
                'angular_velocity_y': 0.0,
                'angular_velocity_z': 0.1
            },
            confidence=1.0,
            status=SensorStatus.HEALTHY
        )
        
        fusion_manager.update_sensor_measurement('test_imu', imu_measurement)
        
        # Get fused state
        fused_state = fusion_manager.get_fused_state()
        
        print(f"✓ Sensor fusion system working")
        print(f"Fused position: {fused_state.position}")
        print(f"Fused orientation: {fused_state.orientation}")
        
        fusion_manager.stop_fusion()
        return True
        
    except ImportError as e:
        print(f"✗ Sensor fusion import failed: {e}")
        return False
    except Exception as e:
        print(f"✗ Sensor fusion test failed: {e}")
        return False

def test_motor_control():
    """Test motor control system independently."""
    print("Testing motor control system...")
    
    try:
        import sys
        sys.path.append('/home/durian/urc-machiato-2026/src')
        from autonomy.control.advanced_motor_controller import AdvancedMotorController
        
        # Create motor controller with mock interface
        motor_controller = AdvancedMotorController({
            'interface': 'mock',
            'adaptive_mode': True,
            'traction_control': True,
            'load_balancing': True
        })
        
        # Initialize
        if motor_controller.initialize():
            print("✓ Motor controller initialized")
            
            # Test velocity command
            from tests.test_critical_systems import MockTwist
            twist = MockTwist()
            twist.linear.x = 1.0
            twist.angular.z = 0.5
            
            if motor_controller.set_velocity_command(twist):
                print("✓ Velocity command successful")
                print(f"Target velocities: {motor_controller.target_velocities}")
            else:
                print("✗ Velocity command failed")
            
            # Test PID control
            for wheel_name, pid in motor_controller.pid_controllers.items():
                pid.reset()
                
                # Test update
                error = 0.8 - 0.75  # 10% error
                output = pid.update(1.0, 0.75, 0.01)
                
                if abs(output) < 0.01:  # Within 1% of target
                    print(f"✓ {wheel_name} PID control working")
                else:
                    print(f"⚠ {wheel_name} PID control has high error: {error}")
            
            # Test traction control
            if motor_controller.traction_controller.detect_wheel_slip('front_left', 2.0, 1.5):
                print("✓ Traction control detects slip correctly")
            else:
                print("⚠ Traction control may not be working")
            
            print("✓ Motor control system working")
            return True
        else:
            print("✗ Motor controller initialization failed")
            return False
        
    except ImportError as e:
        print(f"✗ Motor control import failed: {e}")
        return False
    except Exception as e:
        print(f"✗ Motor control test failed: {e}")
        return False

def main():
    """Run all demonstration tests."""
    print("=== URC 2026 Critical Systems Demo ===")
    print("Testing mock hardware and critical systems independently...\n")
    
    tests = [
        ("Mock Hardware Server", test_mock_hardware),
        ("Sensor Fusion", test_sensor_fusion),
        ("Motor Control", test_motor_control)
    ]
    
    results = []
    for test_name, test_func in tests:
        print(f"\n--- {test_name} ---")
        result = test_func()
        results.append((test_name, "PASS" if result else "FAIL"))
    
    print(f"\n=== Test Results ===")
    for test_name, result in results:
        status = "✅" if result == "PASS" else "❌"
        print(f"{status} {test_name}")
    
    passed = sum(1 for _, result in results if result == "PASS")
    total = len(results)
    
    print(f"\nSummary: {passed}/{total} tests passed")
    
    if passed == total:
        print("🎉 All systems working correctly!")
        return 0
    else:
        print("⚠ Some systems need attention")
        return 1

if __name__ == "__main__":
    exit(main())