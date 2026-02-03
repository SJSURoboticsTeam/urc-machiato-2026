#!/usr/bin/env python3
"""
Comprehensive System Validation Test

Test script for validating the integrated critical systems implementation.
Tests all components individually and as an integrated system.

Author: URC 2026 Validation Team
"""

import time
import asyncio
import logging
import sys
from typing import Dict, Any

# Configure logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


# Mock classes for testing without ROS2 dependencies
class MockLinear:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class MockAngular:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class MockTwist:
    def __init__(self):
        self.linear = MockLinear()
        self.angular = MockAngular()


class SystemValidator:
    """Comprehensive system validator."""

    def __init__(self):
        self.test_results = {
            "can_bridge": False,
            "emergency_stop": False,
            "motor_control": False,
            "sensor_fusion": False,
            "graceful_degradation": False,
            "integration": False,
            "performance": False,
            "overall_success": False,
        }

        self.test_durations = {}
        self.error_messages = []

    def run_all_tests(self) -> Dict[str, Any]:
        """Run all validation tests."""
        logger.info("Starting comprehensive system validation")

        start_time = time.time()

        try:
            # Individual component tests
            self.test_can_bridge()
            self.test_emergency_stop_system()
            self.test_motor_control_system()
            self.test_sensor_fusion_system()
            self.test_graceful_degradation_system()

            # Integration tests
            self.test_system_integration()
            self.test_performance_benchmarks()

            # Calculate overall success
            passed_tests = [
                result
                for result in self.test_results.values()
                if isinstance(result, bool)
            ]
            self.test_results["overall_success"] = all(passed_tests)

        except Exception as e:
            logger.error(f"Test execution failed: {e}")
            self.error_messages.append(f"Test execution error: {e}")

        total_duration = time.time() - start_time

        return {
            "test_results": self.test_results,
            "test_durations": self.test_durations,
            "total_duration": total_duration,
            "error_messages": self.error_messages,
            "summary": self._generate_summary(),
        }

    def test_can_bridge(self):
        """Test CAN bridge (single-channel)."""
        logger.info("Testing CAN bridge...")
        start_time = time.time()

        try:
            import sys

            sys.path.append("/home/durian/urc-machiato-2026/src")
            from src.bridges.can_bridge import CANBridge

            config = {
                "device": "/dev/ttyACM0",
                "baudrate": 115200,
                "fallback_devices": ["/dev/ttyACM1", "/dev/ttyUSB0"],
            }
            can_bridge = CANBridge(config)
            status = can_bridge.get_status()
            if hasattr(status, "is_connected") and hasattr(status, "bridge_type"):
                self.test_results["can_bridge"] = True
                logger.info("CAN bridge test passed")
            else:
                self.test_results["can_bridge"] = False
                logger.warning("CAN bridge test failed: invalid status structure")
        except Exception as e:
            logger.error(f"CAN bridge test error: {e}")
            self.error_messages.append(f"CAN bridge error: {e}")

        self.test_durations["can_bridge"] = time.time() - start_time

    def test_emergency_stop_system(self):
        """Test emergency stop system."""
        logger.info("Testing emergency stop system...")
        start_time = time.time()

        try:
            import sys

            sys.path.append("/home/durian/urc-machiato-2026/src")
            from autonomy.control.hardware_emergency_stop import (
                EmergencyStopConfig,
                HardwareEmergencyStop,
            )

            # Test configuration
            config = EmergencyStopConfig(
                e_stop_gpio_pin=17, motor_power_relay_pin=18, watchdog_timeout_ms=100
            )

            # Create emergency stop (will use mock GPIO)
            e_stop = HardwareEmergencyStop(config)

            # Test basic functionality
            soft_stop_success = e_stop.activate_soft_stop("test_soft_stop")
            reset_success = e_stop.reset_emergency_stop()

            # Test status reporting
            status = e_stop.get_status()

            # Validate
            if soft_stop_success and reset_success and isinstance(status, dict):
                self.test_results["emergency_stop"] = True
                logger.info("✓ Emergency stop system test passed")
            else:
                logger.warning("✗ Emergency stop system test failed")

        except Exception as e:
            logger.error(f"✗ Emergency stop system test error: {e}")
            self.error_messages.append(f"Emergency stop error: {e}")

        self.test_durations["emergency_stop"] = time.time() - start_time

    def test_motor_control_system(self):
        """Test motor control system."""
        logger.info("Testing motor control system...")
        start_time = time.time()

        try:
            import sys

            sys.path.append("/home/durian/urc-machiato-2026/src")
            from autonomy.control.advanced_motor_controller import (
                create_advanced_motor_controller,
            )

            # Test with module-level MockTwist

            # Test configuration
            config = {
                "interface": "mock",  # Use mock interface for testing
                "adaptive_mode": True,
                "traction_control": True,
            }

            # Create motor controller
            motor_controller = create_advanced_motor_controller(config)

            # Test initialization
            initialized = motor_controller.initialize()

            # Test velocity command
            twist = MockTwist()
            twist.linear.x = 1.0
            twist.angular.z = 0.5

            command_success = motor_controller.set_velocity_command(twist)

            # Test status reporting
            status = motor_controller.get_advanced_status()

            # Test emergency stop
            e_stop_success = motor_controller.emergency_stop()

            # Validate
            if (
                initialized
                and command_success
                and e_stop_success
                and isinstance(status, dict)
                and "control_active" in status
            ):
                self.test_results["motor_control"] = True
                logger.info("✓ Motor control system test passed")
            else:
                logger.warning("✗ Motor control system test failed")

        except Exception as e:
            logger.error(f"✗ Motor control system test error: {e}")
            self.error_messages.append(f"Motor control error: {e}")

        self.test_durations["motor_control"] = time.time() - start_time

    def test_sensor_fusion_system(self):
        """Test sensor fusion system."""
        logger.info("Testing sensor fusion system...")
        start_time = time.time()

        try:
            import sys

            sys.path.append("/home/durian/urc-machiato-2026/src")
            from autonomy.perception.sensor_fusion import (
                create_sensor_fusion_manager,
                SensorType,
                SensorMeasurement,
                SensorStatus,
            )
            import numpy as np

            # Create sensor fusion manager
            fusion_manager = create_sensor_fusion_manager()

            # Add test sensors
            fusion_manager.add_sensor("test_imu", SensorType.IMU)
            fusion_manager.add_sensor("test_gps", SensorType.GPS)

            # Start fusion
            fusion_manager.start_fusion()

            # Create test measurement
            test_measurement = SensorMeasurement(
                sensor_type=SensorType.IMU,
                timestamp=time.time(),
                data={
                    "linear_acceleration_x": 0.1,
                    "linear_acceleration_y": 0.0,
                    "linear_acceleration_z": 9.8,
                    "angular_velocity_x": 0.0,
                    "angular_velocity_y": 0.0,
                    "angular_velocity_z": 0.1,
                },
                confidence=1.0,
                status=SensorStatus.HEALTHY,
            )

            # Update with test measurement
            fusion_manager.update_sensor_measurement("test_imu", test_measurement)

            # Get fused state
            fused_state = fusion_manager.get_fused_state()

            # Get sensor status
            sensor_status = fusion_manager.get_sensor_status()

            # Stop fusion
            fusion_manager.stop_fusion()

            # Validate
            if fused_state and sensor_status and "test_imu" in sensor_status:
                self.test_results["sensor_fusion"] = True
                logger.info("✓ Sensor fusion system test passed")
            else:
                logger.warning("✗ Sensor fusion system test failed")

        except Exception as e:
            logger.error(f"✗ Sensor fusion system test error: {e}")
            self.error_messages.append(f"Sensor fusion error: {e}")

        self.test_durations["sensor_fusion"] = time.time() - start_time

    def test_graceful_degradation_system(self):
        """Test graceful degradation system."""
        logger.info("Testing graceful degradation system...")
        start_time = time.time()

        try:
            import sys

            sys.path.append("/home/durian/urc-machiato-2026/src")
            from core.graceful_degradation import (
                create_operation_mode_manager,
                OperationMode,
            )

            # Create operation mode manager
            mode_manager = create_operation_mode_manager()

            # Start mode management
            mode_manager.start_mode_management()

            # Test mode switching
            original_mode = mode_manager.current_mode
            mode_manager.force_mode_switch(OperationMode.SURVIVAL_MODE, "test_switch")

            # Get system status
            status = mode_manager.get_system_status()

            # Test mission context
            mode_manager.set_mission_context("test_phase", 300.0, 50.0)

            # Stop mode management
            mode_manager.stop_mode_management()

            # Validate
            if (
                status
                and "current_mode" in status
                and status["current_mode"] == OperationMode.SURVIVAL_MODE.value
            ):
                self.test_results["graceful_degradation"] = True
                logger.info("✓ Graceful degradation system test passed")
            else:
                logger.warning("✗ Graceful degradation system test failed")

        except Exception as e:
            logger.error(f"✗ Graceful degradation system test error: {e}")
            self.error_messages.append(f"Graceful degradation error: {e}")

        self.test_durations["graceful_degradation"] = time.time() - start_time

    def test_system_integration(self):
        """Test full system integration."""
        logger.info("Testing system integration...")
        start_time = time.time()

        try:
            import sys

            sys.path.append("/home/durian/urc-machiato-2026/src")
            from autonomy.control.integrated_critical_systems import (
                create_integrated_critical_systems,
            )

            # Mock geometry_msgs for testing
            class MockTwist:
                def __init__(self):
                    self.linear.x = 0.0
                    self.linear.y = 0.0
                    self.linear.z = 0.0
                    self.angular.x = 0.0
                    self.angular.y = 0.0
                    self.angular.z = 0.0

            # Test configuration
            config = {
                "motor_interface": "mock",
                "mission_duration": 900.0,
                "adaptive_motor_control": True,
            }

            # Create integrated system
            integrated_system = create_integrated_critical_systems(config)

            # Start systems
            systems_started = integrated_system.start_systems()

            # Test velocity command
            twist = MockTwist()
            twist.linear.x = 0.5
            twist.angular.z = 0.2

            command_success = integrated_system.process_velocity_command(twist)

            # Get system status
            system_status = integrated_system.get_system_status()

            # Run validation
            validation_results = integrated_system.run_competition_validation()

            # Shutdown systems
            integrated_system.shutdown_systems()

            # Validate
            if (
                systems_started
                and command_success
                and system_status
                and validation_results
            ):
                self.test_results["integration"] = True
                logger.info("✓ System integration test passed")
            else:
                logger.warning("✗ System integration test failed")

        except Exception as e:
            logger.error(f"✗ System integration test error: {e}")
            self.error_messages.append(f"Integration error: {e}")

        self.test_durations["integration"] = time.time() - start_time

    def test_performance_benchmarks(self):
        """Test performance benchmarks."""
        logger.info("Testing performance benchmarks...")
        start_time = time.time()

        try:
            # Test import performance
            import_start = time.time()
            from src.autonomy.control.advanced_motor_controller import (
                AdvancedMotorController,
            )
            from src.bridges.can_bridge import CANBridge
            from src.autonomy.perception.sensor_fusion import SensorFusionManager

            import_duration = time.time() - import_start

            # Test instantiation performance
            instantiation_start = time.time()

            motor_config = {"interface": "mock"}
            motor_controller = AdvancedMotorController(motor_config)

            can_config = {"device": "/dev/ttyACM0", "baudrate": 115200}
            can_interface = CANBridge(can_config)

            fusion_config = {}
            fusion_manager = SensorFusionManager(fusion_config)

            instantiation_duration = time.time() - instantiation_start

            # Performance criteria (should be very fast)
            import_ok = import_duration < 2.0  # 2 seconds max
            instantiation_ok = instantiation_duration < 1.0  # 1 second max

            if import_ok and instantiation_ok:
                self.test_results["performance"] = True
                logger.info("✓ Performance benchmarks passed")
                logger.info(f"  Import time: {import_duration:.3f}s")
                logger.info(f"  Instantiation time: {instantiation_duration:.3f}s")
            else:
                logger.warning("✗ Performance benchmarks failed")
                logger.info(f"  Import time: {import_duration:.3f}s (required < 2.0s)")
                logger.info(
                    f"  Instantiation time: {instantiation_duration:.3f}s (required < 1.0s)"
                )

        except Exception as e:
            logger.error(f"✗ Performance benchmarks test error: {e}")
            self.error_messages.append(f"Performance error: {e}")

        self.test_durations["performance"] = time.time() - start_time

    def _generate_summary(self) -> str:
        """Generate test summary."""
        passed = sum(1 for result in self.test_results.values() if result)
        total = len(self.test_results)

        summary = f"Test Summary: {passed}/{total} tests passed\n"

        for test_name, result in self.test_results.items():
            status = "✓ PASS" if result else "✗ FAIL"
            summary += f"  {test_name}: {status}\n"

        if self.error_messages:
            summary += "\nErrors:\n"
            for error in self.error_messages:
                summary += f"  - {error}\n"

        return summary


def main():
    """Main test execution."""
    print("URC Rover Critical Systems Validation")
    print("=" * 50)

    validator = SystemValidator()
    results = validator.run_all_tests()

    print("\n" + results["summary"])
    print(f"Total test duration: {results['total_duration']:.2f}s")

    # Exit with appropriate code
    if results["test_results"]["overall_success"]:
        print("\n🎉 All critical systems validated successfully!")
        sys.exit(0)
    else:
        print("\n❌ Some validation tests failed!")
        sys.exit(1)


if __name__ == "__main__":
    main()
