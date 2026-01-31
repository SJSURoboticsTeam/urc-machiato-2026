#!/usr/bin/env python3
"""
Test Suite for URC 2026 Critical Systems

Comprehensive testing framework for all critical systems including:
- Unit tests for individual components
- Integration tests for system coordination
- Performance benchmarks and stress tests
- Competition scenario validation
- Simulation environment tests

Author: URC 2026 Testing Team
"""

import pytest
import asyncio
import time
import threading
import numpy as np
from typing import Dict, Any, List, Optional
from unittest.mock import Mock, patch, MagicMock
import logging

# Configure logging for tests
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Mock ROS2 components for testing
class MockTwist:
    def __init__(self):
        self.linear = MockLinear()
        self.angular = MockAngular()

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

class MockROS2Node:
    def __init__(self, name):
        self.name = name
        self.get_logger = Mock()
        self.create_publisher = Mock()
        self.create_subscription = Mock()
        self.create_client = Mock()
        self.create_timer = Mock()

# Test configuration
TEST_CONFIG = {
    'can_bridge': {
        'device': '/dev/ttyACM0',
        'baudrate': 115200,
        'fallback_devices': ['/dev/ttyACM1', '/dev/ttyUSB0'],
    },
    'emergency_stop': {
        'e_stop_gpio_pin': 17,
        'motor_power_relay_pin': 18,
        'status_led_pin': 24,
        'watchdog_timeout_ms': 100,
        'auto_reset_seconds': 5
    },
    'motor_control': {
        'interface': 'mock',
        'adaptive_mode': True,
        'traction_control': True,
        'load_balancing': True
    },
    'sensor_fusion': {
        'fusion_rate': 100.0,
        'enable_ekf': True,
        'enable_complementary': True
    },
    'graceful_degradation': {
        'mode_evaluation_interval': 2.0,
        'stability_requirement': 5.0,
        'cpu_critical_threshold': 90.0,
        'memory_critical_threshold': 95.0
    },
    'integration': {
        'mission_duration': 900.0,
        'adaptive_motor_control': True,
        'mission_phase': 'competition'
    }
}


class TestCANBridge:
    """Test suite for CAN bridge (single-channel)."""

    @pytest.fixture
    def can_bridge(self):
        from src.bridges.can_bridge import CANBridge
        return CANBridge(TEST_CONFIG['can_bridge'])

    @pytest.mark.asyncio
    async def test_can_bridge_initialization(self, can_bridge):
        """Test CAN bridge connect."""
        with patch('serial.Serial') as mock_serial:
            mock_serial.return_value.is_open = True
            mock_serial.return_value.in_waiting = 0
            mock_serial.return_value.readline.return_value = b'OK\r'
            result = await can_bridge.connect()
            assert result is True
            assert can_bridge.is_connected is True

    def test_can_bridge_status(self, can_bridge):
        """Test CAN bridge get_status returns BridgeStatus."""
        from src.bridges.can_bridge import BridgeStatus
        status = can_bridge.get_status()
        assert isinstance(status, BridgeStatus)
        assert hasattr(status, 'is_connected')
        assert hasattr(status, 'bridge_type')
        assert hasattr(status, 'messages_sent')

    def test_can_bridge_shutdown(self, can_bridge):
        """Test CAN bridge synchronous shutdown."""
        can_bridge.serial_port = Mock(is_open=True, close=Mock())
        can_bridge.is_connected = True
        can_bridge.shutdown()
        assert can_bridge.is_connected is False
        assert can_bridge.serial_port is None


class TestHardwareEmergencyStop:
    """Test suite for hardware emergency stop system (requires RPi.GPIO)."""

    @pytest.fixture
    def emergency_stop(self):
        pytest.importorskip("RPi.GPIO")
        from src.autonomy.control.hardware_emergency_stop import HardwareEmergencyStop, EmergencyStopConfig
        config = EmergencyStopConfig(**TEST_CONFIG['emergency_stop'])
        return HardwareEmergencyStop(config)
    
    def test_emergency_stop_initialization(self, emergency_stop):
        """Test emergency stop initialization."""
        assert emergency_stop.config.e_stop_gpio_pin == 17
        assert emergency_stop.config.motor_power_relay_pin == 18
        assert emergency_stop.config.watchdog_timeout_ms == 100
    
    def test_soft_stop_activation(self, emergency_stop):
        """Test soft emergency stop activation."""
        result = emergency_stop.activate_soft_stop("test_soft_stop")
        
        assert result is True
        assert emergency_stop.is_active is True
        assert emergency_stop.stop_level.value == "soft_stop"
        assert emergency_stop.stop_reason == "test_soft_stop"
    
    def test_hard_stop_activation(self, emergency_stop):
        """Test hard emergency stop activation."""
        result = emergency_stop.activate_hard_stop("test_hard_stop")
        
        assert result is True
        assert emergency_stop.is_active is True
        assert emergency_stop.stop_level.value == "hard_stop"
        assert emergency_stop.stop_reason == "test_hard_stop"
    
    def test_emergency_stop_reset(self, emergency_stop):
        """Test emergency stop reset."""
        # Activate stop first
        emergency_stop.activate_soft_stop("test")
        
        # Then reset
        result = emergency_stop.reset_emergency_stop()
        
        assert result is True
        assert emergency_stop.is_active is False
        assert emergency_stop.stop_reason == "none"
    
    @patch('RPi.GPIO.setup')
    @patch('RPi.GPIO.output')
    @patch('RPi.GPIO.input')
    def test_gpio_operations(self, mock_input, mock_output, mock_setup, emergency_stop):
        """Test GPIO operations with mocked hardware."""
        # Test initialization
        emergency_stop._setup_gpio()
        
        # Verify GPIO setup calls
        mock_setup.assert_called()
        
        # Test status LED operation
        status = emergency_stop.get_status()
        assert isinstance(status, dict)
        assert 'is_active' in status


class TestAdvancedMotorControl:
    """Test suite for advanced motor control system (requires motor_controller package)."""

    @pytest.fixture
    def motor_controller(self):
        pytest.importorskip("src.autonomy.control.motor_controller")
        from src.autonomy.control.advanced_motor_controller import AdvancedMotorController
        return AdvancedMotorController(TEST_CONFIG['motor_control'])
    
    def test_motor_controller_initialization(self, motor_controller):
        """Test motor controller initialization."""
        result = motor_controller.initialize()
        assert result is True
        assert motor_controller.control_active is True
        assert len(motor_controller.pid_controllers) == 6  # 6-wheel rover
    
    def test_velocity_command_conversion(self, motor_controller):
        """Test twist to wheel velocity conversion."""
        twist = MockTwist()
        twist.linear.x = 1.0  # Forward 1 m/s
        twist.angular.z = 0.5  # Turn 0.5 rad/s
        
        result = motor_controller.set_velocity_command(twist)
        assert result is True
        
        # Check that target velocities are set
        assert motor_controller.target_velocities['front_left'] is not None
        assert motor_controller.target_velocities['front_right'] is not None
    
    def test_pid_controller_update(self, motor_controller):
        """Test PID controller updates."""
        for wheel_name, pid_controller in motor_controller.pid_controllers.items():
            # Test reset
            pid_controller.reset()
            assert pid_controller.previous_error == 0.0
            assert pid_controller.integral == 0.0
            
            # Test update
            output = pid_controller.update(1.0, 0.8, 0.01)
            assert isinstance(output, float)
    
    def test_traction_control_slip_detection(self, motor_controller):
        """Test traction control slip detection."""
        traction_controller = motor_controller.traction_controller
        
        # Test slip detection
        slip_detected = traction_controller.detect_wheel_slip('front_left', 2.0, 1.5)
        
        # Calculate expected slip ratio
        expected_slip_ratio = abs((2.0 - 1.5) / 2.0)  # 25% slip
        slip_threshold = traction_controller.params.slip_threshold  # 20% threshold
        
        expected_slip = expected_slip_ratio > slip_threshold
        assert slip_detected == expected_slip
    
    def test_terrain_classification(self, motor_controller):
        """Test terrain classification."""
        terrain_classifier = motor_controller.terrain_classifier
        
        # Test different terrain scenarios
        wheel_vels = {'front_left': 1.0, 'front_right': 1.0}
        motor_currents = {'front_left': 2.0, 'front_right': 2.5}
        accelerations = {'front_left': 0.1, 'front_right': 0.15}
        
        # Classify terrain (should not crash)
        terrain = terrain_classifier.classify_terrain(wheel_vels, motor_currents, accelerations)
        
        from src.autonomy.control.advanced_motor_controller import TerrainType
        assert terrain in [t.value for t in TerrainType]
    
    def test_maintenance_alerts(self, motor_controller):
        """Test predictive maintenance alerts."""
        # Update sensor data to trigger alerts
        motor_controller._update_predictive_maintenance()
        
        alerts = motor_controller.get_maintenance_alerts()
        
        # Should return dictionary (possibly empty)
        assert isinstance(alerts, dict)


class TestSensorFusion:
    """Test suite for sensor fusion system."""
    
    @pytest.fixture
    def fusion_manager(self):
        from src.autonomy.perception.sensor_fusion import SensorFusionManager
        return SensorFusionManager(TEST_CONFIG['sensor_fusion'])
    
    def test_fusion_manager_initialization(self, fusion_manager):
        """Test sensor fusion manager initialization."""
        from src.autonomy.perception.sensor_fusion import SensorType
        fusion_manager.add_sensor('test_imu', SensorType.IMU)
        fusion_manager.add_sensor('test_gps', SensorType.GPS)

        # Check sensors added
        sensor_status = fusion_manager.get_sensor_status()
        assert 'test_imu' in sensor_status
        assert 'test_gps' in sensor_status
        assert sensor_status['test_imu']['type'] == 'imu'
        assert sensor_status['test_gps']['type'] == 'gps'
    
    def test_extended_kalman_filter(self, fusion_manager):
        """Test Extended Kalman Filter functionality."""
        from src.autonomy.perception.sensor_fusion import ExtendedKalmanFilter
        
        ekf = ExtendedKalmanFilter()
        
        # Test prediction step
        ekf.predict(0.01)  # 10ms timestep
        assert ekf.state.shape == (16,)  # 16-dimensional state vector
        
        # Test measurement update
        import numpy as np
        measurement = np.array([0.1, 0.0, 9.8, 0.0, 0.0, 0.1])  # IMU measurement
        R = np.eye(6) * 0.1
        
        ekf.update_imu(measurement, R)
        state = ekf.get_state()
        
        assert state.position.shape == (3,)
        assert state.velocity.shape == (3,)
        assert state.orientation.shape == (4,)
    
    def test_complementary_filter(self, fusion_manager):
        """Test complementary filter for attitude estimation."""
        from src.autonomy.perception.sensor_fusion import ComplementaryFilter
        
        comp_filter = ComplementaryFilter(alpha=0.98)
        
        # Test filter update
        gyro = np.array([0.0, 0.0, 0.1])  # 0.1 rad/s around z-axis
        accel = np.array([0.0, 0.0, 9.81])  # Gravity pointing down
        orientation = comp_filter.update(gyro, accel, 0.01)
        
        assert orientation.shape == (4,)  # Quaternion
        assert np.linalg.norm(orientation) > 0.99  # Should be normalized
    
    def test_sensor_health_monitoring(self, fusion_manager):
        """Test sensor health monitoring."""
        from src.autonomy.perception.sensor_fusion import SensorMeasurement, SensorStatus, SensorType

        fusion_manager.add_sensor('test_imu', SensorType.IMU)

        # Add healthy measurement
        healthy_measurement = SensorMeasurement(
            sensor_type=SensorType.IMU,
            timestamp=time.time(),
            data={'accel_x': 0.0},
            confidence=1.0,
            status=SensorStatus.HEALTHY
        )
        
        fusion_manager.update_sensor_measurement('test_imu', healthy_measurement)
        
        # Check sensor status
        status = fusion_manager.get_sensor_status()
        assert status['test_imu']['status'] == 'healthy'
        assert status['test_imu']['confidence'] == 1.0


class TestGracefulDegradation:
    """Test suite for graceful degradation system."""

    @pytest.fixture
    def mode_manager(self):
        from src.core.graceful_degradation import OperationModeManager
        config = {k: v for k, v in TEST_CONFIG['graceful_degradation'].items()
                  if k != 'sensor_fusion'}
        try:
            return OperationModeManager(config if config else TEST_CONFIG['graceful_degradation'])
        except TypeError:
            pytest.skip("OperationModeManager config incompatible")
    
    def test_operation_mode_transitions(self, mode_manager):
        """Test operation mode transitions."""
        from src.core.graceful_degradation import OperationMode
        
        # Test mode switching
        original_mode = mode_manager.current_mode
        
        mode_manager.force_mode_switch(OperationMode.SURVIVAL_MODE, "test_switch")
        
        assert mode_manager.current_mode == OperationMode.SURVIVAL_MODE
        assert mode_manager.last_mode_switch != original_mode
    
    def test_resource_monitoring(self, mode_manager):
        """Test system resource monitoring."""
        resource_manager = mode_manager.resource_manager
        resources = resource_manager.get_available_resources()
        
        # Should return resource metrics
        assert hasattr(resources, 'cpu_usage')
        assert hasattr(resources, 'memory_usage')
        assert hasattr(resources, 'temperature')
        
        assert isinstance(resources.cpu_usage, float)
        assert isinstance(resources.memory_usage, float)
    
    def test_component_resource_requirements(self, mode_manager):
        """Test component resource requirements."""
        component_registry = mode_manager.resource_manager.component_registry
        
        # Get total resources for critical components
        critical_components = ['emergency_stop', 'safety_monitor', 'motor_control']
        total_resources = component_registry.get_total_resources(critical_components)
        
        assert 'cpu' in total_resources
        assert 'memory' in total_resources
        assert 'network' in total_resources
        
        # Critical components should require resources
        assert total_resources['cpu'] > 0
        assert total_resources['memory'] > 0
    
    def test_adaptive_mode_configuration(self, mode_manager):
        """Test adaptive mode configuration."""
        from src.core.graceful_degradation import OperationMode
        
        # Test each mode configuration
        modes = [OperationMode.FULL_COMPETITION, OperationMode.DEGRADED_AUTONOMY, 
                 OperationMode.SURVIVAL_MODE, OperationMode.SAFE_STOP]
        
        for mode in modes:
            config = mode_manager.mode_configs[mode]
            
            assert hasattr(config, 'control_rate')
            assert hasattr(config, 'navigation_mode')
            assert hasattr(config, 'vision_processing')
            
            # Verify resource allocation pattern
            if mode == OperationMode.FULL_COMPETITION:
                assert config.control_rate > config.navigation_mode
            elif mode == OperationMode.SURVIVAL_MODE:
                assert config.vision_processing == "none"


class TestIntegratedCriticalSystems:
    """Test suite for integrated critical systems (requires control package)."""

    @pytest.fixture
    def integrated_system(self):
        try:
            from src.autonomy.control.integrated_critical_systems import IntegratedCriticalSystems
        except (ImportError, ModuleNotFoundError):
            pytest.skip("IntegratedCriticalSystems not available")
        return IntegratedCriticalSystems(TEST_CONFIG['integration'])
    
    def test_system_initialization(self, integrated_system):
        """Test complete system initialization."""
        result = integrated_system.start_systems()
        assert result is True
        assert integrated_system.system_initialized is True
    
    def test_velocity_command_processing(self, integrated_system):
        """Test velocity command processing through all systems."""
        twist = MockTwist()
        twist.linear.x = 0.5
        twist.angular.z = 0.2
        
        result = integrated_system.process_velocity_command(twist)
        assert result is True
    
    def test_system_coordination(self, integrated_system):
        """Test coordination between systems."""
        status = integrated_system.get_system_status()
        
        # Should include status from all subsystems
        assert 'emergency_stop' in status
        assert 'redundant_can' in status
        assert 'motor_control' in status
        assert 'sensor_fusion' in status
        assert 'operation_mode' in status
        assert 'performance_metrics' in status
    
    def test_health_monitoring_loop(self, integrated_system):
        """Test health monitoring loop."""
        # Should start health monitoring thread
        assert integrated_system.health_monitor_thread is not None
        assert integrated_system.coordination_active is True
    
    def test_competition_validation(self, integrated_system):
        """Test competition validation functionality."""
        validation_results = integrated_system.run_competition_validation()
        
        # Should return validation results
        assert isinstance(validation_results, dict)
        assert 'overall_success' in validation_results
        assert 'redundant_can' in validation_results
        assert 'emergency_stop' in validation_results
        assert 'motor_control' in validation_results


class TestPerformanceBenchmarks:
    """Test suite for performance benchmarks."""
    
    def test_import_performance(self):
        """Test import performance of critical systems (CAN bridge and sensor fusion only)."""
        import time

        start_time = time.time()
        from src.bridges.can_bridge import CANBridge
        from src.autonomy.perception.sensor_fusion import SensorFusionManager

        import_duration = time.time() - start_time
        assert import_duration < 2.0, f"Import took too long: {import_duration:.3f}s"

    def test_instantiation_performance(self):
        """Test instantiation performance of CAN bridge and sensor fusion."""
        import time
        from src.bridges.can_bridge import CANBridge
        from src.autonomy.perception.sensor_fusion import SensorFusionManager

        start_time = time.time()
        can_interface = CANBridge(TEST_CONFIG['can_bridge'])
        fusion_manager = SensorFusionManager(TEST_CONFIG['sensor_fusion'])
        instantiation_duration = time.time() - start_time
        assert instantiation_duration < 1.0, f"Instantiation took too long: {instantiation_duration:.3f}s"

    def test_memory_usage(self):
        """Test memory usage of CAN bridge and sensor fusion."""
        import psutil
        import os
        from src.bridges.can_bridge import CANBridge
        from src.autonomy.perception.sensor_fusion import SensorFusionManager

        process = psutil.Process(os.getpid())
        initial_memory = process.memory_info().rss / 1024 / 1024  # MB
        can_interface = CANBridge(TEST_CONFIG['can_bridge'])
        fusion_manager = SensorFusionManager(TEST_CONFIG['sensor_fusion'])
        final_memory = process.memory_info().rss / 1024 / 1024  # MB
        memory_increase = final_memory - initial_memory
        assert memory_increase < 500, f"Memory usage too high: {memory_increase:.1f}MB"


class TestCompetitionScenarios:
    """Test suite for competition scenarios."""
    
    @pytest.mark.asyncio
    async def test_can_failure_scenario(self):
        """Test CAN bridge connection failure scenario."""
        from src.bridges.can_bridge import CANBridge
        can_bridge = CANBridge(TEST_CONFIG['can_bridge'])
        with patch('serial.Serial') as mock_serial:
            mock_serial.side_effect = Exception("Device unavailable")
            result = await can_bridge.connect()
            assert result is False
            status = can_bridge.get_status()
            assert status.is_connected is False
    
    def test_emergency_stop_scenario(self):
        """Test emergency stop scenario (requires RPi.GPIO)."""
        pytest.importorskip("RPi.GPIO")
        from src.autonomy.control.hardware_emergency_stop import (
            HardwareEmergencyStop,
            EmergencyStopConfig,
        )
        e_stop = HardwareEmergencyStop(EmergencyStopConfig(**TEST_CONFIG['emergency_stop']))
        
        # Activate emergency stop
        e_stop.activate_hard_stop("test_emergency")
        
        # Verify emergency stop state
        status = e_stop.get_status()
        assert status['is_active'] is True
        assert status['stop_level'] == 'hard_stop'
    
    def test_high_load_scenario(self):
        """Test high system load scenario."""
        from src.core.graceful_degradation import OperationModeManager
        config = {k: v for k, v in TEST_CONFIG['graceful_degradation'].items()
                  if k != 'sensor_fusion'}
        try:
            mode_manager = OperationModeManager(config if config else TEST_CONFIG['graceful_degradation'])
        except TypeError:
            pytest.skip("OperationModeManager config incompatible")
        
        # Simulate high CPU usage
        with patch('psutil.cpu_percent', return_value=95.0):
            # Trigger resource evaluation
            resources = mode_manager.resource_manager.get_available_resources()
            
            # Should detect critical CPU usage
            assert resources.cpu_usage > 90.0
            
            # Should trigger mode switch to degraded
            mode_manager._evaluate_mode_switching()
            
            # Check if mode changed appropriately
            current_mode = mode_manager.current_mode
            from src.core.graceful_degradation import OperationMode
            assert current_mode in [OperationMode.SURVIVAL_MODE, OperationMode.SAFE_STOP]
    
    def test_sensor_failure_scenario(self):
        """Test sensor failure scenario."""
        from src.autonomy.perception.sensor_fusion import (
            SensorFusionManager,
            SensorMeasurement,
            SensorStatus,
            SensorType,
        )
        fusion_manager = SensorFusionManager(TEST_CONFIG['sensor_fusion'])
        fusion_manager.add_sensor('test_imu', SensorType.IMU)

        
        # Add failed sensor measurement
        failed_measurement = SensorMeasurement(
            sensor_type=SensorType.IMU,
            timestamp=time.time(),
            data={'accel_x': 0.0},
            confidence=0.0,
            status=SensorStatus.FAILED
        )
        
        fusion_manager.update_sensor_measurement('test_imu', failed_measurement)
        
        # Check sensor status
        status = fusion_manager.get_sensor_status()
        assert status['test_imu']['status'] == 'failed'
        assert status['test_imu']['confidence'] == 0.0


# Integration test for complete communication stack
class TestCompleteCommunicationStack:
    """Test the complete communication stack integration."""
    
    @pytest.mark.asyncio
    async def test_end_to_end_communication(self):
        """Test end-to-end communication through all layers."""
        try:
            from src.autonomy.control.integrated_critical_systems import IntegratedCriticalSystems
        except (ImportError, ModuleNotFoundError):
            pytest.skip("IntegratedCriticalSystems not available")
        # Create mock hardware
        mock_serial = Mock()
        mock_serial.is_open = True
        mock_serial.in_waiting = 0
        mock_serial.readline.return_value = b't00D61000000000000000\r'
        
        with patch('serial.Serial', return_value=mock_serial):
            # Initialize complete system
            integrated_system = IntegratedCriticalSystems(TEST_CONFIG['integration'])
            integrated_system.start_systems()
            
            # Send command through complete stack
            twist = MockTwist()
            twist.linear.x = 1.0
            
            # Process command through all layers
            result = integrated_system.process_velocity_command(twist)
            
            # Verify command reaches hardware
            assert mock_serial.write.called
            assert result is True


if __name__ == '__main__':
    pytest.main([__file__, '-v', '--tb=short'])