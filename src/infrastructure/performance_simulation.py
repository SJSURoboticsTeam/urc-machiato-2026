#!/usr/bin/env python3
"""
Performance-Based Simulation Environment

High-fidelity simulation for URC 2026 based on real hardware characteristics.
Analogous sensor modeling with 1kHz physics engine for performance testing.

Features:
- Real-world hardware modeling based on STM32/Camera specifications
- Performance-based physics (1kHz updates)
- Analogous sensor noise and latency characteristics
- Failure injection framework for robustness testing
- Zero-copy communication patterns

Author: URC 2026 Simulation Team
"""

import time
import threading
import numpy as np
import random
from typing import Dict, Any, Optional, List, Callable
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
import rclpy.node as Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu, NavSatFix, Image, JointState
from std_msgs.msg import Header
from infrastructure.logging import get_logger, get_performance_logger
from infrastructure.validation import validate_input


class SimulationPerformance(Enum):
    """Simulation performance levels."""
    FAST = "fast"           # Fast physics, lower fidelity
    BALANCED = "balanced"     # Balanced performance vs. accuracy
    REALISTIC = "realistic"    # High fidelity, real-time constraints


@dataclass
class HardwareSpecs:
    """Real hardware specifications for accurate simulation."""
    # STM32 Specifications
    stm32_mhz: int = 168
    stm32_ram_mb: int = 16
    can_frequency_hz: int = 1000000
    can_max_latency_us: int = 500
    control_loop_hz: int = 100
    
    # Camera Specifications (Raspberry Pi)
    camera_resolution: tuple = (1920, 1080)
    camera_fps: int = 30
    camera_focal_length: float = 3.6  # mm
    camera_sensor_size: tuple = (5.67, 4.76)  # μm
    
    # Sensor Characteristics
    imu_noise_deg_per_sqrt_hz: float = 0.1
    gps_accuracy_m: float = 2.0
    lidar_range_m: float = 30.0
    wheel_diameter_m: float = 0.3
    wheel_base_m: float = 0.5


@dataclass
class NoiseModel:
    """Analogous noise modeling for sensors."""
    mean: float = 0.0
    std_dev: float = 1.0
    drift_rate: float = 0.001
    bias: float = 0.0
    dropout_rate: float = 0.001
    latency_mean_us: float = 100.0
    latency_std_us: float = 20.0


class AnalogousSensor:
    """Analogous sensor modeling based on real hardware."""
    
    def __init__(self, hardware_specs: HardwareSpecs):
        self.specs = hardware_specs
        self.logger = get_logger("analogous_sensor")
        
        # Initialize noise models
        self._initialize_noise_models()
        
        # Sensor state
        self.true_values = {}
        self.measured_values = {}
        self.last_update_time = time.time()
        self.update_frequency = 100.0  # Default 100Hz
    
    def _initialize_noise_models(self):
        """Initialize noise models based on hardware specs."""
        self.noise_models = {
            'imu': NoiseModel(
                mean=0.0,
                std_dev=self.specs.imu_noise_deg_per_sqrt_hz,
                drift_rate=0.001,  # 0.001 deg/s drift
                bias=0.05,  # Small gyroscope bias
                dropout_rate=0.0001,  # Very reliable IMU
                latency_mean_us=150.0,
                latency_std_us=30.0
            ),
            'gps': NoiseModel(
                mean=0.0,
                std_dev=self.specs.gps_accuracy_m,
                drift_rate=0.1,  # 0.1m position drift
                bias=0.0,
                dropout_rate=0.01,  # Occasional GPS loss
                latency_mean_us=500.0,  # GPS is slower
                latency_std_us=100.0
            ),
            'lidar': NoiseModel(
                mean=0.0,
                std_dev=0.05,  # 5cm range noise
                drift_rate=0.01,
                bias=0.1,  # Range bias
                dropout_rate=0.0005,
                latency_mean_us=100.0,
                latency_std_us=20.0
            ),
            'camera': NoiseModel(
                mean=0.0,
                std_dev=0.02,  # Small camera noise
                drift_rate=0.001,
                bias=0.0,
                dropout_rate=0.0001,  # Very reliable
                latency_mean_us=50.0,  # Fast camera
                latency_std_us=10.0
            ),
            'motor': NoiseModel(
                mean=0.0,
                std_dev=0.01,  # Motor control noise
                drift_rate=0.005,
                bias=0.0,
                dropout_rate=0.00001,  # Very reliable CAN bus
                latency_mean_us=self.specs.can_max_latency_us,
                latency_std_us=50.0
            )
        }
    
    def update_true_value(self, sensor_type: str, value: Any):
        """Update the true value of a sensor."""
        self.true_values[sensor_type] = value
        self._apply_drift(sensor_type)
    
    def _apply_drift(self, sensor_type: str):
        """Apply drift to true value over time."""
        if sensor_type not in self.true_values:
            return
        
        current_time = time.time()
        dt = current_time - self.last_update_time
        
        noise_model = self.noise_models.get(sensor_type)
        if noise_model and dt > 0:
            # Apply drift
            drift = noise_model.drift_rate * dt
            if isinstance(self.true_values[sensor_type], (list, tuple, np.ndarray)):
                # Apply drift to each element
                if isinstance(self.true_values[sensor_type], np.ndarray):
                    self.true_values[sensor_type] += drift
                else:
                    self.true_values[sensor_type] = [
                        val + drift for val in self.true_values[sensor_type]
                    ]
            else:
                self.true_values[sensor_type] += drift
        
        self.last_update_time = current_time
    
    def get_measured_value(self, sensor_type: str) -> Any:
        """Get measured value with realistic noise and latency."""
        true_value = self.true_values.get(sensor_type)
        if true_value is None:
            return None
        
        noise_model = self.noise_models.get(sensor_type)
        if not noise_model:
            return true_value
        
        # Simulate latency
        latency_samples = max(1, int(noise_model.latency_mean_us / 1000))
        measured_value = true_value
        
        for _ in range(latency_samples):
            # Add noise
            noise = np.random.normal(noise_model.mean, noise_model.std_dev)
            
            if isinstance(measured_value, np.ndarray):
                measured_value = measured_value + noise
            elif isinstance(measured_value, list):
                measured_value = [val + noise for val in measured_value]
            else:
                measured_value = true_value + noise
            
            time.sleep(0.001)  # 1ms per sample for latency
        
        # Apply dropout
        if random.random() < noise_model.dropout_rate:
            self.logger.warning(f"Simulated dropout for {sensor_type}")
            return None  # Simulate signal loss
        
        # Apply bias
        if isinstance(measured_value, (list, np.ndarray)):
            if isinstance(measured_value, np.ndarray):
                measured_value += noise_model.bias
            else:
                measured_value = [val + noise_model.bias for val in measured_value]
        else:
            measured_value += noise_model.bias
        
        return measured_value


class PerformancePhysicsEngine:
    """1kHz physics engine for performance-based simulation."""
    
    def __init__(self, performance_level: SimulationPerformance = SimulationPerformance.BALANCED):
        self.performance_level = performance_level
        self.logger = get_logger("physics_engine")
        self.perf_logger = get_performance_logger("physics_engine")
        
        # Set physics timestep based on performance level
        self.timestep = self._get_timestep()
        self.last_update_time = time.time()
        
        # Physics state
        self.world_state = {
            'rover_position': np.array([0.0, 0.0, 0.0]),
            'rover_orientation': np.array([0.0, 0.0, 0.0, 1.0]),  # Quaternion
            'rover_velocity': np.array([0.0, 0.0, 0.0]),
            'wheel_positions': [0.0, 0.0],  # Left, right
            'wheel_velocities': [0.0, 0.0],
            'battery_voltage': 12.6,
            'motor_currents': [0.5, 0.5]
        }
        
        self.update_frequency = 1000.0  # 1kHz physics updates
        self.performance_stats = {
            'physics_frequency': 0.0,
            'frame_time_ms': 0.0,
            'max_frame_time_ms': 0.0
        }
    
    def _get_timestep(self) -> float:
        """Get physics timestep based on performance level."""
        timesteps = {
            SimulationPerformance.FAST: 0.002,      # 500Hz physics
            SimulationPerformance.BALANCED: 0.001,   # 1kHz physics
            SimulationPerformance.REALISTIC: 0.0005   # 2kHz physics
        }
        return timesteps[self.performance_level]
    
    def update_physics(self, dt: float):
        """Update physics simulation."""
        with self.perf_logger:
            self._update_rover_kinematics(dt)
            self._update_wheel_dynamics(dt)
            self._update_battery_model(dt)
            self._track_performance(dt)
    
    def _update_rover_kinematics(self, dt: float):
        """Update rover position and orientation."""
        # Simple differential drive kinematics
        left_vel, right_vel = self.world_state['wheel_velocities']
        wheel_radius = 0.15  # meters
        wheel_base = 0.5  # meters
        
        # Linear and angular velocities
        linear_vel = (left_vel + right_vel) * wheel_radius / 2.0
        angular_vel = (right_vel - left_vel) * wheel_radius / wheel_base
        
        # Update position
        orientation = self.world_state['rover_orientation']
        
        # Create rotation matrix from quaternion
        qw, qx, qy, qz = orientation
        
        # Angular velocity update
        angle_change = angular_vel * dt
        q_change = np.array([
            0,
            0,
            np.sin(angle_change / 2),
            np.cos(angle_change / 2)
        ])
        
        # Update orientation
        new_orientation = self._quaternion_multiply(orientation, q_change)
        new_orientation = new_orientation / np.linalg.norm(new_orientation)
        
        # Update position
        # Extract heading from quaternion
        heading = np.arctan2(2 * (qw*qz + qx*qy), 1 - 2 * (qy*qy + qw*qw))
        
        position_change = linear_vel * dt * np.array([
            np.cos(heading),
            np.sin(heading),
            0
        ])
        
        self.world_state['rover_position'] += position_change
        self.world_state['rover_orientation'] = new_orientation
        self.world_state['rover_velocity'] = np.array([
            linear_vel * np.cos(heading),
            linear_vel * np.sin(heading),
            0
        ])
    
    def _update_wheel_dynamics(self, dt: float):
        """Update wheel positions based on velocities."""
        left_vel, right_vel = self.world_state['wheel_velocities']
        wheel_radius = 0.15
        
        # Update positions (simplified - would use proper kinematics)
        self.world_state['wheel_positions'][0] += left_vel * dt * wheel_radius
        self.world_state['wheel_positions'][1] += right_vel * dt * wheel_radius
    
    def _update_battery_model(self, dt: float):
        """Update battery voltage based on motor currents."""
        left_current, right_current = self.world_state['motor_currents']
        total_current = left_current + right_current
        
        # Simple battery discharge model
        voltage_drop = total_current * 0.01 * dt  # Simplified model
        self.world_state['battery_voltage'] -= voltage_drop
        
        # Clamp to realistic range
        self.world_state['battery_voltage'] = max(10.0, self.world_state['battery_voltage'])
    
    def _quaternion_multiply(self, q1, q2):
        """Multiply two quaternions."""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ])
    
    def _track_performance(self, dt: float):
        """Track physics engine performance."""
        frame_time_ms = dt * 1000
        
        self.performance_stats['frame_time_ms'] = frame_time_ms
        self.performance_stats['max_frame_time_ms'] = max(
            self.performance_stats['max_frame_time_ms'], frame_time_ms
        )
        
        # Calculate actual frequency
        current_time = time.time()
        time_diff = current_time - self.last_update_time
        if time_diff > 0:
            self.performance_stats['physics_frequency'] = 1.0 / time_diff
        
        self.last_update_time = current_time
        
        # Log performance warnings
        if frame_time_ms > 10.0:  # 10ms threshold
            self.logger.warning(
                f"Physics frame time exceeded: {frame_time_ms:.2f}ms",
                extra={'frame_time_ms': frame_time_ms}
            )
    
    def get_world_state(self) -> Dict[str, Any]:
        """Get complete world state."""
        return {
            **self.world_state,
            'performance_stats': self.performance_stats,
            'timestep': self.timestep,
            'target_frequency': self.update_frequency
        }


class FailureInjectionFramework:
    """Failure injection for robustness testing."""
    
    def __init__(self):
        self.logger = get_logger("failure_injection")
        self.active_failures = {}
        self.failure_scenarios = {
            'sensor_dropout': {
                'sensors': ['imu', 'gps', 'camera'],
                'duration_range': (0.1, 5.0),
                'recovery_time': 2.0
            },
            'communication_loss': {
                'duration_range': (1.0, 10.0),
                'packet_loss_rate': 0.1,
                'recovery_time': 5.0
            },
            'motor_stall': {
                'motors': ['left', 'right'],
                'stall_current': 2.0,  # 2A indicates stall
                'recovery_time': 3.0
            },
            'getting_stuck': {
                'min_velocity': 0.01,  # m/s
                'timeout': 30.0,
                'oscillation_threshold': 0.05
            }
        }
    
    def inject_failure(self, scenario: str, params: Dict[str, Any] = None) -> bool:
        """Inject a specific failure scenario."""
        if scenario not in self.failure_scenarios:
            self.logger.error(f"Unknown failure scenario: {scenario}")
            return False
        
        self.active_failures[scenario] = {
            'start_time': time.time(),
            'params': params or {},
            'active': True
        }
        
        self.logger.warning(
            f"Failure injection activated: {scenario}",
            extra={'scenario': scenario, 'params': params}
        )
        
        return True
    
    def get_injected_failures(self) -> Dict[str, Any]:
        """Get all active failure injections."""
        return self.active_failures.copy()
    
    def clear_failure(self, scenario: str):
        """Clear a specific failure scenario."""
        if scenario in self.active_failures:
            self.active_failures[scenario]['active'] = False
            self.logger.info(f"Failure injection cleared: {scenario}")
    
    def apply_failure_effects(self, sensors: AnalogousSensor, physics_engine: PerformancePhysicsEngine):
        """Apply active failure effects to simulation."""
        current_time = time.time()
        
        for scenario, failure in self.active_failures.items():
            if not failure['active']:
                continue
            
            age = current_time - failure['start_time']
            scenario_config = self.failure_scenarios[scenario]
            
            # Apply scenario-specific effects
            if scenario == 'sensor_dropout':
                self._apply_sensor_dropout(sensors, failure, scenario_config, age)
            elif scenario == 'communication_loss':
                self._apply_communication_loss(sensors, failure, scenario_config, age)
            elif scenario == 'motor_stall':
                self._apply_motor_stall(physics_engine, failure, scenario_config, age)
            elif scenario == 'getting_stuck':
                self._apply_getting_stuck(physics_engine, failure, scenario_config, age)
    
    def _apply_sensor_dropout(self, sensors: AnalogousSensor, failure, config, age):
        """Apply sensor dropout effects."""
        duration = min(age, config['duration_range'][1])
        
        if age < duration:
            # Randomly drop sensors
            for sensor in config['sensors']:
                if random.random() < 0.1:  # 10% dropout chance
                    sensors.true_values[sensor] = None
                    self.logger.warning(f"Sensor dropout: {sensor}")
        
        # Auto-recovery after timeout
        if age > config['recovery_time']:
            self.clear_failure('sensor_dropout')
    
    def _apply_communication_loss(self, sensors: AnalogousSensor, failure, config, age):
        """Apply communication loss effects."""
        duration = min(age, config['duration_range'][1])
        packet_loss_rate = config['packet_loss_rate']
        
        if age < duration:
            # Simulate packet loss
            for sensor_type in ['imu', 'gps', 'camera']:
                if random.random() < packet_loss_rate:
                    # Introduce latency spikes
                    sensors.noise_models[sensor_type].latency_mean_us *= 10.0
                    self.logger.warning(f"Communication loss: {sensor_type}")
        
        # Recovery after timeout
        if age > config['recovery_time']:
            # Reset latency
            sensors._initialize_noise_models()
            self.clear_failure('communication_loss')
    
    def _apply_motor_stall(self, physics_engine: PerformancePhysicsEngine, failure, config, age):
        """Apply motor stall effects."""
        duration = min(age, config['duration_range'][1])
        
        if age < duration:
            stall_current = config['stall_current']
            # Simulate motor stall
            physics_engine.world_state['motor_currents'] = [
                stall_current if motor in config['motors'] else 0.5
                for motor in ['left', 'right']
            ]
            
            # Stop movement
            physics_engine.world_state['wheel_velocities'] = [0.0, 0.0]
        
        # Recovery after timeout
        if age > config['recovery_time']:
            self.clear_failure('motor_stall')
    
    def _apply_getting_stuck(self, physics_engine: PerformancePhysicsEngine, failure, config, age):
        """Apply getting stuck effects."""
        velocity_magnitude = np.linalg.norm(physics_engine.world_state['rover_velocity'])
        
        if velocity_magnitude < config['min_velocity'] and age > config['timeout']:
            # Rover is stuck
            physics_engine.world_state['wheel_velocities'] = [
                random.uniform(-0.1, 0.1),  # Oscillation
                random.uniform(-0.1, 0.1)
            ]
            
            self.logger.warning("Rover stuck - applying oscillation")


# Simulation node wrapper
class PerformanceSimulationNode(Node):
    """High-performance simulation node."""
    
    def __init__(self, performance_level: SimulationPerformance = SimulationPerformance.BALANCED):
        super().__init__('performance_simulation')
        
        self.hardware_specs = HardwareSpecs()
        self.sensors = AnalogousSensor(self.hardware_specs)
        self.physics_engine = PerformancePhysicsEngine(performance_level)
        self.failure_injection = FailureInjectionFramework()
        
        # ROS2 publishers
        self.imu_publisher = self.create_publisher(Imu, '/imu/data', 10)
        self.gps_publisher = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.camera_publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.joint_publisher = self.create_publisher(JointState, '/joint_states', 10)
        
        # Simulation control
        self.simulation_timer = self.create_timer(
            1.0 / 1000.0,  # 1kHz physics updates
            self._simulation_step
        )
        
        # Status publishing
        self.status_timer = self.create_timer(
            0.1,  # 10Hz status publishing
            self._publish_status
        )
        
        # Command subscription
        self.cmd_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self._cmd_vel_callback,
            10
        )
        
        self.logger = get_logger("performance_simulation")
        self.logger.info(f"Performance simulation started: {performance_level.value}")
    
    def _simulation_step(self):
        """Single simulation step."""
        dt = self.physics_engine.timestep
        
        # Update physics
        self.physics_engine.update_physics(dt)
        
        # Apply failure injection effects
        self.failure_injection.apply_failure_effects(self.sensors, self.physics_engine)
        
        # Update sensor true values
        world_state = self.physics_engine.get_world_state()
        self.sensors.update_true_value('imu', world_state['rover_orientation'])
        self.sensors.update_true_value('gps', world_state['rover_position'][:2])
        self.sensors.update_true_value('wheel_position', world_state['wheel_positions'])
        self.sensors.update_true_value('wheel_velocity', world_state['wheel_velocities'])
    
    def _publish_status(self):
        """Publish simulation status."""
        world_state = self.physics_engine.get_world_state()
        active_failures = self.failure_injection.get_injected_failures()
        
        self.get_logger().info(
            "Simulation status",
            extra={
                'fps': world_state['performance_stats']['physics_frequency'],
                'frame_time_ms': world_state['performance_stats']['frame_time_ms'],
                'active_failures': len(active_failures)
            }
        )
    
    def _cmd_vel_callback(self, msg: Twist):
        """Handle velocity commands."""
        # Convert to wheel velocities (simplified differential drive)
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        
        wheel_radius = 0.15
        wheel_base = 0.5
        
        left_wheel_vel = (linear_vel - angular_vel * wheel_base / 2) / wheel_radius
        right_wheel_vel = (linear_vel + angular_vel * wheel_base / 2) / wheel_radius
        
        self.sensors.update_true_value('wheel_velocity', [left_wheel_vel, right_wheel_vel])
    
    def inject_failure(self, scenario: str, params: Dict[str, Any] = None):
        """API for failure injection."""
        return self.failure_injection.inject_failure(scenario, params)


if __name__ == "__main__":
    # Demo performance simulation
    print("🚀 Performance-Based Simulation Demo")
    
    # Test different performance levels
    for level in [SimulationPerformance.FAST, SimulationPerformance.BALANCED, SimulationPerformance.REALISTIC]:
        print(f"\nTesting {level.value} performance...")
        sim = PerformanceSimulationNode(level)
        
        # Test failure injection
        sim.inject_failure('sensor_dropout', {'sensors': ['imu', 'gps']})
        
        print(f"Physics engine: {sim.physics_engine.timestep}s timestep")
        print(f"Target frequency: {sim.physics_engine.update_frequency}Hz")
        
        time.sleep(1.0)
        
        # Clear failure
        sim.failure_injection.clear_failure('sensor_dropout')
        print("Failure cleared")
    
    print("\n🎯 Performance simulation complete!")