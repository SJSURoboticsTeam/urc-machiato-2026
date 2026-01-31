#!/usr/bin/env python3
"""
Multi-Sensor Fusion Validation Test

Test multi-sensor fusion data validation, dropout handling, 
and conflict resolution between GPS, IMU, odometry, and vision.

Author: URC 2026 Perception Systems Team
"""

import asyncio
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Any
from unittest.mock import Mock, patch, AsyncMock
import pytest
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Header
from sensor_msgs.msg import Imu, NavSatFix, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance


# Import sensor fusion components
try:
    from src.autonomy.perception.sensor_fusion import SensorFusion
    from src.autonomy.perception.gps_processor import GPSProcessor
    from src.autonomy.perception.imu_processor import IMUProcessor
    from src.autonomy.perception.odometry_processor import OdometryProcessor
except ImportError as e:
    pytest.skip(f"Skipping sensor fusion tests due to import error: {e}", allow_module_level=True)


@dataclass
class SensorReading:
    """Individual sensor reading with metadata"""
    sensor_type: str
    timestamp: float
    data: Dict[str, Any]
    confidence: float = 1.0
    covariance: Optional[np.ndarray] = None
    
    def is_valid(self) -> bool:
        """Check if sensor reading is valid"""
        if self.confidence < 0.1:
            return False
        if self.data is None:
            return False
        return True


@dataclass
class FusionResult:
    """Result of sensor fusion process"""
    position: np.ndarray
    orientation: np.ndarray
    velocity: np.ndarray
    timestamp: float
    confidence: float
    contributing_sensors: List[str]
    fusion_quality: float
    outliers: List[str] = field(default_factory=list)
    
    @property
    def position_covariance(self) -> np.ndarray:
        return np.eye(3) * (1.0 - self.confidence)
        
    @property
    def orientation_covariance(self) -> np.ndarray:
        return np.eye(4) * (1.0 - self.confidence)


class MockGPSPublisher(Node):
    """Mock GPS sensor publisher"""
    
    def __init__(self):
        super().__init__('mock_gps_publisher')
        self.publisher = self.create_publisher(NavSatFix, '/gps/fix', 10)
        
    def publish_gps_data(self, latitude: float, longitude: float, altitude: float, 
                        confidence: float = 1.0):
        """Publish GPS data"""
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gps_link"
        msg.latitude = latitude
        msg.longitude = longitude
        msg.altitude = altitude
        
        # Set status based on confidence
        if confidence > 0.8:
            msg.status.status = NavSatFix.STATUS_FIX
        elif confidence > 0.5:
            msg.status.status = NavSatFix.STATUS_FIX
        else:
            msg.status.status = NavSatFix.STATUS_NO_FIX
            
        self.publisher.publish(msg)


class MockIMUPublisher(Node):
    """Mock IMU sensor publisher"""
    
    def __init__(self):
        super().__init__('mock_imu_publisher')
        self.publisher = self.create_publisher(Imu, '/imu/data', 10)
        
    def publish_imu_data(self, linear_accel: np.ndarray, angular_vel: np.ndarray,
                        orientation: Optional[np.ndarray] = None):
        """Publish IMU data"""
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"
        
        msg.linear_acceleration.x = linear_accel[0]
        msg.linear_acceleration.y = linear_accel[1]
        msg.linear_acceleration.z = linear_accel[2]
        
        msg.angular_velocity.x = angular_vel[0]
        msg.angular_velocity.y = angular_vel[1]
        msg.angular_velocity.z = angular_vel[2]
        
        if orientation is not None:
            msg.orientation.x = orientation[0]
            msg.orientation.y = orientation[1]
            msg.orientation.z = orientation[2]
            msg.orientation.w = orientation[3]
            
        self.publisher.publish(msg)


class MockOdometryPublisher(Node):
    """Mock odometry sensor publisher"""
    
    def __init__(self):
        super().__init__('mock_odometry_publisher')
        self.publisher = self.create_publisher(Odometry, '/odom', 10)
        
    def publish_odometry_data(self, position: np.ndarray, orientation: np.ndarray,
                            linear_vel: np.ndarray, angular_vel: np.ndarray):
        """Publish odometry data"""
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        
        # Position
        msg.pose.pose.position.x = position[0]
        msg.pose.pose.position.y = position[1]
        msg.pose.pose.position.z = position[2]
        
        # Orientation
        msg.pose.pose.orientation.x = orientation[0]
        msg.pose.pose.orientation.y = orientation[1]
        msg.pose.pose.orientation.z = orientation[2]
        msg.pose.pose.orientation.w = orientation[3]
        
        # Velocity
        msg.twist.twist.linear.x = linear_vel[0]
        msg.twist.twist.linear.y = linear_vel[1]
        msg.twist.twist.linear.z = linear_vel[2]
        
        msg.twist.twist.angular.x = angular_vel[0]
        msg.twist.twist.angular.y = angular_vel[1]
        msg.twist.twist.angular.z = angular_vel[2]
        
        self.publisher.publish(msg)


class MockSensorFusion(Node):
    """Mock sensor fusion system"""
    
    def __init__(self):
        super().__init__('mock_sensor_fusion')
        self.readings: Dict[str, SensorReading] = {}
        self.fusion_results: List[FusionResult] = []
        self.active_sensors: List[str] = []
        
        # Subscriptions
        self.gps_sub = self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odometry_callback, 10)
        
        # Publishers
        self.pose_publisher = self.create_publisher(Odometry, '/fusion/pose', 10)
        
    def gps_callback(self, msg: NavSatFix):
        """Handle GPS data"""
        if msg.status.status != NavSatFix.STATUS_NO_FIX:
            reading = SensorReading(
                sensor_type="gps",
                timestamp=time.time(),
                data={
                    'latitude': msg.latitude,
                    'longitude': msg.longitude,
                    'altitude': msg.altitude
                },
                confidence=0.8 if msg.status.status == NavSatFix.STATUS_FIX else 0.5
            )
            self.readings['gps'] = reading
            
    def imu_callback(self, msg: Imu):
        """Handle IMU data"""
        reading = SensorReading(
            sensor_type="imu",
            timestamp=time.time(),
            data={
                'linear_accel': [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z],
                'angular_vel': [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
            }
        )
        
        # Add orientation if available
        if msg.orientation.w != 0.0:
            reading.data['orientation'] = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
            
        self.readings['imu'] = reading
        
    def odometry_callback(self, msg: Odometry):
        """Handle odometry data"""
        reading = SensorReading(
            sensor_type="odometry",
            timestamp=time.time(),
            data={
                'position': [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z],
                'orientation': [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
                             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w],
                'linear_vel': [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z],
                'angular_vel': [msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z]
            },
            confidence=0.9  # Odometry is generally reliable
        )
        self.readings['odometry'] = reading
        
    def perform_fusion(self) -> FusionResult:
        """Perform sensor fusion"""
        if not self.readings:
            return None
            
        # Get current readings
        current_time = time.time()
        valid_readings = {
            sensor: reading for sensor, reading in self.readings.items() 
            if reading.is_valid() and (current_time - reading.timestamp) < 1.0
        }
        
        if not valid_readings:
            return None
            
        # Fusion logic
        position = np.zeros(3)
        orientation = np.array([0, 0, 0, 1])  # Identity quaternion
        velocity = np.zeros(3)
        
        weights = {
            'gps': 0.3,
            'imu': 0.2,
            'odometry': 0.5
        }
        
        contributing_sensors = []
        total_weight = 0
        
        # Fuse position
        if 'gps' in valid_readings:
            # GPS provides global position
            contributing_sensors.append('gps')
            total_weight += weights['gps']
            
        if 'odometry' in valid_readings:
            # Odometry provides relative position
            pos_data = valid_readings['odometry'].data['position']
            position += weights['odometry'] * np.array(pos_data)
            contributing_sensors.append('odometry')
            total_weight += weights['odometry']
            
        if total_weight > 0:
            position /= total_weight
            
        # Fuse orientation
        if 'imu' in valid_readings and 'orientation' in valid_readings['imu'].data:
            orient_data = valid_readings['imu'].data['orientation']
            orientation = np.array(orient_data)
            contributing_sensors.append('imu')
            
        # Fuse velocity
        if 'odometry' in valid_readings:
            vel_data = valid_readings['odometry'].data['linear_vel']
            velocity = np.array(vel_data)
            
        # Calculate confidence
        confidence = min(len(valid_readings) / 3.0, 1.0)
        fusion_quality = self.calculate_fusion_quality(valid_readings)
        
        result = FusionResult(
            position=position,
            orientation=orientation,
            velocity=velocity,
            timestamp=current_time,
            confidence=confidence,
            contributing_sensors=contributing_sensors,
            fusion_quality=fusion_quality
        )
        
        self.fusion_results.append(result)
        return result
        
    def calculate_fusion_quality(self, readings: Dict[str, SensorReading]) -> float:
        """Calculate quality of fusion based on sensor consistency"""
        if len(readings) < 2:
            return 0.5
            
        # Simple consistency check
        quality = 0.0
        count = 0
        
        for sensor1, reading1 in readings.items():
            for sensor2, reading2 in readings.items():
                if sensor1 < sensor2:  # Avoid duplicate checks
                    # Time consistency
                    time_diff = abs(reading1.timestamp - reading2.timestamp)
                    if time_diff < 0.1:  # Within 100ms
                        quality += 1.0
                    else:
                        quality += max(0, 1.0 - time_diff)
                    count += 1
                    
        return quality / max(count, 1)
        
    def detect_outliers(self) -> List[str]:
        """Detect outlier sensor readings"""
        outliers = []
        
        if len(self.readings) < 2:
            return outliers
            
        # Simple outlier detection based on position differences
        positions = {}
        for sensor, reading in self.readings.items():
            if reading.sensor_type in ['gps', 'odometry']:
                if reading.sensor_type == 'gps':
                    # Convert GPS to local coordinates (simplified)
                    positions[sensor] = np.array([reading.data['latitude'], reading.data['longitude']])
                else:
                    positions[sensor] = np.array(reading.data['position'][:2])
                    
        if len(positions) >= 2:
            # Check for large position differences
            pos_values = list(positions.values())
            mean_pos = np.mean(pos_values, axis=0)
            
            for sensor, pos in positions.items():
                distance = np.linalg.norm(pos - mean_pos)
                if distance > 10.0:  # 10 meters threshold
                    outliers.append(sensor)
                    
        return outliers


class SensorFusionMonitor:
    """Monitor sensor fusion performance and accuracy"""
    
    def __init__(self):
        self.fusion_history: List[FusionResult] = []
        self.sensor_status: Dict[str, bool] = {}
        self.dropout_events: List[Dict] = []
        self.conflict_events: List[Dict] = []
        
    def record_fusion_result(self, result: FusionResult):
        """Record fusion result"""
        self.fusion_history.append(result)
        
        # Update sensor status
        self.sensor_status = {sensor: sensor in result.contributing_sensors 
                            for sensor in ['gps', 'imu', 'odometry']}
                            
    def record_dropout(self, sensor: str, duration: float):
        """Record sensor dropout event"""
        self.dropout_events.append({
            'sensor': sensor,
            'duration': duration,
            'timestamp': time.time()
        })
        
    def record_conflict(self, sensors: List[str], conflict_type: str):
        """Record sensor conflict event"""
        self.conflict_events.append({
            'sensors': sensors,
            'type': conflict_type,
            'timestamp': time.time()
        })
        
    def get_sensor_availability(self) -> Dict[str, float]:
        """Get sensor availability percentages"""
        if not self.fusion_history:
            return {}
            
        total_fusions = len(self.fusion_history)
        availability = {}
        
        for sensor in ['gps', 'imu', 'odometry']:
            available_count = sum(1 for result in self.fusion_history 
                                if sensor in result.contributing_sensors)
            availability[sensor] = available_count / total_fusions
            
        return availability
        
    def get_fusion_accuracy(self) -> Dict[str, float]:
        """Get fusion accuracy metrics"""
        if not self.fusion_history:
            return {}
            
        confidences = [result.confidence for result in self.fusion_history]
        qualities = [result.fusion_quality for result in self.fusion_history]
        
        return {
            'average_confidence': np.mean(confidences),
            'min_confidence': np.min(confidences),
            'average_quality': np.mean(qualities),
            'min_quality': np.min(qualities)
        }


@pytest.fixture
def ros_context():
    """Provide ROS2 context for tests"""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def gps_publisher(ros_context):
    """Provide mock GPS publisher"""
    return MockGPSPublisher()


@pytest.fixture
def imu_publisher(ros_context):
    """Provide mock IMU publisher"""
    return MockIMUPublisher()


@pytest.fixture
def odometry_publisher(ros_context):
    """Provide mock odometry publisher"""
    return MockOdometryPublisher()


@pytest.fixture
def sensor_fusion(ros_context):
    """Provide mock sensor fusion system"""
    return MockSensorFusion()


@pytest.fixture
def fusion_monitor():
    """Provide sensor fusion monitor"""
    return SensorFusionMonitor()


class TestMultiSensorFusion:
    """Test multi-sensor fusion validation and conflict resolution"""
    
    def test_sensor_dropout_handling(
        self, ros_context, gps_publisher, imu_publisher, odometry_publisher, 
        sensor_fusion, fusion_monitor
    ):
        """Test system behavior when individual sensors fail"""
        
        # Publish initial sensor data
        gps_publisher.publish_gps_data(37.7749, -122.4194, 100.0)  # San Francisco
        imu_publisher.publish_imu_data(np.array([0.1, 0.0, 9.8]), np.array([0.0, 0.0, 0.01]))
        odometry_publisher.publish_odometry_data(
            np.array([1.0, 2.0, 0.0]), 
            np.array([0, 0, 0, 1]),
            np.array([0.5, 0.0, 0.0]),
            np.array([0.0, 0.0, 0.1])
        )
        
        # Process messages
        for _ in range(10):
            rclpy.spin_once(ros_context, timeout_sec=0.01)
            
        # Initial fusion should include all sensors
        result = sensor_fusion.perform_fusion()
        fusion_monitor.record_fusion_result(result)
        
        assert len(result.contributing_sensors) == 3, f"Expected 3 sensors, got {len(result.contributing_sensors)}"
        assert result.confidence > 0.8, f"Low initial confidence: {result.confidence}"
        
        # Simulate GPS dropout (no GPS messages)
        print("Simulating GPS dropout...")
        dropout_start = time.time()
        
        # Continue with IMU and odometry only
        for i in range(50):  # Simulate 5 seconds of dropout
            imu_publisher.publish_imu_data(
                np.array([0.1 + 0.01*i, 0.0, 9.8]), 
                np.array([0.0, 0.0, 0.01])
            )
            odometry_publisher.publish_odometry_data(
                np.array([1.0 + 0.01*i, 2.0 + 0.005*i, 0.0]), 
                np.array([0, 0, 0, 1]),
                np.array([0.5, 0.0, 0.0]),
                np.array([0.0, 0.0, 0.1])
            )
            
            rclpy.spin_once(ros_context, timeout_sec=0.01)
            
            if i % 10 == 0:  # Fuse every 10 iterations
                result = sensor_fusion.perform_fusion()
                if result:
                    fusion_monitor.record_fusion_result(result)
                    
        dropout_duration = time.time() - dropout_start
        fusion_monitor.record_dropout('gps', dropout_duration)
        
        # Verify system continued with reduced sensor set
        latest_results = fusion_monitor.fusion_history[-5:]
        avg_confidence = np.mean([r.confidence for r in latest_results])
        
        assert avg_confidence < 0.8, "Confidence should decrease with GPS dropout"
        assert avg_confidence > 0.4, "Confidence should remain reasonable with 2 sensors"
        
        # Verify GPS is not in contributing sensors
        for result in latest_results:
            assert 'gps' not in result.contributing_sensors, "GPS should not contribute during dropout"
            assert 'imu' in result.contributing_sensors, "IMU should still contribute"
            assert 'odometry' in result.contributing_sensors, "Odometry should still contribute"
            
        print(f"✅ GPS dropout handled: {dropout_duration:.1f}s")
        
    def test_sensor_conflict_resolution(
        self, ros_context, gps_publisher, odometry_publisher, 
        sensor_fusion, fusion_monitor
    ):
        """Test handling of conflicting sensor data"""
        
        # Publish conflicting position data
        # GPS says rover is at origin
        gps_publisher.publish_gps_data(0.0, 0.0, 100.0, confidence=0.9)
        
        # Odometry says rover is 50m away
        odometry_publisher.publish_odometry_data(
            np.array([50.0, 50.0, 0.0]), 
            np.array([0, 0, 0, 1]),
            np.array([0.0, 0.0, 0.0]),
            np.array([0.0, 0.0, 0.0])
        )
        
        # Process messages
        for _ in range(10):
            rclpy.spin_once(ros_context, timeout_sec=0.01)
            
        result = sensor_fusion.perform_fusion()
        fusion_monitor.record_fusion_result(result)
        
        # Detect outliers
        outliers = sensor_fusion.detect_outliers()
        
        if outliers:
            fusion_monitor.record_conflict(outliers, "position_mismatch")
            
        # Verify fusion handled conflict
        assert result is not None, "Fusion should still produce result despite conflict"
        assert result.fusion_quality < 0.8, "Fusion quality should be lower with conflicts"
        
        # Position should be weighted average, not extreme values
        position_norm = np.linalg.norm(result.position)
        assert position_norm < 40.0, f"Position too close to odometry: {position_norm}"
        assert position_norm > 10.0, f"Position too close to GPS: {position_norm}"
        
        print(f"✅ Sensor conflict resolved: outliers={outliers}, quality={result.fusion_quality:.2f}")
        
    def test_temporal_sensor_alignment(
        self, ros_context, gps_publisher, imu_publisher, odometry_publisher,
        sensor_fusion, fusion_monitor
    ):
        """Test timestamp synchronization between sensors"""
        
        # Publish sensor data with different timestamps
        base_time = time.time()
        
        # GPS at t=0
        gps_publisher.publish_gps_data(37.7749, -122.4194, 100.0)
        
        # IMU at t=50ms
        time.sleep(0.050)
        imu_publisher.publish_imu_data(np.array([0.1, 0.0, 9.8]), np.array([0.0, 0.0, 0.01]))
        
        # Odometry at t=100ms
        time.sleep(0.050)
        odometry_publisher.publish_odometry_data(
            np.array([1.0, 2.0, 0.0]), 
            np.array([0, 0, 0, 1]),
            np.array([0.5, 0.0, 0.0]),
            np.array([0.0, 0.0, 0.1])
        )
        
        # Process messages
        for _ in range(20):
            rclpy.spin_once(ros_context, timeout_sec=0.01)
            
        result = sensor_fusion.perform_fusion()
        fusion_monitor.record_fusion_result(result)
        
        # Verify temporal alignment
        if result:
            # Check that fusion used time-synchronized data
            assert result.fusion_quality > 0.5, f"Poor temporal alignment: {result.fusion_quality}"
            
            # Verify timestamps are considered
            contributing_times = [sensor_fusion.readings[sensor].timestamp 
                              for sensor in result.contributing_sensors 
                              if sensor in sensor_fusion.readings]
            
            if len(contributing_times) > 1:
                time_variance = np.var(contributing_times)
                assert time_variance < 1.0, f"Poor time synchronization: variance={time_variance:.3f}"
                
        print("✅ Temporal sensor alignment verified")
        
    def test_sensor_weight_adaptation(
        self, ros_context, gps_publisher, imu_publisher, odometry_publisher,
        sensor_fusion, fusion_monitor
    ):
        """Test adaptive sensor weighting based on reliability"""
        
        # Simulate GPS with varying reliability
        gps_reliability_sequence = [0.9, 0.7, 0.3, 0.8, 0.6]  # Varying confidence
        
        for i, reliability in enumerate(gps_reliability_sequence):
            print(f"Testing GPS reliability: {reliability}")
            
            # Publish sensor data
            gps_publisher.publish_gps_data(
                37.7749 + i*0.001,  # Small movement
                -122.4194 + i*0.001,
                100.0,
                confidence=reliability
            )
            
            imu_publisher.publish_imu_data(
                np.array([0.1, 0.0, 9.8]), 
                np.array([0.0, 0.0, 0.01])
            )
            
            odometry_publisher.publish_odometry_data(
                np.array([1.0 + i*0.1, 2.0 + i*0.05, 0.0]), 
                np.array([0, 0, 0, 1]),
                np.array([0.5, 0.0, 0.0]),
                np.array([0.0, 0.0, 0.1])
            )
            
            # Process messages
            for _ in range(10):
                rclpy.spin_once(ros_context, timeout_sec=0.01)
                
            result = sensor_fusion.perform_fusion()
            if result:
                fusion_monitor.record_fusion_result(result)
                
                # Verify GPS contribution adapts to reliability
                gps_contributing = 'gps' in result.contributing_sensors
                
                if reliability < 0.4:
                    # GPS should be excluded when very unreliable
                    assert not gps_contributing, f"GPS should be excluded at reliability {reliability}"
                else:
                    # GPS should contribute when reasonably reliable
                    assert gps_contributing, f"GPS should contribute at reliability {reliability}"
                    
        print("✅ Sensor weight adaptation verified")
        
    def test_fusion_performance_under_noise(
        self, ros_context, gps_publisher, imu_publisher, odometry_publisher,
        sensor_fusion, fusion_monitor
    ):
        """Test fusion performance under noisy conditions"""
        
        noise_levels = [0.0, 0.1, 0.5, 1.0]  # Increasing noise
        
        for noise_level in noise_levels:
            print(f"Testing noise level: {noise_level}")
            
            # Add noise to sensor data
            gps_noise = np.random.normal(0, noise_level * 0.0001, 2)
            imu_noise = np.random.normal(0, noise_level * 0.01, 6)
            odom_noise = np.random.normal(0, noise_level * 0.05, 6)
            
            # Publish noisy data
            gps_publisher.publish_gps_data(
                37.7749 + gps_noise[0],
                -122.4194 + gps_noise[1],
                100.0
            )
            
            imu_publisher.publish_imu_data(
                np.array([0.1, 0.0, 9.8]) + imu_noise[:3],
                np.array([0.0, 0.0, 0.01]) + imu_noise[3:]
            )
            
            odometry_publisher.publish_odometry_data(
                np.array([1.0, 2.0, 0.0]) + odom_noise[:3],
                np.array([0, 0, 0, 1]),
                np.array([0.5, 0.0, 0.0]) + odom_noise[3:4],
                np.array([0.0, 0.0, 0.1]) + odom_noise[4:]
            )
            
            # Process messages
            for _ in range(10):
                rclpy.spin_once(ros_context, timeout_sec=0.01)
                
            result = sensor_fusion.perform_fusion()
            if result:
                fusion_monitor.record_fusion_result(result)
                
                # Verify fusion degrades gracefully with noise
                expected_min_confidence = max(0.3, 1.0 - noise_level * 0.5)
                assert result.confidence >= expected_min_confidence, \
                    f"Confidence too low with noise {noise_level}: {result.confidence}"
                    
        print("✅ Fusion performance under noise verified")
        
    def test_sensor_fusion_reliability_metrics(
        self, ros_context, sensor_fusion, fusion_monitor
    ):
        """Test sensor fusion reliability metrics"""
        
        # Generate a series of fusion results
        for i in range(100):
            # Simulate varying sensor availability
            available_sensors = []
            
            if i % 4 != 0:  # GPS available 75% of time
                available_sensors.append('gps')
            if i % 5 != 0:  # IMU available 80% of time
                available_sensors.append('imu')
            if i % 10 != 0:  # Odometry available 90% of time
                available_sensors.append('odometry')
                
            # Create mock fusion result
            result = FusionResult(
                position=np.random.rand(3),
                orientation=np.array([0, 0, 0, 1]),
                velocity=np.random.rand(3),
                timestamp=time.time(),
                confidence=len(available_sensors) / 3.0,
                contributing_sensors=available_sensors,
                fusion_quality=0.8
            )
            
            fusion_monitor.record_fusion_result(result)
            
        # Get reliability metrics
        availability = fusion_monitor.get_sensor_availability()
        accuracy = fusion_monitor.get_fusion_accuracy()
        
        # Verify expected availability
        assert 0.7 <= availability['gps'] <= 0.8, f"GPS availability unexpected: {availability['gps']}"
        assert 0.75 <= availability['imu'] <= 0.85, f"IMU availability unexpected: {availability['imu']}"
        assert 0.85 <= availability['odometry'] <= 0.95, f"Odometry availability unexpected: {availability['odometry']}"
        
        # Verify accuracy metrics are reasonable
        assert 0.5 <= accuracy['average_confidence'] <= 1.0, f"Average confidence unexpected: {accuracy['average_confidence']}"
        assert 0.0 <= accuracy['min_confidence'] <= accuracy['average_confidence'], f"Min confidence unexpected: {accuracy['min_confidence']}"
        
        print(f"✅ Reliability metrics: availability={availability}, accuracy={accuracy}")


if __name__ == "__main__":
    # Run tests manually for debugging
    pytest.main([__file__, "-v"])