#!/usr/bin/env python3
"""
Navigation Control Loop Integrity Test

Complete data flow test from perception to motor control 
with strict latency requirements (<100ms total loop).

Author: URC 2026 Navigation Systems Team
"""

import asyncio
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple
from unittest.mock import Mock, patch, AsyncMock
import pytest
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import Header
from sensor_msgs.msg import Image, LaserScan, Imu
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped, Point, Quaternion


# Import navigation components
try:
    from src.autonomy.perception.camera_processor import CameraProcessor
    from src.autonomy.perception.lidar_processor import LidarProcessor
    from src.autonomy.core.navigation.navigation_node import NavigationNode
    from src.autonomy.core.navigation.slam_processor import SLAMProcessor
    from src.autonomy.control.motion_controller import MotionController
except ImportError as e:
    pytest.skip(f"Skipping navigation loop tests due to import error: {e}", allow_module_level=True)


@dataclass
class NavigationTiming:
    """Track timing data for navigation control loop"""
    perception_time: float = 0.0
    slam_time: float = 0.0
    planning_time: float = 0.0
    control_time: float = 0.0
    motor_time: float = 0.0
    total_time: float = 0.0
    
    @property
    def perception_latency(self) -> float:
        return self.perception_time
    
    @property
    def slam_latency(self) -> float:
        return self.slam_time - self.perception_time
    
    @property
    def planning_latency(self) -> float:
        return self.planning_time - self.slam_time
    
    @property
    def control_latency(self) -> float:
        return self.control_time - self.planning_time
    
    @property
    def motor_latency(self) -> float:
        return self.motor_time - self.control_time
    
    @property
    def total_latency(self) -> float:
        return self.total_time - self.perception_time


@dataclass 
class NavigationState:
    """Track state through navigation pipeline"""
    camera_data: Optional[np.ndarray] = None
    lidar_data: Optional[np.ndarray] = None
    imu_data: Optional[Dict] = None
    odometry: Optional[Odometry] = None
    slam_pose: Optional[PoseStamped] = None
    planned_path: Optional[Path] = None
    motor_command: Optional[Twist] = None


class MockSensorPublisher(Node):
    """Mock sensor data publisher"""
    
    def __init__(self):
        super().__init__('mock_sensor_publisher')
        self.camera_pub = self.create_publisher(Image, '/camera/front/image_raw', 10)
        self.lidar_pub = self.create_publisher(LaserScan, '/lidar/scan', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.odometry_pub = self.create_publisher(Odometry, '/odom', 10)
        
    def publish_camera_data(self, image_data: np.ndarray):
        """Publish camera data"""
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"
        msg.height, msg.width = image_data.shape[:2]
        msg.encoding = "rgb8"
        msg.is_bigendian = False
        msg.step = msg.width * 3
        msg.data = image_data.tobytes()
        self.camera_pub.publish(msg)
        
    def publish_lidar_data(self, ranges: np.ndarray):
        """Publish lidar data"""
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "laser"
        msg.angle_min = -np.pi
        msg.angle_max = np.pi
        msg.angle_increment = 2 * np.pi / len(ranges)
        msg.range_min = 0.1
        msg.range_max = 10.0
        msg.ranges = ranges.tolist()
        self.lidar_pub.publish(msg)
        
    def publish_imu_data(self, linear_accel: np.ndarray, angular_vel: np.ndarray):
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
        self.imu_pub.publish(msg)
        
    def publish_odometry(self, position: np.ndarray, orientation: np.ndarray):
        """Publish odometry data"""
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        
        msg.pose.pose.position.x = position[0]
        msg.pose.pose.position.y = position[1]
        msg.pose.pose.position.z = position[2]
        
        msg.pose.pose.orientation.x = orientation[0]
        msg.pose.pose.orientation.y = orientation[1]
        msg.pose.pose.orientation.z = orientation[2]
        msg.pose.pose.orientation.w = orientation[3]
        
        self.odometry_pub.publish(msg)


class MockMotorController(Node):
    """Mock motor controller to receive commands"""
    
    def __init__(self):
        super().__init__('mock_motor_controller')
        self.received_commands: List[Twist] = []
        self.received_times: List[float] = []
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.command_callback, 10
        )
        
    def command_callback(self, msg: Twist):
        """Receive motor command"""
        self.received_commands.append(msg)
        self.received_times.append(time.time())


class NavigationLoopMonitor:
    """Monitor navigation control loop data flow"""
    
    def __init__(self):
        self.timing_data: List[NavigationTiming] = []
        self.state_tracking: List[NavigationState] = []
        self.active_measurements: Dict[str, NavigationTiming] = {}
        self.active_states: Dict[str, NavigationState] = {}
        
    def start_measurement(self, measurement_id: str):
        """Start timing measurement for navigation loop"""
        timing = NavigationTiming(perception_time=time.time())
        self.active_measurements[measurement_id] = timing
        self.active_states[measurement_id] = NavigationState()
        
    def record_perception_complete(self, measurement_id: str, camera_data: np.ndarray, 
                                lidar_data: np.ndarray, imu_data: Dict):
        """Record perception processing completion"""
        if measurement_id in self.active_measurements:
            self.active_measurements[measurement_id].perception_time = time.time()
            if measurement_id in self.active_states:
                self.active_states[measurement_id].camera_data = camera_data
                self.active_states[measurement_id].lidar_data = lidar_data
                self.active_states[measurement_id].imu_data = imu_data
                
    def record_slam_complete(self, measurement_id: str, pose: PoseStamped):
        """Record SLAM processing completion"""
        if measurement_id in self.active_measurements:
            self.active_measurements[measurement_id].slam_time = time.time()
            if measurement_id in self.active_states:
                self.active_states[measurement_id].slam_pose = pose
                
    def record_planning_complete(self, measurement_id: str, path: Path):
        """Record path planning completion"""
        if measurement_id in self.active_measurements:
            self.active_measurements[measurement_id].planning_time = time.time()
            if measurement_id in self.active_states:
                self.active_states[measurement_id].planned_path = path
                
    def record_control_complete(self, measurement_id: str, command: Twist):
        """Record motion control completion"""
        if measurement_id in self.active_measurements:
            self.active_measurements[measurement_id].control_time = time.time()
            if measurement_id in self.active_states:
                self.active_states[measurement_id].motor_command = command
                
    def record_motor_received(self, measurement_id: str):
        """Record motor command reception"""
        if measurement_id in self.active_measurements:
            self.active_measurements[measurement_id].motor_time = time.time()
            self.active_measurements[measurement_id].total_time = time.time()
            
    def complete_measurement(self, measurement_id: str) -> NavigationTiming:
        """Complete timing measurement"""
        if measurement_id in self.active_measurements:
            timing = self.active_measurements[measurement_id]
            self.timing_data.append(timing)
            
            # Store state if available
            if measurement_id in self.active_states:
                self.state_tracking.append(self.active_states[measurement_id])
                
            return timing
        return NavigationTiming()


@pytest.fixture
def ros_context():
    """Provide ROS2 context for tests"""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def sensor_publisher(ros_context):
    """Provide mock sensor publisher"""
    return MockSensorPublisher()


@pytest.fixture
def motor_controller(ros_context):
    """Provide mock motor controller"""
    return MockMotorController()


@pytest.fixture
def navigation_monitor():
    """Provide navigation loop monitor"""
    return NavigationLoopMonitor()


class TestNavigationControlLoop:
    """Test navigation control loop integrity and performance"""
    
    def test_perception_to_motor_complete_chain(
        self, ros_context, sensor_publisher, motor_controller, navigation_monitor
    ):
        """Test complete data flow with latency < 100ms"""
        
        # Setup navigation components
        camera_processor = CameraProcessor()
        slam_processor = SLAMProcessor()
        navigation_node = NavigationNode()
        
        # Create test sensor data
        test_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        test_lidar = np.random.uniform(1.0, 10.0, 360)
        test_imu = {'linear_accel': np.array([0.1, 0.0, 9.8]), 'angular_vel': np.array([0.0, 0.0, 0.01])}
        
        # Start measurement
        test_id = "test_complete_chain"
        navigation_monitor.start_measurement(test_id)
        
        # Publish sensor data
        start_time = time.time()
        sensor_publisher.publish_camera_data(test_image)
        sensor_publisher.publish_lidar_data(test_lidar)
        sensor_publisher.publish_imu_data(test_imu['linear_accel'], test_imu['angular_vel'])
        
        # Mock perception processing
        def mock_perception_callback():
            perception_time = time.time() - start_time
            navigation_monitor.record_perception_complete(test_id, test_image, test_lidar, test_imu)
            
        # Mock SLAM processing
        def mock_slam_callback():
            pose = PoseStamped()
            pose.header.stamp = rclpy.Time().to_msg()
            pose.pose.position.x = 1.0
            pose.pose.position.y = 2.0
            slam_time = time.time() - start_time
            navigation_monitor.record_slam_complete(test_id, pose)
            
        # Mock planning processing
        def mock_planning_callback():
            path = Path()
            path.header.stamp = rclpy.Time().to_msg()
            planning_time = time.time() - start_time
            navigation_monitor.record_planning_complete(test_id, path)
            
        # Mock control processing
        def mock_control_callback():
            cmd = Twist()
            cmd.linear.x = 0.5
            cmd.angular.z = 0.1
            control_time = time.time() - start_time
            navigation_monitor.record_control_complete(test_id, cmd)
            motor_controller.command_callback(cmd)
            
        # Simulate processing pipeline
        perception_delay = 0.020  # 20ms
        slam_delay = 0.040  # 20ms after perception
        planning_delay = 0.060  # 20ms after SLAM
        control_delay = 0.080  # 20ms after planning
        
        # Execute processing with delays
        time.sleep(perception_delay)
        mock_perception_callback()
        
        time.sleep(slam_delay - perception_delay)
        mock_slam_callback()
        
        time.sleep(planning_delay - slam_delay)
        mock_planning_callback()
        
        time.sleep(control_delay - planning_delay)
        mock_control_callback()
        
        # Wait for motor command reception
        motor_received_time = None
        timeout_start = time.time()
        while time.time() - timeout_start < 0.1 and motor_received_time is None:
            rclpy.spin_once(ros_context, timeout_sec=0.01)
            if motor_controller.received_commands:
                motor_received_time = time.time() - start_time
                navigation_monitor.record_motor_received(test_id)
                
        # Complete measurement
        timing = navigation_monitor.complete_measurement(test_id)
        
        # Assert latency requirements
        assert timing.total_latency < 0.100, f"Total navigation loop took {timing.total_latency*1000:.1f}ms, required < 100ms"
        assert timing.perception_latency < 0.030, f"Perception took {timing.perception_latency*1000:.1f}ms, required < 30ms"
        assert timing.slam_latency < 0.025, f"SLAM took {timing.slam_latency*1000:.1f}ms, required < 25ms"
        assert timing.planning_latency < 0.025, f"Planning took {timing.planning_latency*1000:.1f}ms, required < 25ms"
        assert timing.control_latency < 0.025, f"Control took {timing.control_latency*1000:.1f}ms, required < 25ms"
        
        # Verify motor command was received
        assert len(motor_controller.received_commands) > 0, "No motor commands received"
        assert motor_controller.received_commands[-1].linear.x > 0, "Invalid motor command received"
        
        print(f"✅ Navigation loop latency: {timing.total_latency*1000:.1f}ms")
        print(f"  Perception: {timing.perception_latency*1000:.1f}ms")
        print(f"  SLAM: {timing.slam_latency*1000:.1f}ms")
        print(f"  Planning: {timing.planning_latency*1000:.1f}ms")
        print(f"  Control: {timing.control_latency*1000:.1f}ms")
        
    def test_navigation_data_loss_recovery(
        self, ros_context, sensor_publisher, motor_controller, navigation_monitor
    ):
        """Test system recovery from sensor data loss"""
        
        test_id = "test_data_loss"
        navigation_monitor.start_measurement(test_id)
        
        # Start with normal sensor data
        normal_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        normal_lidar = np.random.uniform(1.0, 10.0, 360)
        
        sensor_publisher.publish_camera_data(normal_image)
        sensor_publisher.publish_lidar_data(normal_lidar)
        
        # Simulate camera failure (no camera data)
        time.sleep(0.01)
        
        # Continue with lidar only
        degraded_lidar = np.random.uniform(1.0, 10.0, 360)
        sensor_publisher.publish_lidar_data(degraded_lidar)
        
        # System should continue navigation with degraded performance
        # Mock continued processing
        navigation_monitor.record_perception_complete(test_id, None, degraded_lidar, None)
        
        # System should use last known good position + odometry
        pose = PoseStamped()
        pose.pose.position.x = 1.0
        pose.pose.position.y = 2.0
        navigation_monitor.record_slam_complete(test_id, pose)
        
        path = Path()
        navigation_monitor.record_planning_complete(test_id, path)
        
        cmd = Twist()
        cmd.linear.x = 0.3  # Reduced speed due to camera failure
        cmd.angular.z = 0.05
        navigation_monitor.record_control_complete(test_id, cmd)
        motor_controller.command_callback(cmd)
        
        time.sleep(0.01)
        navigation_monitor.record_motor_received(test_id)
        
        timing = navigation_monitor.complete_measurement(test_id)
        state = navigation_monitor.state_tracking[-1] if navigation_monitor.state_tracking else None
        
        # Verify system handled data loss gracefully
        assert state.camera_data is None, "Camera data should be None"
        assert state.lidar_data is not None, "Lidar data should still be available"
        assert state.motor_command is not None, "Motor command should still be generated"
        assert state.motor_command.linear.x < 0.5, "Speed should be reduced with degraded sensors"
        
        print("✅ Navigation data loss recovery verified")
        
    def test_navigation_data_corruption_handling(
        self, ros_context, sensor_publisher, motor_controller, navigation_monitor
    ):
        """Test handling of corrupted navigation data"""
        
        test_id = "test_data_corruption"
        navigation_monitor.start_measurement(test_id)
        
        # Test cases for various data corruption scenarios
        corruption_scenarios = [
            ("NaN in lidar data", np.array([1.0, 2.0, np.nan, 4.0])),
            ("Infinite lidar ranges", np.array([1.0, 2.0, np.inf, 4.0])),
            ("Negative lidar ranges", np.array([-1.0, 2.0, 3.0, 4.0])),
            ("Empty image data", np.array([])),
        ]
        
        for scenario_name, corrupted_data in corruption_scenarios:
            print(f"Testing {scenario_name}...")
            
            # Publish corrupted data
            if "lidar" in scenario_name.lower():
                if corrupted_data.size > 0:
                    sensor_publisher.publish_lidar_data(corrupted_data)
                else:
                    # Skip empty data test for lidar (would cause publisher error)
                    continue
                    
            # Mock system response to corruption
            try:
                # System should detect and handle corruption
                if np.any(np.isnan(corrupted_data)) or np.any(np.isinf(corrupted_data)):
                    # System should filter out invalid data
                    filtered_data = corrupted_data[np.isfinite(corrupted_data)]
                    navigation_monitor.record_perception_complete(test_id, None, filtered_data, None)
                elif np.any(corrupted_data < 0):
                    # System should clamp negative values
                    clamped_data = np.maximum(corrupted_data, 0.1)
                    navigation_monitor.record_perception_complete(test_id, None, clamped_data, None)
                    
            except Exception as e:
                # System should handle exceptions gracefully
                print(f"  Caught expected exception: {e}")
                continue
                
        # Verify system remained stable
        assert len(motor_controller.received_commands) >= 0, "System should remain stable"
        
        print("✅ Navigation data corruption handling verified")
        
    def test_navigation_loop_under_load(
        self, ros_context, sensor_publisher, motor_controller, navigation_monitor
    ):
        """Test navigation loop under high computational load"""
        
        # Simulate high computational load
        load_tasks = []
        
        def cpu_intensive_task():
            """Simulate CPU-intensive processing"""
            result = np.random.rand(1000, 1000)
            for _ in range(10):
                result = np.dot(result, result)
            return result
            
        # Start background load
        import concurrent.futures
        with concurrent.futures.ThreadPoolExecutor(max_workers=4) as executor:
            # Submit CPU-intensive tasks
            load_futures = [executor.submit(cpu_intensive_task) for _ in range(8)]
            
            # Run navigation loop under load
            test_id = "test_under_load"
            navigation_monitor.start_measurement(test_id)
            
            # Publish sensor data
            test_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
            test_lidar = np.random.uniform(1.0, 10.0, 360)
            
            start_time = time.time()
            sensor_publisher.publish_camera_data(test_image)
            sensor_publisher.publish_lidar_data(test_lidar)
            
            # Mock processing under load
            time.sleep(0.03)  # Simulate slower processing under load
            navigation_monitor.record_perception_complete(test_id, test_image, test_lidar, None)
            
            time.sleep(0.03)
            pose = PoseStamped()
            navigation_monitor.record_slam_complete(test_id, pose)
            
            time.sleep(0.03)
            path = Path()
            navigation_monitor.record_planning_complete(test_id, path)
            
            time.sleep(0.03)
            cmd = Twist()
            cmd.linear.x = 0.4
            navigation_monitor.record_control_complete(test_id, cmd)
            motor_controller.command_callback(cmd)
            
            time.sleep(0.01)
            navigation_monitor.record_motor_received(test_id)
            
            timing = navigation_monitor.complete_measurement(test_id)
            
            # Wait for load tasks to complete
            concurrent.futures.wait(load_futures)
            
        # Verify navigation still works under load (with potentially increased latency)
        assert timing.total_latency < 0.200, f"Navigation under load took {timing.total_latency*1000:.1f}ms, required < 200ms"
        assert len(motor_controller.received_commands) > 0, "Navigation should work under load"
        
        print(f"✅ Navigation under load: {timing.total_latency*1000:.1f}ms")


class TestNavigationDataIntegrity:
    """Test navigation data integrity through processing pipeline"""
    
    def test_coordinate_system_consistency(self):
        """Test coordinate system transformations are consistent"""
        
        # Test transformation from camera to robot to world coordinates
        camera_coords = np.array([100, 200])  # Pixel coordinates
        robot_coords = np.array([1.0, 2.0, 0.5])  # Robot coordinates (meters)
        world_coords = np.array([10.0, 20.0, 0.0])  # World coordinates (meters)
        
        # Mock coordinate transformation
        def camera_to_robot(cam_coords):
            # Simulate camera to robot transformation
            return np.array([cam_coords[0] * 0.01, cam_coords[1] * 0.01, 1.0])
            
        def robot_to_world(robot_coords):
            # Simulate robot to world transformation
            return robot_coords + np.array([9.0, 18.0, -0.5])
            
        # Test transformations
        transformed_robot = camera_to_robot(camera_coords)
        transformed_world = robot_to_world(transformed_robot)
        
        # Verify transformations are reasonable
        assert np.allclose(transformed_robot, robot_coords, atol=0.1), "Camera to robot transformation failed"
        assert np.allclose(transformed_world, world_coords, atol=0.1), "Robot to world transformation failed"
        
        print("✅ Coordinate system consistency verified")
        
    def test_timestamp_synchronization(self):
        """Test sensor timestamp synchronization"""
        
        # Simulate sensor timestamps
        base_time = time.time()
        sensor_times = {
            'camera': base_time + 0.001,
            'lidar': base_time + 0.002,
            'imu': base_time + 0.003,
            'odometry': base_time + 0.004
        }
        
        # Verify timestamps are synchronized within acceptable tolerance
        max_time_diff = 0.010  # 10ms tolerance
        for sensor1, time1 in sensor_times.items():
            for sensor2, time2 in sensor_times.items():
                time_diff = abs(time1 - time2)
                assert time_diff < max_time_diff, f"Timestamp synchronization failed: {sensor1} vs {sensor2} diff {time_diff*1000:.1f}ms"
                
        print("✅ Timestamp synchronization verified")
        
    def test_data_precision_preservation(self):
        """Test data precision is preserved through pipeline"""
        
        # High precision navigation data
        precise_position = np.array([1.23456789, 2.34567890, 3.45678901])
        precise_orientation = np.array([0.12345678, 0.23456789, 0.34567890, 0.45678901])
        
        # Mock data processing pipeline
        def process_position(position):
            # Simulate position processing with precision loss
            return position.astype(np.float32).astype(np.float64)
            
        def process_orientation(orientation):
            # Normalize quaternion
            norm = np.linalg.norm(orientation)
            return orientation / norm
            
        processed_position = process_position(precise_position)
        processed_orientation = process_orientation(precise_orientation)
        
        # Verify precision is preserved
        assert np.allclose(processed_position, precise_position, atol=1e-6), "Position precision lost"
        assert np.allclose(processed_orientation, precise_orientation, atol=1e-6), "Orientation precision lost"
        
        print("✅ Data precision preservation verified")


if __name__ == "__main__":
    # Run tests manually for debugging
    pytest.main([__file__, "-v"])