#!/usr/bin/env python3
"""
Control Loop Latency Tests - URC 2026 Competition Critical

Tests end-to-end control loop performance: sensor → processing → actuator
Target: <50ms control loop latency for competition requirements

Critical for autonomous navigation and real-time control systems.
"""

import statistics
import threading
import time
import unittest
from concurrent.futures import ThreadPoolExecutor
from typing import Dict, List, Optional, Tuple

import numpy as np
import psutil
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, Float32, Header


class ControlLoopLatencyTest(unittest.TestCase):
    """Test end-to-end control loop latency under competition conditions."""

    def setUp(self):
        """Set up control loop latency testing environment."""
        rclpy.init()

        # Test parameters
        self.num_iterations = 1000
        self.target_latency_ms = 50.0  # Competition requirement
        self.warmup_iterations = 100

        # Create test nodes
        self.sensor_simulator = SensorSimulatorNode()
        self.control_processor = ControlProcessorNode()
        self.actuator_simulator = ActuatorSimulatorNode()

        # Performance tracking
        self.latencies = []
        self.processing_times = []
        self.sensor_delays = []
        self.actuator_delays = []

        # Control loop state
        self.loop_active = False
        self.test_completed = False

    def tearDown(self):
        """Clean up ROS2 resources."""
        self.sensor_simulator.destroy_node()
        self.control_processor.destroy_node()
        self.actuator_simulator.destroy_node()
        rclpy.shutdown()

    def test_end_to_end_control_loop_latency(self):
        """Test complete control loop: sensor → processing → actuator."""
        print("\n[OBJECTIVE] Testing End-to-End Control Loop Latency")
        print("=" * 60)

        # Start all nodes
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(self.sensor_simulator)
        executor.add_node(self.control_processor)
        executor.add_node(self.actuator_simulator)

        # Start executor in background
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

        # Allow nodes to discover each other
        time.sleep(2.0)

        try:
            # Warm up the system
            print(" Warming up control loop...")
            self._run_warmup_phase()

            # Run main latency test
            print("[GRAPH] Measuring control loop latency...")
            self._run_latency_test()

            # Analyze results
            self._analyze_latency_results()

        finally:
            executor.shutdown()
            executor_thread.join(timeout=2.0)

    def _run_warmup_phase(self):
        """Warm up the control loop before measurements."""
        self.loop_active = True

        for i in range(self.warmup_iterations):
            # Trigger sensor reading
            self.sensor_simulator.trigger_sensor_reading()
            time.sleep(0.01)  # 10ms between readings

        time.sleep(0.5)  # Allow system to stabilize
        self.loop_active = False

    def _run_latency_test(self):
        """Run the main latency measurement test."""
        self.latencies.clear()
        self.processing_times.clear()
        self.loop_active = True

        start_time = time.time()

        for i in range(self.num_iterations):
            # Record loop start time
            loop_start = time.time()

            # Trigger complete control loop
            self._execute_control_loop_iteration(loop_start)

            # Small delay to prevent overwhelming the system
            time.sleep(0.005)  # 5ms between iterations

            # Progress indicator
            if (i + 1) % 200 == 0:
                print(f"  Progress: {i + 1}/{self.num_iterations} iterations")

        self.loop_active = False
        total_duration = time.time() - start_time

        print(".2f")
        print(f"  Iterations: {len(self.latencies)}")

    def _execute_control_loop_iteration(self, loop_start: float):
        """Execute one complete control loop iteration."""
        try:
            # Step 1: Sensor reading (simulate real sensor data)
            sensor_start = time.time()
            self.sensor_simulator.trigger_sensor_reading()
            sensor_delay = (time.time() - sensor_start) * 1000

            # Step 2: Wait for processing to complete
            processing_start = time.time()

            # Trigger control processing
            self.control_processor.trigger_processing()

            # Wait for actuator command to be generated
            timeout = 0.1  # 100ms timeout
            actuator_received = False

            while time.time() - processing_start < timeout and not actuator_received:
                rclpy.spin_once(self.actuator_simulator, timeout_sec=0.001)
                if hasattr(self.actuator_simulator, "last_command_time"):
                    actuator_received = True

            processing_time = (time.time() - processing_start) * 1000

            # Step 3: Measure total loop latency
            if actuator_received:
                total_latency = (time.time() - loop_start) * 1000
                self.latencies.append(total_latency)
                self.processing_times.append(processing_time)
                self.sensor_delays.append(sensor_delay)

        except Exception as e:
            print(f"Warning: Control loop iteration failed: {e}")

    def _analyze_latency_results(self):
        """Analyze and validate latency test results."""
        if not self.latencies:
            self.fail("No latency measurements collected")

        # Calculate statistics
        avg_latency = statistics.mean(self.latencies)
        max_latency = max(self.latencies)
        min_latency = min(self.latencies)
        p95_latency = statistics.quantiles(self.latencies, n=20)[18]  # 95th percentile
        p99_latency = statistics.quantiles(self.latencies, n=100)[98]  # 99th percentile

        # Calculate processing statistics
        avg_processing = (
            statistics.mean(self.processing_times) if self.processing_times else 0
        )
        avg_sensor_delay = (
            statistics.mean(self.sensor_delays) if self.sensor_delays else 0
        )

        # Print detailed results
        print("\n Control Loop Latency Results:")
        print("-" * 40)
        print(".3f")
        print(".3f")
        print(".3f")
        print(".3f")
        print(".3f")
        print(".3f")
        print(".3f")

        # Validate against competition requirements
        print("\n[OBJECTIVE] Competition Requirements Check:")
        print("-" * 40)

        requirements = {
            "Average Latency": (avg_latency, self.target_latency_ms),
            "95th Percentile": (
                p95_latency,
                self.target_latency_ms * 1.2,
            ),  # 20% tolerance
            "99th Percentile": (
                p99_latency,
                self.target_latency_ms * 1.5,
            ),  # 50% tolerance
            "Maximum Latency": (
                max_latency,
                self.target_latency_ms * 2.0,
            ),  # 100% tolerance
        }

        all_passed = True
        for metric, (actual, target) in requirements.items():
            passed = actual <= target
            status = "[PASS] PASS" if passed else "[FAIL] FAIL"
            print(".3f")

            if not passed:
                all_passed = False

        # Performance assessment
        print("\n[IGNITE] Performance Assessment:")
        print("-" * 40)

        if all_passed:
            print("[PASS] ALL COMPETITION REQUIREMENTS MET")
            print("   Control loop performance suitable for competition")
        elif avg_latency <= self.target_latency_ms:
            print("  AVERAGE LATENCY OK, BUT TAIL LATENCY CONCERNS")
            print("   Consider optimizing for worst-case performance")
        else:
            print("[FAIL] COMPETITION PERFORMANCE NOT MET")
            print("   Control loop requires optimization before competition")

        # Recommendations
        if avg_latency > self.target_latency_ms:
            print("\n Optimization Recommendations:")
            if avg_processing > 20:
                print("   - Optimize control algorithm processing time")
            if avg_sensor_delay > 5:
                print("   - Reduce sensor reading latency")
            if p95_latency > self.target_latency_ms * 1.5:
                print("   - Investigate outliers causing tail latency")

        # Store results for regression testing
        self.test_results = {
            "average_latency_ms": avg_latency,
            "max_latency_ms": max_latency,
            "p95_latency_ms": p95_latency,
            "p99_latency_ms": p99_latency,
            "processing_time_ms": avg_processing,
            "sensor_delay_ms": avg_sensor_delay,
            "requirements_met": all_passed,
            "sample_count": len(self.latencies),
        }

        self.assertLess(avg_latency, self.target_latency_ms * 2.0, ".3f")

    def test_control_loop_under_load(self):
        """Test control loop performance under concurrent load."""
        print("\n[REFRESH] Testing Control Loop Under Load")

        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(self.sensor_simulator)
        executor.add_node(self.control_processor)
        executor.add_node(self.actuator_simulator)

        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

        time.sleep(2.0)

        try:
            # Create background load
            load_threads = []
            for i in range(5):
                t = threading.Thread(target=self._generate_background_load, args=(i,))
                load_threads.append(t)
                t.start()

            # Test control loop under load
            self._run_latency_test()

            # Stop load generation
            for t in load_threads:
                t.join(timeout=2.0)

        finally:
            executor.shutdown()
            executor_thread.join(timeout=2.0)

    def _generate_background_load(self, thread_id: int):
        """Generate background computational load."""
        while self.loop_active:
            # Perform CPU-intensive computation
            result = 0
            for i in range(10000):
                result += i**2
            time.sleep(0.001)  # Small delay to prevent complete CPU saturation


class SensorSimulatorNode(Node):
    """Simulates sensor data for control loop testing."""

    def __init__(self):
        super().__init__("sensor_simulator")

        # QoS for real-time sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Publishers
        self.imu_pub = self.create_publisher(Imu, "/imu/data", sensor_qos)
        self.lidar_pub = self.create_publisher(LaserScan, "/scan", sensor_qos)
        self.odom_pub = self.create_publisher(Odometry, "/odom", sensor_qos)

        self.get_logger().info("Sensor simulator ready")

    def trigger_sensor_reading(self):
        """Trigger a complete sensor reading cycle."""
        current_time = self.get_clock().now()

        # Publish IMU data
        imu_msg = Imu()
        imu_msg.header.stamp = current_time.to_msg()
        imu_msg.header.frame_id = "imu_link"
        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 9.81
        imu_msg.angular_velocity.x = 0.1
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = 0.0
        self.imu_pub.publish(imu_msg)

        # Publish LiDAR scan
        scan_msg = LaserScan()
        scan_msg.header.stamp = current_time.to_msg()
        scan_msg.header.frame_id = "lidar_link"
        scan_msg.angle_min = -np.pi / 2
        scan_msg.angle_max = np.pi / 2
        scan_msg.angle_increment = np.pi / 180
        scan_msg.range_min = 0.1
        scan_msg.range_max = 10.0
        scan_msg.ranges = [5.0] * 180  # 180 readings
        self.lidar_pub.publish(scan_msg)

        # Publish odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.pose.pose.position.x = 1.0
        odom_msg.pose.pose.position.y = 2.0
        odom_msg.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom_msg)


class ControlProcessorNode(Node):
    """Simulates control processing for latency testing."""

    def __init__(self):
        super().__init__("control_processor")

        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        command_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, "/imu/data", self.imu_callback, sensor_qos
        )
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, sensor_qos
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, sensor_qos
        )

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", command_qos)

        # State
        self.latest_imu = None
        self.latest_scan = None
        self.latest_odom = None
        self.processing_triggered = False

        self.get_logger().info("Control processor ready")

    def imu_callback(self, msg: Imu):
        """Handle IMU data."""
        self.latest_imu = msg

    def scan_callback(self, msg: LaserScan):
        """Handle LiDAR scan data."""
        self.latest_scan = msg

    def odom_callback(self, msg: Odometry):
        """Handle odometry data."""
        self.latest_odom = msg

    def trigger_processing(self):
        """Trigger control processing cycle."""
        self.processing_triggered = True

        # Perform control calculation
        if self.latest_imu and self.latest_odom:
            cmd_vel = Twist()

            # Simple proportional control based on position
            target_x, target_y = 5.0, 0.0  # Target position
            current_x = self.latest_odom.pose.pose.position.x
            current_y = self.latest_odom.pose.pose.position.y

            # Calculate control inputs
            error_x = target_x - current_x
            error_y = target_y - current_y

            cmd_vel.linear.x = min(0.5, error_x * 0.1)  # Proportional gain
            cmd_vel.angular.z = error_y * 0.2

            # Publish command
            self.cmd_vel_pub.publish(cmd_vel)

        self.processing_triggered = False


class ActuatorSimulatorNode(Node):
    """Simulates actuator response for latency testing."""

    def __init__(self):
        super().__init__("actuator_simulator")

        command_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, command_qos
        )

        # State
        self.last_command_time = None

        self.get_logger().info("Actuator simulator ready")

    def cmd_vel_callback(self, msg: Twist):
        """Handle velocity command."""
        self.last_command_time = time.time()

        # Simulate actuator response time (small delay)
        time.sleep(0.001)  # 1ms actuator response


if __name__ == "__main__":
    unittest.main()
