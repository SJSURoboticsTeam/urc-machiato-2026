#!/usr/bin/env python3
"""
Pytest Configuration and Shared Fixtures

Provides shared test fixtures and configuration for the autonomy test suite.
"""

import time
from unittest.mock import Mock

import numpy as np
import pytest

# ROS2 testing imports (optional)
try:
    import rclpy
    from rclpy.node import Node
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    rclpy = None
    Node = None

# Import simulation sensor factory
from simulation.sensors.sensor_factory import SensorFactory


@pytest.fixture(scope="session", autouse=True)
def set_random_seed():
    """Set random seed for reproducible tests."""
    np.random.seed(42)


@pytest.fixture
def mock_gps():
    """Provide mock GPS sensor."""
    factory = SensorFactory()
    return factory.create("gps", {"name": "test_gps", "type": "gps"})


@pytest.fixture
def mock_imu():
    """Provide mock IMU sensor."""
    factory = SensorFactory()
    return factory.create("imu", {"name": "test_imu", "type": "imu"})


@pytest.fixture
def mock_camera():
    """Provide mock camera sensor."""
    factory = SensorFactory()
    return factory.create("gps", {"name": "test_camera", "type": "gps"})  # Using GPS as placeholder


@pytest.fixture
def mock_lidar():
    """Provide mock LiDAR sensor."""
    factory = SensorFactory()
    return factory.create("gps", {"name": "test_lidar", "type": "gps"})  # Using GPS as placeholder


@pytest.fixture
def mock_encoders():
    """Provide mock wheel encoders (4 wheels)."""
    factory = SensorFactory()
    return [factory.create("gps", {"name": f"encoder_{i}", "type": "gps"}) for i in range(4)]  # Using GPS as placeholder


@pytest.fixture
def mock_actuators():
    """Provide mock actuators."""
    factory = SensorFactory()
    return [factory.create("gps", {"name": f"actuator_{i}", "type": "gps"}) for i in range(6)]  # Using GPS as placeholder


@pytest.fixture
def mock_battery():
    """Provide mock battery sensor."""
    factory = SensorFactory()
    return factory.create("gps", {"name": "test_battery", "type": "gps"})  # Using GPS as placeholder


@pytest.fixture
def sensor_fusion_mock():
    """Provide complete sensor fusion mock."""
    factory = SensorFactory()
    return {
        'gps': factory.create("gps", {"name": "fusion_gps", "type": "gps"}),
        'imu': factory.create("imu", {"name": "fusion_imu", "type": "imu"}),
    }


@pytest.fixture
def mock_sensor_suite():
    """Provide complete mock sensor suite."""
    factory = SensorFactory()
    return {
        'sensors': [
            factory.create("gps", {"name": "suite_gps", "type": "gps"}),
            factory.create("imu", {"name": "suite_imu", "type": "imu"}),
        ]
    }


@pytest.fixture
def mock_safety_manager():
    """Provide mock safety manager."""
    safety_manager = Mock()
    safety_manager.trigger_safety = Mock()
    safety_manager.clear_trigger = Mock()
    safety_manager.get_safety_status = Mock(return_value={
        'active_triggers': [],
        'highest_severity': None,
        'system_safe': True
    })
    return safety_manager


@pytest.fixture
def mock_navigation():
    """Provide mock navigation system."""
    navigation = Mock()
    navigation.navigate_to_waypoint = Mock()
    navigation.get_navigation_status = Mock(return_value={
        'is_navigating': False,
        'current_position': (0.0, 0.0),
        'target_waypoint': None,
        'distance_to_target': 0.0,
        'velocity': 0.0
    })
    navigation.stop_navigation = Mock()
    navigation.resume_navigation = Mock()
    return navigation


@pytest.fixture
def mock_vision():
    """Provide mock computer vision system."""
    vision = Mock()
    vision.detect_objects = Mock(return_value=[])
    vision.detect_aruco_markers = Mock(return_value=[])
    vision.get_camera_intrinsics = Mock(return_value={
        'fx': 600.0, 'fy': 600.0, 'cx': 320.0, 'cy': 240.0
    })
    vision.get_health_status = Mock(return_value={
        'healthy': True,
        'fps': 30.0,
        'resolution': (640, 480)
    })
    return vision


@pytest.fixture
def mock_control():
    """Provide mock control system."""
    control = Mock()
    control.set_velocity = Mock()
    control.stop_all = Mock()
    control.get_control_status = Mock(return_value={
        'active': True,
        'velocity': 0.0,
        'effort': 0.0,
        'temperature': 25.0
    })
    return control


@pytest.fixture
def mock_state_machine():
    """Provide mock state machine."""
    state_machine = Mock()
    state_machine.current_state = Mock()
    state_machine.transition_to = Mock()
    state_machine.start_mission = Mock()
    state_machine.stop_mission = Mock()
    return state_machine


@pytest.fixture
def integration_test_setup(mock_safety_manager, mock_navigation,
                           mock_vision, mock_control, mock_state_machine):
    """Provide complete integration test setup."""
    return {
        'safety': mock_safety_manager,
        'navigation': mock_navigation,
        'vision': mock_vision,
        'control': mock_control,
        'state_machine': mock_state_machine
    }


# Test data fixtures
@pytest.fixture
def test_waypoints():
    """Provide test mission waypoints."""
    return [
        {'id': 1, 'position': (0.0, 0.0), 'type': 'gnss_waypoint'},
        {'id': 2, 'position': (10.0, 0.0), 'type': 'aruco_marker'},
        {'id': 3, 'position': (10.0, 10.0), 'type': 'ground_object'},
        {'id': 4, 'position': (0.0, 10.0), 'type': 'gnss_waypoint'},
    ]


@pytest.fixture
def test_trajectory():
    """Provide test trajectory data."""
    trajectory = []
    for i in range(50):
        x = i * 0.2  # 20cm steps
        y = np.sin(i * 0.1) * 1.0  # Sine wave
        z = 0.0
        trajectory.append((x, y, z))
    return trajectory


@pytest.fixture
def performance_monitor():
    """Provide performance monitoring fixture."""
    class PerformanceMonitor:
        def __init__(self):
            self.metrics = []
            self.start_time = None

        def start(self):
            self.start_time = time.time()

        def record_metric(self, name, value):
            self.metrics.append({
                'timestamp': time.time(),
                'name': name,
                'value': value
            })

        def get_average(self, name):
            values = [m['value'] for m in self.metrics if m['name'] == name]
            return sum(values) / len(values) if values else 0

        def get_elapsed_time(self):
            return time.time() - self.start_time if self.start_time else 0

    return PerformanceMonitor()


# Custom pytest marks
def pytest_configure(config):
    """Configure custom pytest marks."""
    config.addinivalue_line("markers", "slow: marks tests as slow (deselect with '-m \"not slow\"')")
    config.addinivalue_line("markers", "integration: marks tests as integration tests")
    config.addinivalue_line("markers", "performance: marks tests as performance tests")
    config.addinivalue_line("markers", "safety: marks tests as safety-critical")
    config.addinivalue_line("markers", "ros2: marks tests as requiring ROS2 environment")
    config.addinivalue_line("markers", "launch: marks tests as using ROS2 launch files")


# Test utilities
def assert_pose_equal(pose1, pose2, tolerance=1e-6):
    """Assert that two poses are equal within tolerance."""
    assert len(pose1) == len(pose2), f"Pose dimensions don't match: {len(pose1)} vs {len(pose2)}"

    for i, (p1, p2) in enumerate(zip(pose1, pose2)):
        assert abs(p1 - p2) < tolerance, f"Pose component {i} differs: {p1} vs {p2}"


def assert_transforms_equal(transform1, transform2, tolerance=1e-6):
    """Assert that two transforms are equal within tolerance."""
    assert 'position' in transform1 and 'position' in transform2
    assert 'orientation' in transform1 and 'orientation' in transform2

    assert_pose_equal(transform1['position'], transform2['position'], tolerance)
    assert_pose_equal(transform1['orientation'], transform2['orientation'], tolerance)


def wait_for_condition(condition_func, timeout=5.0, check_interval=0.1):
    """Wait for a condition to become true."""
    start_time = time.time()

    while time.time() - start_time < timeout:
        if condition_func():
            return True
        time.sleep(check_interval)

    return False


# ROS2 Testing Fixtures
@pytest.fixture(scope="session", autouse=True)
def ros2_init_shutdown():
    """Initialize and shutdown ROS2 context for all ROS2 tests."""
    if not ROS2_AVAILABLE:
        pytest.skip("ROS2 not available - install ROS2 to run ROS2 tests")

    # Initialize ROS2
    rclpy.init()

    # Set test domain ID to avoid conflicts
    import os
    os.environ['ROS_DOMAIN_ID'] = '42'

    yield

    # Shutdown ROS2
    rclpy.shutdown()


@pytest.fixture
def ros_node():
    """Provide a ROS2 node for testing."""
    if not ROS2_AVAILABLE:
        pytest.skip("ROS2 not available")

    node = Node('test_node')
    yield node
    node.destroy_node()


@pytest.fixture
def ros_executor():
    """Provide a ROS2 executor for testing."""
    if not ROS2_AVAILABLE:
        pytest.skip("ROS2 not available")

    from rclpy.executors import SingleThreadedExecutor
    executor = SingleThreadedExecutor()
    yield executor


@pytest.fixture
def ros_context():
    """Provide ROS2 context initialization/cleanup (legacy compatibility)."""
    if not ROS2_AVAILABLE:
        pytest.skip("ROS2 not available")

    # Context is already initialized by ros2_init_shutdown
    yield
    # Cleanup handled by ros2_init_shutdown


import pytest
import os
import signal
import subprocess
import sys
import threading
import time

# Import mock autonomy interfaces first
import mock_autonomy_interfaces

# Import ROS2 for context management
try:
    import rclpy
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    rclpy = None

# Global variables for ROS2 processes
_ros2_processes = []
_ros2_daemon_started = False

def _start_ros2_daemon():
    """Start ROS2 daemon if not already running."""
    global _ros2_daemon_started
    if _ros2_daemon_started:
        return

    try:
        # Stop any existing daemon first
        subprocess.run(['bash', '-c', 'source /opt/ros/humble/setup.bash && ros2 daemon stop'],
                      capture_output=True, timeout=5)
        time.sleep(1)

        # Start daemon
        result = subprocess.run(['bash', '-c', 'source /opt/ros/humble/setup.bash && ros2 daemon start'],
                               capture_output=True, timeout=10)
        if result.returncode == 0:
            _ros2_daemon_started = True
            print("✅ ROS2 daemon started")
        else:
            print("⚠️  ROS2 daemon start failed")
    except Exception as e:
        print(f"⚠️  ROS2 daemon setup failed: {e}")

def _launch_ros2_node(script_path, node_name):
    """Launch a ROS2 node as background process."""
    try:
        env = os.environ.copy()
        env['ROS_DOMAIN_ID'] = '42'
        env['PYTHONPATH'] = f"{os.getcwd()}:{os.getcwd()}/Autonomy:{os.getcwd()}/Autonomy/code"

        cmd = f"cd {os.getcwd()} && source /opt/ros/humble/setup.bash && python3 {script_path}"
        process = subprocess.Popen(
            ['bash', '-c', cmd],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env=env,
            preexec_fn=os.setsid  # Create new process group
        )

        _ros2_processes.append((node_name, process))
        print(f"🚀 {node_name} launched (PID: {process.pid})")

        # Give node time to start
        time.sleep(2)

        return process
    except Exception as e:
        print(f"❌ Failed to launch {node_name}: {e}")
        return None

@pytest.fixture(scope="session", autouse=False)
def ros2_integration_environment():
    """Set up complete ROS2 integration environment for all tests."""
    print("\n🔧 Setting up ROS2 integration environment...")

    # Start ROS2 daemon
    _start_ros2_daemon()

    # Launch ROS2 nodes
    nodes_to_launch = [
        ("tests/mock_topics_publisher.py", "Mock Topics Publisher"),
        ("tests/state_machine_director_node.py", "State Machine Director"),
        ("tests/slam_nodes.py", "SLAM Processing Nodes"),
        ("tests/navigation_service_node.py", "Navigation Service"),
    ]

    launched_nodes = []
    for script, name in nodes_to_launch:
        process = _launch_ros2_node(script, name)
        if process:
            launched_nodes.append((name, process))

    # Wait for all nodes to initialize
    print("⏳ Waiting for ROS2 nodes to initialize...")
    time.sleep(5)

    # Verify environment is ready
    try:
        env = os.environ.copy()
        env['ROS_DOMAIN_ID'] = '42'

        result = subprocess.run(
            ['bash', '-c', 'source /opt/ros/humble/setup.bash && ros2 topic list'],
            env=env, capture_output=True, text=True, timeout=5
        )

        topics = result.stdout.strip().split('\n')
        print(f"📡 ROS2 topics available: {len([t for t in topics if t.strip()])}")

        # Check for key topics
        key_topics = ['/state_machine/current_state', '/gps/fix', '/imu/data', '/cmd_vel']
        found_topics = [t for t in key_topics if t in topics]

        if found_topics:
            print(f"   ✅ Found: {', '.join(found_topics)}")
        if len(found_topics) >= 3:
            print("✅ ROS2 integration environment ready")
        else:
            print("⚠️  Limited ROS2 topics detected - some tests may fail")

    except Exception as e:
        print(f"⚠️  ROS2 environment verification failed: {e}")

    yield  # Tests run here

    # Cleanup
    print("\n🧹 Cleaning up ROS2 integration environment...")

    # Terminate all ROS2 processes
    for name, process in launched_nodes:
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            process.wait(timeout=5)
            print(f"✅ {name} stopped")
        except Exception as e:
            try:
                process.kill()
                print(f"⚠️  {name} force killed")
            except:
                print(f"⚠️  {name} cleanup failed: {e}")

    # Stop ROS2 daemon
    try:
        subprocess.run(['bash', '-c', 'source /opt/ros/humble/setup.bash && ros2 daemon stop'],
                      capture_output=True, timeout=5)
        print("✅ ROS2 daemon stopped")
    except:
        pass

@pytest.fixture
def ros_context():
    """Initialize and cleanup ROS context for individual tests."""
    # For integration tests, ROS2 context is managed externally
    # Don't skip - let tests handle ROS2 availability
    if ROS2_AVAILABLE:
        rclpy.init()
        yield
        rclpy.shutdown()
    else:
        yield  # Allow tests to run even without ROS2


def simulate_time_passing(seconds):
    """Context manager to simulate time passing for testing."""
    class TimeSimulator:
        def __enter__(self):
            self.start_time = time.time()
            return self

        def __exit__(self, exc_type, exc_val, exc_tb):
            # Time has "passed" - just return
            pass

        def elapsed(self):
            return time.time() - self.start_time

    return TimeSimulator()
