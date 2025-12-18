#!/usr/bin/env python3
"""
Pytest Configuration and Shared Fixtures

Provides shared test fixtures and configuration for the autonomy test suite.
Ensures tests use newest implementation and start from clean state.
"""

import importlib
import os
import sys
import time
from unittest.mock import Mock

import numpy as np
import pytest

# Ensure we use the absolute latest code - clear any cached imports
modules_to_clear = [
    "bridges.competition_bridge",
    "bridges.telemetry_manager",
    "bridges.websocket_manager",
    "bridges.mission_orchestrator",
    "core.dds_domain_redundancy_manager",
    "core.state_synchronization_manager",
    "core.recovery_coordinator",
    "config.config_manager",
]

for module in modules_to_clear:
    if module in sys.modules:
        del sys.modules[module]

# Set up project paths to ensure latest code
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SRC_ROOT = os.path.join(PROJECT_ROOT, "src")

# Clear and reset Python path for clean imports
original_path = sys.path[:]
sys.path.clear()
sys.path.append(PROJECT_ROOT)
sys.path.append(SRC_ROOT)
sys.path.extend(original_path)

# ROS2 testing imports (optional)
try:
    import rclpy
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import Imu, NavSatFix
    from std_msgs.msg import Bool, Float32, String

    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    rclpy = None
    Node = None
    SingleThreadedExecutor = None
    Context = None
    String = None
    Bool = None
    Float32 = None
    NavSatFix = None
    Imu = None
    TwistStamped = None
    PoseStamped = None

# Import simulation sensor factory with fresh import
try:
    from simulation.sensors.sensor_factory import SensorFactory
except ImportError:
    SensorFactory = None


# Clean environment setup
@pytest.fixture(scope="function", autouse=True)
def clean_test_environment():
    """Ensure clean test environment with fresh imports and no cached state."""
    # Reset global config manager
    try:
        from config.config_manager import reset_global_config

        reset_global_config()
    except ImportError:
        pass

    # Clear critical modules to force fresh imports
    modules_to_clear = [
        "bridges.competition_bridge",
        "bridges.telemetry_manager",
        "bridges.websocket_manager",
        "config.config_manager",
        "core.dds_domain_redundancy_manager",
    ]

    for module in modules_to_clear:
        if module in sys.modules:
            del sys.modules[module]

    # Clear environment variables that might persist
    env_vars_to_clear = ["ROS_DOMAIN_ID", "URC_ENV"]
    for var in env_vars_to_clear:
        os.environ.pop(var, None)

    yield

    # Test cleanup - ensure clean state
    if ROS2_AVAILABLE:
        try:
            rclpy.shutdown()
        except:
            pass


@pytest.fixture(scope="session", autouse=True)
def set_random_seed():
    """Set random seed for reproducible tests."""
    np.random.seed(42)


@pytest.fixture
def ros2_node():
    """Provide a ROS2 node for testing."""
    if not ROS2_AVAILABLE:
        pytest.skip("ROS2 not available")

    node = Node("test_node")
    yield node
    node.destroy_node()


@pytest.fixture
def ros2_executor():
    """Provide a ROS2 executor for testing."""
    if not ROS2_AVAILABLE:
        pytest.skip("ROS2 not available")

    executor = SingleThreadedExecutor()
    yield executor


@pytest.fixture
def mock_websocket_server():
    """Provide a mock WebSocket server for testing."""

    class MockWebSocketServer:
        def __init__(self):
            self.clients = set()
            self.messages = []

        def add_client(self, client):
            self.clients.add(client)

        def remove_client(self, client):
            self.clients.discard(client)

        def broadcast(self, message):
            self.messages.append(message)
            return len(self.clients)

    return MockWebSocketServer()


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
    return factory.create(
        "gps", {"name": "test_camera", "type": "gps"}
    )  # Using GPS as placeholder


@pytest.fixture
def mock_lidar():
    """Provide mock LiDAR sensor."""
    factory = SensorFactory()
    return factory.create(
        "gps", {"name": "test_lidar", "type": "gps"}
    )  # Using GPS as placeholder


@pytest.fixture
def mock_encoders():
    """Provide mock wheel encoders (4 wheels)."""
    factory = SensorFactory()
    return [
        factory.create("gps", {"name": f"encoder_{i}", "type": "gps"}) for i in range(4)
    ]  # Using GPS as placeholder


@pytest.fixture
def mock_actuators():
    """Provide mock actuators."""
    factory = SensorFactory()
    return [
        factory.create("gps", {"name": f"actuator_{i}", "type": "gps"})
        for i in range(6)
    ]  # Using GPS as placeholder


@pytest.fixture
def mock_battery():
    """Provide mock battery sensor."""
    factory = SensorFactory()
    return factory.create(
        "gps", {"name": "test_battery", "type": "gps"}
    )  # Using GPS as placeholder


@pytest.fixture
def sensor_fusion_mock():
    """Provide complete sensor fusion mock."""
    factory = SensorFactory()
    return {
        "gps": factory.create("gps", {"name": "fusion_gps", "type": "gps"}),
        "imu": factory.create("imu", {"name": "fusion_imu", "type": "imu"}),
    }


@pytest.fixture
def mock_sensor_suite():
    """Provide complete mock sensor suite."""
    factory = SensorFactory()
    return {
        "sensors": [
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
    safety_manager.get_safety_status = Mock(
        return_value={
            "active_triggers": [],
            "highest_severity": None,
            "system_safe": True,
        }
    )
    return safety_manager


@pytest.fixture
def mock_navigation():
    """Provide mock navigation system."""
    navigation = Mock()
    navigation.navigate_to_waypoint = Mock()
    navigation.get_navigation_status = Mock(
        return_value={
            "is_navigating": False,
            "current_position": (0.0, 0.0),
            "target_waypoint": None,
            "distance_to_target": 0.0,
            "velocity": 0.0,
        }
    )
    navigation.stop_navigation = Mock()
    navigation.resume_navigation = Mock()
    return navigation


@pytest.fixture
def mock_vision():
    """Provide mock computer vision system."""
    vision = Mock()
    vision.detect_objects = Mock(return_value=[])
    vision.detect_aruco_markers = Mock(return_value=[])
    vision.get_camera_intrinsics = Mock(
        return_value={"fx": 600.0, "fy": 600.0, "cx": 320.0, "cy": 240.0}
    )
    vision.get_health_status = Mock(
        return_value={"healthy": True, "fps": 30.0, "resolution": (640, 480)}
    )
    return vision


@pytest.fixture
def mock_control():
    """Provide mock control system."""
    control = Mock()
    control.set_velocity = Mock()
    control.stop_all = Mock()
    control.get_control_status = Mock(
        return_value={
            "active": True,
            "velocity": 0.0,
            "effort": 0.0,
            "temperature": 25.0,
        }
    )
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
def integration_test_setup(
    mock_safety_manager, mock_navigation, mock_vision, mock_control, mock_state_machine
):
    """Provide complete integration test setup."""
    return {
        "safety": mock_safety_manager,
        "navigation": mock_navigation,
        "vision": mock_vision,
        "control": mock_control,
        "state_machine": mock_state_machine,
    }


# Test data fixtures
@pytest.fixture
def test_waypoints():
    """Provide test mission waypoints."""
    return [
        {"id": 1, "position": (0.0, 0.0), "type": "gnss_waypoint"},
        {"id": 2, "position": (10.0, 0.0), "type": "aruco_marker"},
        {"id": 3, "position": (10.0, 10.0), "type": "ground_object"},
        {"id": 4, "position": (0.0, 10.0), "type": "gnss_waypoint"},
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
            self.metrics.append(
                {"timestamp": time.time(), "name": name, "value": value}
            )

        def get_average(self, name):
            values = [m["value"] for m in self.metrics if m["name"] == name]
            return sum(values) / len(values) if values else 0

        def get_elapsed_time(self):
            return time.time() - self.start_time if self.start_time else 0

    return PerformanceMonitor()


# Custom pytest marks
def pytest_configure(config):
    """Configure custom pytest marks."""
    config.addinivalue_line(
        "markers", "slow: marks tests as slow (deselect with '-m \"not slow\"')"
    )
    config.addinivalue_line("markers", "integration: marks tests as integration tests")
    config.addinivalue_line("markers", "performance: marks tests as performance tests")
    config.addinivalue_line("markers", "safety: marks tests as safety-critical")
    config.addinivalue_line(
        "markers", "ros2: marks tests as requiring ROS2 environment"
    )
    config.addinivalue_line("markers", "launch: marks tests as using ROS2 launch files")


# Test utilities
def assert_pose_equal(pose1, pose2, tolerance=1e-6):
    """Assert that two poses are equal within tolerance."""
    assert len(pose1) == len(
        pose2
    ), f"Pose dimensions don't match: {len(pose1)} vs {len(pose2)}"

    for i, (p1, p2) in enumerate(zip(pose1, pose2)):
        assert abs(p1 - p2) < tolerance, f"Pose component {i} differs: {p1} vs {p2}"


def assert_transforms_equal(transform1, transform2, tolerance=1e-6):
    """Assert that two transforms are equal within tolerance."""
    assert "position" in transform1 and "position" in transform2
    assert "orientation" in transform1 and "orientation" in transform2

    assert_pose_equal(transform1["position"], transform2["position"], tolerance)
    assert_pose_equal(transform1["orientation"], transform2["orientation"], tolerance)


def wait_for_condition(condition_func, timeout=5.0, check_interval=0.1):
    """Wait for a condition to become true."""
    start_time = time.time()

    while time.time() - start_time < timeout:
        if condition_func():
            return True
        time.sleep(check_interval)

    return False


# ROS2 Testing Fixtures - Clean initialization per test
@pytest.fixture(scope="function")
def ros2_context():
    """Provide clean ROS2 context for each test function."""
    if not ROS2_AVAILABLE:
        pytest.skip("ROS2 not available - install ROS2 to run ROS2 tests")

    # Ensure clean state - shutdown any existing context
    try:
        rclpy.shutdown()
    except:
        pass

    # Initialize fresh context for this test
    rclpy.init()

    # Set isolated domain ID for this test
    test_domain_id = f"test_{hash(time.time()) % 10000}"
    os.environ["ROS_DOMAIN_ID"] = test_domain_id

    yield

    # Clean shutdown
    try:
        rclpy.shutdown()
    except:
        pass

    # Clean up environment
    os.environ.pop("ROS_DOMAIN_ID", None)


@pytest.fixture
def ros_node(ros2_context):
    """Provide a ROS2 node for testing with clean context."""
    if not ROS2_AVAILABLE:
        pytest.skip("ROS2 not available")

    node = Node(f"test_node_{hash(time.time()) % 10000}")
    yield node
    node.destroy_node()


@pytest.fixture
def ros_executor(ros2_context):
    """Provide a ROS2 executor for testing."""
    if not ROS2_AVAILABLE:
        pytest.skip("ROS2 not available")

    from rclpy.executors import SingleThreadedExecutor

    executor = SingleThreadedExecutor()
    yield executor


# Integration testing utilities
@pytest.fixture(scope="function")
def clean_module_cache():
    """Ensure fresh module imports for each test."""
    # Clear critical modules to force fresh imports
    critical_modules = [
        "bridges.competition_bridge",
        "bridges.telemetry_manager",
        "bridges.websocket_manager",
        "config.config_manager",
        "core.dds_domain_redundancy_manager",
    ]

    for module in critical_modules:
        if module in sys.modules:
            del sys.modules[module]

    yield

    # Re-clear after test
    for module in critical_modules:
        if module in sys.modules:
            del sys.modules[module]


@pytest.fixture(scope="function")
def isolated_config():
    """Provide isolated configuration for each test."""
    import shutil
    import tempfile

    from config.config_manager import ConfigurationManager

    # Create temporary config directory
    temp_dir = tempfile.mkdtemp()
    config_dir = os.path.join(temp_dir, "config")
    os.makedirs(config_dir)

    manager = ConfigurationManager(config_dir)

    yield manager

    # Cleanup
    shutil.rmtree(temp_dir)


def simulate_time_passing(_seconds):
    """Context manager to simulate time passing for testing."""

    class TimeSimulator:
        def __enter__(self):
            self.start_time = time.time()
            return self

        def __exit__(self, _exc_type, _exc_val, _exc_tb):
            # Time has "passed" - just return
            pass

        def elapsed(self):
            return time.time() - self.start_time

    return TimeSimulator()
