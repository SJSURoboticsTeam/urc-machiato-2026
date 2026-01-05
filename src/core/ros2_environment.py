#!/usr/bin/env python3
"""
ROS2 Environment Manager - Intelligent ROS2 Detection and Fallbacks

Provides intelligent ROS2 environment detection with graceful fallbacks
for development and testing environments without full ROS2 installation.

Features:
- Automatic ROS2 environment detection
- Mock implementations for development
- Conditional imports with fallbacks
- Lightweight alternatives for embedded systems
- Environment-specific optimizations

Author: URC 2026 ROS2 Environment Team
"""

import importlib
import os
import sys
import logging
from typing import Any, Dict, List, Optional, Type, Callable, Union
from dataclasses import dataclass, field
import time

logger = logging.getLogger(__name__)


@dataclass
class ROS2Environment:
    """ROS2 environment information."""
    available: bool = False
    version: Optional[str] = None
    distro: Optional[str] = None
    workspace_path: Optional[str] = None
    python_path: Optional[str] = None
    domain_id: Optional[int] = None
    use_sim_time: bool = False
    mock_mode: bool = False


class ROS2EnvironmentManager:
    """
    Manages ROS2 environment detection and provides fallbacks.

    Automatically detects ROS2 availability and provides appropriate
    implementations for development, testing, and production environments.
    """

    def __init__(self):
        self.environment = self._detect_environment()
        self._mock_implementations: Dict[str, Any] = {}
        self._lazy_imports: Dict[str, Any] = {}
        self._performance_mode = os.getenv('URC_PERFORMANCE_MODE', 'normal')

        # Setup environment
        self._setup_environment()
        self._initialize_mock_implementations()

    def _detect_environment(self) -> ROS2Environment:
        """Detect ROS2 environment availability and configuration."""
        env = ROS2Environment()

        # Check for ROS2 environment variables
        ros_distro = os.getenv('ROS_DISTRO')
        ros_python_version = os.getenv('ROS_PYTHON_VERSION')
        ros_workspace = os.getenv('ROS_WORKSPACE')
        ros_domain_id = os.getenv('ROS_DOMAIN_ID')

        # Check for ROS2 Python path
        ros_python_path = None
        for path in sys.path:
            if 'ros2' in path.lower() or 'opt/ros' in path:
                ros_python_path = path
                break

        # Check for ROS2 availability by trying to import rclpy
        ros2_available = False
        ros2_version = None

        try:
            import rclpy
            ros2_available = True
            try:
                ros2_version = getattr(rclpy, '__version__', 'unknown')
            except:
                ros2_version = 'unknown'
        except ImportError:
            ros2_available = False

        # Determine if we should use mock mode
        use_mock = os.getenv('URC_ROS2_MOCK', 'auto').lower()
        if use_mock == 'true':
            mock_mode = True
        elif use_mock == 'false':
            mock_mode = False
        else:  # auto
            mock_mode = not ros2_available

        env.available = ros2_available
        env.version = ros2_version
        env.distro = ros_distro
        env.workspace_path = ros_workspace
        env.python_path = ros_python_path
        env.domain_id = int(ros_domain_id) if ros_domain_id else None
        env.use_sim_time = os.getenv('ROS_USE_SIM_TIME', 'false').lower() == 'true'
        env.mock_mode = mock_mode

        logger.info(f"ROS2 Environment: available={ros2_available}, mock_mode={mock_mode}, "
                   f"distro={ros_distro}, version={ros2_version}")

        return env

    def _setup_environment(self):
        """Setup ROS2 environment if available."""
        if self.environment.available and not self.environment.mock_mode:
            try:
                # Initialize ROS2 if not already done
                import rclpy
                if not rclpy.ok():
                    rclpy.init()
                    logger.info("ROS2 initialized successfully")
            except Exception as e:
                logger.warning(f"Failed to initialize ROS2: {e}")
                self.environment.mock_mode = True

    def _initialize_mock_implementations(self):
        """Initialize mock implementations for development."""
        # QoS Profiles mock
        self._mock_implementations['QoSProfile'] = self._create_mock_qos_profile()
        self._mock_implementations['ReliabilityPolicy'] = self._create_mock_reliability_policy()
        self._mock_implementations['DurabilityPolicy'] = self._create_mock_durability_policy()
        self._mock_implementations['HistoryPolicy'] = self._create_mock_history_policy()

        # Node mocks
        self._mock_implementations['LifecycleNode'] = self._create_mock_lifecycle_node()
        self._mock_implementations['LifecycleState'] = self._create_mock_lifecycle_state()
        self._mock_implementations['TransitionCallbackReturn'] = self._create_mock_transition_callback()

        # Message mocks
        self._mock_implementations['PoseStamped'] = self._create_mock_message('PoseStamped')
        self._mock_implementations['Twist'] = self._create_mock_message('Twist')
        self._mock_implementations['Imu'] = self._create_mock_message('Imu')
        self._mock_implementations['NavSatFix'] = self._mock_navsatfix_message()
        self._mock_implementations['String'] = self._create_mock_message('String')

        # Service mocks
        self._mock_implementations['Trigger'] = self._create_mock_service('Trigger')

        # Action mocks
        self._mock_implementations['NavigateToPose'] = self._create_mock_action('NavigateToPose')

    def _create_mock_qos_profile(self):
        """Create mock QoS profile class."""
        class MockQoSProfile:
            def __init__(self, **kwargs):
                self.reliability = kwargs.get('reliability', 'reliable')
                self.durability = kwargs.get('durability', 'volatile')
                self.history = kwargs.get('history', 'keep_last')
                self.depth = kwargs.get('depth', 10)

        return MockQoSProfile

    def _create_mock_reliability_policy(self):
        """Create mock reliability policy enum."""
        class MockReliabilityPolicy:
            RELIABLE = 'reliable'
            BEST_EFFORT = 'best_effort'

        return MockReliabilityPolicy

    def _create_mock_durability_policy(self):
        """Create mock durability policy enum."""
        class MockDurabilityPolicy:
            VOLATILE = 'volatile'
            TRANSIENT_LOCAL = 'transient_local'

        return MockDurabilityPolicy

    def _create_mock_history_policy(self):
        """Create mock history policy enum."""
        class MockHistoryPolicy:
            KEEP_LAST = 'keep_last'
            KEEP_ALL = 'keep_all'

        return MockHistoryPolicy

    def _create_mock_lifecycle_node(self):
        """Create mock lifecycle node class."""
        class MockLifecycleNode:
            def __init__(self, node_name: str, **kwargs):
                self.node_name = node_name
                self.logger = logger
                self._publishers = {}
                self._subscribers = {}
                self._services = {}
                self._actions = {}
                self._timers = []

            def get_logger(self):
                return self.logger

            def create_publisher(self, msg_type, topic, **kwargs):
                pub_id = f"{topic}_{len(self._publishers)}"
                self._publishers[pub_id] = {'topic': topic, 'type': msg_type}
                return MockPublisher(topic, msg_type)

            def create_subscription(self, msg_type, topic, callback, **kwargs):
                sub_id = f"{topic}_{len(self._subscribers)}"
                self._subscribers[sub_id] = {'topic': topic, 'type': msg_type, 'callback': callback}
                return MockSubscription(topic, msg_type, callback)

            def create_service(self, srv_type, service_name, callback, **kwargs):
                svc_id = f"{service_name}_{len(self._services)}"
                self._services[svc_id] = {'name': service_name, 'type': srv_type, 'callback': callback}
                return MockService(service_name, srv_type, callback)

            def create_timer(self, timer_period_sec, callback, **kwargs):
                timer = MockTimer(timer_period_sec, callback)
                self._timers.append(timer)
                return timer

        return MockLifecycleNode

    def _create_mock_lifecycle_state(self):
        """Create mock lifecycle state enum."""
        class MockLifecycleState:
            UNCONFIGURED = 'unconfigured'
            INACTIVE = 'inactive'
            ACTIVE = 'active'
            FINALIZED = 'finalized'

        return MockLifecycleState

    def _create_mock_transition_callback(self):
        """Create mock transition callback enum."""
        class MockTransitionCallbackReturn:
            SUCCESS = 'success'
            FAILURE = 'failure'
            ERROR = 'error'

        return MockTransitionCallbackReturn

    def _create_mock_message(self, msg_type: str):
        """Create mock message class."""
        class MockMessage:
            def __init__(self, **kwargs):
                for key, value in kwargs.items():
                    setattr(self, key, value)

        MockMessage.__name__ = msg_type
        return MockMessage

    def _mock_navsatfix_message(self):
        """Create specialized NavSatFix mock."""
        class MockNavSatFix:
            def __init__(self, **kwargs):
                self.latitude = kwargs.get('latitude', 0.0)
                self.longitude = kwargs.get('longitude', 0.0)
                self.altitude = kwargs.get('altitude', 0.0)
                self.status = kwargs.get('status', MockNavSatStatus())
                self.position_covariance = kwargs.get('position_covariance', [0.0] * 9)
                self.position_covariance_type = kwargs.get('position_covariance_type', 0)

        class MockNavSatStatus:
            def __init__(self):
                self.status = 0
                self.service = 1

        return MockNavSatFix

    def _create_mock_service(self, srv_type: str):
        """Create mock service class."""
        class MockServiceRequest:
            pass

        class MockServiceResponse:
            success = True
            message = "Mock service response"

        class MockService:
            def __init__(self):
                self.Request = MockServiceRequest
                self.Response = MockServiceResponse

        MockService.__name__ = srv_type
        return MockService

    def _create_mock_action(self, action_type: str):
        """Create mock action class."""
        class MockActionGoal:
            pass

        class MockActionResult:
            pass

        class MockActionFeedback:
            pass

        class MockAction:
            def __init__(self):
                self.Goal = MockActionGoal
                self.Result = MockActionResult
                self.Feedback = MockActionFeedback

        MockAction.__name__ = action_type
        return MockAction

    def safe_import(self, module_name: str, fallback: Any = None) -> Any:
        """
        Safely import a module with ROS2-aware fallbacks.

        Args:
            module_name: Module to import
            fallback: Fallback implementation if import fails

        Returns:
            Imported module or fallback
        """
        try:
            # Try direct import first
            return importlib.import_module(module_name)
        except ImportError:
            # Try ROS2-specific imports if in ROS2 environment
            if self.environment.available and not self.environment.mock_mode:
                try:
                    # Handle ROS2 specific imports
                    if module_name == 'rclpy':
                        import rclpy
                        return rclpy
                    elif module_name == 'rclpy.lifecycle':
                        from rclpy import lifecycle
                        return lifecycle
                    elif module_name == 'rclpy.qos':
                        from rclpy import qos
                        return qos
                    elif module_name.startswith('autonomy_interfaces'):
                        # Try importing from autonomy_interfaces
                        return importlib.import_module(module_name)
                except ImportError:
                    pass

            # Use mock implementation
            if module_name in self._mock_implementations:
                logger.debug(f"Using mock implementation for {module_name}")
                return self._mock_implementations[module_name]

            # Use provided fallback
            if fallback:
                logger.debug(f"Using fallback for {module_name}")
                return fallback

            # Create a dummy module to prevent crashes
            logger.warning(f"No implementation available for {module_name}, creating dummy")
            return self._create_dummy_module(module_name)

    def _create_dummy_module(self, module_name: str):
        """Create a dummy module to prevent import errors."""
        class DummyModule:
            def __getattr__(self, name):
                # Return a lambda that logs when called
                def dummy_function(*args, **kwargs):
                    logger.debug(f"Dummy {module_name}.{name} called with {args} {kwargs}")
                    return None
                return dummy_function

        dummy = DummyModule()
        dummy.__name__ = module_name
        return dummy

    def lazy_import(self, module_name: str, import_path: str = None) -> Callable:
        """
        Create a lazy import function for heavy dependencies.

        Args:
            module_name: Name to store the imported module under
            import_path: Actual import path (defaults to module_name)

        Returns:
            Function that performs the import when called
        """
        import_path = import_path or module_name

        def lazy_import_func():
            if module_name not in self._lazy_imports:
                try:
                    start_time = time.time()
                    module = importlib.import_module(import_path)
                    import_time = time.time() - start_time

                    self._lazy_imports[module_name] = module
                    logger.info(f"Lazy imported {module_name} in {import_time:.3f}s")
                except ImportError as e:
                    logger.warning(f"Failed to lazy import {module_name}: {e}")
                    self._lazy_imports[module_name] = None

            return self._lazy_imports[module_name]

        return lazy_import_func

    def get_optimized_imports(self) -> Dict[str, Any]:
        """
        Get optimized imports based on environment and performance mode.

        Returns:
            Dictionary of import name -> implementation
        """
        imports = {}

        # Core ROS2 imports
        imports['rclpy'] = self.safe_import('rclpy')
        imports['lifecycle'] = self.safe_import('rclpy.lifecycle')
        imports['qos'] = self.safe_import('rclpy.qos')

        # QoS components
        if not self.environment.mock_mode:
            try:
                from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
                imports['QoSProfile'] = QoSProfile
                imports['ReliabilityPolicy'] = ReliabilityPolicy
                imports['DurabilityPolicy'] = DurabilityPolicy
                imports['HistoryPolicy'] = HistoryPolicy
            except ImportError:
                pass

        # Use mocks if not available
        if 'QoSProfile' not in imports:
            imports['QoSProfile'] = self._mock_implementations['QoSProfile']
            imports['ReliabilityPolicy'] = self._mock_implementations['ReliabilityPolicy']
            imports['DurabilityPolicy'] = self._mock_implementations['DurabilityPolicy']
            imports['HistoryPolicy'] = self._mock_implementations['HistoryPolicy']

        # Messages
        imports['PoseStamped'] = self.safe_import('geometry_msgs.msg', self._mock_implementations['PoseStamped'])
        imports['Twist'] = self.safe_import('geometry_msgs.msg', self._mock_implementations['Twist'])
        imports['Imu'] = self.safe_import('sensor_msgs.msg', self._mock_implementations['Imu'])
        imports['NavSatFix'] = self.safe_import('sensor_msgs.msg', self._mock_implementations['NavSatFix'])
        imports['String'] = self.safe_import('std_msgs.msg', self._mock_implementations['String'])

        # Services
        imports['Trigger'] = self.safe_import('std_srvs.srv', self._mock_implementations['Trigger'])

        # Actions
        imports['NavigateToPose'] = self.safe_import('autonomy_interfaces.action',
                                                   self._mock_implementations['NavigateToPose'])

        return imports

    def is_performance_mode(self) -> bool:
        """Check if we're in performance optimization mode."""
        return self._performance_mode.lower() in ['performance', 'embedded', 'minimal']

    def get_environment_info(self) -> Dict[str, Any]:
        """Get comprehensive environment information."""
        return {
            'ros2_available': self.environment.available,
            'ros2_version': self.environment.version,
            'ros2_distro': self.environment.distro,
            'mock_mode': self.environment.mock_mode,
            'performance_mode': self._performance_mode,
            'domain_id': self.environment.domain_id,
            'use_sim_time': self.environment.use_sim_time,
            'workspace_path': self.environment.workspace_path
        }


# Mock classes for ROS2 components
class MockPublisher:
    def __init__(self, topic: str, msg_type):
        self.topic = topic
        self.msg_type = msg_type

    def publish(self, msg):
        logger.debug(f"Mock publish to {self.topic}: {msg}")

class MockSubscription:
    def __init__(self, topic: str, msg_type, callback):
        self.topic = topic
        self.msg_type = msg_type
        self.callback = callback

class MockService:
    def __init__(self, service_name: str, srv_type, callback):
        self.service_name = service_name
        self.srv_type = srv_type
        self.callback = callback

class MockTimer:
    def __init__(self, period: float, callback: Callable):
        self.period = period
        self.callback = callback
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def start(self):
        if not self._running:
            self._running = True
            self._thread = threading.Thread(target=self._run_timer, daemon=True)
            self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)

    def _run_timer(self):
        while self._running:
            time.sleep(self.period)
            try:
                self.callback()
            except Exception as e:
                logger.error(f"Timer callback error: {e}")


# Global instance
_ros2_env_manager = None

def get_ros2_environment_manager() -> ROS2EnvironmentManager:
    """Get global ROS2 environment manager instance."""
    global _ros2_env_manager
    if _ros2_env_manager is None:
        _ros2_env_manager = ROS2EnvironmentManager()
    return _ros2_env_manager

def safe_ros2_import(module_name: str, fallback: Any = None):
    """Convenience function for safe ROS2 imports."""
    manager = get_ros2_environment_manager()
    return manager.safe_import(module_name, fallback)

def lazy_ros2_import(module_name: str, import_path: str = None):
    """Convenience function for lazy ROS2 imports."""
    manager = get_ros2_environment_manager()
    return manager.lazy_import(module_name, import_path)

# Export key functions
__all__ = [
    'ROS2EnvironmentManager',
    'get_ros2_environment_manager',
    'safe_ros2_import',
    'lazy_ros2_import'
]

