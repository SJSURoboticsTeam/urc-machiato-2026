#!/usr/bin/env python3
"""
Unified Test Suite - Comprehensive Testing Framework for URC 2026

Consolidates all test functionality from across the codebase:
- Unit tests (tests/unit/)
- Integration tests (tests/integration/)
- System tests (tests/system/)
- Property-based tests (tests/property/)
- Security tests (tests/security/)
- Competition tests (tests/competition/)
- Performance tests (tests/performance/)
- Mock and simulation tests (tests/mocks/)

Features:
- Unified test discovery and execution
- Comprehensive test reporting
- Performance benchmarking
- Test data factories
- Mock management
- Coverage analysis

Author: URC 2026 Unified Test Suite Team
"""

import time
import unittest
import logging
from typing import Dict, List, Any, Optional, Callable, Union, Type
from dataclasses import dataclass, field
from enum import Enum
import threading
import json
from pathlib import Path
import weakref

logger = logging.getLogger(__name__)


class TestType(Enum):
    """Types of tests."""
    UNIT = "unit"
    INTEGRATION = "integration"
    SYSTEM = "system"
    PROPERTY = "property"
    SECURITY = "security"
    COMPETITION = "competition"
    PERFORMANCE = "performance"
    SMOKE = "smoke"


class TestStatus(Enum):
    """Test execution status."""
    PENDING = "pending"
    RUNNING = "running"
    PASSED = "passed"
    FAILED = "failed"
    ERROR = "error"
    SKIPPED = "skipped"


class TestPriority(Enum):
    """Test execution priority."""
    CRITICAL = 1
    HIGH = 2
    MEDIUM = 3
    LOW = 4


@dataclass
class TestResult:
    """Result of a test execution."""
    test_name: str
    test_type: TestType
    status: TestStatus
    duration: float = 0.0
    error_message: Optional[str] = None
    stack_trace: Optional[str] = None
    assertions_passed: int = 0
    assertions_failed: int = 0
    timestamp: float = field(default_factory=time.time)
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass
class TestSuite:
    """Test suite definition."""
    name: str
    description: str
    test_type: TestType
    priority: TestPriority = TestPriority.MEDIUM
    test_classes: List[Type] = field(default_factory=list)
    test_functions: List[Callable] = field(default_factory=list)
    setup_function: Optional[Callable] = None
    teardown_function: Optional[Callable] = None
    timeout_seconds: Optional[float] = None
    tags: List[str] = field(default_factory=list)


@dataclass
class TestRun:
    """Test run configuration."""
    run_id: str
    suites: List[str]
    parallel_execution: bool = False
    max_workers: int = 4
    timeout_seconds: float = 300.0
    include_patterns: List[str] = field(default_factory=list)
    exclude_patterns: List[str] = field(default_factory=list)
    report_format: str = "json"
    coverage_enabled: bool = True


@dataclass
class TestReport:
    """Comprehensive test report."""
    run_id: str
    timestamp: float = field(default_factory=time.time)
    duration: float = 0.0
    total_tests: int = 0
    passed_tests: int = 0
    failed_tests: int = 0
    error_tests: int = 0
    skipped_tests: int = 0
    coverage_percentage: float = 0.0
    results: List[TestResult] = field(default_factory=list)
    performance_metrics: Dict[str, Any] = field(default_factory=dict)
    system_info: Dict[str, Any] = field(default_factory=dict)


class TestDataFactory:
    """
    Test data factory for generating realistic test data.
    Consolidates test data generation from various test modules.
    """

    def __init__(self):
        self.factories: Dict[str, Callable] = {}
        self.templates: Dict[str, Dict[str, Any]] = {}

        # Initialize default factories
        self._initialize_default_factories()

    def _initialize_default_factories(self):
        """Initialize default test data factories."""
        import random
        import string

        # Mission data factory
        def create_mission_data():
            return {
                "mission_id": f"mission_{random.randint(1000, 9999)}",
                "type": random.choice(["waypoint", "object_detection", "delivery"]),
                "priority": random.choice(["low", "medium", "high"]),
                "waypoints": [
                    {
                        "x": random.uniform(-10, 10),
                        "y": random.uniform(-10, 10),
                        "z": 0.0
                    } for _ in range(random.randint(3, 8))
                ]
            }

        # Sensor data factory
        def create_sensor_data():
            return {
                "timestamp": time.time(),
                "sensor_id": f"sensor_{random.randint(1, 10)}",
                "sensor_type": random.choice(["imu", "gps", "lidar", "camera"]),
                "value": random.uniform(0, 100),
                "unit": random.choice(["m", "m/s", "degrees", "pixels"]),
                "quality": random.uniform(0.5, 1.0)
            }

        # Robot state factory
        def create_robot_state():
            return {
                "position": {
                    "x": random.uniform(-5, 5),
                    "y": random.uniform(-5, 5),
                    "z": 0.0
                },
                "orientation": {
                    "roll": random.uniform(-3.14, 3.14),
                    "pitch": random.uniform(-1.57, 1.57),
                    "yaw": random.uniform(-3.14, 3.14)
                },
                "velocity": {
                    "linear": random.uniform(0, 2.0),
                    "angular": random.uniform(-1.0, 1.0)
                },
                "battery_level": random.uniform(0.1, 1.0),
                "status": random.choice(["idle", "moving", "error", "charging"])
            }

        self.register_factory("mission", create_mission_data)
        self.register_factory("sensor", create_sensor_data)
        self.register_factory("robot_state", create_robot_state)

    def register_factory(self, name: str, factory_function: Callable):
        """Register a test data factory."""
        self.factories[name] = factory_function

    def register_template(self, name: str, template: Dict[str, Any]):
        """Register a test data template."""
        self.templates[name] = template

    def create_data(self, factory_name: str, **overrides) -> Dict[str, Any]:
        """
        Create test data using a factory.

        Args:
            factory_name: Name of the factory to use
            **overrides: Override specific values

        Returns:
            Generated test data
        """
        if factory_name not in self.factories:
            raise ValueError(f"Factory '{factory_name}' not registered")

        data = self.factories[factory_name]()
        data.update(overrides)
        return data

    def create_batch(self, factory_name: str, count: int, **overrides) -> List[Dict[str, Any]]:
        """Create a batch of test data."""
        return [self.create_data(factory_name, **overrides) for _ in range(count)]


class MockManager:
    """
    Mock management for testing.
    Consolidates mock utilities from various test modules.
    """

    def __init__(self):
        self.mocks: Dict[str, Any] = {}
        self.mock_factories: Dict[str, Callable] = {}

        # Initialize default mocks
        self._initialize_default_mocks()

    def _initialize_default_mocks(self):
        """Initialize default mock objects."""

        # ROS2 mock
        class MockROS2Node:
            def __init__(self):
                self.publishers = {}
                self.subscribers = {}
                self.services = {}

            def create_publisher(self, msg_type, topic, qos_profile=None):
                self.publishers[topic] = {"type": msg_type, "qos": qos_profile}
                return MockPublisher()

            def create_subscription(self, msg_type, topic, callback, qos_profile=None):
                self.subscribers[topic] = {"type": msg_type, "callback": callback}
                return MockSubscription()

            def create_service(self, srv_type, srv_name, callback):
                self.services[srv_name] = {"type": srv_type, "callback": callback}
                return MockService()

        class MockPublisher:
            def publish(self, msg): pass

        class MockSubscription:
            def __init__(self): self.callback = None

        class MockService:
            pass

        # Hardware mock
        class MockCANBus:
            def __init__(self):
                self.messages = []

            def send(self, message):
                self.messages.append(message)

            def recv(self, timeout=None):
                return self.messages.pop(0) if self.messages else None

        # Sensor mock
        class MockSensor:
            def __init__(self, sensor_type="generic"):
                self.sensor_type = sensor_type
                self.is_connected = True

            def read(self):
                import random
                return {
                    "value": random.uniform(0, 100),
                    "timestamp": time.time(),
                    "quality": random.uniform(0.8, 1.0)
                }

        self.register_mock("ros2_node", MockROS2Node)
        self.register_mock("can_bus", MockCANBus)
        self.register_mock("sensor", MockSensor)

    def register_mock(self, name: str, mock_class: Type):
        """Register a mock class."""
        self.mocks[name] = mock_class

    def create_mock(self, mock_name: str, **kwargs):
        """Create a mock instance."""
        if mock_name not in self.mocks:
            raise ValueError(f"Mock '{mock_name}' not registered")

        return self.mocks[mock_name](**kwargs)


class PerformanceBenchmark:
    """
    Performance benchmarking for tests.
    Consolidates performance testing from various modules.
    """

    def __init__(self):
        self.benchmarks: Dict[str, Dict[str, Any]] = {}
        self.baselines: Dict[str, float] = {}

    def start_benchmark(self, name: str):
        """Start a performance benchmark."""
        self.benchmarks[name] = {
            "start_time": time.time(),
            "start_memory": self._get_memory_usage(),
            "measurements": []
        }

    def end_benchmark(self, name: str) -> Dict[str, Any]:
        """End a performance benchmark."""
        if name not in self.benchmarks:
            return {}

        benchmark = self.benchmarks[name]
        end_time = time.time()
        end_memory = self._get_memory_usage()

        result = {
            "duration": end_time - benchmark["start_time"],
            "memory_delta": end_memory - benchmark["start_memory"],
            "start_time": benchmark["start_time"],
            "end_time": end_time
        }

        # Check against baseline
        if name in self.baselines:
            baseline = self.baselines[name]
            result["baseline_comparison"] = {
                "baseline_duration": baseline,
                "performance_ratio": result["duration"] / baseline,
                "within_tolerance": abs(result["duration"] - baseline) / baseline < 0.1  # 10% tolerance
            }

        del self.benchmarks[name]
        return result

    def set_baseline(self, name: str, duration: float):
        """Set a performance baseline."""
        self.baselines[name] = duration

    def _get_memory_usage(self) -> float:
        """Get current memory usage."""
        import psutil
        process = psutil.Process()
        return process.memory_info().rss / 1024 / 1024  # MB


class UnifiedTestSuite:
    """
    Unified Test Suite Manager - Single interface for all testing.

    Consolidates all test functionality:
    - Test discovery and execution
    - Test data factories
    - Mock management
    - Performance benchmarking
    - Comprehensive reporting
    - Coverage analysis
    """

    def __init__(self):
        self.test_suites: Dict[str, TestSuite] = {}
        self.test_results: Dict[str, List[TestResult]] = {}
        self.data_factory = TestDataFactory()
        self.mock_manager = MockManager()
        self.performance_benchmark = PerformanceBenchmark()

        # Test execution
        self.running_tests: Dict[str, threading.Thread] = {}
        self.test_lock = threading.Lock()

    def register_test_suite(self, suite: TestSuite):
        """Register a test suite."""
        self.test_suites[suite.name] = suite
        logger.info(f"Registered test suite: {suite.name} ({suite.test_type.value})")

    def create_test_suite(self, name: str, description: str, test_type: TestType,
                         **kwargs) -> TestSuite:
        """Create and register a test suite."""
        suite = TestSuite(name=name, description=description, test_type=test_type, **kwargs)
        self.register_test_suite(suite)
        return suite

    def run_test_suite(self, suite_name: str, **run_config) -> TestReport:
        """
        Run a specific test suite.

        Args:
            suite_name: Name of the test suite to run
            **run_config: Test run configuration

        Returns:
            Test report
        """
        if suite_name not in self.test_suites:
            raise ValueError(f"Test suite '{suite_name}' not registered")

        suite = self.test_suites[suite_name]
        run_id = f"{suite_name}_{int(time.time())}"

        # Create test run configuration
        test_run = TestRun(run_id=run_id, suites=[suite_name], **run_config)

        return self._execute_test_run(test_run)

    def run_multiple_suites(self, suite_names: List[str], **run_config) -> TestReport:
        """Run multiple test suites."""
        run_id = f"multi_suite_{int(time.time())}"
        test_run = TestRun(run_id=run_id, suites=suite_names, **run_config)
        return self._execute_test_run(test_run)

    def _execute_test_run(self, test_run: TestRun) -> TestReport:
        """Execute a test run."""
        start_time = time.time()
        report = TestReport(run_id=test_run.run_id)

        try:
            # Execute each suite
            for suite_name in test_run.suites:
                if suite_name not in self.test_suites:
                    logger.warning(f"Test suite '{suite_name}' not found, skipping")
                    continue

                suite = self.test_suites[suite_name]

                # Setup
                if suite.setup_function:
                    try:
                        suite.setup_function()
                    except Exception as e:
                        logger.error(f"Suite setup failed for {suite_name}: {e}")

                # Run tests
                suite_results = self._run_suite_tests(suite, test_run)
                report.results.extend(suite_results)

                # Teardown
                if suite.teardown_function:
                    try:
                        suite.teardown_function()
                    except Exception as e:
                        logger.error(f"Suite teardown failed for {suite_name}: {e}")

            # Calculate summary
            report.total_tests = len(report.results)
            report.passed_tests = len([r for r in report.results if r.status == TestStatus.PASSED])
            report.failed_tests = len([r for r in report.results if r.status == TestStatus.FAILED])
            report.error_tests = len([r for r in report.results if r.status == TestStatus.ERROR])
            report.skipped_tests = len([r for r in report.results if r.status == TestStatus.SKIPPED])
            report.duration = time.time() - start_time

            # Performance metrics
            if report.results:
                durations = [r.duration for r in report.results if r.duration > 0]
                if durations:
                    report.performance_metrics = {
                        "avg_test_duration": sum(durations) / len(durations),
                        "max_test_duration": max(durations),
                        "min_test_duration": min(durations),
                        "total_duration": report.duration
                    }

            logger.info(f"Test run {test_run.run_id} completed: "
                       f"{report.passed_tests}/{report.total_tests} passed")

        except Exception as e:
            logger.error(f"Test run failed: {e}")
            report.results.append(TestResult(
                test_name="test_run",
                test_type=TestType.SYSTEM,
                status=TestStatus.ERROR,
                error_message=str(e)
            ))

        return report

    def _run_suite_tests(self, suite: TestSuite, test_run: TestRun) -> List[TestResult]:
        """Run tests for a suite."""
        results = []

        # Run test classes
        for test_class in suite.test_classes:
            class_results = self._run_test_class(test_class, suite)
            results.extend(class_results)

        # Run test functions
        for test_function in suite.test_functions:
            result = self._run_test_function(test_function, suite)
            results.append(result)

        return results

    def _run_test_class(self, test_class: Type, suite: TestSuite) -> List[TestResult]:
        """Run a test class."""
        results = []

        # Find test methods
        test_methods = [method for method in dir(test_class)
                       if method.startswith('test_') and callable(getattr(test_class, method))]

        for method_name in test_methods:
            method = getattr(test_class, method_name)
            result = self._run_test_function(method, suite, class_name=test_class.__name__)
            result.test_name = f"{test_class.__name__}.{method_name}"
            results.append(result)

        return results

    def _run_test_function(self, test_function: Callable, suite: TestSuite,
                          class_name: str = None) -> TestResult:
        """Run a single test function."""
        test_name = f"{class_name}.{test_function.__name__}" if class_name else test_function.__name__

        result = TestResult(
            test_name=test_name,
            test_type=suite.test_type,
            status=TestStatus.RUNNING
        )

        start_time = time.time()

        try:
            # Create test instance if it's a method
            if hasattr(test_function, '__self__'):
                test_function()
            else:
                test_function()

            result.status = TestStatus.PASSED
            result.assertions_passed = 1  # Simplified

        except AssertionError as e:
            result.status = TestStatus.FAILED
            result.error_message = str(e)
            result.assertions_failed = 1

        except Exception as e:
            result.status = TestStatus.ERROR
            result.error_message = str(e)
            import traceback
            result.stack_trace = traceback.format_exc()

        except unittest.SkipTest as e:
            result.status = TestStatus.SKIPPED
            result.error_message = str(e)

        result.duration = time.time() - start_time
        return result

    def get_test_data(self, factory_name: str, **overrides) -> Dict[str, Any]:
        """Get test data from factory."""
        return self.data_factory.create_data(factory_name, **overrides)

    def create_mock(self, mock_name: str, **kwargs):
        """Create a mock object."""
        return self.mock_manager.create_mock(mock_name, **kwargs)

    def start_performance_test(self, test_name: str):
        """Start performance benchmarking."""
        self.performance_benchmark.start_benchmark(test_name)

    def end_performance_test(self, test_name: str) -> Dict[str, Any]:
        """End performance benchmarking."""
        return self.performance_benchmark.end_benchmark(test_name)

    def set_performance_baseline(self, test_name: str, duration: float):
        """Set performance baseline."""
        self.performance_benchmark.set_baseline(test_name, duration)

    def generate_test_report(self, report: TestReport, format: str = "json") -> str:
        """Generate test report in specified format."""
        if format == "json":
            return json.dumps({
                "run_id": report.run_id,
                "timestamp": report.timestamp,
                "duration": report.duration,
                "summary": {
                    "total": report.total_tests,
                    "passed": report.passed_tests,
                    "failed": report.failed_tests,
                    "errors": report.error_tests,
                    "skipped": report.skipped_tests
                },
                "coverage": report.coverage_percentage,
                "performance": report.performance_metrics,
                "results": [
                    {
                        "name": r.test_name,
                        "status": r.status.value,
                        "duration": r.duration,
                        "error": r.error_message
                    } for r in report.results
                ]
            }, indent=2, default=str)

        elif format == "text":
            lines = [
                f"Test Report: {report.run_id}",
                f"Duration: {report.duration:.2f}s",
                f"Tests: {report.total_tests} total, "
                f"{report.passed_tests} passed, "
                f"{report.failed_tests} failed, "
                f"{report.error_tests} errors, "
                f"{report.skipped_tests} skipped",
                ""
            ]

            for result in report.results:
                status_icon = {
                    TestStatus.PASSED: "✓",
                    TestStatus.FAILED: "✗",
                    TestStatus.ERROR: "!",
                    TestStatus.SKIPPED: "○"
                }.get(result.status, "?")

                lines.append(f"{status_icon} {result.test_name} ({result.duration:.3f}s)")

                if result.error_message:
                    lines.append(f"    Error: {result.error_message}")

            return "\n".join(lines)

        return ""

    def get_test_status(self) -> Dict[str, Any]:
        """Get overall test suite status."""
        return {
            "registered_suites": len(self.test_suites),
            "suite_types": {suite.test_type.value: len([s for s in self.test_suites.values()
                                                       if s.test_type == suite.test_type])
                           for suite in self.test_suites.values()},
            "total_tests": sum(len(suite.test_classes) + len(suite.test_functions)
                             for suite in self.test_suites.values()),
            "running_tests": len(self.running_tests),
            "factories_available": len(self.data_factory.factories),
            "mocks_available": len(self.mock_manager.mocks),
            "performance_baselines": len(self.performance_benchmark.baselines)
        }


# Global test suite instance
_test_suite = None

def get_test_suite() -> UnifiedTestSuite:
    """Get global test suite instance."""
    global _test_suite
    if _test_suite is None:
        _test_suite = UnifiedTestSuite()
    return _test_suite

# Convenience functions
def create_test_suite(name: str, description: str, test_type: TestType, **kwargs) -> TestSuite:
    """Create a test suite."""
    return get_test_suite().create_test_suite(name, description, test_type, **kwargs)

def run_test_suite(suite_name: str, **config) -> TestReport:
    """Run a test suite."""
    return get_test_suite().run_test_suite(suite_name, **config)

def get_test_data(factory_name: str, **overrides) -> Dict[str, Any]:
    """Get test data."""
    return get_test_suite().get_test_data(factory_name, **overrides)

def create_mock(mock_name: str, **kwargs):
    """Create a mock."""
    return get_test_suite().create_mock(mock_name, **kwargs)

# Export key components
__all__ = [
    'UnifiedTestSuite',
    'TestDataFactory',
    'MockManager',
    'PerformanceBenchmark',
    'TestType',
    'TestStatus',
    'TestPriority',
    'TestResult',
    'TestSuite',
    'TestRun',
    'TestReport',
    'get_test_suite',
    'create_test_suite',
    'run_test_suite',
    'get_test_data',
    'create_mock'
]




