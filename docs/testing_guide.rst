.. _testing_guide:

================
Testing Team Guide
================

Welcome to the Testing Team! You are the **quality gatekeepers** for the entire rover system. While other teams build features, you ensure nothing breaks, performance is maintained, and the rover is safe to operate. Testing is not just about finding bugs - it's about proving the system works reliably under all conditions.

.. image:: _static/test_state.png
   :alt: Testing State Machine
   :align: center
   :width: 80%

What You Own
============

**Quality Assurance Infrastructure:**
- Unit testing framework and coverage metrics
- Integration testing across system boundaries
- Performance benchmarking and regression detection
- Hardware-in-the-loop (HIL) testing
- Chaos engineering and failure simulation
- Continuous integration pipeline maintenance
- Test automation and reporting

**Key Files:**
- ``tests/`` - Complete test suite organization
- ``tests/conftest.py`` - Test configuration and fixtures
- ``tests/run_tests.py`` - Test execution orchestration
- ``scripts/testing/`` - Test automation scripts
- ``tools/`` - Testing utilities and helpers

Your Day-to-Day
===============

**Quality Health Checks:**
```bash
# Run full test suite
python -m pytest tests/ -v --tb=short

# Check test coverage
python -m pytest tests/ --cov=src --cov-report=html

# Monitor performance regressions
python tools/performance/benchmark.py

# Check CI status
python scripts/testing/check_ci_status.py
```

**Common Issues You Prevent/Fix:**
- Code regressions breaking existing functionality
- Performance degradation over time
- Integration issues between system components
- Hardware compatibility problems
- Race conditions and concurrency issues
- Memory leaks and resource exhaustion

Week 1: Getting Started
=======================

**Day 1: Understand Testing Philosophy**
1. Read the Big Picture guide (focus on quality across all layers)
2. Study the testing pyramid (unit → integration → system → chaos)
3. Run ``python -m pytest tests/unit/ -v`` to see basic tests
4. Execute ``python -m pytest tests/ --cov=src`` to check coverage

**Day 2: Learn Test Organization**
```bash
# Understand test structure
ls -la tests/
# unit/ integration/ hardware/ performance/ chaos/ etc.

# Run different test types
python -m pytest tests/unit/ -v        # Fast unit tests
python -m pytest tests/integration/ -v # Component integration
python -m pytest tests/hardware/ -v    # Hardware validation
python -m pytest tests/performance/ -v # Performance checks
```

**Day 3: Test Automation Basics**
```bash
# Run comprehensive test suite
python tests/run_comprehensive_integration.sh

# Generate test reports
python -m pytest tests/ --junitxml=reports/test_results.xml

# Check test coverage
python -m pytest tests/ --cov=src --cov-report=term-missing
```

**Day 4-5: Your First Test Contribution**
1. Find an untested function in the codebase
2. Write a unit test for it
3. Add it to the appropriate test file
4. Ensure it passes and improves coverage

Testing Philosophy & Strategy
=============================

**Testing Pyramid:**
```
   Chaos Tests (Few)     - System resilience under extreme conditions
   ┌─────────────────────────────────────────────────────────────┐
   │ Integration Tests     - Component interactions and boundaries │
   ┌─────────────────────────────────────────────────────────────┐
   │ Unit Tests (Many)     - Individual function/method correctness │
   └─────────────────────────────────────────────────────────────┘
```

**Test Categories:**
- **Unit Tests**: Test individual functions/methods in isolation
- **Integration Tests**: Test interactions between components
- **System Tests**: Test end-to-end workflows
- **Performance Tests**: Ensure speed and resource usage requirements
- **Chaos Tests**: Test system behavior under failure conditions
- **Hardware Tests**: Validate real hardware integration

**Quality Gates:**
- Code coverage >80% for new code
- All tests pass on CI/CD pipeline
- Performance regressions <5% degradation
- No critical security vulnerabilities
- Documentation updated for API changes

Test Organization & Structure
=============================

**Test Directory Structure:**
```
tests/
├── conftest.py              # Shared test configuration
├── unit/                    # Unit tests (fast, isolated)
│   ├── test_navigation.py
│   ├── test_state_machine.py
│   └── ...
├── integration/             # Integration tests (slower, realistic)
│   ├── test_mission_execution.py
│   ├── test_network_integration.py
│   └── ...
├── hardware/                # Hardware validation tests
│   ├── test_arm_hardware.py
│   └── test_gps_hardware.py
├── performance/             # Performance benchmarks
│   ├── test_navigation_performance.py
│   └── test_network_throughput.py
├── chaos/                   # Failure simulation tests
│   ├── test_chaos_engineering.py
│   └── test_network_chaos.py
├── extreme/                 # Extreme condition tests
│   ├── test_competition_extremes.py
│   └── test_resource_exhaustion.py
└── factories/               # Test data factories
    ├── basic_test_factories.py
    └── test_data_factories.py
```

**Test Naming Conventions:**
- ``test_function_name.py`` for unit tests
- ``test_component_integration.py`` for integration tests
- ``test_feature_end_to_end.py`` for system tests
- ``test_performance_metric.py`` for performance tests

Writing Effective Tests
=======================

**Unit Test Best Practices:**
```python
import pytest
from unittest.mock import Mock, patch

def test_calculate_distance():
    """Test distance calculation between two points."""
    # Arrange
    point1 = Point(x=0, y=0)
    point2 = Point(x=3, y=4)
    expected_distance = 5.0

    # Act
    result = calculate_distance(point1, point2)

    # Assert
    assert result == pytest.approx(expected_distance, abs=0.01)

def test_navigation_with_mocked_sensors():
    """Test navigation logic with mocked sensor inputs."""
    # Arrange
    mock_gps = Mock()
    mock_gps.get_position.return_value = Position(lat=42.0, lon=-71.0)

    navigator = Navigator()

    # Act
    with patch.object(navigator, 'gps', mock_gps):
        success = navigator.navigate_to_goal(goal)

    # Assert
    assert success is True
    mock_gps.get_position.assert_called_once()
```

**Integration Test Patterns:**
```python
@pytest.mark.integration
def test_navigation_and_arm_integration(ros_test_fixture):
    """Test navigation and arm working together."""
    # Setup ROS2 test environment
    with ros_test_fixture:
        # Start navigation and arm nodes
        nav_node = launch_navigation_node()
        arm_node = launch_arm_node()

        # Execute integrated mission
        mission = SampleCollectionMission()
        result = mission.execute()

        # Verify both systems worked together
        assert result.success
        assert nav_node.reached_goal()
        assert arm_node.sample_collected()
```

**Performance Test Templates:**
```python
import time
import statistics
from tools.performance.profiler import PerformanceProfiler

def test_navigation_performance_under_load():
    """Test navigation performance with multiple concurrent goals."""
    profiler = PerformanceProfiler()

    with profiler.measure("navigation_multi_goal"):
        # Generate multiple navigation goals
        goals = generate_test_goals(count=10)

        # Execute goals concurrently
        start_time = time.time()
        results = execute_goals_concurrently(goals)
        total_time = time.time() - start_time

        # Performance assertions
        assert total_time < 30.0  # 30 second max for 10 goals
        assert all(result.success for result in results)

        # Resource usage checks
        cpu_usage = profiler.get_cpu_usage()
        memory_usage = profiler.get_memory_usage()

        assert cpu_usage < 80.0  # Max 80% CPU
        assert memory_usage < 500  # Max 500MB RAM
```

Test Fixtures & Utilities
=========================

**Common Test Fixtures:**
```python
# conftest.py
import pytest
import rclpy
from unittest.mock import Mock

@pytest.fixture
def ros_context():
    """Provide ROS2 context for tests."""
    rclpy.init()
    yield
    rclpy.shutdown()

@pytest.fixture
def mock_gps():
    """Mock GPS sensor for testing."""
    gps = Mock()
    gps.get_position.return_value = Position(lat=42.0, lon=-71.0)
    gps.get_accuracy.return_value = 1.5  # meters
    return gps

@pytest.fixture
def navigation_test_setup(ros_context, mock_gps):
    """Complete navigation test setup."""
    navigator = Navigator()
    navigator.gps = mock_gps
    return navigator
```

**Test Data Factories:**
```python
# tests/factories/navigation_factories.py
from tests.factories.basic_test_factories import Point, Pose

def create_test_goal(x=10.0, y=5.0, theta=0.0):
    """Create a test navigation goal."""
    return Pose(
        position=Point(x=x, y=y, z=0.0),
        orientation=create_test_orientation(theta)
    )

def create_obstacle_field(radius=2.0, count=5):
    """Create a field of obstacles for testing."""
    obstacles = []
    for i in range(count):
        angle = (2 * 3.14159 * i) / count
        x = radius * cos(angle)
        y = radius * sin(angle)
        obstacles.append(Point(x=x, y=y, z=0.0))
    return obstacles
```

Continuous Integration
======================

**CI/CD Pipeline:**
```yaml
# .github/workflows/ci.yml
name: CI
on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2
      - name: Setup Python
        uses: actions/setup-python@v2
        with:
          python-version: '3.10'
      - name: Install dependencies
        run: pip install -e .[dev]
      - name: Run tests
        run: python -m pytest tests/ --cov=src --cov-report=xml
      - name: Upload coverage
        uses: codecov/codecov-action@v1
```

**Quality Gates:**
- **Test Coverage**: >80% overall, >90% for critical components
- **Performance**: No regressions >5% from baseline
- **Security**: Pass bandit security scanning
- **Style**: Pass black, flake8, mypy checks
- **Documentation**: API docs build successfully

Debugging Test Failures
=======================

**Common Test Issues & Solutions:**

**Flaky Tests:**
```python
# Use retry decorator for network-dependent tests
@pytest.mark.flaky(reruns=3, reruns_delay=1)
def test_network_resilience():
    # Test that may occasionally fail due to network timing
    pass

# Stabilize timing-dependent tests
def test_timing_dependent_feature():
    with freeze_time("2023-01-01 12:00:00"):
        # Test logic that depends on time
        pass
```

**Async Test Issues:**
```python
# Proper async test handling
@pytest.mark.asyncio
async def test_async_operation():
    result = await async_operation()
    assert result.success

# Test timeouts for async operations
@pytest.mark.timeout(30)
@pytest.mark.asyncio
async def test_long_running_operation():
    result = await long_operation()
    assert result.completed
```

**ROS2 Test Challenges:**
```python
# ROS2 integration test setup
def test_ros2_integration():
    # Use proper ROS2 test fixtures
    with launch_testing.tools.launch_file_from_package(
        'autonomy', 'test_launch.py'
    ) as launch_file:
        with launch_testing.tools.ready_to_test(
            processes=[launch_file]
        ):
            # Test ROS2 node interactions
            assert node_communication_works()
```

Performance Testing & Benchmarking
===================================

**Performance Test Categories:**
- **Latency Tests**: Response time for critical operations
- **Throughput Tests**: Operations per second under load
- **Resource Tests**: CPU, memory, network usage
- **Scalability Tests**: Performance with increased load
- **Regression Tests**: Detect performance degradation

**Benchmarking Framework:**
```python
# tools/performance/benchmark.py
import time
import psutil
import statistics

class PerformanceProfiler:
    def __init__(self):
        self.metrics = {}

    def measure(self, name):
        return PerformanceContext(self, name)

    def record_metric(self, name, value, unit):
        if name not in self.metrics:
            self.metrics[name] = []
        self.metrics[name].append((value, unit))

    def get_statistics(self, name):
        values = [v[0] for v in self.metrics[name]]
        return {
            'mean': statistics.mean(values),
            'median': statistics.median(values),
            'stddev': statistics.stdev(values),
            'min': min(values),
            'max': max(values)
        }
```

Chaos Engineering
=================

**Chaos Test Patterns:**
```python
# Network failure simulation
def test_navigation_with_network_failures():
    with network_failure_simulation():
        navigator = Navigator()
        # Test graceful degradation
        assert navigator.handles_network_failure()

# Resource exhaustion testing
def test_system_under_memory_pressure():
    with memory_limit_simulation(limit_mb=100):
        # Test system behavior under memory pressure
        assert system_gracefully_handles_low_memory()

# Component failure simulation
def test_arm_fallback_when_navigation_fails():
    with component_failure_simulation('navigation'):
        arm_controller = ArmController()
        # Test fallback behaviors
        assert arm_controller.uses_backup_navigation()
```

**Chaos Test Framework:**
```python
# tests/chaos/test_chaos_engineering.py
class ChaosTestSuite:
    def test_system_resilience_under_chaos(self):
        """Test system behavior when multiple failures occur."""
        failures = [
            network_partition(),
            cpu_spike(),
            memory_pressure(),
            disk_full()
        ]

        with simulate_multiple_failures(failures):
            # Test that critical functions still work
            assert emergency_stop_works()
            assert safety_system_active()
            assert can_return_to_safe_state()
```

Hardware-in-the-Loop Testing
============================

**HIL Test Setup:**
```python
# tests/hardware/hardware_interface_simulator.py
class HardwareInterfaceSimulator:
    def __init__(self):
        self.motor_positions = {}
        self.sensor_readings = {}

    def simulate_motor_response(self, motor_id, command):
        # Simulate real motor behavior
        self.motor_positions[motor_id] = self.calculate_new_position(
            self.motor_positions.get(motor_id, 0),
            command
        )
        return self.motor_positions[motor_id]

    def simulate_sensor_noise(self, sensor_type):
        # Add realistic noise to sensor readings
        base_reading = self.get_base_sensor_reading(sensor_type)
        noise = random.gauss(0, self.get_sensor_noise_stddev(sensor_type))
        return base_reading + noise
```

**HIL Test Execution:**
```python
def test_arm_control_hil():
    """Test arm control with hardware simulation."""
    simulator = HardwareInterfaceSimulator()
    arm_controller = ArmController(hardware_interface=simulator)

    # Test arm movement
    goal_position = [0.5, 0.3, 0.2]
    success = arm_controller.move_to_position(goal_position)

    assert success
    assert arm_controller.at_goal_tolerance(goal_position, tolerance=0.01)
```

Test Reporting & Analytics
==========================

**Test Report Generation:**
```python
# tools/testing/generate_report.py
def generate_test_report(test_results):
    """Generate comprehensive test report."""
    report = {
        'summary': {
            'total_tests': len(test_results),
            'passed': sum(1 for r in test_results if r.passed),
            'failed': sum(1 for r in test_results if not r.passed),
            'coverage': calculate_coverage(test_results)
        },
        'performance_trends': analyze_performance_trends(),
        'failure_analysis': categorize_failures(test_results),
        'recommendations': generate_recommendations(test_results)
    }

    return report
```

**Test Dashboard:**
```python
# tools/testing/test_dashboard.py
def create_test_dashboard():
    """Create interactive test results dashboard."""
    dashboard = TestDashboard()

    # Add test metrics
    dashboard.add_metric('test_coverage', get_current_coverage())
    dashboard.add_metric('performance_score', get_performance_score())
    dashboard.add_metric('failure_rate', get_failure_rate())

    # Add trend charts
    dashboard.add_chart('coverage_trend', get_coverage_history())
    dashboard.add_chart('performance_trend', get_performance_history())

    return dashboard.render()
```

Development Workflow
====================

**Testing Code Changes:**
1. **Write tests first**: TDD approach for new features
2. **Run relevant tests**: Don't run entire suite for small changes
3. **Check coverage**: Ensure new code is adequately tested
4. **Performance validation**: Verify no performance regressions
5. **Integration testing**: Test with dependent systems

**Test Code Review Checklist:**
- [ ] Tests exist for new functionality
- [ ] Test coverage maintained or improved
- [ ] Performance tests included for critical paths
- [ ] Integration tests for component interactions
- [ ] Edge cases and error conditions tested
- [ ] Test documentation clear and helpful
- [ ] No flaky or slow tests introduced

Getting Help
============

**Testing-Specific Resources:**
- pytest documentation and best practices
- ROS2 testing framework guides
- Performance testing methodologies
- Chaos engineering principles

**Team Resources:**
- Test automation scripts in ``scripts/testing/``
- Performance profiling tools in ``tools/performance/``
- Test data factories in ``tests/factories/``
- CI/CD pipeline configuration

**When You're Stuck:**
1. Check test logs: ``tail -f log/test_*.log``
2. Run tests in isolation: ``python -m pytest tests/unit/test_specific.py -v -s``
3. Debug with pdb: ``python -m pytest tests/ --pdb``
4. Check CI failures: Review GitHub Actions logs
5. Ask in #testing-team channel

Remember: You are the last line of defense against bugs and performance issues. Your thorough testing gives the team confidence to deploy changes and ensures the rover performs reliably in competition. Quality is not just a goal - it's your responsibility!

