.. _quality_standards:

==================
Quality Standards
==================

This document outlines the coding standards, practices, and quality requirements for the URC 2026 Mars Rover project. These standards ensure maintainable, testable, and reliable code that new team members can understand and contribute to.

Code Quality Standards
======================

Python Code Style
-----------------

All Python code must follow these standards:

**Imports**
```python
# 1. Standard library imports
import logging
from typing import Dict, List, Optional

# 2. Third-party imports (alphabetical)
import numpy as np
import rclpy

# 3. Local imports (alphabetical)
from autonomy_interfaces.msg import SensorData
from mission_executor import MissionExecutor
```

**Type Hints**
```python
# ✅ Good: Always use type hints for public APIs
def process_sensor_data(self, data: SensorData) -> bool:
    """Process sensor data and return success status."""
    # Implementation
    pass

# ✅ Good: Use Union for multiple types
def navigate_to_point(self, x: float, y: float, timeout: Optional[float] = None) -> NavigationResult:
    """Navigate to point with optional timeout."""
    pass

# ✅ Good: Use generics for collections
def get_sensor_readings(self) -> List[SensorData]:
    """Get all available sensor readings."""
    pass
```

**Docstrings**
```python
# ✅ Good: Complete docstring with Args/Returns/Raises
def execute_mission(self, mission_type: str, params: Dict[str, Any]) -> MissionResult:
    """
    Execute a mission with given parameters.

    Args:
        mission_type: Type of mission to execute ('sample_collection', 'delivery', etc.)
        params: Mission-specific parameters

    Returns:
        MissionResult with execution status and data

    Raises:
        MissionError: If mission execution fails
        ValidationError: If parameters are invalid
    """
    pass

# ❌ Bad: Missing details
def execute_mission(self, mission_type, params):
    """Execute mission"""
    pass
```

**Error Handling**
```python
# ✅ Good: Specific exceptions with context
try:
    result = self.execute_navigation(waypoint)
except NavigationTimeoutError as e:
    self.get_logger().error(f"Navigation timeout after {e.timeout}s: {e}")
    self.handle_navigation_failure()
    raise MissionError(f"Navigation failed: {e}") from e
except HardwareError as e:
    self.get_logger().critical(f"Hardware failure: {e}")
    self.emergency_stop()
    raise

# ❌ Bad: Bare except or generic exceptions
try:
    result = self.execute_navigation(waypoint)
except:
    raise Exception("Navigation failed")  # Too generic
```

ROS2-Specific Standards
=======================

Package Structure
-----------------

Every ROS2 package must follow this structure:

```
autonomy_package_name/
├── package.xml                 # Package metadata and dependencies
├── CMakeLists.txt              # Build configuration
├── README.md                   # Package documentation
├── setup.py                    # Python package setup
├── resource/                   # Package resource files
├── launch/                     # Launch files
├── config/                     # Configuration files
├── autonomy_package_name/      # Python package
│   ├── __init__.py
│   ├── node.py                 # Main ROS2 node
│   ├── types.py                # Type definitions
│   └── utils.py                # Utilities
└── tests/                      # Unit and integration tests
    ├── test_node.py
    └── test_integration.py
```

**package.xml Standards**
```xml
<?xml version="1.0"?>
<package format="3">
  <name>autonomy_navigation</name>
  <version>1.0.0</version>
  <description>Navigation system for URC 2026</description>

  <!-- Required dependencies -->
  <depend>rclpy</depend>
  <depend>autonomy_interfaces</depend>
  <depend>geometry_msgs</depend>

  <!-- Test dependencies -->
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
</package>
```

Node Implementation
-------------------

**Node Class Structure**
```python
import logging
from typing import Any, Dict

import rclpy
from rclpy.node import Node

logger = logging.getLogger(__name__)


class NavigationNode(Node):
    """ROS2 node for autonomous navigation."""

    def __init__(self) -> None:
        """Initialize navigation node with ROS2 interfaces."""
        super().__init__("navigation_node")

        # Declare parameters with defaults
        self.declare_parameter("max_velocity", 2.0)
        self.declare_parameter("timeout", 30.0)

        # Create publishers/subscribers/services
        self._setup_interfaces()

        # Initialize state
        self.current_waypoint = None
        self.is_navigating = False

        self.get_logger().info("Navigation node initialized")

    def _setup_interfaces(self) -> None:
        """Setup ROS2 publishers, subscribers, and services."""
        # Publishers
        self.status_pub = self.create_publisher(
            String, "/navigation/status", QoSProfile(depth=10)
        )

        # Subscribers
        self.waypoint_sub = self.create_subscription(
            PoseStamped, "/navigation/waypoint", self._waypoint_callback, 10
        )

        # Services
        self.navigate_srv = self.create_service(
            NavigateToPose, "/navigation/navigate", self._navigate_callback
        )

    def _waypoint_callback(self, msg: PoseStamped) -> None:
        """Handle incoming waypoint messages."""
        try:
            # Process waypoint
            pass
        except Exception as e:
            self.get_logger().error(f"Waypoint processing failed: {e}")

    def _navigate_callback(
        self,
        request: NavigateToPose.Request,
        response: NavigateToPose.Response
    ) -> NavigateToPose.Response:
        """Handle navigation service requests."""
        try:
            # Execute navigation
            response.success = True
            return response
        except Exception as e:
            self.get_logger().error(f"Navigation service failed: {e}")
            response.success = False
            response.message = str(e)
            return response
```

**QoS Settings**
```python
# For real-time control
control_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    depth=1  # Keep only latest message
)

# For status monitoring
status_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=10  # Keep history for monitoring
)
```

Mission Standards
=================

Mission Class Interface
-----------------------

All mission classes must implement this interface:

```python
from abc import ABC, abstractmethod
from typing import Any, Dict, Optional
from enum import Enum

import rclpy
from rclpy.node import Node


class MissionState(Enum):
    """Standard mission states."""
    IDLE = "idle"
    ACTIVE = "active"
    PAUSED = "paused"
    COMPLETED = "completed"
    FAILED = "failed"


class BaseMission(Node, ABC):
    """Base class for all URC missions."""

    def __init__(self, mission_name: str) -> None:
        """Initialize mission with standard parameters."""
        super().__init__(f"{mission_name}_mission")

        # Standard mission parameters
        self.declare_parameter("timeout", 300.0)
        self.declare_parameter("retry_count", 3)

        # Mission state
        self.state = MissionState.IDLE
        self.start_time = None
        self.error_count = 0

        # Setup standard interfaces
        self._setup_mission_interfaces()

    def _setup_mission_interfaces(self) -> None:
        """Setup standard mission ROS2 interfaces."""
        # Status publisher
        self.status_pub = self.create_publisher(
            String, f"/mission/{self.get_name()}/status", 10
        )

        # Control subscriber
        self.control_sub = self.create_subscription(
            String, "/mission/control", self._control_callback, 10
        )

    @abstractmethod
    def execute_mission(self, params: Dict[str, Any]) -> bool:
        """
        Execute the mission with given parameters.

        Args:
            params: Mission-specific parameters

        Returns:
            True if mission completed successfully
        """
        pass

    def start_mission(self, params: Optional[Dict[str, Any]] = None) -> bool:
        """Start mission execution."""
        if self.state != MissionState.IDLE:
            self.get_logger().warning("Mission already active")
            return False

        try:
            self.state = MissionState.ACTIVE
            self.start_time = self.get_clock().now()
            self._publish_status()

            success = self.execute_mission(params or {})
            self.state = MissionState.COMPLETED if success else MissionState.FAILED
            self._publish_status()

            return success
        except Exception as e:
            self.get_logger().error(f"Mission execution failed: {e}")
            self.state = MissionState.FAILED
            self._publish_status()
            return False

    def stop_mission(self) -> None:
        """Stop mission execution."""
        self.state = MissionState.IDLE
        self._publish_status()

    def _control_callback(self, msg: String) -> None:
        """Handle mission control commands."""
        command = msg.data.lower()
        if command == "start":
            self.start_mission()
        elif command == "stop":
            self.stop_mission()
        elif command == "pause":
            self.state = MissionState.PAUSED
        elif command == "resume" and self.state == MissionState.PAUSED:
            self.state = MissionState.ACTIVE

        self._publish_status()

    def _publish_status(self) -> None:
        """Publish current mission status."""
        status_msg = String()
        status_msg.data = self.state.value
        self.status_pub.publish(status_msg)
```

Testing Standards
=================

Unit Tests
----------

```python
import pytest
import rclpy
from unittest.mock import MagicMock

from autonomy_navigation.navigation_node import NavigationNode


class TestNavigationNode:
    """Test cases for NavigationNode."""

    @pytest.fixture
    def node(self):
        """Create test node."""
        rclpy.init()
        node = NavigationNode()
        yield node
        node.destroy_node()
        rclpy.shutdown()

    def test_initialization(self, node):
        """Test node initializes correctly."""
        assert node.get_name() == "navigation_node"
        assert node.max_velocity == 2.0

    def test_waypoint_navigation(self, node):
        """Test waypoint navigation execution."""
        waypoint = PoseStamped()
        # Setup waypoint...

        result = node.navigate_to_waypoint(waypoint)
        assert result.success == True

    @pytest.mark.integration
    def test_hardware_integration(self, node):
        """Test integration with hardware interfaces."""
        # This would test actual hardware if available
        pass
```

Integration Tests
-----------------

```python
import pytest
import rclpy
import time

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest


@pytest.mark.launch_test
def generate_test_description():
    """Generate launch description for integration tests."""
    return LaunchDescription([
        Node(
            package='autonomy_navigation',
            executable='navigation_node',
            name='test_navigation'
        ),
        Node(
            package='mission_executor',
            executable='mission_executor',
            name='test_executor'
        ),
        ReadyToTest()
    ])


class TestNavigationIntegration:
    """Integration tests for navigation system."""

    def test_mission_execution_flow(self, launch_service):
        """Test complete mission execution flow."""
        # Setup test scenario
        # Execute mission
        # Verify results
        pass
```

Code Review Checklist
=====================

Pre-Commit Checklist
--------------------

- [ ] Code formatted with black (`black .`)
- [ ] Imports sorted with isort (`isort .`)
- [ ] Linting passes (`flake8 .`)
- [ ] Type checking passes (`mypy .`)
- [ ] Tests pass (`pytest`)
- [ ] Coverage >90% (`pytest --cov`)

Code Review Checklist
---------------------

**Code Quality**
- [ ] Follows established patterns and standards
- [ ] Proper error handling with specific exceptions
- [ ] Comprehensive logging with appropriate levels
- [ ] Type hints on all public APIs
- [ ] Docstrings complete and accurate

**Architecture**
- [ ] Clean separation of concerns
- [ ] Proper dependency injection
- [ ] No circular dependencies
- [ ] Follows ROS2 best practices

**Testing**
- [ ] Unit tests for all new functionality
- [ ] Integration tests for system interactions
- [ ] Error conditions tested
- [ ] Test coverage maintained

**Documentation**
- [ ] Code is self-documenting
- [ ] README updated for new features
- [ ] API documentation complete
- [ ] Breaking changes documented

**Performance**
- [ ] No obvious performance issues
- [ ] Resource usage reasonable
- [ ] Real-time constraints met

Quality Gates
=============

**Definition of Done**
1. Code reviewed and approved
2. All tests pass (unit + integration)
3. Documentation updated
4. Linting and type checking pass
5. No critical security issues
6. Performance requirements met

**Quality Metrics**
- **Code Coverage**: >90% for new code
- **Linting Score**: 100% clean (flake8)
- **Type Coverage**: 100% public APIs (mypy)
- **Documentation**: 100% public APIs (sphinx)
- **Performance**: Meet real-time requirements

**Automated Checks**
```bash
# Run all quality checks
./scripts/check_quality.sh

# Individual checks
black --check .                    # Code formatting
isort --check-only .              # Import sorting
flake8 .                          # Linting
mypy .                            # Type checking
pytest --cov=. --cov-fail-under=90  # Testing
```

Continuous Integration
======================

**GitHub Actions Workflow**
```yaml
name: Quality Checks
on: [push, pull_request]

jobs:
  quality:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-python@v4
        with:
          python-version: '3.10'

      - name: Install dependencies
        run: |
          pip install -e .[dev]
          rosdep install --from-paths src --ignore-src -y

      - name: Code formatting
        run: black --check .

      - name: Import sorting
        run: isort --check-only .

      - name: Linting
        run: flake8 .

      - name: Type checking
        run: mypy .

      - name: Testing
        run: pytest --cov=. --cov-fail-under=90

      - name: Build
        run: colcon build --symlink-install
```

Getting Help
============

**Quality Issues**
- Check this document first
- Look at existing code for examples
- Ask in code review for guidance

**Tool Issues**
- `black --help` for formatting options
- `mypy --help` for type checking
- `pytest --help` for testing options

**Standards Questions**
- Check existing approved PRs for examples
- Review recent code reviews for patterns
- Ask architecture questions in design discussions

Remember: **Quality is everyone's responsibility**. Following these standards ensures our code is maintainable, reliable, and accessible to new team members! 🚀




