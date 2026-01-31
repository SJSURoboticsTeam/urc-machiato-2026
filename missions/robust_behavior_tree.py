#!/usr/bin/env python3
"""
Enhanced Behavior Tree System - URC 2026

Py-trees based behavior tree implementation with comprehensive failure handling:
- Mature, battle-tested BT library (used by Boston Dynamics, NASA)
- ROS2 integration through py_trees_ros
- State persistence and restoration
- Execution monitoring and timeout handling
- Circuit breaker integration for robustness
- Hierarchical error propagation with recovery

Author: URC 2026 Risk Mitigation Team
"""

import time
import threading
import logging
from typing import Any, Dict, List, Optional, Callable, Union
from enum import Enum
from dataclasses import dataclass, field

# Py-trees imports (HIGH PRIORITY REPLACEMENT)
try:
    import py_trees
    from py_trees.blackboard import Blackboard
    from py_trees.behaviours import Running, Success, Failure
    from py_trees.composites import Sequence, Selector, Parallel
    from py_trees.decorators import Timeout, Retry, Condition
    from py_trees.trees import BehaviourTree
    PY_TREES_AVAILABLE = True
    PY_TREES_BLACKBOARD_AVAILABLE = True
except ImportError:
    PY_TREES_AVAILABLE = False
    PY_TREES_BLACKBOARD_AVAILABLE = False
    logger.warning("py-trees not available, falling back to basic implementation")

# Py-trees ROS2 integration
try:
    import py_trees_ros
    from py_trees_ros.behaviours import ToBlackboard, FromBlackboard
    from py_trees_ros.actions import ActionClient
    from py_trees_ros.trees import BehaviourTree as ROSBehaviourTree
    PY_TREES_ROS_AVAILABLE = True
except ImportError:
    PY_TREES_ROS_AVAILABLE = False
    # Logger not yet defined, use print or skip warning
    import warnings
    warnings.warn("py-trees-ros not available, ROS2 integration limited", ImportWarning)

from src.core.error_handling import circuitbreaker
from src.infrastructure.config import get_config

# Import mission resource manager
try:
    from src.core.mission_resource_manager import get_mission_resource_manager
    RESOURCE_MANAGER_AVAILABLE = True
except ImportError:
    RESOURCE_MANAGER_AVAILABLE = False

logger = logging.getLogger(__name__)


# Use py-trees status when available, fallback to custom
if PY_TREES_AVAILABLE:
    BTNodeStatus = py_trees.common.Status
else:
    class BTNodeStatus(Enum):
        """Behavior tree node execution status."""
        SUCCESS = "success"
        FAILURE = "failure"
        RUNNING = "running"
        IDLE = "idle"


class BTNodeType(Enum):
    """Behavior tree node types."""
    ACTION = "action"
    CONDITION = "condition"
    COMPOSITE = "composite"
    DECORATOR = "decorator"


# Enhanced BT Node with py-trees integration
class EnhancedBTNode(py_trees.behaviour.Behaviour if PY_TREES_AVAILABLE else object):
    """Enhanced behavior tree node with circuit breaker and monitoring."""

    def __init__(self, name: str, node_type: BTNodeType = BTNodeType.ACTION):
        if PY_TREES_AVAILABLE:
            super().__init__(name)
        else:
            self.name = name

        self.node_type = node_type
        self.execution_count = 0
        self.success_count = 0
        self.failure_count = 0
        self.total_execution_time = 0.0
        self.last_execution_time = 0.0
        self.circuit_breaker_enabled = False
        self.timeout_seconds = None
        self.retry_count = 0
        self.max_retries = 3

        # Circuit breaker for robustness
        self.circuit_breaker = circuitbreaker.CircuitBreaker(
            failure_threshold=5,
            recovery_timeout=30.0
        )

        # Monitoring
        self.execution_history: List[Dict[str, Any]] = []
        self.performance_metrics: Dict[str, Any] = {}

    def execute_with_monitoring(self, context: Dict[str, Any]) -> BTNodeStatus:
        """Execute node with comprehensive monitoring and circuit breaker."""
        start_time = time.time()
        self.execution_count += 1

        try:
            # Check circuit breaker
            if self.circuit_breaker_enabled:
                result = self.circuit_breaker.call(self._execute_impl, context)
            else:
                result = self._execute_impl(context)

            execution_time = time.time() - start_time
            self.total_execution_time += execution_time
            self.last_execution_time = execution_time

            # Update metrics
            if result == BTNodeStatus.SUCCESS:
                self.success_count += 1
            else:
                self.failure_count += 1

            # Record execution
            self._record_execution(result, execution_time, context)

            return result

        except circuitbreaker.CircuitBreakerOpenException:
            logger.warning(f"Circuit breaker open for node {self.name}")
            return BTNodeStatus.FAILURE
        except Exception as e:
            logger.error(f"Node {self.name} execution failed: {e}")
            return BTNodeStatus.FAILURE

    def _execute_impl(self, context: Dict[str, Any]) -> BTNodeStatus:
        """Implementation-specific execution logic. Override in subclasses."""
        raise NotImplementedError("Subclasses must implement _execute_impl")

    def _record_execution(self, status: BTNodeStatus, execution_time: float,
                         context: Dict[str, Any]):
        """Record execution for monitoring and debugging."""
        record = {
            'timestamp': time.time(),
            'status': status.value if hasattr(status, 'value') else str(status),
            'execution_time': execution_time,
            'context_snapshot': dict(list(context.items())[:5])  # Limit context size
        }

        self.execution_history.append(record)

        # Keep only recent history
        if len(self.execution_history) > 100:
            self.execution_history = self.execution_history[-100:]

    def get_performance_metrics(self) -> Dict[str, Any]:
        """Get performance metrics for this node."""
        total_executions = self.execution_count
        if total_executions == 0:
            return {'success_rate': 0.0, 'avg_execution_time': 0.0}

        success_rate = self.success_count / total_executions
        avg_execution_time = self.total_execution_time / total_executions

        return {
            'success_rate': success_rate,
            'avg_execution_time': avg_execution_time,
            'total_executions': total_executions,
            'circuit_breaker_enabled': self.circuit_breaker_enabled,
            'circuit_breaker_state': self.circuit_breaker.get_state().value
        }

    def reset_metrics(self):
        """Reset performance metrics."""
        self.execution_count = 0
        self.success_count = 0
        self.failure_count = 0
        self.total_execution_time = 0.0
        self.execution_history.clear()

    # Py-trees compatibility methods
    if PY_TREES_AVAILABLE:
        def update(self):
            """Py-trees update method."""
            context = {}  # Would be passed from tree execution context
            result = self.execute_with_monitoring(context)

            # Convert to py-trees status
            if result == BTNodeStatus.SUCCESS:
                return py_trees.common.Status.SUCCESS
            elif result == BTNodeStatus.FAILURE:
                return py_trees.common.Status.FAILURE
            else:
                return py_trees.common.Status.RUNNING

    def __str__(self):
        metrics = self.get_performance_metrics()
        return (f"{self.__class__.__name__}('{self.name}', "
                ".1f"                f"success_rate={metrics['success_rate']:.1%})")


@dataclass
class BTNodeResult:
    """Result of behavior tree node execution (legacy compatibility)."""
    status: BTNodeStatus
    data: Dict[str, Any] = field(default_factory=dict)
    error_message: Optional[str] = None
    execution_time: float = 0.0
    retry_count: int = 0


@dataclass
class BTExecutionContext:
    """Execution context for behavior tree nodes."""
    node_id: str
    start_time: float
    timeout: Optional[float] = None
    retry_count: int = 0
    max_retries: int = 3
    parent_context: Optional['BTExecutionContext'] = None
    blackboard: Any = field(default=None)  # py_trees.blackboard.Blackboard or dict
    
    def __post_init__(self):
        """Initialize blackboard based on availability."""
        if self.blackboard is None:
            if PY_TREES_BLACKBOARD_AVAILABLE:
                self.blackboard = Blackboard()
            else:
                self.blackboard = {}


class BTNode(ABC):
    """Base class for all behavior tree nodes with robust error handling."""

    def __init__(self, node_id: str, node_type: BTNodeType):
        self.node_id = node_id
        self.node_type = node_type
        self.logger = logging.getLogger(f"{__name__}.{node_id}")

        # Execution tracking
        self.execution_count = 0
        self.success_count = 0
        self.failure_count = 0
        self.last_execution_time = 0
        self.average_execution_time = 0.0

        # Failure handling
        self.failure_handlers: List[Callable] = []
        self.recovery_actions: List[Callable] = []

        # Monitoring
        self.timeout = None
        self.max_retries = 3
        self.circuit_breaker_enabled = False
        self.failure_threshold = 5
        self.recovery_timeout = 30.0
        self.consecutive_failures = 0
        self.last_failure_time = 0
        self.circuit_open = False

    @abstractmethod
    def execute(self, context: BTExecutionContext) -> BTNodeResult:
        """Execute the node logic."""
        pass

    def execute_with_monitoring(self, context: BTExecutionContext) -> BTNodeResult:
        """Execute node with comprehensive monitoring and error handling."""
        self.execution_count += 1
        start_time = time.time()

        # Check circuit breaker
        if self.circuit_breaker_enabled and self._is_circuit_open():
            return BTNodeResult(
                status=BTNodeStatus.FAILURE,
                error_message="Circuit breaker open",
                execution_time=time.time() - start_time
            )

        try:
            # Execute with timeout monitoring
            if self.timeout:
                context.timeout = self.timeout

            result = self._execute_with_timeout(context)

            # Update statistics
            execution_time = time.time() - start_time
            self.last_execution_time = execution_time
            self.average_execution_time = (
                (self.average_execution_time * (self.execution_count - 1)) + execution_time
            ) / self.execution_count

            # Handle result
            if result.status == BTNodeStatus.SUCCESS:
                self.success_count += 1
                self.consecutive_failures = 0
                self._close_circuit()
            else:
                self.failure_count += 1
                self.consecutive_failures += 1
                self.last_failure_time = time.time()

                # Execute failure handlers
                self._execute_failure_handlers(result)

                # Check if circuit should open
                if self.consecutive_failures >= self.failure_threshold:
                    self._open_circuit()

            result.execution_time = execution_time
            return result

        except Exception as e:
            execution_time = time.time() - start_time
            self.logger.error(f"Node {self.node_id} execution failed: {e}")

            result = BTNodeResult(
                status=BTNodeStatus.FAILURE,
                error_message=str(e),
                execution_time=execution_time
            )

            # Execute failure handlers
            self._execute_failure_handlers(result)

            return result

    def _execute_with_timeout(self, context: BTExecutionContext) -> BTNodeResult:
        """Execute with timeout protection."""
        if not context.timeout:
            return self.execute(context)

        # Execute in separate thread with timeout
        result = [None]
        exception = [None]

        def execute_thread():
            try:
                result[0] = self.execute(context)
            except Exception as e:
                exception[0] = e

        thread = threading.Thread(target=execute_thread, daemon=True)
        thread.start()
        thread.join(timeout=context.timeout)

        if thread.is_alive():
            # Timeout occurred
            return BTNodeResult(
                status=BTNodeStatus.FAILURE,
                error_message=f"Execution timeout after {context.timeout}s"
            )

        if exception[0]:
            raise exception[0]

        return result[0]

    def _is_circuit_open(self) -> bool:
        """Check if circuit breaker is open."""
        if not self.circuit_breaker_enabled:
            return False

        if not self.circuit_open:
            return False

        # Check if recovery timeout has passed
        if time.time() - self.last_failure_time > self.recovery_timeout:
            self.logger.info(f"Circuit breaker recovery timeout passed for {self.node_id}")
            self._close_circuit()
            return False

        return True

    def _open_circuit(self):
        """Open circuit breaker."""
        if not self.circuit_breaker_enabled:
            return

        self.circuit_open = True
        self.logger.warning(f"Circuit breaker opened for {self.node_id} after {self.consecutive_failures} failures")

    def _close_circuit(self):
        """Close circuit breaker."""
        if self.circuit_open:
            self.circuit_open = False
            self.consecutive_failures = 0
            self.logger.info(f"Circuit breaker closed for {self.node_id}")

    def _execute_failure_handlers(self, result: BTNodeResult):
        """Execute registered failure handlers."""
        for handler in self.failure_handlers:
            try:
                handler(result)
            except Exception as e:
                self.logger.error(f"Failure handler error: {e}")

    def add_failure_handler(self, handler: Callable):
        """Add a failure handler function."""
        self.failure_handlers.append(handler)

    def add_recovery_action(self, action: Callable):
        """Add a recovery action function."""
        self.recovery_actions.append(action)

    def enable_circuit_breaker(self, failure_threshold: int = 5, recovery_timeout: float = 30.0):
        """Enable circuit breaker pattern."""
        self.circuit_breaker_enabled = True
        self.failure_threshold = failure_threshold
        self.recovery_timeout = recovery_timeout

    def get_health_status(self) -> Dict[str, Any]:
        """Get node health status."""
        total_executions = self.execution_count
        success_rate = self.success_count / total_executions if total_executions > 0 else 0

        health_status = "healthy"
        if success_rate < 0.5:
            health_status = "unhealthy"
        elif success_rate < 0.8:
            health_status = "degraded"

        return {
            'node_id': self.node_id,
            'health_status': health_status,
            'success_rate': success_rate,
            'total_executions': total_executions,
            'consecutive_failures': self.consecutive_failures,
            'circuit_breaker_open': self.circuit_open,
            'average_execution_time': self.average_execution_time
        }


class BTActionNode(BTNode):
    """Action node that performs operations."""

    def __init__(self, node_id: str, action_func: Callable):
        super().__init__(node_id, BTNodeType.ACTION)
        self.action_func = action_func

    def execute(self, context: BTExecutionContext) -> BTNodeResult:
        """Execute the action function."""
        try:
            result = self.action_func(context)
            if isinstance(result, dict):
                status = result.get('status', BTNodeStatus.SUCCESS)
                data = result.get('data', {})
                return BTNodeResult(status=status, data=data)
            elif isinstance(result, bool):
                status = BTNodeStatus.SUCCESS if result else BTNodeStatus.FAILURE
                return BTNodeResult(status=status)
            else:
                return BTNodeResult(status=BTNodeStatus.SUCCESS, data={'result': result})
        except Exception as e:
            return BTNodeResult(
                status=BTNodeStatus.FAILURE,
                error_message=str(e)
            )


class BTConditionNode(BTNode):
    """Condition node that checks conditions."""

    def __init__(self, node_id: str, condition_func: Callable):
        super().__init__(node_id, BTNodeType.CONDITION)
        self.condition_func = condition_func

    def execute(self, context: BTExecutionContext) -> BTNodeResult:
        """Execute the condition check."""
        try:
            result = self.condition_func(context)
            if isinstance(result, bool):
                status = BTNodeStatus.SUCCESS if result else BTNodeStatus.FAILURE
            else:
                status = BTNodeStatus.SUCCESS  # Assume truthy result is success
            return BTNodeResult(status=status, data={'condition_result': result})
        except Exception as e:
            return BTNodeResult(
                status=BTNodeStatus.FAILURE,
                error_message=str(e)
            )


class BTCompositeNode(BTNode):
    """Base class for composite nodes (Sequence, Selector, etc.)."""

    def __init__(self, node_id: str, children: List[BTNode]):
        super().__init__(node_id, BTNodeType.COMPOSITE)
        self.children = children

    def add_child(self, child: BTNode):
        """Add a child node."""
        self.children.append(child)

    def get_child_health_status(self) -> List[Dict[str, Any]]:
        """Get health status of all children."""
        return [child.get_health_status() for child in self.children]


class BTSequence(BTCompositeNode):
    """Sequence composite - executes children in order until one fails."""

    def execute(self, context: BTExecutionContext) -> BTNodeResult:
        """Execute children in sequence."""
        results = []

        for i, child in enumerate(self.children):
            child_context = BTExecutionContext(
                node_id=f"{context.node_id}_child_{i}",
                start_time=time.time(),
                timeout=context.timeout,
                parent_context=context,
                blackboard=context.blackboard
            )

            result = child.execute_with_monitoring(child_context)
            results.append(result)

            if result.status == BTNodeStatus.FAILURE:
                return BTNodeResult(
                    status=BTNodeStatus.FAILURE,
                    data={'failed_child': i, 'results': results},
                    error_message=f"Child {child.node_id} failed: {result.error_message}"
                )
            elif result.status == BTNodeStatus.RUNNING:
                return BTNodeResult(
                    status=BTNodeStatus.RUNNING,
                    data={'running_child': i, 'results': results}
                )

        return BTNodeResult(
            status=BTNodeStatus.SUCCESS,
            data={'results': results}
        )


class BTSelector(BTCompositeNode):
    """Selector composite - executes children until one succeeds."""

    def execute(self, context: BTExecutionContext) -> BTNodeResult:
        """Execute children until one succeeds."""
        results = []

        for i, child in enumerate(self.children):
            child_context = BTExecutionContext(
                node_id=f"{context.node_id}_child_{i}",
                start_time=time.time(),
                timeout=context.timeout,
                parent_context=context,
                blackboard=context.blackboard
            )

            result = child.execute_with_monitoring(child_context)
            results.append(result)

            if result.status == BTNodeStatus.SUCCESS:
                return BTNodeResult(
                    status=BTNodeStatus.SUCCESS,
                    data={'successful_child': i, 'results': results}
                )
            elif result.status == BTNodeStatus.RUNNING:
                return BTNodeResult(
                    status=BTNodeStatus.RUNNING,
                    data={'running_child': i, 'results': results}
                )

        return BTNodeResult(
            status=BTNodeStatus.FAILURE,
            data={'results': results},
            error_message="All children failed"
        )


class BTDecorator(BTNode):
    """Base class for decorator nodes."""

    def __init__(self, node_id: str, child: BTNode):
        super().__init__(node_id, BTNodeType.DECORATOR)
        self.child = child


class BTRetryDecorator(BTDecorator):
    """Retry decorator - retries child execution on failure."""

    def __init__(self, node_id: str, child: BTNode, max_retries: int = 3):
        super().__init__(node_id, child)
        self.max_retries = max_retries

    def execute(self, context: BTExecutionContext) -> BTNodeResult:
        """Execute child with retry logic."""
        last_result = None

        for attempt in range(self.max_retries + 1):  # +1 for initial attempt
            child_context = BTExecutionContext(
                node_id=f"{context.node_id}_attempt_{attempt}",
                start_time=time.time(),
                timeout=context.timeout,
                retry_count=attempt,
                max_retries=self.max_retries,
                parent_context=context,
                blackboard=context.blackboard
            )

            result = self.child.execute_with_monitoring(child_context)
            last_result = result

            if result.status == BTNodeStatus.SUCCESS:
                return BTNodeResult(
                    status=BTNodeStatus.SUCCESS,
                    data=result.data,
                    retry_count=attempt
                )
            elif result.status == BTNodeStatus.RUNNING:
                return BTNodeResult(
                    status=BTNodeStatus.RUNNING,
                    data=result.data,
                    retry_count=attempt
                )

        # All retries failed
        return BTNodeResult(
            status=BTNodeStatus.FAILURE,
            data=last_result.data if last_result else {},
            error_message=f"Failed after {self.max_retries + 1} attempts",
            retry_count=self.max_retries
        )


class BTTimeoutDecorator(BTDecorator):
    """Timeout decorator - adds timeout to child execution."""

    def __init__(self, node_id: str, child: BTNode, timeout: float):
        super().__init__(node_id, child)
        self.timeout = timeout

    def execute(self, context: BTExecutionContext) -> BTNodeResult:
        """Execute child with timeout."""
        child_context = BTExecutionContext(
            node_id=f"{context.node_id}_timeout",
            start_time=time.time(),
            timeout=self.timeout,
            parent_context=context,
            blackboard=context.blackboard
        )

        result = self.child.execute_with_monitoring(child_context)
        return result


class BTInverterDecorator(BTDecorator):
    """Inverter decorator - inverts child result."""

    def execute(self, context: BTExecutionContext) -> BTNodeResult:
        """Execute child and invert result."""
        result = self.child.execute_with_monitoring(context)

        inverted_status = (
            BTNodeStatus.FAILURE if result.status == BTNodeStatus.SUCCESS
            else BTNodeStatus.SUCCESS if result.status == BTNodeStatus.FAILURE
            else result.status  # RUNNING remains RUNNING
        )

        return BTNodeResult(
            status=inverted_status,
            data=result.data,
            error_message=result.error_message
        )


class BehaviorTree:
    """Robust behavior tree implementation with failure recovery."""

    def __init__(self, root: BTNode, name: str = "BehaviorTree"):
        self.root = root
        self.name = name
        self.logger = logging.getLogger(f"{__name__}.{name}")

        # Execution state
        self.is_running = False
        self.execution_context = None
        self.last_execution_result = None

        # State persistence
        self.state_persistence_enabled = False
        self.state_file = f"{name}_state.json"

        # Monitoring
        self.execution_monitor = BTExecutionMonitor()

        # Recovery
        self.recovery_actions: List[Callable] = []

    def execute(self, blackboard: Optional[Any] = None) -> BTNodeResult:
        """Execute the behavior tree with monitoring."""
        if blackboard is None:
            if PY_TREES_BLACKBOARD_AVAILABLE:
                blackboard = Blackboard()
            else:
                blackboard = {}

        self.is_running = True

        # Create execution context
        self.execution_context = BTExecutionContext(
            node_id=self.name,
            start_time=time.time(),
            blackboard=blackboard
        )

        try:
            # Load persisted state if enabled
            if self.state_persistence_enabled:
                self._load_execution_state()

            # Execute with monitoring
            self.execution_monitor.start_execution(self.name)
            result = self.root.execute_with_monitoring(self.execution_context)
            self.execution_monitor.end_execution(result)

            # Persist state if enabled
            if self.state_persistence_enabled:
                self._save_execution_state(result)

            self.last_execution_result = result
            return result

        except Exception as e:
            self.logger.error(f"Behavior tree execution failed: {e}")
            result = BTNodeResult(
                status=BTNodeStatus.FAILURE,
                error_message=str(e)
            )
            self.execution_monitor.end_execution(result)
            return result
        finally:
            self.is_running = False

    def tick(self) -> BTNodeResult:
        """Tick the behavior tree (for continuous execution)."""
        if not self.is_running:
            return self.execute()

        # Continue execution if running
        if self.last_execution_result and self.last_execution_result.status == BTNodeStatus.RUNNING:
            # Re-execute to continue
            return self.execute(self.execution_context.blackboard)

        return self.last_execution_result

    def stop(self):
        """Stop behavior tree execution."""
        self.is_running = False
        self.logger.info(f"Behavior tree {self.name} stopped")

    def get_health_status(self) -> Dict[str, Any]:
        """Get comprehensive health status of the behavior tree."""
        root_health = self.root.get_health_status()

        return {
            'tree_name': self.name,
            'is_running': self.is_running,
            'root_health': root_health,
            'execution_monitor': self.execution_monitor.get_stats(),
            'last_execution': {
                'status': self.last_execution_result.status.value if self.last_execution_result else None,
                'execution_time': self.last_execution_result.execution_time if self.last_execution_result else 0,
                'error_message': self.last_execution_result.error_message if self.last_execution_result else None
            } if self.last_execution_result else None
        }

    def enable_state_persistence(self, state_file: Optional[str] = None):
        """Enable state persistence for fault recovery."""
        self.state_persistence_enabled = True
        if state_file:
            self.state_file = state_file
        self.logger.info(f"State persistence enabled: {self.state_file}")

    def add_recovery_action(self, action: Callable):
        """Add a recovery action for execution failures."""
        self.recovery_actions.append(action)

    def execute_recovery(self) -> bool:
        """Execute recovery actions."""
        success = False
        for action in self.recovery_actions:
            try:
                if action():
                    success = True
                    break
            except Exception as e:
                self.logger.error(f"Recovery action failed: {e}")

        return success

    def _load_execution_state(self):
        """Load persisted execution state."""
        try:
            import json
            with open(self.state_file, 'r') as f:
                state = json.load(f)
                # Restore relevant state
                self.execution_context.blackboard.update(state.get('blackboard', {}))
                self.logger.info("Execution state loaded successfully")
        except Exception as e:
            self.logger.warning(f"Failed to load execution state: {e}")

    def _save_execution_state(self, result: BTNodeResult):
        """Save execution state for persistence."""
        try:
            import json
            state = {
                'timestamp': time.time(),
                'blackboard': self.execution_context.blackboard,
                'last_result': {
                    'status': result.status.value,
                    'execution_time': result.execution_time,
                    'error_message': result.error_message
                }
            }
            with open(self.state_file, 'w') as f:
                json.dump(state, f, indent=2)
        except Exception as e:
            self.logger.error(f"Failed to save execution state: {e}")


class BTExecutionMonitor:
    """Monitor behavior tree execution performance and health."""

    def __init__(self):
        self.executions = []
        self.current_execution = None

    def start_execution(self, tree_name: str):
        """Start monitoring execution."""
        self.current_execution = {
            'tree_name': tree_name,
            'start_time': time.time(),
            'status': None,
            'execution_time': 0
        }

    def end_execution(self, result: BTNodeResult):
        """End monitoring execution."""
        if self.current_execution:
            self.current_execution['status'] = result.status.value
            self.current_execution['execution_time'] = result.execution_time
            self.current_execution['error_message'] = result.error_message

            self.executions.append(self.current_execution)
            self.current_execution = None

            # Keep only last 100 executions
            if len(self.executions) > 100:
                self.executions = self.executions[-100:]

    def get_stats(self) -> Dict[str, Any]:
        """Get execution statistics."""
        if not self.executions:
            return {'total_executions': 0}

        total_time = sum(ex['execution_time'] for ex in self.executions)
        success_count = sum(1 for ex in self.executions if ex['status'] == 'success')
        failure_count = sum(1 for ex in self.executions if ex['status'] == 'failure')

        return {
            'total_executions': len(self.executions),
            'success_rate': success_count / len(self.executions),
            'failure_rate': failure_count / len(self.executions),
            'average_execution_time': total_time / len(self.executions),
            'total_execution_time': total_time
        }


# Example usage and test functions
def create_robust_navigation_tree() -> BehaviorTree:
    """Create a robust navigation behavior tree with failure handling."""

    # Action nodes with failure handling
    def navigate_to_waypoint(context):
        try:
            waypoint = context.blackboard.get('current_waypoint')
            if not waypoint:
                return {'status': BTNodeStatus.FAILURE, 'error': 'No waypoint set'}

            # Simulate navigation logic
            time.sleep(0.1)  # Simulate work

            # Check for navigation success (simplified)
            if context.blackboard.get('navigation_simulate_failure', False):
                return {'status': BTNodeStatus.FAILURE, 'error': 'Navigation failed'}

            return {'status': BTNodeStatus.SUCCESS, 'data': {'waypoint_reached': waypoint}}
        except Exception as e:
            return {'status': BTNodeStatus.FAILURE, 'error': str(e)}

    def check_obstacles(context):
        obstacles = context.blackboard.get('obstacles', [])
        if obstacles:
            return False  # Obstacles detected
        return True

    def emergency_stop(context):
        context.blackboard['emergency_stop_triggered'] = True
        return {'status': BTNodeStatus.SUCCESS}

    # Create nodes
    navigate_action = BTActionNode("navigate_to_waypoint", navigate_to_waypoint)
    navigate_action.enable_circuit_breaker(failure_threshold=3, recovery_timeout=10.0)

    obstacle_condition = BTConditionNode("check_obstacles", check_obstacles)
    emergency_action = BTActionNode("emergency_stop", emergency_stop)

    # Create composite with retry decorator
    retry_navigate = BTRetryDecorator("retry_navigate", navigate_action, max_retries=2)

    # Sequence: Check obstacles -> Navigate with retry -> Emergency stop fallback
    navigation_sequence = BTSequence("navigation_sequence", [
        obstacle_condition,
        retry_navigate
    ])

    # Selector: Try normal navigation, fallback to emergency stop
    navigation_selector = BTSelector("navigation_selector", [
        navigation_sequence,
        emergency_action
    ])

    # Create behavior tree
    tree = BehaviorTree(navigation_selector, "RobustNavigationTree")

    # Add recovery actions
    def reset_navigation():
        # Reset navigation state
        return True

    tree.add_recovery_action(reset_navigation)

    return tree


if __name__ == "__main__":
    # Example usage
    logging.basicConfig(level=logging.INFO)

    tree = create_robust_navigation_tree()

    # Test successful execution
    blackboard = {'current_waypoint': {'x': 10, 'y': 5}}
    result = tree.execute(blackboard)
    print(f"Execution result: {result.status}")

    # Test failure with recovery
    blackboard['navigation_simulate_failure'] = True
    result = tree.execute(blackboard)
    print(f"Failure result: {result.status}")

    # Check health status
    health = tree.get_health_status()
    print(f"Tree health: {health}")


# =============================================================================
# PY-TREES ENHANCED IMPLEMENTATIONS (HIGH PRIORITY REPLACEMENT)
# =============================================================================

class PyTreesBehaviorTree:
    """
    Py-trees based behavior tree implementation.

    This replaces the custom BT implementation with the mature py-trees library,
    providing better reliability, ROS2 integration, and visualization capabilities.
    """

    def __init__(self, root_node: Optional['py_trees.behaviour.Behaviour'] = None, name: str = "URC_BT"):
        if not PY_TREES_AVAILABLE:
            raise ImportError("py-trees library not available. Install with: pip install py-trees py-trees-ros")

        self.name = name
        self.logger = logging.getLogger(f"{__name__}.{name}")

        # Initialize py-trees components
        self.root = root_node or py_trees.composites.Selector(name="RootSelector")
        self.tree = py_trees.trees.BehaviourTree(self.root)

        # Enhanced monitoring
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key="mission_state", access=py_trees.blackboard.Access.WRITE)
        self.blackboard.register_key(key="health_status", access=py_trees.blackboard.Access.WRITE)

        # Mission resource manager integration
        if RESOURCE_MANAGER_AVAILABLE:
            self.resource_manager = get_mission_resource_manager()
            self.logger.info("Mission Resource Manager integrated with behavior tree")
        else:
            self.resource_manager = None
            self.logger.warning("Mission Resource Manager not available - basic BT operation only")

        # Performance tracking
        self.execution_count = 0
        self.success_count = 0
        self.failure_count = 0
        self.execution_times: List[float] = []

        # Circuit breaker for tree-level protection
        self.circuit_breaker = circuitbreaker.CircuitBreaker(
            failure_threshold=10,
            recovery_timeout=60.0
        )

        # Setup tree introspection
        self.tree.add_pre_tick_handler(self._pre_tick_handler)
        self.tree.add_post_tick_handler(self._post_tick_handler)

        self.logger.info(f"Py-trees behavior tree '{name}' initialized")

    def _pre_tick_handler(self, tree: py_trees.trees.BehaviourTree):
        """Handler called before each tree tick."""
        self.execution_count += 1
        self._tick_start_time = time.time()

    def _post_tick_handler(self, tree: py_trees.trees.BehaviourTree):
        """Handler called after each tree tick."""
        execution_time = time.time() - self._tick_start_time
        self.execution_times.append(execution_time)

        # Keep only recent execution times
        if len(self.execution_times) > 100:
            self.execution_times = self.execution_times[-50:]

        # Update success/failure counts
        if tree.root.status == py_trees.common.Status.SUCCESS:
            self.success_count += 1
        elif tree.root.status == py_trees.common.Status.FAILURE:
            self.failure_count += 1

    def execute(self, blackboard_data: Optional[Dict[str, Any]] = None) -> BTNodeResult:
        """Execute the behavior tree with circuit breaker protection."""
        try:
            # Update blackboard if data provided
            if blackboard_data:
                for key, value in blackboard_data.items():
                    self.blackboard.set(key, value)

            # Execute with circuit breaker
            result = self.circuit_breaker.call(self._execute_impl)

            # Convert py-trees status to our status
            if result == py_trees.common.Status.SUCCESS:
                status = BTNodeStatus.SUCCESS
            elif result == py_trees.common.Status.FAILURE:
                status = BTNodeStatus.FAILURE
            else:
                status = BTNodeStatus.RUNNING

            return BTNodeResult(
                status=status,
                data={'tree_status': result, 'blackboard': dict(self.blackboard)},
                execution_time=self.execution_times[-1] if self.execution_times else 0.0
            )

        except circuitbreaker.CircuitBreakerOpenException:
            self.logger.warning("Behavior tree circuit breaker is open")
            return BTNodeResult(
                status=BTNodeStatus.FAILURE,
                error_message="Circuit breaker open",
                execution_time=0.0
            )

    def _execute_impl(self) -> py_trees.common.Status:
        """Internal tree execution."""
        self.tree.tick()
        return self.tree.root.status

    def add_node(self, node: 'py_trees.behaviour.Behaviour', parent: Optional['py_trees.behaviour.Behaviour'] = None):
        """Add a node to the tree."""
        parent = parent or self.root
        if isinstance(parent, py_trees.composites.Composite):
            parent.add_child(node)
        else:
            self.logger.warning(f"Cannot add child to non-composite node {parent.name}")

    def get_health_status(self) -> Dict[str, Any]:
        """Get comprehensive tree health status."""
        total_executions = self.execution_count
        if total_executions == 0:
            success_rate = 0.0
        else:
            success_rate = self.success_count / total_executions

        avg_execution_time = (
            sum(self.execution_times) / len(self.execution_times)
            if self.execution_times else 0.0
        )

        return {
            'overall_health': 'healthy' if success_rate > 0.8 else 'degraded',
            'success_rate': success_rate,
            'total_executions': total_executions,
            'circuit_breaker_state': self.circuit_breaker.get_state().value,
            'average_execution_time': avg_execution_time,
            'tree_structure': self._get_tree_structure(),
            'blackboard_snapshot': dict(self.blackboard)
        }

    def _get_tree_structure(self) -> Dict[str, Any]:
        """Get tree structure for debugging."""
        def node_to_dict(node):
            return {
                'name': node.name,
                'type': type(node).__name__,
                'status': node.status.value if hasattr(node, 'status') else 'unknown',
                'children': [node_to_dict(child) for child in getattr(node, 'children', [])]
            }

        return node_to_dict(self.root)

    def reset(self):
        """Reset tree state and statistics."""
        self.execution_count = 0
        self.success_count = 0
        self.failure_count = 0
        self.execution_times.clear()

        # Reset all nodes
        for node in self.tree.root.iterate():
            if hasattr(node, 'reset'):
                node.reset()

        self.logger.info("Behavior tree reset")

    def switch_mission_profile(self, mission_type: str) -> bool:
        """
        Switch to a different mission profile, enabling/disabling components accordingly.

        Args:
            mission_type: Mission profile name (e.g., 'waypoint_navigation', 'sample_collection')

        Returns:
            True if profile switch was successful
        """
        if not self.resource_manager:
            self.logger.warning("Mission Resource Manager not available - cannot switch profiles")
            return False

        # Switch the resource manager to the new profile
        success = self.resource_manager.switch_mission_profile(mission_type)

        if success:
            self.logger.info(f"Switched behavior tree to mission profile: {mission_type}")

            # Update blackboard with new mission state
            self.blackboard.mission_state = mission_type

            # Log component status
            resource_status = self.resource_manager.get_resource_status()
            self.logger.info(f"Active components: {list(resource_status.get('component_status', {}).keys())}")
        else:
            self.logger.error(f"Failed to switch to mission profile: {mission_type}")

        return success

    def get_resource_status(self) -> Dict[str, Any]:
        """
        Get current resource usage and component status.

        Returns:
            Dictionary with resource status information
        """
        if not self.resource_manager:
            return {"error": "Mission Resource Manager not available"}

        return self.resource_manager.get_resource_status()

    def visualize(self, file_path: Optional[str] = None) -> str:
        """Generate tree visualization."""
        if PY_TREES_AVAILABLE:
            try:
                from py_trees.display import ascii_tree
                return ascii_tree(self.root)
            except ImportError:
                return "Visualization requires py_trees.display"
        return "Py-trees not available for visualization"


# Enhanced Action Nodes with py-trees
class EnhancedActionNode(py_trees.behaviour.Behaviour):
    """Enhanced action node with monitoring and circuit breaker."""

    def __init__(self, name: str, action_func: Callable[[], bool]):
        super().__init__(name)
        self.action_func = action_func
        self.execution_count = 0
        self.success_count = 0
        self.circuit_breaker = circuitbreaker.CircuitBreaker(failure_threshold=3)

    def update(self) -> py_trees.common.Status:
        """Execute action with monitoring."""
        self.execution_count += 1

        try:
            success = self.circuit_breaker.call(self.action_func)
            if success:
                self.success_count += 1
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE
        except circuitbreaker.CircuitBreakerOpenException:
            return py_trees.common.Status.FAILURE
        except Exception:
            return py_trees.common.Status.FAILURE


class EnhancedConditionNode(py_trees.behaviour.Behaviour):
    """Enhanced condition node with caching and monitoring."""

    def __init__(self, name: str, condition_func: Callable[[], bool], cache_timeout: float = 1.0):
        super().__init__(name)
        self.condition_func = condition_func
        self.cache_timeout = cache_timeout
        self.last_check_time = 0
        self.cached_result = None
        self.check_count = 0

    def update(self) -> py_trees.common.Status:
        """Evaluate condition with caching."""
        current_time = time.time()

        # Use cached result if recent
        if current_time - self.last_check_time < self.cache_timeout and self.cached_result is not None:
            return self.cached_result

        self.check_count += 1
        self.last_check_time = current_time

        try:
            result = self.condition_func()
            status = py_trees.common.Status.SUCCESS if result else py_trees.common.Status.FAILURE
            self.cached_result = status
            return status
        except Exception:
            return py_trees.common.Status.FAILURE


# ROS2 Integration Nodes (if py_trees_ros available)
if PY_TREES_ROS_AVAILABLE:
    class ROS2ActionNode(py_trees_ros.actions.ActionClient):
        """ROS2 action client as a behavior tree node (proper py_trees_ros integration)."""
        
        def __init__(self, name: str, action_type: Any, action_name: str, node: Any = None):
            """Initialize ROS2 action client node.
            
            Args:
                name: Node name
                action_type: ROS2 action type (e.g., NavigateToPose)
                action_name: Action server name
                node: ROS2 node instance (required for py_trees_ros)
            """
            if node is None:
                raise ValueError("ROS2 node required for py_trees_ros ActionClient")
            
            super().__init__(
                name=name,
                action_type=action_type,
                action_name=action_name,
                node=node
            )
    
    class ROS2ToBlackboard(ToBlackboard):
        """ROS2 topic to blackboard node (proper py_trees_ros integration)."""
        pass
    
    class ROS2FromBlackboard(FromBlackboard):
        """Blackboard to ROS2 topic node (proper py_trees_ros integration)."""
        pass

elif PY_TREES_AVAILABLE:
    # Fallback implementation if py_trees_ros not available
    class ROS2ActionNode(py_trees.behaviour.Behaviour):
        """Fallback ROS2 action client (limited functionality)."""
        
        def __init__(self, name: str, action_type: Any, action_name: str, node: Any = None):
            super().__init__(name)
            self.action_type = action_type
            self.action_name = action_name
            self.node = node
            self.logger.warning("py_trees_ros not available, using fallback implementation")
        
        def update(self) -> py_trees.common.Status:
            """Fallback update - always returns RUNNING."""
            self.logger.warning("ROS2 action not available without py_trees_ros")
            return py_trees.common.Status.RUNNING


# Factory functions for easy tree construction
def create_mission_sequence(name: str = "MissionSequence") -> py_trees.composites.Sequence:
    """Create a mission execution sequence."""
    return py_trees.composites.Sequence(name=name, memory=True)


def create_fallback_selector(name: str = "FallbackSelector") -> py_trees.composites.Selector:
    """Create a fallback selector for error recovery."""
    return py_trees.composites.Selector(name=name, memory=False)


def create_parallel_tasks(name: str = "ParallelTasks", policy=py_trees.common.ParallelPolicy.SuccessOnAll()) -> py_trees.composites.Parallel:
    """Create parallel task execution."""
    return py_trees.composites.Parallel(name=name, policy=policy)


# Mission-specific node factories
def create_navigation_action(name: str, waypoint: Dict[str, float]) -> EnhancedActionNode:
    """Create navigation action node."""
    def navigate():
        # Integration point for navigation system
        config = get_config()
        # Simulate navigation to waypoint
        return True  # Would integrate with actual navigation

    return EnhancedActionNode(name, navigate)


def create_sample_collection_action(name: str, sample_type: str) -> EnhancedActionNode:
    """Create sample collection action node."""
    def collect_sample():
        # Integration point for science payload
        config = get_config()
        # Simulate sample collection
        return True  # Would integrate with actual science system

    return EnhancedActionNode(name, collect_sample)


def create_system_health_condition(name: str = "SystemHealthy") -> EnhancedConditionNode:
    """Create system health condition node."""
    def check_health():
        # Integration point for health monitoring
        config = get_config()
        # Would check actual system health
        return True

    return EnhancedConditionNode(name, check_health)


# Demonstration and testing
if __name__ == "__main__":
    print("🧪 PY-TREES ENHANCED BEHAVIOR TREE DEMO")
    print("=" * 50)

    if not PY_TREES_AVAILABLE:
        print("❌ py-trees not available. Install with: pip install py-trees py-trees-ros")
        exit(1)

    try:
        # Create enhanced behavior tree
        tree = PyTreesBehaviorTree(name="URC_Demo_BT")

        # Add mission sequence
        mission_seq = create_mission_sequence("MainMission")
        tree.add_node(mission_seq)

        # Add navigation action
        nav_action = create_navigation_action("NavigateToSite", {"x": 10.0, "y": 5.0})
        mission_seq.add_child(nav_action)

        # Add health check condition
        health_condition = create_system_health_condition("PreFlightCheck")
        mission_seq.add_child(health_condition)

        # Add sample collection
        sample_action = create_sample_collection_action("CollectSoilSample", "soil")
        mission_seq.add_child(sample_action)

        print("✅ Enhanced behavior tree created")
        print(f"📊 Tree structure: {tree._get_tree_structure()}")

        # Execute tree
        result = tree.execute({'mission_phase': 'exploration'})
        print(f"🎯 Execution result: {result.status}")

        # Show health status
        health = tree.get_health_status()
        print(f"🏥 Tree health: {health['overall_health']} ({health['success_rate']:.1%} success rate)")

        # Demonstrate mission profile switching
        print("\n🔄 MISSION PROFILE SWITCHING DEMO")
        print("-" * 40)

        # Switch to waypoint navigation profile
        print("Switching to 'waypoint_navigation' profile...")
        success = tree.switch_mission_profile('waypoint_navigation')
        if success:
            resource_status = tree.get_resource_status()
            print("✅ Profile switch successful")
            print(f"📊 Mission profile: {resource_status.get('mission_profile', 'unknown')}")
            print(f"🔧 Component status: {resource_status.get('component_status', {})}")
        else:
            print("❌ Profile switch failed")

        # Switch to sample collection profile
        print("\nSwitching to 'sample_collection' profile...")
        success = tree.switch_mission_profile('sample_collection')
        if success:
            resource_status = tree.get_resource_status()
            print("✅ Profile switch successful")
            print(f"📊 Mission profile: {resource_status.get('mission_profile', 'unknown')}")
            print(f"🔧 Component status: {resource_status.get('component_status', {})}")
        else:
            print("❌ Profile switch failed")

        # Show visualization
        print("\n🌳 Tree Visualization:")
        print(tree.visualize())

        print("\n🎉 Py-trees enhanced behavior tree demo completed!")

    except Exception as e:
        print(f"❌ Demo failed: {e}")
        import traceback
        traceback.print_exc()
