#!/usr/bin/env python3
"""
Mission Debugger - Interactive debugging and monitoring tools

Provides debugging interfaces for:
- Mission execution tracing
- Component state inspection
- Performance profiling
- Real-time monitoring
- Error analysis and replay

Author: URC 2026 Debugging Team
"""

import time
import json
import threading
from typing import Dict, List, Any, Optional, Callable
from dataclasses import dataclass, field
from collections import deque
import logging

logger = logging.getLogger(__name__)


@dataclass
class MissionExecutionTrace:
    """Mission execution trace record."""
    mission_id: str
    timestamp: float
    component: str
    action: str
    state: Dict[str, Any]
    duration: Optional[float] = None
    error: Optional[str] = None
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass
class ComponentState:
    """Component state snapshot."""
    component_name: str
    timestamp: float
    state: Dict[str, Any]
    health_status: str
    performance_metrics: Dict[str, float] = field(default_factory=dict)


class MissionDebugger:
    """
    Interactive mission debugging and monitoring system.

    Provides real-time debugging capabilities for mission execution,
    component monitoring, and performance analysis.
    """

    def __init__(self):
        self.logger = logging.getLogger(f"{__name__}.MissionDebugger")

        # Execution tracing
        self.execution_traces: deque = deque(maxlen=10000)
        self.active_traces: Dict[str, MissionExecutionTrace] = {}

        # Component state monitoring
        self.component_states: Dict[str, ComponentState] = {}
        self.state_update_callbacks: List[Callable] = []

        # Performance profiling
        self.performance_profiles: Dict[str, List[float]] = {}
        self.profiling_active = False

        # Error tracking
        self.error_history: deque = deque(maxlen=1000)
        self.error_callbacks: List[Callable] = []

        # Debugging state
        self.debug_mode = False
        self.breakpoints: Dict[str, List[Callable]] = {}
        self.watch_variables: Dict[str, Any] = {}

        self.logger.info("Mission debugger initialized")

    def start_debugging(self) -> bool:
        """Start debugging session."""
        self.debug_mode = True
        self.logger.info("Mission debugging started")
        return True

    def stop_debugging(self) -> bool:
        """Stop debugging session."""
        self.debug_mode = False
        self.logger.info("Mission debugging stopped")
        return True

    def trace_execution_start(self, mission_id: str, component: str,
                            action: str, initial_state: Dict[str, Any]) -> str:
        """
        Start tracing a mission execution step.

        Args:
            mission_id: Mission identifier
            component: Component name
            action: Action being performed
            initial_state: Initial state

        Returns:
            Trace ID for later completion
        """
        trace_id = f"{mission_id}_{component}_{action}_{time.time()}"

        trace = MissionExecutionTrace(
            mission_id=mission_id,
            timestamp=time.time(),
            component=component,
            action=action,
            state=initial_state.copy(),
            metadata={'trace_id': trace_id}
        )

        self.active_traces[trace_id] = trace

        if self.debug_mode:
            self.logger.debug(f"Execution trace started: {trace_id}")

        return trace_id

    def trace_execution_end(self, trace_id: str, final_state: Dict[str, Any],
                          error: str = None) -> bool:
        """
        Complete an execution trace.

        Args:
            trace_id: Trace ID from trace_execution_start
            final_state: Final state after execution
            error: Error message if execution failed

        Returns:
            True if trace completed successfully
        """
        if trace_id not in self.active_traces:
            self.logger.warning(f"Unknown trace ID: {trace_id}")
            return False

        trace = self.active_traces.pop(trace_id)
        trace.duration = time.time() - trace.timestamp
        trace.error = error

        # Update final state
        trace.state.update(final_state)

        # Store completed trace
        self.execution_traces.append(trace)

        if self.debug_mode:
            status = "ERROR" if error else "SUCCESS"
            self.logger.debug(f"Execution trace completed: {trace_id} ({status}, {trace.duration:.3f}s)")

        # Check breakpoints
        self._check_breakpoints(trace)

        return True

    def update_component_state(self, component_name: str, state: Dict[str, Any],
                             health_status: str = "healthy",
                             performance_metrics: Dict[str, float] = None):
        """
        Update component state for monitoring.

        Args:
            component_name: Component name
            state: Current component state
            health_status: Component health status
            performance_metrics: Performance metrics
        """
        component_state = ComponentState(
            component_name=component_name,
            timestamp=time.time(),
            state=state.copy(),
            health_status=health_status,
            performance_metrics=performance_metrics or {}
        )

        self.component_states[component_name] = component_state

        if self.debug_mode:
            self.logger.debug(f"Component state updated: {component_name}")

        # Notify callbacks
        for callback in self.state_update_callbacks:
            try:
                callback(component_state)
            except Exception as e:
                self.logger.error(f"State update callback error: {e}")

    def log_error(self, component: str, error_type: str, error_message: str,
                 context: Dict[str, Any] = None) -> str:
        """
        Log an error for debugging analysis.

        Args:
            component: Component where error occurred
            error_type: Type of error
            error_message: Error message
            context: Additional error context

        Returns:
            Error ID for tracking
        """
        error_record = {
            'error_id': f"{component}_{error_type}_{time.time()}",
            'timestamp': time.time(),
            'component': component,
            'error_type': error_type,
            'message': error_message,
            'context': context or {}
        }

        self.error_history.append(error_record)

        self.logger.error(f"Error logged: {component} - {error_type}: {error_message}")

        # Call error callbacks
        for callback in self.error_callbacks:
            try:
                callback(error_record)
            except Exception as e:
                self.logger.error(f"Error callback error: {e}")

        return error_record['error_id']

    def set_breakpoint(self, component: str, condition: Callable[[MissionExecutionTrace], bool],
                      action: Callable[[MissionExecutionTrace], None]):
        """
        Set a debugging breakpoint.

        Args:
            component: Component to break on
            condition: Condition function that returns True to trigger breakpoint
            action: Action to take when breakpoint is hit
        """
        if component not in self.breakpoints:
            self.breakpoints[component] = []

        self.breakpoints[component].append({
            'condition': condition,
            'action': action
        })

        self.logger.info(f"Breakpoint set for component: {component}")

    def watch_variable(self, name: str, getter: Callable[[], Any]):
        """
        Watch a variable for changes.

        Args:
            name: Variable name
            getter: Function to get current variable value
        """
        self.watch_variables[name] = {
            'getter': getter,
            'last_value': None,
            'change_count': 0
        }

        self.logger.debug(f"Variable watch set: {name}")

    def check_variable_watches(self):
        """Check all variable watches for changes."""
        for name, watch in self.watch_variables.items():
            try:
                current_value = watch['getter']()
                last_value = watch['last_value']

                if current_value != last_value:
                    watch['change_count'] += 1
                    watch['last_value'] = current_value

                    if self.debug_mode:
                        self.logger.debug(f"Variable changed: {name} = {current_value} (changes: {watch['change_count']})")

            except Exception as e:
                self.logger.error(f"Error checking watch variable {name}: {e}")

    def _check_breakpoints(self, trace: MissionExecutionTrace):
        """Check if any breakpoints should be triggered."""
        if trace.component not in self.breakpoints:
            return

        for breakpoint in self.breakpoints[trace.component]:
            try:
                if breakpoint['condition'](trace):
                    self.logger.warning(f"Breakpoint triggered: {trace.component} - {trace.action}")
                    breakpoint['action'](trace)
            except Exception as e:
                self.logger.error(f"Breakpoint check error: {e}")

    def start_performance_profiling(self):
        """Start performance profiling."""
        self.profiling_active = True
        self.logger.info("Performance profiling started")

    def stop_performance_profiling(self):
        """Stop performance profiling."""
        self.profiling_active = False
        self.logger.info("Performance profiling stopped")

    def record_performance_metric(self, component: str, operation: str, duration: float):
        """
        Record a performance metric.

        Args:
            component: Component name
            operation: Operation name
            duration: Operation duration in seconds
        """
        if not self.profiling_active:
            return

        key = f"{component}:{operation}"
        if key not in self.performance_profiles:
            self.performance_profiles[key] = []

        self.performance_profiles[key].append(duration)

        # Keep only recent measurements
        if len(self.performance_profiles[key]) > 100:
            self.performance_profiles[key] = self.performance_profiles[key][-50:]

    def get_execution_report(self, mission_id: str = None,
                           component: str = None) -> Dict[str, Any]:
        """
        Get execution report for analysis.

        Args:
            mission_id: Filter by mission ID
            component: Filter by component

        Returns:
            Execution report data
        """
        traces = list(self.execution_traces)

        # Apply filters
        if mission_id:
            traces = [t for t in traces if t.mission_id == mission_id]
        if component:
            traces = [t for t in traces if t.component == component]

        # Calculate statistics
        total_traces = len(traces)
        error_traces = [t for t in traces if t.error]
        successful_traces = total_traces - len(error_traces)

        avg_duration = 0.0
        if traces:
            durations = [t.duration for t in traces if t.duration is not None]
            if durations:
                avg_duration = sum(durations) / len(durations)

        # Group by component
        component_stats = {}
        for trace in traces:
            comp = trace.component
            if comp not in component_stats:
                component_stats[comp] = {
                    'total': 0,
                    'errors': 0,
                    'avg_duration': 0.0
                }

            component_stats[comp]['total'] += 1
            if trace.error:
                component_stats[comp]['errors'] += 1

            if trace.duration is not None:
                current_avg = component_stats[comp]['avg_duration']
                count = component_stats[comp]['total']
                component_stats[comp]['avg_duration'] = (current_avg * (count - 1) + trace.duration) / count

        return {
            'total_traces': total_traces,
            'successful_traces': successful_traces,
            'error_traces': len(error_traces),
            'average_duration': avg_duration,
            'component_stats': component_stats,
            'recent_traces': [
                {
                    'mission_id': t.mission_id,
                    'component': t.component,
                    'action': t.action,
                    'duration': t.duration,
                    'error': t.error,
                    'timestamp': t.timestamp
                }
                for t in traces[-10:]  # Last 10 traces
            ]
        }

    def get_debug_snapshot(self) -> Dict[str, Any]:
        """
        Get complete debug snapshot of system state.

        Returns:
            Comprehensive debug information
        """
        return {
            'debug_mode': self.debug_mode,
            'active_traces': len(self.active_traces),
            'total_traces': len(self.execution_traces),
            'component_states': {
                name: {
                    'health': state.health_status,
                    'timestamp': state.timestamp,
                    'metrics': state.performance_metrics
                }
                for name, state in self.component_states.items()
            },
            'error_count': len(self.error_history),
            'breakpoints': list(self.breakpoints.keys()),
            'watched_variables': list(self.watch_variables.keys()),
            'profiling_active': self.profiling_active,
            'performance_profiles': {
                key: {
                    'count': len(values),
                    'avg': sum(values) / len(values) if values else 0,
                    'min': min(values) if values else 0,
                    'max': max(values) if values else 0
                }
                for key, values in self.performance_profiles.items()
            }
        }

    def register_state_callback(self, callback: Callable):
        """Register component state update callback."""
        self.state_update_callbacks.append(callback)

    def register_error_callback(self, callback: Callable):
        """Register error callback."""
        self.error_callbacks.append(callback)

    def export_debug_data(self, filename: str) -> bool:
        """
        Export all debug data to file.

        Args:
            filename: Output filename

        Returns:
            True if export successful
        """
        try:
            debug_data = {
                'execution_traces': [
                    {
                        'mission_id': t.mission_id,
                        'timestamp': t.timestamp,
                        'component': t.component,
                        'action': t.action,
                        'state': t.state,
                        'duration': t.duration,
                        'error': t.error,
                        'metadata': t.metadata
                    }
                    for t in self.execution_traces
                ],
                'component_states': {
                    name: {
                        'component_name': state.component_name,
                        'timestamp': state.timestamp,
                        'state': state.state,
                        'health_status': state.health_status,
                        'performance_metrics': state.performance_metrics
                    }
                    for name, state in self.component_states.items()
                },
                'error_history': list(self.error_history),
                'performance_profiles': dict(self.performance_profiles),
                'debug_snapshot': self.get_debug_snapshot()
            }

            with open(filename, 'w') as f:
                json.dump(debug_data, f, indent=2, default=str)

            self.logger.info(f"Debug data exported to {filename}")
            return True

        except Exception as e:
            self.logger.error(f"Failed to export debug data: {e}")
            return False


# Global debugger instance
_debugger = None

def get_mission_debugger() -> MissionDebugger:
    """Get global mission debugger instance."""
    global _debugger
    if _debugger is None:
        _debugger = MissionDebugger()
    return _debugger

