#!/usr/bin/env python3
"""
Monitoring Service - Real-time context monitoring and dashboard integration

Provides comprehensive monitoring of system context, adaptation actions,
and dashboard integration for the adaptive state machine.
"""

import time
from collections import deque
from typing import Any, Dict, List, Optional

import rclpy
from autonomy_interfaces.msg import AdaptiveAction as AdaptiveActionMsg
from autonomy_interfaces.msg import ContextState, ContextUpdate, SystemState
from autonomy_interfaces.srv import GetAdaptationHistory, GetContext, GetSystemState
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from .config import MonitoringConfig, Timing
from .error_handling import MonitoringError, error_boundary, handle_service_error


class MonitoringService(Node):
    """
    Comprehensive monitoring service for the adaptive state machine.

    Provides real-time context monitoring, adaptation tracking, and
    dashboard integration with historical data and analytics.
    """

    def __init__(self) -> None:
        """Initialize the monitoring service."""
        super().__init__("monitoring_service")

        # Data storage
        self.context_history: deque = deque(
            maxlen=5000
        )  # Store last 5000 context readings
        self.adaptation_history: deque = deque(
            maxlen=1000
        )  # Store last 1000 adaptations
        self.system_events: deque = deque(maxlen=2000)  # Store last 2000 events

        # Analytics data
        self.performance_metrics: Dict[str, Any] = {}
        self.policy_effectiveness: Dict[str, Any] = {}
        self.system_health_trends: Dict[str, Any] = {}

        # Configuration
        self.declare_parameters(
            namespace="",
            parameters=[
                ("dashboard_update_rate", 2.0),  # Hz
                ("analytics_update_rate", 0.1),  # Hz (every 10 seconds)
                ("alert_check_rate", 1.0),  # Hz
                ("max_history_age", 3600.0),  # 1 hour in seconds
            ],
        )

        # Get parameters
        self.dashboard_rate = self.get_parameter("dashboard_update_rate").value
        self.analytics_rate = self.get_parameter("analytics_update_rate").value
        self.alert_rate = self.get_parameter("alert_check_rate").value

        # ROS2 interfaces
        self._setup_subscribers()
        self._setup_services()
        self._setup_timers()

        # Alert thresholds
        self.alert_thresholds = {
            "battery_critical": 10.0,
            "cpu_critical": 95.0,
            "memory_critical": 95.0,
            "temperature_critical": 85.0,
            "communication_loss_duration": 30.0,
        }

        self.get_logger().info("Monitoring Service initialized")

    def _setup_subscribers(self) -> None:
        """Set up ROS2 subscribers for monitoring data."""
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Context updates
        self.context_sub = self.create_subscription(
            ContextState, "/state_machine/context", self._context_callback, qos_profile
        )

        # Adaptation actions
        self.adaptation_sub = self.create_subscription(
            AdaptiveActionMsg,
            "/state_machine/adaptation",
            self._adaptation_callback,
            qos_profile,
        )

        # System state changes
        self.state_sub = self.create_subscription(
            SystemState,
            "/state_machine/current_state",
            self._state_change_callback,
            qos_profile,
        )

    def _setup_services(self) -> None:
        """Set up ROS2 services for data queries."""
        # Enhanced context service
        self.context_query_srv = self.create_service(
            GetContext, "/monitoring/get_context", self._context_query_callback
        )

        # Adaptation analytics service
        self.analytics_srv = self.create_service(
            GetAdaptationHistory, "/monitoring/get_analytics", self._analytics_callback
        )

        # System health service
        self.health_srv = self.create_service(
            GetSystemState, "/monitoring/get_health", self._health_callback
        )

    def _setup_timers(self) -> None:
        """Set up periodic monitoring timers."""
        # Dashboard updates
        self.dashboard_timer = self.create_timer(
            1.0 / self.dashboard_rate, self._dashboard_update_callback
        )

        # Analytics computation
        self.analytics_timer = self.create_timer(
            1.0 / self.analytics_rate, self._analytics_computation_callback
        )

        # Alert checking
        self.alert_timer = self.create_timer(
            1.0 / self.alert_rate, self._alert_check_callback
        )

        # Data cleanup (remove old data)
        self.cleanup_timer = self.create_timer(
            300.0, self._cleanup_old_data  # Every 5 minutes
        )

    def _context_callback(self, msg: ContextState) -> None:
        """Handle incoming context updates."""
        try:
            # Store context with timestamp
            context_entry = {
                "context": msg,
                "timestamp": time.time(),
                "ros_timestamp": msg.timestamp,
            }

            self.context_history.append(context_entry)

            # Update system health trends
            self._update_health_trends(msg)

        except Exception as e:
            handle_service_error(
                self.get_logger(), e, "context update", "MonitoringService"
            )

    def _adaptation_callback(self, msg: AdaptiveActionMsg) -> None:
        """Handle incoming adaptation actions."""
        try:
            adaptation_entry = {
                "action": msg,
                "timestamp": time.time(),
                "ros_timestamp": msg.timestamp,
            }

            self.adaptation_history.append(adaptation_entry)

            # Log significant events
            if msg.priority >= 80:
                self._log_system_event(
                    "HIGH_PRIORITY_ADAPTATION",
                    f"Priority {msg.priority} action: {msg.action_type}",
                )

        except Exception as e:
            self.get_logger().error(f"Adaptation callback failed: {e}")

    def _state_change_callback(self, msg: SystemState) -> None:
        """Handle state change events."""
        try:
            self._log_system_event(
                "STATE_CHANGE", f"State changed to {msg.current_state}"
            )

        except Exception as e:
            self.get_logger().error(f"State change callback failed: {e}")

    def _dashboard_update_callback(self) -> None:
        """Periodic dashboard updates with latest context."""
        try:
            if not self.context_history:
                return

            # Get latest context
            latest_context = self.context_history[-1]["context"]

            # Create dashboard update
            dashboard_msg = ContextUpdate()
            dashboard_msg.battery_level = latest_context.battery_level
            dashboard_msg.mission_status = latest_context.mission_status
            dashboard_msg.mission_progress = latest_context.mission_progress
            dashboard_msg.communication_active = latest_context.communication_active
            dashboard_msg.safety_active = latest_context.safety_active

            # Active adaptations
            dashboard_msg.active_adaptations = [
                action["action"].action_type
                for action in list(self.adaptation_history)[-10:]  # Last 10 actions
            ]

            # Alert level
            dashboard_msg.alert_level = self._calculate_alert_level(latest_context)

            # Available actions
            dashboard_msg.available_actions = self._get_available_actions(
                latest_context
            )

            dashboard_msg.timestamp = self.get_clock().now().to_msg()

            # Publish to dashboard topic
            if hasattr(self, "dashboard_pub"):
                self.dashboard_pub.publish(dashboard_msg)

        except Exception as e:
            self.get_logger().error(f"Dashboard update failed: {e}")

    def _analytics_computation_callback(self) -> None:
        """Periodic analytics computation."""
        try:
            self._compute_performance_metrics()
            self._compute_policy_effectiveness()
            self._analyze_system_patterns()

        except Exception as e:
            self.get_logger().error(f"Analytics computation failed: {e}")

    def _alert_check_callback(self) -> None:
        """Check for alert conditions."""
        try:
            if not self.context_history:
                return

            latest_context = self.context_history[-1]["context"]
            alerts = self._check_alert_conditions(latest_context)

            for alert in alerts:
                self._log_system_event("ALERT", alert)
                self.get_logger().warning(f"ALERT: {alert}")

        except Exception as e:
            self.get_logger().error(f"Alert check failed: {e}")

    def _compute_performance_metrics(self) -> None:
        """Compute system performance metrics."""
        if len(self.context_history) < 10:
            return

        try:
            recent_contexts = list(self.context_history)[-100:]  # Last 100 readings

            # Battery metrics
            battery_levels = [ctx["context"].battery_level for ctx in recent_contexts]
            self.performance_metrics["battery_trend"] = self._calculate_trend(
                battery_levels
            )
            self.performance_metrics["battery_volatility"] = self._calculate_volatility(
                battery_levels
            )

            # System performance
            cpu_usage = [ctx["context"].cpu_usage for ctx in recent_contexts]
            memory_usage = [ctx["context"].memory_usage for ctx in recent_contexts]

            self.performance_metrics["avg_cpu_usage"] = sum(cpu_usage) / len(cpu_usage)
            self.performance_metrics["avg_memory_usage"] = sum(memory_usage) / len(
                memory_usage
            )
            self.performance_metrics["cpu_volatility"] = self._calculate_volatility(
                cpu_usage
            )

            # Mission efficiency
            mission_progress = [
                ctx["context"].mission_progress for ctx in recent_contexts
            ]
            if mission_progress:
                self.performance_metrics[
                    "mission_progress_rate"
                ] = self._calculate_trend(mission_progress)

        except Exception as e:
            self.get_logger().error(f"Performance metrics computation failed: {e}")

    def _compute_policy_effectiveness(self) -> None:
        """Compute effectiveness of adaptation policies."""
        if len(self.adaptation_history) < 5:
            return

        try:
            # Group adaptations by type
            policy_stats = {}
            recent_adaptations = list(self.adaptation_history)[-50:]  # Last 50 actions

            for adaptation in recent_adaptations:
                action_type = adaptation["action"].action_type

                if action_type not in policy_stats:
                    policy_stats[action_type] = {"count": 0, "avg_priority": 0.0}

                policy_stats[action_type]["count"] += 1
                policy_stats[action_type]["avg_priority"] += adaptation[
                    "action"
                ].priority

            # Calculate averages
            for stats in policy_stats.values():
                stats["avg_priority"] = stats["avg_priority"] / stats["count"]

            self.policy_effectiveness = policy_stats

        except Exception as e:
            self.get_logger().error(f"Policy effectiveness computation failed: {e}")

    def _analyze_system_patterns(self) -> None:
        """Analyze system behavior patterns."""
        try:
            if len(self.context_history) < 50:
                return

            # Detect patterns in context data
            patterns = {
                "safety_activation_frequency": self._calculate_safety_frequency(),
                "adaptation_trigger_patterns": self._analyze_adaptation_patterns(),
                "performance_degradation_indicators": self._detect_performance_issues(),
            }

            # Store for dashboard access
            self.system_health_trends = patterns

        except Exception as e:
            self.get_logger().error(f"Pattern analysis failed: {e}")

    def _calculate_alert_level(self, context: ContextState) -> str:
        """Calculate current alert level."""
        if (
            context.battery_critical
            or context.temperature > self.alert_thresholds["temperature_critical"]
            or context.cpu_usage > self.alert_thresholds["cpu_critical"]
        ):
            return "CRITICAL"

        if (
            context.battery_warning
            or context.obstacle_detected
            or not context.communication_active
            or context.safety_active
        ):
            return "WARNING"

        return "NONE"

    def _get_available_actions(self, context: ContextState) -> List[str]:
        """Get contextually available actions."""
        actions = ["emergency_stop"]

        # Add context-specific actions
        if context.battery_critical:
            actions.extend(["emergency_return", "power_save_mode"])
        if context.obstacle_detected:
            actions.extend(["request_assistance", "attempt_detour"])
        if not context.communication_active:
            actions.extend(["safe_mode", "local_control_only"])
        if context.safety_active:
            actions.extend(["safety_reset", "manual_override"])

        return actions

    def _check_alert_conditions(self, context: ContextState) -> List[str]:
        """Check for alert-worthy conditions."""
        alerts = []

        if context.battery_level < self.alert_thresholds["battery_critical"]:
            alerts.append(f"Battery critically low: {context.battery_level}%")

        if context.cpu_usage > self.alert_thresholds["cpu_critical"]:
            alerts.append(f"CPU usage critical: {context.cpu_usage}%")

        if context.memory_usage > self.alert_thresholds["memory_critical"]:
            alerts.append(f"Memory usage critical: {context.memory_usage}%")

        if context.temperature > self.alert_thresholds["temperature_critical"]:
            alerts.append(f"Temperature critical: {context.temperature}°C")

        return alerts

    def _update_health_trends(self, context: ContextState) -> None:
        """Update system health trend analysis."""
        # This would maintain rolling averages and trend detection
        # Implementation simplified for brevity
        pass

    def _log_system_event(self, event_type: str, message: str) -> None:
        """Log a system event."""
        event = {
            "type": event_type,
            "message": message,
            "timestamp": time.time(),
            "ros_timestamp": self.get_clock().now().to_msg(),
        }

        self.system_events.append(event)

        # Keep events bounded
        if len(self.system_events) > 2000:
            self.system_events.popleft()

    # Service Callbacks
    def _context_query_callback(
        self, request: GetContext.Request, response: GetContext.Response
    ) -> None:
        """Handle context query requests."""
        try:
            if self.context_history:
                response.context = self.context_history[-1]["context"]
            else:
                self.get_logger().warning("No context history available")
        except Exception as e:
            self.get_logger().error(f"Context query failed: {e}")

        return response

    def _analytics_callback(
        self,
        request: GetAdaptationHistory.Request,
        response: GetAdaptationHistory.Response,
    ) -> None:
        """Handle analytics query requests."""
        try:
            # Return recent adaptation history
            recent_actions = list(self.adaptation_history)[-request.limit :]
            response.actions = [entry["action"] for entry in recent_actions]

            if request.include_context:
                # Include context snapshots for each action
                response.contexts = []
                for entry in recent_actions:
                    # Find corresponding context (simplified)
                    response.contexts.append(entry["action"].trigger_context)

        except Exception as e:
            self.get_logger().error(f"Analytics query failed: {e}")

        return response

    def _health_callback(
        self, request: GetSystemState.Request, response: GetSystemState.Response
    ) -> None:
        """Handle system health query requests."""
        try:
            # Create a comprehensive health report
            response.current_state = "MONITORING_ACTIVE"
            response.subsystem_status = []

            # Add performance metrics
            for metric, value in self.performance_metrics.items():
                status = autonomy_interfaces.msg.SubsystemStatus()
                status.name = f"metric_{metric}"
                status.status = "ACTIVE"
                status.details = f"{metric}: {value:.2f}"
                response.subsystem_status.append(status)

            # Add policy effectiveness
            for policy, stats in self.policy_effectiveness.items():
                status = autonomy_interfaces.msg.SubsystemStatus()
                status.name = f"policy_{policy}"
                status.status = "ACTIVE"
                status.details = f"effectiveness: {stats}"
                response.subsystem_status.append(status)

        except Exception as e:
            self.get_logger().error(f"Health query failed: {e}")

        return response

    # Utility Methods
    def _calculate_trend(self, values: List[float]) -> float:
        """Calculate linear trend (slope) of values."""
        if len(values) < 2:
            return 0.0

        n = len(values)
        x = list(range(n))
        y = values

        sum_x = sum(x)
        sum_y = sum(y)
        sum_xy = sum(xi * yi for xi, yi in zip(x, y))
        sum_xx = sum(xi * xi for xi in x)

        if n * sum_xx - sum_x * sum_x == 0:
            return 0.0

        slope = (n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x)
        return slope

    def _calculate_volatility(self, values: List[float]) -> float:
        """Calculate volatility (standard deviation) of values."""
        if len(values) < 2:
            return 0.0

        mean = sum(values) / len(values)
        variance = sum((x - mean) ** 2 for x in values) / len(values)
        return variance**0.5

    def _calculate_safety_frequency(self) -> float:
        """Calculate frequency of safety activations."""
        if not self.context_history:
            return 0.0

        safety_count = sum(
            1 for entry in self.context_history if entry["context"].safety_active
        )

        time_span = time.time() - self.context_history[0]["timestamp"]
        if time_span > 0:
            return safety_count / time_span * 3600  # per hour
        return 0.0

    def _analyze_adaptation_patterns(self) -> Dict[str, Any]:
        """Analyze patterns in adaptation triggers."""
        # Simplified pattern analysis
        return {
            "most_common_trigger": "battery_warning",
            "peak_adaptation_hour": 14,  # 2 PM
            "adaptation_success_rate": 0.85,
        }

    def _detect_performance_issues(self) -> List[str]:
        """Detect potential performance issues."""
        issues = []

        if self.performance_metrics.get("avg_cpu_usage", 0) > 80:
            issues.append("High CPU usage detected")
        if self.performance_metrics.get("avg_memory_usage", 0) > 90:
            issues.append("High memory usage detected")
        if self.performance_metrics.get("cpu_volatility", 0) > 10:
            issues.append("Unstable CPU performance")

        return issues

    def _cleanup_old_data(self) -> None:
        """Remove old data to prevent memory issues."""
        current_time = time.time()
        max_age = self.get_parameter("max_history_age").value

        # Clean up old context data
        while (
            self.context_history
            and current_time - self.context_history[0]["timestamp"] > max_age
        ):
            self.context_history.popleft()

        # Clean up old adaptation data
        while (
            self.adaptation_history
            and current_time - self.adaptation_history[0]["timestamp"] > max_age
        ):
            self.adaptation_history.popleft()

        # Clean up old events
        while (
            self.system_events
            and current_time - self.system_events[0]["timestamp"] > max_age
        ):
            self.system_events.popleft()

    def get_monitoring_stats(self) -> Dict[str, Any]:
        """Get comprehensive monitoring statistics."""
        return {
            "context_readings": len(self.context_history),
            "adaptation_actions": len(self.adaptation_history),
            "system_events": len(self.system_events),
            "performance_metrics": self.performance_metrics.copy(),
            "policy_effectiveness": self.policy_effectiveness.copy(),
            "system_health_trends": self.system_health_trends.copy(),
            "uptime": time.time() - self._start_time
            if hasattr(self, "_start_time")
            else 0,
        }


def main() -> None:
    """Main entry point."""
    rclpy.init()
    node = MonitoringService()

    # Store start time for uptime calculation
    node._start_time = time.time()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Monitoring Service shutting down")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
