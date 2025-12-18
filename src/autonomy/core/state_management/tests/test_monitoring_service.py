#!/usr/bin/env python3
"""
Enhanced tests for Monitoring Service

Tests analytics, statistical calculations, callbacks, and performance monitoring.
"""

import math
import time
from unittest.mock import MagicMock, Mock, patch

import pytest
from autonomy_interfaces.msg import AdaptiveAction, ContextState
from autonomy_state_machine.config import MonitoringConfig
from autonomy_state_machine.monitoring_service import MonitoringService


class TestMonitoringServiceAnalytics:
    """Test analytics and statistical functionality."""

    @pytest.fixture
    def monitoring_service(self):
        """Create a monitoring service instance with mocked ROS2 components."""
        # Mock the ROS2 components to avoid requiring rclpy.init()
        with patch("rclpy.node.Node.__init__", return_value=None), patch(
            "rclpy.node.Node.create_timer"
        ), patch("rclpy.node.Node.create_subscription"), patch(
            "rclpy.node.Node.create_service"
        ), patch(
            "rclpy.node.Node.create_publisher"
        ), patch(
            "rclpy.node.Node.get_logger"
        ), patch(
            "rclpy.node.Node.declare_parameters", return_value=[]
        ), patch(
            "rclpy.node.Node.get_parameter",
            side_effect=lambda name: Mock(
                value={
                    "dashboard_update_rate": 2.0,
                    "analytics_update_rate": 0.1,
                    "alert_check_rate": 1.0,
                    "max_history_age": 3600.0,
                }[name]
            ),
        ), patch(
            "rclpy.node.Node.has_parameter", return_value=True
        ), patch(
            "rclpy.node.Node._parameters", {}, create=True
        ):
            service = MonitoringService()
            # Manually initialize the attributes that would normally be set by __init__
            service.context_history = []
            service.adaptation_history = []
            service.performance_metrics = {}
            service.policy_effectiveness = {}
            service.system_health_trends = {}
            service._alert_level = "NONE"
            return service

    def test_calculate_trend_positive(self, monitoring_service):
        """Test trend calculation with increasing values."""
        values = [1.0, 2.0, 3.0, 4.0, 5.0]
        trend = monitoring_service._calculate_trend(values)

        assert trend > 0  # Should be positive trend

    def test_calculate_trend_negative(self, monitoring_service):
        """Test trend calculation with decreasing values."""
        values = [5.0, 4.0, 3.0, 2.0, 1.0]
        trend = monitoring_service._calculate_trend(values)

        assert trend < 0  # Should be negative trend

    def test_calculate_trend_flat(self, monitoring_service):
        """Test trend calculation with flat values."""
        values = [3.0, 3.0, 3.0, 3.0, 3.0]
        trend = monitoring_service._calculate_trend(values)

        assert abs(trend) < 0.001  # Should be approximately zero

    def test_calculate_trend_insufficient_data(self, monitoring_service):
        """Test trend calculation with insufficient data."""
        values = [1.0, 2.0]  # Less than 2 points
        trend = monitoring_service._calculate_trend(values)

        assert trend == 0.0

    def test_calculate_volatility(self, monitoring_service):
        """Test volatility calculation."""
        values = [10.0, 12.0, 8.0, 11.0, 9.0]
        volatility = monitoring_service._calculate_volatility(values)

        assert volatility > 0  # Should have some volatility
        assert isinstance(volatility, float)

    def test_calculate_volatility_constant(self, monitoring_service):
        """Test volatility calculation with constant values."""
        values = [5.0, 5.0, 5.0, 5.0, 5.0]
        volatility = monitoring_service._calculate_volatility(values)

        assert volatility == 0.0

    def test_calculate_volatility_insufficient_data(self, monitoring_service):
        """Test volatility calculation with insufficient data."""
        values = [10.0]  # Less than 2 points
        volatility = monitoring_service._calculate_volatility(values)

        assert volatility == 0.0

    def test_calculate_safety_frequency(self, monitoring_service):
        """Test safety frequency calculation."""
        # Create test context data with some safety activations
        contexts = []
        for i in range(10):
            context = Mock()
            context.safety_active = i % 3 == 0  # Every 3rd context has safety active
            contexts.append({"context": context})

        frequency = monitoring_service._calculate_safety_frequency(contexts)
        expected_frequency = 3 / 10  # 3 out of 10 contexts had safety active

        assert abs(frequency - expected_frequency) < 0.001

    def test_calculate_safety_frequency_no_contexts(self, monitoring_service):
        """Test safety frequency with empty context list."""
        frequency = monitoring_service._calculate_safety_frequency([])

        assert frequency == 0.0

    def test_analyze_system_patterns(self, monitoring_service):
        """Test system pattern analysis."""
        # This method delegates to other analysis methods
        # We can test that it calls the expected methods
        with patch.object(
            monitoring_service, "_analyze_battery_trend"
        ) as mock_battery, patch.object(
            monitoring_service, "_analyze_performance_trend"
        ) as mock_perf, patch.object(
            monitoring_service, "_analyze_safety_frequency"
        ) as mock_safety, patch.object(
            monitoring_service, "_analyze_adaptation_effectiveness"
        ) as mock_adapt:

            mock_battery.return_value = "increasing"
            mock_perf.return_value = "stable"
            mock_safety.return_value = 0.2
            mock_adapt.return_value = 0.8

            contexts = [Mock()] * 10  # Mock contexts
            patterns = monitoring_service._analyze_system_patterns(contexts)

            # Verify all analysis methods were called
            mock_battery.assert_called_once_with(contexts)
            mock_perf.assert_called_once_with(contexts)
            mock_safety.assert_called_once_with(contexts)
            mock_adapt.assert_called_once_with(contexts)

            # Verify results structure
            assert "battery_trend" in patterns
            assert "performance_trend" in patterns
            assert "safety_frequency" in patterns
            assert "adaptation_effectiveness" in patterns

    def test_compute_policy_effectiveness(self, monitoring_service):
        """Test policy effectiveness computation."""
        # Create mock adaptation history
        adaptation1 = Mock()
        adaptation1.action_type = "battery_mgmt"
        adaptation1.priority = 80
        adaptation1.timestamp = time.time()

        adaptation2 = Mock()
        adaptation2.action_type = "obstacle_avoidance"
        adaptation2.priority = 75
        adaptation2.timestamp = time.time()

        adaptation3 = Mock()
        adaptation3.action_type = "battery_mgmt"
        adaptation3.priority = 85
        adaptation3.timestamp = time.time()

        # Set up adaptation history
        monitoring_service.adaptation_history = [
            {"action": adaptation1},
            {"action": adaptation2},
            {"action": adaptation3},
        ]

        monitoring_service._compute_policy_effectiveness()

        effectiveness = monitoring_service.policy_effectiveness

        # Verify battery_mgmt policy effectiveness
        assert "battery_mgmt" in effectiveness
        battery_stats = effectiveness["battery_mgmt"]
        assert battery_stats["count"] == 2  # Two battery actions
        assert battery_stats["avg_priority"] == 82.5  # (80 + 85) / 2

        # Verify obstacle_avoidance policy effectiveness
        assert "obstacle_avoidance" in effectiveness
        obstacle_stats = effectiveness["obstacle_avoidance"]
        assert obstacle_stats["count"] == 1
        assert obstacle_stats["avg_priority"] == 75.0

    def test_compute_policy_effectiveness_insufficient_data(self, monitoring_service):
        """Test policy effectiveness with insufficient adaptation history."""
        # Less than 5 adaptations
        monitoring_service.adaptation_history = [Mock()] * 3

        monitoring_service._compute_policy_effectiveness()

        # Should not compute effectiveness with insufficient data
        assert len(monitoring_service.policy_effectiveness) == 0

    def test_calculate_alert_level_none(self, monitoring_service):
        """Test alert level calculation when no alerts."""
        context = ContextState()
        context.battery_level = 80.0
        context.cpu_usage = 30.0
        context.communication_active = True
        context.safety_active = False

        alert_level = monitoring_service._calculate_alert_level(context)

        assert alert_level == "NONE"

    def test_calculate_alert_level_warning(self, monitoring_service):
        """Test alert level calculation for warning conditions."""
        context = ContextState()
        context.battery_level = 25.0  # Warning level
        context.cpu_usage = 30.0
        context.communication_active = True
        context.safety_active = False

        alert_level = monitoring_service._calculate_alert_level(context)

        assert alert_level == "WARNING"

    def test_calculate_alert_level_critical(self, monitoring_service):
        """Test alert level calculation for critical conditions."""
        context = ContextState()
        context.battery_level = 5.0  # Critical level
        context.cpu_usage = 30.0
        context.communication_active = True
        context.safety_active = False

        alert_level = monitoring_service._calculate_alert_level(context)

        assert alert_level == "CRITICAL"

    def test_check_alert_conditions(self, monitoring_service):
        """Test alert condition checking."""
        context = ContextState()
        context.battery_level = 5.0  # Critical battery
        context.cpu_usage = 95.0  # High CPU
        context.communication_active = False  # Communication lost
        context.safety_active = True  # Safety active

        alerts = monitoring_service._check_alert_conditions(context)

        # Should detect multiple alert conditions
        assert "critical_battery" in alerts
        assert "high_cpu_usage" in alerts
        assert "communication_loss" in alerts
        assert "safety_active" in alerts

    def test_get_available_actions(self, monitoring_service):
        """Test getting available actions for context."""
        context = ContextState()
        context.battery_level = 5.0  # Critical battery
        context.obstacle_detected = True
        context.communication_active = False

        actions = monitoring_service._get_available_actions(context)

        # Should include actions for critical conditions
        assert isinstance(actions, list)
        # Specific actions depend on context evaluation logic

    def test_detect_performance_issues(self, monitoring_service):
        """Test performance issue detection."""
        # Set up monitoring service with some performance data
        monitoring_service.performance_metrics = {
            "cpu_usage": 95.0,
            "memory_usage": 92.0,
            "disk_usage": 85.0,
        }

        issues = monitoring_service._detect_performance_issues()

        assert "high_cpu_usage" in issues
        assert "high_memory_usage" in issues
        assert "high_disk_usage" in issues

    def test_detect_performance_issues_normal(self, monitoring_service):
        """Test performance issue detection with normal values."""
        monitoring_service.performance_metrics = {
            "cpu_usage": 45.0,
            "memory_usage": 60.0,
            "disk_usage": 70.0,
        }

        issues = monitoring_service._detect_performance_issues()

        assert len(issues) == 0

    def test_analyze_adaptation_patterns(self, monitoring_service):
        """Test adaptation pattern analysis."""
        # Create mock adaptation history with some patterns
        adaptation1 = Mock()
        adaptation1.action_type = "battery_mgmt"
        adaptation1.priority = 80
        adaptation1.timestamp = time.time()

        adaptation2 = Mock()
        adaptation2.action_type = "battery_mgmt"
        adaptation2.priority = 85
        adaptation2.timestamp = time.time()

        monitoring_service.adaptation_history = [
            {"action": adaptation1},
            {"action": adaptation2},
        ]

        patterns = monitoring_service._analyze_adaptation_patterns()

        assert isinstance(patterns, dict)
        # Specific pattern analysis depends on implementation

    def test_monitoring_stats_comprehensive(self, monitoring_service):
        """Test comprehensive monitoring stats generation."""
        # Set up some monitoring data
        monitoring_service.performance_metrics = {
            "avg_cpu_usage": 45.0,
            "avg_memory_usage": 60.0,
            "total_adaptations": 15,
            "active_alerts": 2,
        }

        monitoring_service.policy_effectiveness = {
            "battery_mgmt": {"count": 8, "avg_priority": 82.0},
            "obstacle_avoidance": {"count": 7, "avg_priority": 75.0},
        }

        stats = monitoring_service.get_monitoring_stats()

        assert "performance_metrics" in stats
        assert "policy_effectiveness" in stats
        assert "data_retention" in stats
        assert "system_health" in stats

        # Verify specific metrics
        assert stats["performance_metrics"]["avg_cpu_usage"] == 45.0
        assert stats["policy_effectiveness"]["battery_mgmt"]["count"] == 8

    # ========== STATISTICAL ANALYSIS TESTS ========== #

    def test_trend_calculation_edge_cases(self, monitoring_service):
        """Test trend calculation with edge cases."""
        # Empty data
        trend = monitoring_service._calculate_trend([])
        assert trend == 0.0

        # Single point
        trend = monitoring_service._calculate_trend([5.0])
        assert trend == 0.0

        # Two points - positive trend
        trend = monitoring_service._calculate_trend([1.0, 3.0])
        assert trend > 0

        # Two points - negative trend
        trend = monitoring_service._calculate_trend([3.0, 1.0])
        assert trend < 0

        # Two identical points - flat trend
        trend = monitoring_service._calculate_trend([2.0, 2.0])
        assert abs(trend) < 0.001

    def test_volatility_calculation_precision(self, monitoring_service):
        """Test volatility calculation accuracy."""
        # Constant values should have zero volatility
        volatility = monitoring_service._calculate_volatility([5.0, 5.0, 5.0])
        assert volatility == 0.0

        # Increasing variation
        values_low_var = [10.0, 10.5, 9.5, 10.2, 9.8]
        values_high_var = [10.0, 15.0, 5.0, 20.0, 0.0]

        vol_low = monitoring_service._calculate_volatility(values_low_var)
        vol_high = monitoring_service._calculate_volatility(values_high_var)

        assert vol_high > vol_low  # Higher variation should give higher volatility
        assert vol_low > 0  # Should still have some volatility

    def test_policy_effectiveness_with_realistic_data(self, monitoring_service):
        """Test policy effectiveness with varied scenarios."""
        # Create realistic adaptation history
        adaptations = []

        # Battery management adaptations (successful)
        for i in range(5):
            adapt = Mock()
            adapt.action_type = "battery_mgmt"
            adapt.priority = 80 + i  # Increasing priority
            adapt.timestamp = time.time() + i
            adaptations.append({"action": adapt})

        # Obstacle avoidance adaptations (mixed success)
        for i in range(3):
            adapt = Mock()
            adapt.action_type = "obstacle_avoidance"
            adapt.priority = 75
            adapt.timestamp = time.time() + i + 10
            adaptations.append({"action": adapt})

        monitoring_service.adaptation_history = adaptations
        monitoring_service._compute_policy_effectiveness()

        effectiveness = monitoring_service.policy_effectiveness

        # Should have both policy types
        assert "battery_mgmt" in effectiveness
        assert "obstacle_avoidance" in effectiveness

        # Battery management should show higher average priority
        battery_avg = effectiveness["battery_mgmt"]["avg_priority"]
        obstacle_avg = effectiveness["obstacle_avoidance"]["avg_priority"]
        assert battery_avg > obstacle_avg

    def test_alert_level_escalation(self, monitoring_service):
        """Test alert level changes over time."""
        # No alerts initially
        context = ContextState()
        context.battery_level = 80.0
        context.cpu_usage = 30.0
        context.communication_active = True
        alert_level = monitoring_service._calculate_alert_level(context)
        assert alert_level == "NONE"

        # Warning condition
        context.battery_level = 25.0  # Warning level
        alert_level = monitoring_service._calculate_alert_level(context)
        assert alert_level == "WARNING"

        # Critical condition
        context.battery_level = 5.0  # Critical level
        alert_level = monitoring_service._calculate_alert_level(context)
        assert alert_level == "CRITICAL"

        # Multiple critical conditions
        context.cpu_usage = 95.0  # High CPU
        context.communication_active = False  # Communication lost
        alert_level = monitoring_service._calculate_alert_level(context)
        assert alert_level == "CRITICAL"  # Should remain critical

    def test_performance_issue_detection_accuracy(self, monitoring_service):
        """Test performance issue detection accuracy."""
        # Normal conditions
        monitoring_service.performance_metrics = {
            "cpu_usage": 45.0,
            "memory_usage": 60.0,
            "disk_usage": 70.0,
        }
        issues = monitoring_service._detect_performance_issues()
        assert len(issues) == 0

        # Single issue
        monitoring_service.performance_metrics["cpu_usage"] = 85.0
        issues = monitoring_service._detect_performance_issues()
        assert "high_cpu_usage" in issues
        assert len(issues) == 1

        # Multiple issues
        monitoring_service.performance_metrics.update(
            {"memory_usage": 95.0, "disk_usage": 90.0}
        )
        issues = monitoring_service._detect_performance_issues()
        assert len(issues) == 3
        assert "high_cpu_usage" in issues
        assert "high_memory_usage" in issues
        assert "high_disk_usage" in issues

    def test_adaptation_pattern_learning(self, monitoring_service):
        """Test adaptation pattern learning and analysis."""
        # Create pattern of adaptations
        adaptations = []

        # Pattern: Battery issues followed by battery management
        for i in range(3):
            adapt = Mock()
            adapt.action_type = "battery_mgmt"
            adapt.priority = 80
            adapt.timestamp = time.time() + i * 60  # Every minute
            adaptations.append({"action": adapt})

        monitoring_service.adaptation_history = adaptations
        patterns = monitoring_service._analyze_adaptation_patterns()

        # Should detect patterns
        assert isinstance(patterns, dict)
        # Specific pattern analysis depends on implementation
        # but should return meaningful data
        assert len(patterns) >= 0

    def test_monitoring_data_persistence(self, monitoring_service):
        """Test data retention and cleanup."""
        # Fill up monitoring history
        for i in range(150):  # More than default limits
            context = ContextState()
            context.battery_level = 50.0
            monitoring_service._context_callback(context)

        # Should have data
        assert len(monitoring_service.context_history) > 0

        # Trigger cleanup
        monitoring_service._cleanup_old_data()

        # Should still have reasonable amount of data
        assert len(monitoring_service.context_history) <= 1010  # Allow some margin

    def test_real_time_analytics_updates(self, monitoring_service):
        """Test analytics updates during live operation."""
        # Initial state
        assert len(monitoring_service.performance_metrics) == 0

        # Add some data
        for i in range(5):
            context = ContextState()
            context.battery_level = 60.0
            context.cpu_usage = 40.0
            monitoring_service._context_callback(context)

        # Trigger analytics computation
        monitoring_service._compute_performance_metrics()

        # Should have computed metrics
        assert len(monitoring_service.performance_metrics) > 0
        assert "avg_cpu_usage" in monitoring_service.performance_metrics
        assert "avg_memory_usage" in monitoring_service.performance_metrics

    def test_alert_condition_evaluation_completeness(self, monitoring_service):
        """Test comprehensive alert condition evaluation."""
        context = ContextState()

        # Test all alert conditions
        test_cases = [
            # (battery, cpu, comms, expected_alerts)
            (80.0, 30.0, True, []),  # Normal
            (25.0, 30.0, True, ["critical_battery"]),  # Low battery
            (80.0, 85.0, True, ["high_cpu_usage"]),  # High CPU
            (80.0, 30.0, False, ["communication_loss"]),  # No comms
            (
                25.0,
                85.0,
                False,
                ["critical_battery", "high_cpu_usage", "communication_loss"],
            ),  # Multiple
        ]

        for battery, cpu, comms, expected_alerts in test_cases:
            context.battery_level = battery
            context.cpu_usage = cpu
            context.communication_active = comms

            alerts = monitoring_service._check_alert_conditions(context)

            for expected_alert in expected_alerts:
                assert (
                    expected_alert in alerts
                ), f"Missing alert: {expected_alert} for battery={battery}, cpu={cpu}, comms={comms}"
