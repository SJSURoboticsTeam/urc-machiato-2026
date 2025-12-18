#!/usr/bin/env python3
"""
Configuration Constants - Centralized configuration for the state machine

Contains all magic numbers, thresholds, timeouts, and other configurable
parameters used throughout the state machine components.
"""

from typing import Dict, Any


# Timing Constants (seconds)
class Timing:
    """Timing-related configuration constants."""

    # Cache timeouts
    CONTEXT_CACHE_TIMEOUT = 0.5
    BATTERY_CACHE_TIMEOUT = 0.5
    MISSION_STATUS_CACHE_TIMEOUT = 0.5

    # Timer intervals
    CONTEXT_EVALUATION_INTERVAL = 1.0
    ADAPTATION_CHECK_INTERVAL = 0.5
    STATE_PUBLISH_INTERVAL = 0.1
    DASHBOARD_UPDATE_INTERVAL = 1.0
    ANALYTICS_COMPUTATION_INTERVAL = 5.0
    ALERT_CHECK_INTERVAL = 2.0

    # Timeouts
    MISSION_TIMEOUT_DEFAULT = 1800.0  # 30 minutes
    MISSION_ABORT_TIMEOUT = 300.0     # 5 minutes
    COMMUNICATION_TIMEOUT = 10.0      # 10 seconds
    EMERGENCY_RETURN_TIMEOUT = 900.0  # 15 minutes
    COMPLETE_AND_RETURN_TIMEOUT = 600.0  # 10 minutes
    AUTO_RETURN_TIMEOUT = 300.0       # 5 minutes

    # Cooldowns
    ADAPTIVE_ACTION_COOLDOWN = 5.0    # 5 seconds between same action
    HUMAN_INTERVENTION_COOLDOWN = 300.0  # 5 minutes


# Threshold Constants
class Thresholds:
    """System threshold values."""

    # Battery levels (%)
    BATTERY_CRITICAL = 10.0
    BATTERY_WARNING = 20.0
    BATTERY_LOW = 30.0

    # System performance
    CPU_WARNING = 80.0
    CPU_CRITICAL = 90.0
    MEMORY_WARNING = 85.0
    MEMORY_CRITICAL = 95.0
    TEMPERATURE_WARNING = 70.0
    TEMPERATURE_CRITICAL = 85.0

    # Mission progress
    MISSION_PROGRESS_COMPLETE = 0.9
    MISSION_PROGRESS_ABORT = 0.1
    MISSION_PROGRESS_WARNING = 0.7

    # Environmental
    OBSTACLE_CRITICAL_DISTANCE = 0.3  # meters
    OBSTACLE_WARNING_DISTANCE = 1.0   # meters
    HAZARDOUS_AREA_DISTANCE = 5.0     # meters

    # Communication
    COMMUNICATION_LATENCY_WARNING = 100.0  # ms
    COMMUNICATION_LATENCY_CRITICAL = 500.0  # ms

    # System overload
    SYSTEM_OVERLOAD_CPU = 85.0
    SYSTEM_OVERLOAD_MEMORY = 90.0


# Policy Configuration
class PolicyConfig:
    """Configuration for adaptive policies."""

    EMERGENCY_RETURN_PRIORITY = 95
    COMPLETE_AND_RETURN_PRIORITY = 95
    MISSION_ABORT_PRIORITY = 90
    REQUEST_HUMAN_INTERVENTION_PRIORITY = 90
    COMMUNICATION_SAFE_MODE_PRIORITY = 80
    OBSTACLE_AVOIDANCE_PRIORITY = 75
    REDUCE_POWER_PRIORITY = 70
    SYSTEM_THROTTLE_PRIORITY = 65
    MISSION_PAUSE_PRIORITY = 60
    AUTO_RETURN_PRIORITY = 85

    # Duration estimates (seconds)
    OBSTACLE_AVOIDANCE_DURATION = 5.0
    SYSTEM_THROTTLE_DURATION = 10.0
    COMMUNICATION_SAFE_MODE_DURATION = 60.0

    # Power reduction factors
    POWER_REDUCTION_LOW = 0.8
    POWER_REDUCTION_MEDIUM = 0.6
    POWER_REDUCTION_HIGH = 0.4

    # System resource limits when throttling
    CPU_LIMIT_THROTTLE = 50
    MEMORY_LIMIT_THROTTLE = 60


# Quality of Service Configuration
class QoSConfig:
    """ROS2 QoS profile configurations."""

    # Reliability policies
    RELIABLE = "RELIABLE"
    BEST_EFFORT = "BEST_EFFORT"

    # History policies
    KEEP_LAST = "KEEP_LAST"
    KEEP_ALL = "KEEP_ALL"

    # Durability policies
    VOLATILE = "VOLATILE"
    TRANSIENT_LOCAL = "TRANSIENT_LOCAL"

    # Default QoS settings
    DEFAULT_DEPTH = 10
    SENSOR_DEPTH = 5
    STATE_DEPTH = 1

    # Profile configurations
    STATE_MACHINE_QOS = {
        "reliability": RELIABLE,
        "history": KEEP_LAST,
        "depth": STATE_DEPTH,
        "durability": TRANSIENT_LOCAL
    }

    SENSOR_QOS = {
        "reliability": BEST_EFFORT,
        "history": KEEP_LAST,
        "depth": SENSOR_DEPTH,
        "durability": VOLATILE
    }

    COMMAND_QOS = {
        "reliability": RELIABLE,
        "history": KEEP_ALL,
        "depth": DEFAULT_DEPTH,
        "durability": VOLATILE
    }


# Monitoring Configuration
class MonitoringConfig:
    """Configuration for monitoring and analytics."""

    # Data retention
    CONTEXT_HISTORY_SIZE = 1000
    ADAPTATION_HISTORY_SIZE = 500
    SYSTEM_EVENTS_SIZE = 200
    PERFORMANCE_METRICS_SIZE = 100

    # Analytics windows
    PERFORMANCE_ANALYSIS_WINDOW = 50
    TREND_ANALYSIS_WINDOW = 20
    PATTERN_ANALYSIS_WINDOW = 10

    # Alert thresholds
    ALERT_HIGH_PRIORITY_THRESHOLD = 80
    ALERT_CRITICAL_PRIORITY_THRESHOLD = 90

    # Dashboard update limits
    DASHBOARD_UPDATE_RATE = 1.0  # Hz
    DASHBOARD_ACTIVE_ACTIONS_LIMIT = 5


# System Limits
class SystemLimits:
    """System resource limits and safety bounds."""

    # Rate limits
    MAX_TRANSITIONS_PER_MINUTE = 60
    MAX_ADAPTIVE_ACTIONS_PER_MINUTE = 30
    MAX_ERROR_RATE_PER_MINUTE = 10

    # Size limits
    MAX_LOG_MESSAGE_LENGTH = 1000
    MAX_CONTEXT_DATA_SIZE = 10000
    MAX_HISTORY_RETENTION_DAYS = 7

    # Performance limits
    MAX_EVALUATION_TIME = 0.1  # seconds
    MAX_POLICY_EVALUATION_TIME = 0.05  # seconds
    MAX_TRANSITION_TIME = 0.2  # seconds


# Default Configurations
DEFAULT_CONFIG = {
    "timing": Timing,
    "thresholds": Thresholds,
    "policy": PolicyConfig,
    "qos": QoSConfig,
    "monitoring": MonitoringConfig,
    "limits": SystemLimits,
}


def get_config(section: str = "") -> Any:
    """
    Get configuration section or full config.

    Args:
        section: Configuration section name (optional)

    Returns:
        Configuration section or full config dictionary
    """
    if section and hasattr(DEFAULT_CONFIG, section):
        return getattr(DEFAULT_CONFIG, section)
    return DEFAULT_CONFIG


def validate_config(config: Dict[str, Any]) -> bool:
    """
    Validate configuration parameters.

    Args:
        config: Configuration dictionary to validate

    Returns:
        True if valid, False otherwise
    """
    # Basic validation - could be expanded
    required_sections = ["timing", "thresholds", "policy"]

    for section in required_sections:
        if section not in config:
            return False

    return True


