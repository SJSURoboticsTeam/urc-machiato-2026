#!/usr/bin/env python3
"""
Unified Utilities - Organized Utility Functions for URC 2026

Consolidates utility functions from across the codebase:
- Safety utilities (from safety_system.py)
- Hardware validation (from hardware_validator.py)
- Recovery coordination (from recovery_coordinator.py)
- Network resilience (from network_resilience.py)
- General utilities (from various modules)

Features:
- Organized by functionality
- Comprehensive error handling
- Performance optimized
- Well documented and tested

Author: URC 2026 Unified Utilities Team
"""

import time
import psutil
import threading
import logging
from typing import Dict, List, Any, Optional, Callable, Union, Tuple
from dataclasses import dataclass, field
from enum import Enum
import weakref

logger = logging.getLogger(__name__)


class SafetyLevel(Enum):
    """Safety levels for operations."""
    NOMINAL = "nominal"
    CAUTION = "caution"
    WARNING = "warning"
    CRITICAL = "critical"
    EMERGENCY = "emergency"


class SystemHealth(Enum):
    """Overall system health status."""
    HEALTHY = "healthy"
    DEGRADED = "degraded"
    UNHEALTHY = "unhealthy"
    FAILED = "failed"


@dataclass
class SafetyCheck:
    """Safety check result."""
    component: str
    check_type: str
    status: SafetyLevel
    message: str
    timestamp: float = field(default_factory=time.time)
    details: Dict[str, Any] = field(default_factory=dict)


@dataclass
class RecoveryAction:
    """Recovery action definition."""
    name: str
    description: str
    action_function: Callable
    priority: int = 1
    max_attempts: int = 3
    cooldown_seconds: float = 5.0


class SafetyManager:
    """
    Safety management utilities - consolidated from safety_system.py.

    Provides comprehensive safety checking, emergency handling,
    and system protection mechanisms.
    """

    def __init__(self):
        self.safety_checks: Dict[str, SafetyCheck] = {}
        self.emergency_handlers: List[Callable] = []
        self.safety_thresholds: Dict[str, Any] = {}
        self.system_health = SystemHealth.HEALTHY

        # Initialize default safety thresholds
        self._initialize_safety_thresholds()

    def _initialize_safety_thresholds(self):
        """Initialize default safety thresholds."""
        self.safety_thresholds = {
            "cpu_usage_percent": 90,
            "memory_usage_percent": 85,
            "disk_usage_percent": 95,
            "temperature_celsius": 70,
            "emergency_stop_timeout_sec": 5.0,
            "heartbeat_interval_sec": 1.0,
            "max_sensor_age_sec": 2.0
        }

    def perform_safety_check(self, component: str, check_type: str,
                           check_function: Callable) -> SafetyCheck:
        """
        Perform a safety check.

        Args:
            component: Component name
            check_type: Type of safety check
            check_function: Function that returns (is_safe, message, details)

        Returns:
            SafetyCheck result
        """
        try:
            result = check_function()
            if isinstance(result, tuple) and len(result) >= 2:
                is_safe, message, *details = result
                details = details[0] if details else {}
            else:
                is_safe = bool(result)
                message = "Check passed" if is_safe else "Check failed"
                details = {}

            # Determine safety level
            if is_safe:
                status = SafetyLevel.NOMINAL
            else:
                # Check severity based on check_type
                if "emergency" in check_type.lower() or "critical" in check_type.lower():
                    status = SafetyLevel.CRITICAL
                elif "warning" in check_type.lower():
                    status = SafetyLevel.WARNING
                else:
                    status = SafetyLevel.CAUTION

            safety_check = SafetyCheck(
                component=component,
                check_type=check_type,
                status=status,
                message=message,
                details=details
            )

            # Store result
            self.safety_checks[f"{component}_{check_type}"] = safety_check

            # Trigger emergency if critical
            if status == SafetyLevel.CRITICAL:
                self._trigger_emergency(safety_check)

            return safety_check

        except Exception as e:
            safety_check = SafetyCheck(
                component=component,
                check_type=check_type,
                status=SafetyLevel.CRITICAL,
                message=f"Safety check failed: {e}",
                details={"error": str(e)}
            )

            self.safety_checks[f"{component}_{check_type}"] = safety_check
            self._trigger_emergency(safety_check)

            return safety_check

    def _trigger_emergency(self, safety_check: SafetyCheck):
        """Trigger emergency procedures."""
        logger.critical(f"EMERGENCY TRIGGERED: {safety_check.component} - {safety_check.message}")

        for handler in self.emergency_handlers:
            try:
                handler(safety_check)
            except Exception as e:
                logger.error(f"Emergency handler failed: {e}")

    def add_emergency_handler(self, handler: Callable):
        """Add an emergency handler."""
        self.emergency_handlers.append(handler)

    def get_system_safety_status(self) -> Dict[str, Any]:
        """Get overall system safety status."""
        checks = list(self.safety_checks.values())

        if not checks:
            return {
                "overall_status": SafetyLevel.NOMINAL.value,
                "system_health": SystemHealth.HEALTHY.value,
                "checks_performed": 0,
                "critical_issues": 0
            }

        # Determine overall status
        status_levels = [check.status for check in checks]
        if SafetyLevel.CRITICAL in status_levels:
            overall_status = SafetyLevel.CRITICAL
            system_health = SystemHealth.FAILED
        elif SafetyLevel.WARNING in status_levels:
            overall_status = SafetyLevel.WARNING
            system_health = SystemHealth.DEGRADED
        elif SafetyLevel.CAUTION in status_levels:
            overall_status = SafetyLevel.CAUTION
            system_health = SystemHealth.DEGRADED
        else:
            overall_status = SafetyLevel.NOMINAL
            system_health = SystemHealth.HEALTHY

        critical_issues = len([c for c in checks if c.status == SafetyLevel.CRITICAL])

        return {
            "overall_status": overall_status.value,
            "system_health": system_health.value,
            "checks_performed": len(checks),
            "critical_issues": critical_issues,
            "last_check": max((c.timestamp for c in checks), default=time.time())
        }


class HardwareValidator:
    """
    Hardware validation utilities - consolidated from hardware_validator.py.

    Provides hardware compatibility checking, validation, and monitoring.
    """

    def __init__(self):
        self.hardware_specs: Dict[str, Dict[str, Any]] = {}
        self.validation_rules: Dict[str, Callable] = {}
        self.hardware_status: Dict[str, Dict[str, Any]] = {}

        # Initialize default hardware specs
        self._initialize_hardware_specs()

    def _initialize_hardware_specs(self):
        """Initialize default hardware specifications."""
        self.hardware_specs = {
            "can_bus": {
                "interface": "socketcan",
                "channel": "can0",
                "bitrate": 500000,
                "timeout_ms": 100
            },
            "motor_controller": {
                "max_current_a": 10.0,
                "max_voltage_v": 24.0,
                "max_rpm": 3000,
                "control_mode": "velocity"
            },
            "robotic_arm": {
                "degrees_of_freedom": 6,
                "payload_kg": 5.0,
                "reach_m": 1.2,
                "precision_mm": 1.0
            },
            "sensors": {
                "imu": {"update_rate_hz": 100, "accuracy_deg": 0.1},
                "gps": {"accuracy_m": 2.0, "update_rate_hz": 10},
                "lidar": {"range_m": 30, "fov_deg": 270}
            }
        }

    def validate_hardware_config(self, hardware_type: str,
                               config: Dict[str, Any]) -> Tuple[bool, List[str]]:
        """
        Validate hardware configuration.

        Args:
            hardware_type: Type of hardware
            config: Hardware configuration

        Returns:
            Tuple of (is_valid, error_messages)
        """
        errors = []

        if hardware_type not in self.hardware_specs:
            errors.append(f"Unknown hardware type: {hardware_type}")
            return False, errors

        specs = self.hardware_specs[hardware_type]

        # Validate required fields
        required_fields = ["can_bus", "motor_controller", "robotic_arm", "sensors"]
        if hardware_type in required_fields:
            for field in required_fields:
                if field not in config:
                    errors.append(f"Missing required hardware component: {field}")

        # Validate specifications
        for key, expected_value in specs.items():
            if key in config:
                actual_value = config[key]

                if isinstance(expected_value, dict):
                    # Nested validation
                    continue
                elif isinstance(expected_value, (int, float)):
                    if isinstance(actual_value, (int, float)):
                        if actual_value > expected_value * 1.1:  # 10% tolerance
                            errors.append(f"{key}: {actual_value} exceeds recommended {expected_value}")
                        elif actual_value < expected_value * 0.9:
                            errors.append(f"{key}: {actual_value} below recommended {expected_value}")
                    else:
                        errors.append(f"{key}: expected numeric value, got {type(actual_value)}")

        return len(errors) == 0, errors

    def monitor_hardware_health(self, hardware_type: str,
                              health_data: Dict[str, Any]) -> Dict[str, Any]:
        """
        Monitor hardware health status.

        Args:
            hardware_type: Type of hardware
            health_data: Health monitoring data

        Returns:
            Health assessment
        """
        assessment = {
            "hardware_type": hardware_type,
            "status": "unknown",
            "issues": [],
            "recommendations": []
        }

        if hardware_type == "can_bus":
            # CAN bus health checks
            error_count = health_data.get("error_count", 0)
            bus_load = health_data.get("bus_load_percent", 0)

            if error_count > 100:
                assessment["status"] = "critical"
                assessment["issues"].append("High error count on CAN bus")
                assessment["recommendations"].append("Check bus termination and cable quality")

            if bus_load > 80:
                assessment["status"] = "warning"
                assessment["issues"].append("High bus utilization")
                assessment["recommendations"].append("Consider reducing message frequency")

        elif hardware_type == "motor_controller":
            # Motor controller health checks
            temperature = health_data.get("temperature_c", 25)
            current = health_data.get("current_a", 0)

            if temperature > 70:
                assessment["status"] = "critical"
                assessment["issues"].append("Motor controller overheating")
                assessment["recommendations"].append("Check cooling system and reduce load")

            if current > 8.0:  # 80% of max
                assessment["status"] = "warning"
                assessment["issues"].append("High current draw")
                assessment["recommendations"].append("Check for mechanical binding")

        # Default to healthy if no issues found
        if not assessment["issues"]:
            assessment["status"] = "healthy"

        # Store status
        self.hardware_status[hardware_type] = {
            **assessment,
            "timestamp": time.time(),
            "health_data": health_data
        }

        return assessment

    def get_hardware_status(self) -> Dict[str, Any]:
        """Get overall hardware status."""
        return {
            "hardware_components": len(self.hardware_status),
            "component_status": self.hardware_status,
            "last_updated": max((status.get("timestamp", 0) for status in self.hardware_status.values()), default=time.time())
        }


class RecoveryCoordinator:
    """
    Recovery coordination utilities - consolidated from recovery_coordinator.py.

    Provides automated recovery mechanisms and coordination.
    """

    def __init__(self):
        self.recovery_actions: Dict[str, RecoveryAction] = {}
        self.active_recoveries: Dict[str, Dict[str, Any]] = {}
        self.recovery_history: List[Dict[str, Any]] = []

    def register_recovery_action(self, failure_type: str, action: RecoveryAction):
        """Register a recovery action for a failure type."""
        self.recovery_actions[failure_type] = action

    def initiate_recovery(self, failure_type: str, context: Dict[str, Any]) -> bool:
        """
        Initiate recovery for a failure type.

        Args:
            failure_type: Type of failure
            context: Recovery context information

        Returns:
            True if recovery initiated successfully
        """
        if failure_type not in self.recovery_actions:
            logger.error(f"No recovery action registered for: {failure_type}")
            return False

        action = self.recovery_actions[failure_type]

        # Check if already recovering
        if failure_type in self.active_recoveries:
            active_recovery = self.active_recoveries[failure_type]
            if time.time() - active_recovery["start_time"] < action.cooldown_seconds:
                logger.info(f"Recovery for {failure_type} already in progress")
                return False

        # Start recovery
        recovery_id = f"{failure_type}_{int(time.time())}"
        self.active_recoveries[failure_type] = {
            "recovery_id": recovery_id,
            "action": action,
            "start_time": time.time(),
            "attempts": 0,
            "context": context
        }

        logger.info(f"Initiated recovery for {failure_type}: {action.description}")

        # Execute recovery in background
        threading.Thread(
            target=self._execute_recovery,
            args=(failure_type,),
            daemon=True
        ).start()

        return True

    def _execute_recovery(self, failure_type: str):
        """Execute recovery action."""
        recovery_info = self.active_recoveries[failure_type]
        action = recovery_info["action"]

        success = False
        for attempt in range(action.max_attempts):
            try:
                recovery_info["attempts"] = attempt + 1
                logger.info(f"Executing recovery attempt {attempt + 1}/{action.max_attempts} for {failure_type}")

                result = action.action_function(recovery_info["context"])

                if result:
                    success = True
                    logger.info(f"Recovery successful for {failure_type}")
                    break
                else:
                    logger.warning(f"Recovery attempt {attempt + 1} failed for {failure_type}")

            except Exception as e:
                logger.error(f"Recovery attempt {attempt + 1} error for {failure_type}: {e}")

            # Wait before next attempt
            if attempt < action.max_attempts - 1:
                time.sleep(action.cooldown_seconds)

        # Record result
        recovery_record = {
            "failure_type": failure_type,
            "recovery_id": recovery_info["recovery_id"],
            "success": success,
            "attempts": recovery_info["attempts"],
            "duration": time.time() - recovery_info["start_time"],
            "timestamp": time.time()
        }

        self.recovery_history.append(recovery_record)

        # Clean up
        del self.active_recoveries[failure_type]

        if not success:
            logger.error(f"All recovery attempts failed for {failure_type}")

    def get_recovery_status(self) -> Dict[str, Any]:
        """Get recovery system status."""
        return {
            "active_recoveries": len(self.active_recoveries),
            "registered_actions": len(self.recovery_actions),
            "recovery_history_count": len(self.recovery_history),
            "active_recovery_types": list(self.active_recoveries.keys())
        }


class NetworkResilienceManager:
    """
    Network resilience utilities - consolidated from network_resilience.py.

    Provides network fault tolerance and optimization.
    """

    def __init__(self):
        self.network_conditions: Dict[str, Any] = {}
        self.resilience_measures: Dict[str, Callable] = {}
        self.network_stats: Dict[str, Any] = {}

    def set_network_condition(self, condition: str, **params):
        """Set network condition for simulation."""
        self.network_conditions[condition] = params

    def get_adaptive_timeout(self, base_timeout: float = 5.0) -> float:
        """Get adaptive timeout based on network conditions."""
        multiplier = 1.0

        if "high_latency" in self.network_conditions:
            multiplier = self.network_conditions["high_latency"].get("multiplier", 2.0)
        elif "packet_loss" in self.network_conditions:
            loss_rate = self.network_conditions["packet_loss"].get("rate", 0.0)
            multiplier = 1.0 + (loss_rate * 3.0)  # Up to 3x for 100% loss

        return base_timeout * multiplier

    def should_retry_request(self, attempt: int, error_type: str) -> bool:
        """Determine if request should be retried."""
        max_retries = self.network_conditions.get("max_retries", 3)

        if attempt >= max_retries:
            return False

        # Retry on network errors
        retry_errors = ["timeout", "connection", "network"]
        return any(err in error_type.lower() for err in retry_errors)

    def get_retry_delay(self, attempt: int) -> float:
        """Get retry delay with exponential backoff."""
        base_delay = self.network_conditions.get("base_retry_delay", 1.0)
        max_delay = self.network_conditions.get("max_retry_delay", 30.0)

        delay = base_delay * (2 ** attempt)  # Exponential backoff
        return min(delay, max_delay)

    def monitor_network_health(self) -> Dict[str, Any]:
        """Monitor network health status."""
        # This would integrate with actual network monitoring
        return {
            "latency_ms": 50,  # Mock values
            "packet_loss_percent": 0.1,
            "bandwidth_mbps": 10.0,
            "status": "healthy"
        }


class SystemUtilities:
    """
    General system utilities - consolidated from various modules.

    Provides common utility functions used across the system.
    """

    @staticmethod
    def get_system_info() -> Dict[str, Any]:
        """Get comprehensive system information."""
        import sys
        return {
            "cpu_count": psutil.cpu_count(),
            "cpu_percent": psutil.cpu_percent(),
            "memory_total": psutil.virtual_memory().total,
            "memory_available": psutil.virtual_memory().available,
            "memory_percent": psutil.virtual_memory().percent,
            "disk_usage": psutil.disk_usage('/').percent,
            "uptime_seconds": time.time() - psutil.boot_time(),
            "python_version": f"{sys.version}",
            "platform": psutil.sys.platform
        }

    @staticmethod
    def format_bytes(bytes_value: int) -> str:
        """Format bytes to human readable format."""
        for unit in ['B', 'KB', 'MB', 'GB', 'TB']:
            if bytes_value < 1024.0:
                return ".1f"
            bytes_value /= 1024.0
        return ".1f"

    @staticmethod
    def calculate_rate(current: float, previous: float, time_delta: float) -> float:
        """Calculate rate of change."""
        if time_delta == 0:
            return 0.0
        return (current - previous) / time_delta

    @staticmethod
    def validate_ip_address(ip: str) -> bool:
        """Validate IP address format."""
        import re
        pattern = r'^(\d{1,3})\.(\d{1,3})\.(\d{1,3})\.(\d{1,3})$'
        match = re.match(pattern, ip)

        if not match:
            return False

        for part in match.groups():
            if not 0 <= int(part) <= 255:
                return False

        return True

    @staticmethod
    def deep_merge_dicts(base: Dict[str, Any], update: Dict[str, Any]) -> Dict[str, Any]:
        """Deep merge two dictionaries."""
        result = base.copy()

        for key, value in update.items():
            if key in result and isinstance(result[key], dict) and isinstance(value, dict):
                result[key] = SystemUtilities.deep_merge_dicts(result[key], value)
            else:
                result[key] = value

        return result


# Global utility instances
_safety_manager = None
_hardware_validator = None
_recovery_coordinator = None
_network_resilience = None

def get_safety_manager() -> SafetyManager:
    """Get global safety manager instance."""
    global _safety_manager
    if _safety_manager is None:
        _safety_manager = SafetyManager()
    return _safety_manager

def get_hardware_validator() -> HardwareValidator:
    """Get global hardware validator instance."""
    global _hardware_validator
    if _hardware_validator is None:
        _hardware_validator = HardwareValidator()
    return _hardware_validator

def get_recovery_coordinator() -> RecoveryCoordinator:
    """Get global recovery coordinator instance."""
    global _recovery_coordinator
    if _recovery_coordinator is None:
        _recovery_coordinator = RecoveryCoordinator()
    return _recovery_coordinator

def get_network_resilience_manager() -> NetworkResilienceManager:
    """Get global network resilience manager instance."""
    global _network_resilience
    if _network_resilience is None:
        _network_resilience = NetworkResilienceManager()
    return _network_resilience

# Export all utility classes and functions
__all__ = [
    'SafetyManager',
    'HardwareValidator',
    'RecoveryCoordinator',
    'NetworkResilienceManager',
    'SystemUtilities',
    'SafetyLevel',
    'SystemHealth',
    'SafetyCheck',
    'RecoveryAction',
    'get_safety_manager',
    'get_hardware_validator',
    'get_recovery_coordinator',
    'get_network_resilience_manager'
]
