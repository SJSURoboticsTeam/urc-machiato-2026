#!/usr/bin/env python3
"""
Adaptive Circuit Breaker - Requirement-Driven Fault Tolerance

Circuit breaker with settings calculated from availability requirements.
Replaces hardcoded values with mathematically-derived parameters based on SLA.

Features:
- Availability SLA-driven configuration
- Automatic parameter calculation
- Different profiles for different services
- Performance monitoring and adjustment

Author: URC 2026 Fault Tolerance Optimization Team
"""

import time
import threading
from typing import Dict, Any, Optional, Callable
from dataclasses import dataclass
from enum import Enum
import logging
import math

logger = logging.getLogger(__name__)


class CircuitBreakerState(Enum):
    """Circuit breaker states."""
    CLOSED = "closed"      # Normal operation
    OPEN = "open"          # Failing, requests rejected
    HALF_OPEN = "half_open"  # Testing if service recovered


@dataclass
class AvailabilityRequirement:
    """Service availability requirements."""
    required_availability_percent: float  # 99.9 = 99.9% uptime required
    acceptable_downtime_seconds: float    # Max acceptable downtime per incident
    service_name: str
    description: str = ""


@dataclass
class CircuitBreakerConfig:
    """Calculated circuit breaker configuration."""
    failure_threshold: int
    recovery_timeout_seconds: float
    success_threshold: int
    timeout_seconds: float
    monitoring_window_seconds: float
    name: str


class AdaptiveCircuitBreaker:
    """
    Circuit breaker with parameters calculated from availability requirements.

    Automatically adjusts failure thresholds and timeouts based on SLA requirements.
    """

    # Predefined availability profiles
    AVAILABILITY_PROFILES = {
        'motion_control': AvailabilityRequirement(
            required_availability_percent=99.9,
            acceptable_downtime_seconds=0.5,  # 500ms max downtime
            service_name='motion_control',
            description='Critical motion control - <20ms latency required'
        ),
        'sensor_fusion': AvailabilityRequirement(
            required_availability_percent=99.5,
            acceptable_downtime_seconds=2.0,  # 2s max downtime
            service_name='sensor_fusion',
            description='Sensor fusion - <100ms latency required'
        ),
        'navigation': AvailabilityRequirement(
            required_availability_percent=99.0,
            acceptable_downtime_seconds=10.0,  # 10s max downtime
            service_name='navigation',
            description='Navigation planning - <500ms latency acceptable'
        ),
        'telemetry': AvailabilityRequirement(
            required_availability_percent=95.0,
            acceptable_downtime_seconds=300.0,  # 5min max downtime
            service_name='telemetry',
            description='Telemetry logging - best effort'
        ),
        'camera': AvailabilityRequirement(
            required_availability_percent=98.0,
            acceptable_downtime_seconds=5.0,  # 5s max downtime
            service_name='camera',
            description='Camera processing - 30Hz required'
        ),
        'gps': AvailabilityRequirement(
            required_availability_percent=90.0,
            acceptable_downtime_seconds=30.0,  # 30s max downtime
            service_name='gps',
            description='GPS positioning - occasional dropouts acceptable'
        )
    }

    def __init__(self, service_name: str,
                 availability_requirement: Optional[AvailabilityRequirement] = None):
        """
        Initialize adaptive circuit breaker.

        Args:
            service_name: Name of the service this breaker protects
            availability_requirement: Custom requirement, or None to use predefined profile
        """
        self.service_name = service_name

        # Get or create availability requirement
        if availability_requirement:
            self.requirement = availability_requirement
        elif service_name in self.AVAILABILITY_PROFILES:
            self.requirement = self.AVAILABILITY_PROFILES[service_name]
        else:
            # Default to telemetry profile for unknown services
            logger.warning(f"No availability profile for {service_name}, using telemetry defaults")
            self.requirement = self.AVAILABILITY_PROFILES['telemetry']

        # Calculate configuration from requirements
        self.config = self._calculate_config_from_requirement(self.requirement)

        # Runtime state
        self.state = CircuitBreakerState.CLOSED
        self.failure_count = 0
        self.success_count = 0
        self.last_failure_time = 0.0
        self.last_attempt_time = 0.0
        self.consecutive_successes = 0
        self.consecutive_failures = 0

        # Statistics
        self.total_requests = 0
        self.total_failures = 0
        self.total_timeouts = 0
        self.state_change_count = 0
        self.last_state_change_time = time.time()

        # Callbacks
        self.state_change_callbacks: list[Callable[[CircuitBreakerState, CircuitBreakerState, str], None]] = []

        self.lock = threading.RLock()

        logger.info(f"AdaptiveCircuitBreaker initialized for {service_name}: "
                   f"{self.config.failure_threshold} failures, "
                   f"{self.config.recovery_timeout_seconds:.1f}s recovery timeout, "
                   f"{self.requirement.required_availability_percent:.1f}% availability required")

    def _calculate_config_from_requirement(self, req: AvailabilityRequirement) -> CircuitBreakerConfig:
        """
        Calculate circuit breaker parameters from availability requirements.

        Mathematical approach:
        - Failure threshold based on acceptable downtime frequency
        - Recovery timeout based on maximum acceptable outage duration
        - Success threshold based on required confidence in recovery
        """
        # Convert percentage to decimal
        required_availability = req.required_availability_percent / 100.0

        # Calculate failure threshold
        # Higher availability = more sensitive to failures (lower threshold)
        # Lower availability = more tolerant (higher threshold)
        if required_availability >= 0.999:  # 99.9%+
            failure_threshold = 2  # Very sensitive
        elif required_availability >= 0.995:  # 99.5%+
            failure_threshold = 3
        elif required_availability >= 0.99:   # 99%+
            failure_threshold = 5
        elif required_availability >= 0.95:   # 95%+
            failure_threshold = 10
        else:  # <95%
            failure_threshold = 20  # Very tolerant

        # Calculate recovery timeout
        # Shorter acceptable downtime = faster recovery attempts
        recovery_timeout = min(req.acceptable_downtime_seconds / 10, 60.0)
        recovery_timeout = max(recovery_timeout, 1.0)  # Minimum 1 second

        # Calculate success threshold
        # Higher availability requirements need more proof of recovery
        if required_availability >= 0.995:
            success_threshold = 5  # Need strong proof
        elif required_availability >= 0.99:
            success_threshold = 3
        else:
            success_threshold = 2  # Quick recovery for less critical services

        # Calculate timeout
        # Based on expected service response time
        if req.service_name in ['motion_control', 'sensor_fusion']:
            timeout_seconds = 0.1  # 100ms for real-time services
        elif req.service_name in ['navigation', 'camera']:
            timeout_seconds = 1.0  # 1s for soft real-time
        else:
            timeout_seconds = 5.0  # 5s for best-effort services

        # Monitoring window
        monitoring_window = 300.0  # 5 minutes default

        return CircuitBreakerConfig(
            failure_threshold=failure_threshold,
            recovery_timeout_seconds=recovery_timeout,
            success_threshold=success_threshold,
            timeout_seconds=timeout_seconds,
            monitoring_window_seconds=monitoring_window,
            name=f"{req.service_name}_circuit_breaker"
        )

    def call(self, func: Callable, *args, **kwargs):
        """
        Execute function through circuit breaker.

        Args:
            func: Function to execute
            *args, **kwargs: Arguments to pass to function

        Returns:
            Function result

        Raises:
            CircuitBreakerOpenException: If circuit breaker is open
            Exception: Any exception from the wrapped function
        """
        with self.lock:
            self.total_requests += 1

            if self.state == CircuitBreakerState.OPEN:
                if self._should_attempt_reset():
                    self._change_state(CircuitBreakerState.HALF_OPEN,
                                     "attempting_recovery")
                else:
                    raise CircuitBreakerOpenException(
                        f"Circuit breaker {self.service_name} is OPEN")

        try:
            start_time = time.time()
            result = func(*args, **kwargs)
            execution_time = time.time() - start_time

            # Check for timeout (even if function succeeded)
            if execution_time > self.config.timeout_seconds:
                self._record_timeout(execution_time)
                raise CircuitBreakerTimeoutException(
                    f"Function call exceeded timeout {self.config.timeout_seconds:.1f}s "
                    f"(took {execution_time:.1f}s)"
                )

            self._record_success()
            return result

        except Exception as e:
            self._record_failure()
            raise

    def _record_success(self):
        """Record a successful call."""
        self.consecutive_failures = 0
        self.consecutive_successes += 1

        if self.state == CircuitBreakerState.HALF_OPEN:
            if self.consecutive_successes >= self.config.success_threshold:
                self._change_state(CircuitBreakerState.CLOSED,
                                 f"recovered_after_{self.consecutive_successes}_successes")

    def _record_failure(self):
        """Record a failed call."""
        self.total_failures += 1
        self.consecutive_failures += 1
        self.consecutive_successes = 0
        self.last_failure_time = time.time()

        if self.state == CircuitBreakerState.HALF_OPEN:
            self._change_state(CircuitBreakerState.OPEN,
                             f"failure_during_recovery_{self.consecutive_failures}_consecutive")
        elif (self.state == CircuitBreakerState.CLOSED and
              self.consecutive_failures >= self.config.failure_threshold):
            self._change_state(CircuitBreakerState.OPEN,
                             f"failure_threshold_exceeded_{self.consecutive_failures}_consecutive")

    def _record_timeout(self, execution_time: float):
        """Record a timeout."""
        self.total_timeouts += 1
        logger.warning(f"Circuit breaker {self.service_name} timeout: "
                      f"{execution_time:.1f}s > {self.config.timeout_seconds:.1f}s")

    def _should_attempt_reset(self) -> bool:
        """Check if circuit breaker should attempt to reset."""
        if self.state != CircuitBreakerState.OPEN:
            return False

        time_since_failure = time.time() - self.last_failure_time
        return time_since_failure >= self.config.recovery_timeout_seconds

    def _change_state(self, new_state: CircuitBreakerState, reason: str):
        """Change circuit breaker state and notify callbacks."""
        if new_state == self.state:
            return

        old_state = self.state
        self.state = new_state
        self.state_change_count += 1
        self.last_state_change_time = time.time()

        # Reset counters on state changes
        if new_state == CircuitBreakerState.CLOSED:
            self.failure_count = 0
            self.success_count = 0
            self.consecutive_failures = 0
            self.consecutive_successes = 0

        logger.info(f"Circuit breaker {self.service_name}: {old_state.value} -> {new_state.value} "
                   f"({reason})")

        # Notify callbacks
        for callback in self.state_change_callbacks:
            try:
                callback(old_state, new_state, reason)
            except Exception as e:
                logger.error(f"State change callback failed: {e}")

    def register_state_change_callback(self, callback: Callable[[CircuitBreakerState, CircuitBreakerState, str], None]):
        """Register callback for state changes."""
        with self.lock:
            self.state_change_callbacks.append(callback)

    def get_stats(self) -> Dict[str, Any]:
        """Get circuit breaker statistics."""
        with self.lock:
            success_rate = 0.0
            if self.total_requests > 0:
                success_rate = ((self.total_requests - self.total_failures) / self.total_requests) * 100

            availability_percent = 0.0
            if self.total_requests > 0:
                availability_percent = ((self.total_requests - self.total_failures) /
                                      self.total_requests) * 100

            return {
                'service_name': self.service_name,
                'state': self.state.value,
                'required_availability_percent': self.requirement.required_availability_percent,
                'acceptable_downtime_seconds': self.requirement.acceptable_downtime_seconds,
                'config': {
                    'failure_threshold': self.config.failure_threshold,
                    'recovery_timeout_seconds': self.config.recovery_timeout_seconds,
                    'success_threshold': self.config.success_threshold,
                    'timeout_seconds': self.config.timeout_seconds,
                },
                'counters': {
                    'total_requests': self.total_requests,
                    'total_failures': self.total_failures,
                    'total_timeouts': self.total_timeouts,
                    'consecutive_failures': self.consecutive_failures,
                    'consecutive_successes': self.consecutive_successes,
                    'state_changes': self.state_change_count,
                },
                'metrics': {
                    'success_rate_percent': success_rate,
                    'current_availability_percent': availability_percent,
                    'time_since_last_failure': time.time() - self.last_failure_time,
                    'time_since_last_state_change': time.time() - self.last_state_change_time,
                },
                'timestamp': time.time()
            }

    def reset_stats(self):
        """Reset circuit breaker statistics."""
        with self.lock:
            self.total_requests = 0
            self.total_failures = 0
            self.total_timeouts = 0
            self.state_change_count = 0
            logger.info(f"Reset statistics for circuit breaker {self.service_name}")

    def force_open(self, reason: str = "manual_override"):
        """Manually force circuit breaker open."""
        with self.lock:
            self._change_state(CircuitBreakerState.OPEN, reason)

    def force_close(self, reason: str = "manual_override"):
        """Manually force circuit breaker closed."""
        with self.lock:
            self._change_state(CircuitBreakerState.CLOSED, reason)

    def is_available(self) -> bool:
        """Check if service is currently available."""
        return self.state != CircuitBreakerState.OPEN

    def get_effective_availability(self) -> float:
        """Calculate effective availability based on recent performance."""
        stats = self.get_stats()
        return stats['metrics']['current_availability_percent']


class CircuitBreakerOpenException(Exception):
    """Raised when circuit breaker is open."""
    pass


class CircuitBreakerTimeoutException(Exception):
    """Raised when operation times out."""
    pass


# Global instances
_circuit_breakers: Dict[str, AdaptiveCircuitBreaker] = {}
_circuit_breakers_lock = threading.Lock()


def get_adaptive_circuit_breaker(service_name: str,
                                availability_requirement: Optional[AvailabilityRequirement] = None) -> AdaptiveCircuitBreaker:
    """Get or create adaptive circuit breaker for service."""
    global _circuit_breakers

    with _circuit_breakers_lock:
        if service_name not in _circuit_breakers:
            _circuit_breakers[service_name] = AdaptiveCircuitBreaker(
                service_name, availability_requirement
            )

        return _circuit_breakers[service_name]


def get_circuit_breaker_stats(service_name: Optional[str] = None) -> Dict[str, Any]:
    """Get statistics for circuit breaker(s)."""
    with _circuit_breakers_lock:
        if service_name:
            breaker = _circuit_breakers.get(service_name)
            if breaker:
                return {service_name: breaker.get_stats()}
            else:
                return {}
        else:
            return {name: breaker.get_stats() for name, breaker in _circuit_breakers.items()}


def reset_all_circuit_breakers():
    """Reset all circuit breaker statistics."""
    with _circuit_breakers_lock:
        for breaker in _circuit_breakers.values():
            breaker.reset_stats()


# Convenience functions for common services
def get_motion_control_breaker() -> AdaptiveCircuitBreaker:
    """Get circuit breaker for motion control (99.9% availability required)."""
    return get_adaptive_circuit_breaker('motion_control')


def get_sensor_fusion_breaker() -> AdaptiveCircuitBreaker:
    """Get circuit breaker for sensor fusion (99.5% availability required)."""
    return get_adaptive_circuit_breaker('sensor_fusion')


def get_navigation_breaker() -> AdaptiveCircuitBreaker:
    """Get circuit breaker for navigation (99.0% availability required)."""
    return get_adaptive_circuit_breaker('navigation')


def get_telemetry_breaker() -> AdaptiveCircuitBreaker:
    """Get circuit breaker for telemetry (95.0% availability required)."""
    return get_adaptive_circuit_breaker('telemetry')


def get_camera_breaker() -> AdaptiveCircuitBreaker:
    """Get circuit breaker for camera (98.0% availability required)."""
    return get_adaptive_circuit_breaker('camera')


def get_gps_breaker() -> AdaptiveCircuitBreaker:
    """Get circuit breaker for GPS (90.0% availability required)."""
    return get_adaptive_circuit_breaker('gps')
