#!/usr/bin/env python3
"""
Safety Monitoring System for URC Rover

Provides runtime verification of critical safety properties using temporal logic.
Monitors system state against formal specifications to ensure safety invariants
are maintained during operation.

Key Safety Properties:
- Battery safety: Never operate with critically low battery
- Geofencing: Never violate competition boundaries during mission
- Control timing: Maintain real-time control loop deadlines
- Emergency stops: Always respond to emergency conditions
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from typing import Dict, List, Any, Optional, Callable
import time
from collections import deque
from enum import Enum

from autonomy_interfaces.msg import SystemState, ContextState, AdaptiveAction as AdaptiveActionMsg
from autonomy_interfaces.srv import GetSafetyStatus, VerifySafetyProperty
from sensor_msgs.msg import BatteryState, NavSatFix


class SafetyLevel(Enum):
    """Safety violation severity levels."""
    NOMINAL = "nominal"
    WARNING = "warning"
    CRITICAL = "critical"
    EMERGENCY = "emergency"


class SafetyProperty:
    """Represents a safety property with its specification and monitoring."""

    def __init__(self, name: str, description: str, specification: str,
                 evaluator: Callable, severity: SafetyLevel, violation_threshold: int = 1):
        self.name = name
        self.description = description
        self.specification = specification  # Temporal logic specification
        self.evaluator = evaluator  # Function to evaluate the property
        self.severity = severity
        self.violation_threshold = violation_threshold
        self.violation_count = 0
        self.last_violation_time = None
        self.consecutive_violations = 0
        self.violation_history = deque(maxlen=100)  # Store last 100 violations

    def evaluate(self, system_state: Dict[str, Any]) -> Tuple[bool, str]:
        """Evaluate the safety property. Returns (satisfied, details)."""
        try:
            satisfied = self.evaluator(system_state)
            if not satisfied:
                self.violation_count += 1
                self.consecutive_violations += 1
                self.last_violation_time = time.time()
                self.violation_history.append({
                    'timestamp': time.time(),
                    'system_state': system_state.copy()
                })
                return False, f"Safety property '{self.name}' violated"
            else:
                self.consecutive_violations = 0  # Reset consecutive counter
                return True, ""
        except Exception as e:
            return False, f"Error evaluating safety property '{self.name}': {str(e)}"


class SafetyMonitor(Node):
    """
    Runtime safety monitoring for the URC rover system.

    Monitors critical safety properties and provides real-time verification
    of system behavior against formal safety specifications.
    """

    def __init__(self):
        super().__init__('safety_monitor')

        # System state tracking
        self.system_state = {
            'battery_level': 100.0,
            'battery_voltage': 0.0,
            'current': 0.0,
            'gps_position': {'lat': 0.0, 'lon': 0.0, 'alt': 0.0},
            'velocity': {'linear': [0.0, 0.0, 0.0], 'angular': [0.0, 0.0, 0.0]},
            'current_mission': 'none',
            'mission_active': False,
            'autonomous_mode': False,
            'emergency_stop': False,
            'boundary_violation': False,
            'control_loop_active': False,
            'last_control_update': time.time(),
            'communication_healthy': True,
            'last_communication': time.time(),
            'terrain_safety': True,
            'motor_temperatures': [],
            'joint_positions': [],
            'system_errors': []
        }

        # Define safety properties
        self.safety_properties = self._define_safety_properties()

        # Monitoring statistics
        self.monitoring_stats = {
            'total_evaluations': 0,
            'total_violations': 0,
            'active_alerts': [],
            'violation_counts': {},
            'evaluation_rate': 10.0,  # Hz
            'last_evaluation': time.time()
        }

        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('monitoring_rate', 10.0),           # Hz - safety monitoring frequency
                ('violation_alert_threshold', 3),     # Consecutive violations before alert
                ('communication_timeout', 5.0),       # Seconds before communication loss alert
                ('control_loop_timeout', 0.1),        # Seconds before control loop timeout
                ('emergency_stop_response_time', 0.05),  # Seconds max response time
                ('geofence_tolerance', 5.0),          # Meters tolerance for geofence
            ]
        )

        # Get parameters
        self.monitoring_rate = self.get_parameter('monitoring_rate').value
        self.violation_alert_threshold = self.get_parameter('violation_alert_threshold').value
        self.communication_timeout = self.get_parameter('communication_timeout').value
        self.control_loop_timeout = self.get_parameter('control_loop_timeout').value
        self.emergency_response_time = self.get_parameter('emergency_stop_response_time').value
        self.geofence_tolerance = self.get_parameter('geofence_tolerance').value

        # Setup ROS2 interfaces
        self._setup_subscribers()
        self._setup_services()
        self._setup_timers()

        self.get_logger().info("Safety Monitor initialized with runtime verification")

    def _define_safety_properties(self) -> Dict[str, SafetyProperty]:
        """Define all safety properties to be monitored."""

        properties = {}

        # Battery Safety: Never operate with critically low battery
        def battery_safety_evaluator(state):
            battery_level = state.get('battery_level', 100.0)
            mission_active = state.get('mission_active', False)
            emergency_stop = state.get('emergency_stop', False)

            # Allow operation down to 15% if emergency stop is engaged
            if emergency_stop:
                return battery_level > 5.0

            # Require higher battery level during active missions
            if mission_active:
                return battery_level > 20.0

            # General operation requires 10% minimum
            return battery_level > 10.0

        properties['battery_safety'] = SafetyProperty(
            name='battery_safety',
            description='Battery level must remain above critical thresholds',
            specification='G(battery_level > 10% ∧ (mission_active → battery_level > 20%))',
            evaluator=battery_safety_evaluator,
            severity=SafetyLevel.CRITICAL,
            violation_threshold=1
        )

        # Geofencing: Never violate boundaries during mission
        def geofence_evaluator(state):
            mission_active = state.get('mission_active', False)
            boundary_violation = state.get('boundary_violation', False)

            # Only enforce geofencing during active missions
            if mission_active:
                return not boundary_violation
            return True

        properties['geofence_compliance'] = SafetyProperty(
            name='geofence_compliance',
            description='Must not violate competition boundaries during missions',
            specification='G(mission_active → ¬boundary_violation)',
            evaluator=geofence_evaluator,
            severity=SafetyLevel.EMERGENCY,
            violation_threshold=1
        )

        # Control Loop Timing: Maintain real-time deadlines
        def control_timing_evaluator(state):
            last_update = state.get('last_control_update', time.time())
            time_since_update = time.time() - last_update
            control_active = state.get('control_loop_active', False)

            if control_active:
                return time_since_update < 0.020  # 20ms deadline
            return True

        properties['control_loop_timing'] = SafetyProperty(
            name='control_loop_timing',
            description='Control loop must maintain 20ms timing deadlines',
            specification='G(control_active → (time_since_last_update < 20ms))',
            evaluator=control_timing_evaluator,
            severity=SafetyLevel.CRITICAL,
            violation_threshold=2
        )

        # Emergency Stop Response: Always respond quickly to emergency conditions
        def emergency_response_evaluator(state):
            emergency_stop = state.get('emergency_stop', False)
            if emergency_stop:
                # Check if emergency actions were taken within response time
                # This would require tracking when emergency was declared vs when actions completed
                return True  # Placeholder - implement actual timing check
            return True

        properties['emergency_stop_response'] = SafetyProperty(
            name='emergency_stop_response',
            description='Must respond to emergency stops within timing requirements',
            specification='G(emergency_declared → ◇(emergency_actions_completed))_≤50ms',
            evaluator=emergency_response_evaluator,
            severity=SafetyLevel.EMERGENCY,
            violation_threshold=1
        )

        # Communication Health: Maintain communication links
        def communication_evaluator(state):
            last_comm = state.get('last_communication', time.time())
            time_since_comm = time.time() - last_comm
            mission_active = state.get('mission_active', False)

            if mission_active:
                return time_since_comm < self.communication_timeout
            return time_since_comm < (self.communication_timeout * 2)  # More lenient when idle

        properties['communication_health'] = SafetyProperty(
            name='communication_health',
            description='Must maintain communication links within timeout limits',
            specification='G(mission_active → (time_since_comm < 5s))',
            evaluator=communication_evaluator,
            severity=SafetyLevel.WARNING,
            violation_threshold=5
        )

        # Terrain Safety: Avoid hazardous terrain during navigation
        def terrain_safety_evaluator(state):
            terrain_safety = state.get('terrain_safety', True)
            autonomous_mode = state.get('autonomous_mode', False)

            # Only enforce during autonomous operation
            if autonomous_mode:
                return terrain_safety
            return True

        properties['terrain_safety'] = SafetyProperty(
            name='terrain_safety',
            description='Must avoid hazardous terrain during autonomous navigation',
            specification='G(autonomous_mode → terrain_safe)',
            evaluator=terrain_safety_evaluator,
            severity=SafetyLevel.WARNING,
            violation_threshold=3
        )

        return properties

    def _setup_subscribers(self):
        """Setup ROS2 subscribers for safety-relevant data."""
        callback_group = MutuallyExclusiveCallbackGroup()

        # System state monitoring
        self.create_subscription(
            SystemState, '/state_machine/current_state',
            self._system_state_callback,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=1),
            callback_group=callback_group
        )

        # Context monitoring
        self.create_subscription(
            ContextState, '/state_machine/context',
            self._context_callback,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=1),
            callback_group=callback_group
        )

        # Battery monitoring
        self.create_subscription(
            BatteryState, '/hardware/battery_state',
            self._battery_callback,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=1),
            callback_group=callback_group
        )

        # GPS position monitoring
        self.create_subscription(
            NavSatFix, '/hardware/gps',
            self._gps_callback,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1),
            callback_group=callback_group
        )

    def _setup_services(self):
        """Setup ROS2 services for safety status access."""
        self.create_service(
            GetSafetyStatus,
            '/safety_monitor/get_status',
            self._handle_get_safety_status
        )

        self.create_service(
            VerifySafetyProperty,
            '/safety_monitor/verify_property',
            self._handle_verify_property
        )

    def _setup_timers(self):
        """Setup periodic timers for safety monitoring."""
        # Main safety monitoring timer
        self.create_timer(
            1.0 / self.monitoring_rate,
            self._safety_monitoring_callback,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        # Alert checking timer
        self.create_timer(
            1.0,  # 1Hz alert checking
            self._alert_check_callback,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

    def _system_state_callback(self, msg):
        """Handle system state updates for safety monitoring."""
        self.system_state.update({
            'current_mission': msg.current_mission,
            'mission_active': msg.mission_active,
            'autonomous_mode': msg.autonomous_mode,
            'emergency_stop': msg.emergency_stop,
            'boundary_violation': msg.boundary_violation,
            'control_loop_active': msg.control_loop_active,
            'last_control_update': time.time(),  # Update timestamp when we receive state
            'system_errors': list(msg.system_errors) if hasattr(msg, 'system_errors') else []
        })

    def _context_callback(self, msg):
        """Handle context updates."""
        # Update context-relevant safety state
        pass

    def _battery_callback(self, msg):
        """Handle battery state updates."""
        self.system_state.update({
            'battery_level': msg.percentage * 100.0,  # Convert to percentage
            'battery_voltage': msg.voltage,
            'current': msg.current
        })

    def _gps_callback(self, msg):
        """Handle GPS position updates."""
        self.system_state['gps_position'] = {
            'lat': msg.latitude,
            'lon': msg.longitude,
            'alt': msg.altitude
        }

    def _safety_monitoring_callback(self):
        """Periodic safety property evaluation."""
        self.monitoring_stats['total_evaluations'] += 1
        violations = []

        # Evaluate all safety properties
        for property_name, safety_property in self.safety_properties.items():
            satisfied, details = safety_property.evaluate(self.system_state)

            if not satisfied:
                violations.append({
                    'property': property_name,
                    'severity': safety_property.severity.value,
                    'details': details,
                    'consecutive_violations': safety_property.consecutive_violations
                })

                self.monitoring_stats['total_violations'] += 1
                self.monitoring_stats['violation_counts'][property_name] = \
                    self.monitoring_stats['violation_counts'].get(property_name, 0) + 1

        # Handle violations
        for violation in violations:
            if violation['consecutive_violations'] >= self.violation_alert_threshold:
                self._handle_safety_violation(violation)

        self.monitoring_stats['last_evaluation'] = time.time()

    def _alert_check_callback(self):
        """Check for ongoing alerts and cleanup resolved ones."""
        current_time = time.time()

        # Check for communication timeouts
        time_since_comm = current_time - self.system_state.get('last_communication', current_time)
        if time_since_comm > self.communication_timeout:
            self._handle_safety_violation({
                'property': 'communication_timeout',
                'severity': SafetyLevel.WARNING.value,
                'details': f'No communication for {time_since_comm:.1f} seconds',
                'consecutive_violations': 1
            })

    def _handle_safety_violation(self, violation: Dict[str, Any]):
        """Handle a safety property violation."""
        severity = violation['severity']
        property_name = violation['property']
        details = violation['details']

        # Add to active alerts if not already present
        alert_exists = any(alert['property'] == property_name for alert in self.monitoring_stats['active_alerts'])

        if not alert_exists:
            self.monitoring_stats['active_alerts'].append({
                'property': property_name,
                'severity': severity,
                'details': details,
                'timestamp': time.time(),
                'acknowledged': False
            })

        # Log based on severity
        if severity == SafetyLevel.EMERGENCY.value:
            self.get_logger().fatal(f"EMERGENCY SAFETY VIOLATION: {property_name} - {details}")
        elif severity == SafetyLevel.CRITICAL.value:
            self.get_logger().error(f"CRITICAL SAFETY VIOLATION: {property_name} - {details}")
        elif severity == SafetyLevel.WARNING.value:
            self.get_logger().warn(f"WARNING SAFETY VIOLATION: {property_name} - {details}")
        else:
            self.get_logger().info(f"SAFETY VIOLATION: {property_name} - {details}")

        # Trigger emergency actions for critical violations
        if severity in [SafetyLevel.EMERGENCY.value, SafetyLevel.CRITICAL.value]:
            self._trigger_emergency_actions(property_name, details)

    def _trigger_emergency_actions(self, property_name: str, details: str):
        """Trigger emergency response actions for critical safety violations."""
        if property_name == 'battery_safety':
            # Emergency stop due to battery
            self._emergency_stop("Critical battery level violation")
        elif property_name == 'geofence_compliance':
            # Emergency stop due to boundary violation
            self._emergency_stop("Competition boundary violation")
        elif property_name == 'control_loop_timing':
            # Control system failure
            self._emergency_stop("Control loop timing violation - system unstable")

    def _emergency_stop(self, reason: str):
        """Execute emergency stop procedure."""
        self.get_logger().fatal(f"EMERGENCY STOP TRIGGERED: {reason}")

        # Publish emergency stop command
        # This would require coordination with the state machine and hardware interfaces

        # Update system state
        self.system_state['emergency_stop'] = True

    def _handle_get_safety_status(self, request, response):
        """Handle safety status service requests."""
        response.overall_safety = self._calculate_overall_safety_level()
        response.active_alerts = []

        for alert in self.monitoring_stats['active_alerts']:
            alert_msg = {
                'property': alert['property'],
                'severity': alert['severity'],
                'details': alert['details'],
                'timestamp': alert['timestamp'],
                'acknowledged': alert['acknowledged']
            }
            response.active_alerts.append(alert_msg)

        response.monitoring_stats.total_evaluations = self.monitoring_stats['total_evaluations']
        response.monitoring_stats.total_violations = self.monitoring_stats['total_violations']
        response.monitoring_stats.evaluation_rate = self.monitoring_stats['evaluation_rate']

        return response

    def _handle_verify_property(self, request, response):
        """Handle property verification service requests."""
        if request.property_name in self.safety_properties:
            property_obj = self.safety_properties[request.property_name]
            satisfied, details = property_obj.evaluate(self.system_state)

            response.property_name = request.property_name
            response.satisfied = satisfied
            response.details = details
            response.violation_count = property_obj.violation_count
            response.last_violation = property_obj.last_violation_time or 0.0
        else:
            response.property_name = request.property_name
            response.satisfied = False
            response.details = f"Unknown safety property: {request.property_name}"
            response.violation_count = 0
            response.last_violation = 0.0

        return response

    def _calculate_overall_safety_level(self) -> str:
        """Calculate overall system safety level."""
        if any(alert['severity'] == SafetyLevel.EMERGENCY.value for alert in self.monitoring_stats['active_alerts']):
            return SafetyLevel.EMERGENCY.value
        elif any(alert['severity'] == SafetyLevel.CRITICAL.value for alert in self.monitoring_stats['active_alerts']):
            return SafetyLevel.CRITICAL.value
        elif any(alert['severity'] == SafetyLevel.WARNING.value for alert in self.monitoring_stats['active_alerts']):
            return SafetyLevel.WARNING.value
        else:
            return SafetyLevel.NOMINAL.value


def main(args=None):
    rclpy.init(args=args)

    safety_monitor = SafetyMonitor()

    try:
        rclpy.spin(safety_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        safety_monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


