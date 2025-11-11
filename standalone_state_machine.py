#!/usr/bin/env python3
"""
Standalone State Machine for URC 2026 Testing

Provides ROS-compatible state machine services and topics for testing
the frontend without requiring the full ROS autonomy stack.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from autonomy_interfaces.srv import ChangeState
import json
import time

class StandaloneStateMachine(Node):
    """Standalone state machine node for testing."""

    def __init__(self):
        super().__init__('standalone_state_machine')

        # Current state
        self.current_state = 'CALIBRATION'
        self.current_substate = 'INITIALIZING'
        self.last_transition_time = time.time()

        # Publishers
        self.state_pub = self.create_publisher(String, '/state_machine/current_state', 10)
        self.transition_pub = self.create_publisher(String, '/state_machine/state_transition', 10)
        self.substate_pub = self.create_publisher(String, '/state_machine/substate', 10)

        # Services - using comprehensive ChangeState service
        self.change_state_srv = self.create_service(
            ChangeState, '/state_machine/change_state',
            self.handle_change_state_request
        )

        self.get_current_state_srv = self.create_service(
            Trigger, '/state_machine/get_current_state',
            self.handle_get_current_state
        )

        # Timer to publish state periodically
        self.timer = self.create_timer(1.0, self.publish_state)

        self.get_logger().info('Standalone State Machine started')
        self.get_logger().info(f'Initial state: {self.current_state}')

    def handle_change_state_request(self, request, response):
        """Handle comprehensive state change requests using ChangeState service."""
        try:
            desired_state = request.desired_state.upper()  # Normalize to uppercase
            reason = request.reason or 'frontend_request'
            operator_id = request.operator_id or 'system'
            force = request.force or False

            self.get_logger().info(f'State change request: {self.current_state} -> {desired_state} by {operator_id} ({reason})')

            # Define valid state transitions for a realistic autonomous robot
            # Emergency states can be entered from anywhere
            # Normal states follow operational flow
            valid_transitions = {
                'BOOT': ['CALIBRATION'],  # Boot sequence
                'CALIBRATION': ['IDLE'],  # After calibration, go to ready state
                'IDLE': ['TELEOPERATION', 'AUTONOMOUS', 'SAFETY', 'SHUTDOWN'],  # Ready to operate
                'TELEOPERATION': ['IDLE', 'SAFETY', 'SHUTDOWN'],  # Can stop or emergency stop
                'AUTONOMOUS': ['IDLE', 'SAFETY', 'SHUTDOWN'],  # Can stop or emergency stop
                'SAFETY': ['IDLE', 'SHUTDOWN'],  # Emergency recovery - go to safe state
                'SHUTDOWN': ['BOOT']  # Restart sequence
            }

            # Emergency transitions - SAFETY can be entered from any state
            emergency_transitions = {
                'SAFETY': ['BOOT', 'CALIBRATION', 'IDLE', 'TELEOPERATION', 'AUTONOMOUS', 'SHUTDOWN']
            }

            # Precondition checks (simplified for demo)
            preconditions_met = True
            failed_preconditions = []

            # Check if system is in a valid state to transition
            if self.current_state not in valid_transitions and not force:
                failed_preconditions.append(f"Current state '{self.current_state}' is not recognized")
                preconditions_met = False

            # Check if desired transition is valid
            is_valid_transition = False
            if self.current_state in valid_transitions and desired_state in valid_transitions[self.current_state]:
                is_valid_transition = True
            elif desired_state in emergency_transitions.get(self.current_state, []):
                is_valid_transition = True

            if not is_valid_transition and not force:
                failed_preconditions.append(f"Invalid transition: {self.current_state} -> {desired_state}")
                preconditions_met = False

            response.preconditions_met = preconditions_met
            response.failed_preconditions = failed_preconditions
            response.warnings = []

            if preconditions_met or force:
                # Execute the transition
                old_state = self.current_state
                self.current_state = desired_state
                transition_start = time.time()
                self.last_transition_time = transition_start

                # Publish transition event
                transition_data = {
                    'from_state': old_state,
                    'to_state': desired_state,
                    'timestamp': self.last_transition_time,
                    'success': True,
                    'reason': reason,
                    'operator_id': operator_id,
                    'forced': force,
                    'transition_type': 'change_state_service'
                }
                self.transition_pub.publish(String(data=json.dumps(transition_data)))

                response.success = True
                response.message = f'Successfully transitioned from {old_state} to {desired_state}'
                response.actual_state = desired_state
                response.actual_substate = ''
                response.actual_calibration_substate = ''
                response.transition_time = time.time() - transition_start

                if force:
                    response.warnings.append('Transition was forced, bypassing normal validation')

                self.get_logger().info(f'State transition completed: {old_state} -> {desired_state} ({response.transition_time:.2f}s)')

            else:
                # Failed preconditions
                response.success = False
                response.message = f'Transition blocked: {failed_preconditions.join(", ")}'
                response.actual_state = self.current_state
                response.actual_substate = ''
                response.actual_calibration_substate = ''
                response.transition_time = 0.0

                self.get_logger().warn(f'State transition blocked: {response.message}')

        except Exception as e:
            response.success = False
            response.message = f'State transition failed: {str(e)}'
            response.actual_state = self.current_state
            response.actual_substate = ''
            response.actual_calibration_substate = ''
            response.transition_time = 0.0
            response.preconditions_met = False
            response.failed_preconditions = [str(e)]
            response.warnings = []
            self.get_logger().error(f'State transition error: {str(e)}')

        return response

    def handle_change_state(self, request, response):
        """Handle state change requests with realistic transitions."""
        try:
            # Define valid state transitions (allow staying in critical states)
            valid_transitions = {
                'BOOT': ['CALIBRATION'],
                'CALIBRATION': ['IDLE'],
                'IDLE': ['TELEOPERATION', 'AUTONOMOUS', 'SAFETY'],  # Allow direct access to critical states
                'TELEOPERATION': ['IDLE', 'AUTONOMOUS', 'SAFETY'],
                'AUTONOMOUS': ['IDLE', 'TELEOPERATION', 'SAFETY'],
                'SAFETY': ['IDLE', 'TELEOPERATION', 'AUTONOMOUS'],  # Allow staying in SAFETY or transitioning out
                'SHUTDOWN': ['BOOT']
            }

            # If we're in a valid state, cycle through available options
            if self.current_state in valid_transitions:
                possible_states = valid_transitions[self.current_state]

                # For states with multiple options, cycle through them predictably
                if len(possible_states) > 1:
                    # Use a simple round-robin approach for demo purposes
                    # Track which option we used last for each state
                    if not hasattr(self, 'transition_counters'):
                        self.transition_counters = {}

                    state_key = self.current_state
                    if state_key not in self.transition_counters:
                        self.transition_counters[state_key] = 0

                    # Cycle through options
                    current_idx = self.transition_counters[state_key]
                    new_state = possible_states[current_idx % len(possible_states)]
                    self.transition_counters[state_key] = (current_idx + 1) % len(possible_states)
                else:
                    # Only one option, go there
                    new_state = possible_states[0]
            else:
                # Fallback to original cycling behavior for unknown states
                state_sequence = ['BOOT', 'CALIBRATION', 'IDLE', 'TELEOPERATION', 'AUTONOMOUS', 'SAFETY', 'SHUTDOWN']
                current_index = state_sequence.index(self.current_state) if self.current_state in state_sequence else 0
                next_index = (current_index + 1) % len(state_sequence)
                new_state = state_sequence[next_index]

            # Validate the transition is reasonable
            if self.current_state == new_state:
                response.success = False
                response.message = f'Already in state: {self.current_state}'
                return response

            # Update state
            old_state = self.current_state
            self.current_state = new_state
            self.last_transition_time = time.time()

            # Publish transition
            transition_data = {
                'from_state': old_state,
                'to_state': new_state,
                'timestamp': self.last_transition_time,
                'success': True,
                'reason': f'Transition from {old_state} to {new_state}'
            }
            self.transition_pub.publish(String(data=json.dumps(transition_data)))

            response.success = True
            response.message = f'State changed from {old_state} to {new_state}'

            self.get_logger().info(f'State transition: {old_state} -> {new_state}')

        except Exception as e:
            response.success = False
            response.message = f'State change failed: {str(e)}'
            self.get_logger().error(f'State change error: {e}')

        return response

    def handle_get_current_state(self, request, response):
        """Handle get current state requests."""
        response.success = True
        response.message = f'Current state: {self.current_state}'
        return response

    def publish_state(self):
        """Publish current state periodically."""
        state_data = {
            'state': self.current_state,
            'substate': self.current_substate,
            'timestamp': time.time(),
            'last_transition': self.last_transition_time
        }

        self.state_pub.publish(String(data=json.dumps(state_data)))
        self.substate_pub.publish(String(data=json.dumps({
            'substate': self.current_substate,
            'timestamp': time.time()
        })))

def main():
    rclpy.init()
    node = StandaloneStateMachine()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()