#!/usr/bin/env python3
"""
Adaptive State Machine - ROS2 Service for BT Integration

Provides the adaptive state machine service that BT orchestrator queries
to determine system state before making autonomous decisions.

Author: URC 2026 State Machine Team
"""

import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from enum import Enum
import json
from typing import Dict, Any, Optional
import time

# Import autonomy interfaces
from autonomy_interfaces.srv import GetSystemState


class SystemState(Enum):
    """System states matching URC requirements."""
    BOOT = "boot"
    IDLE = "idle"
    AUTONOMOUS = "autonomous"
    TELEOPERATION = "teleoperation"
    EMERGENCY_STOP = "emergency_stop"
    ERROR = "error"


class AutonomousSubstate(Enum):
    """Autonomous operation substates."""
    NONE = "none"
    WAYPOINT_NAVIGATION = "waypoint_navigation"
    SAMPLE_COLLECTION = "sample_collection"
    DELIVERY = "delivery"
    FOLLOW_ME = "follow_me"
    RETURN_TO_OPERATOR = "return_to_operator"


class TeleoperationSubstate(Enum):
    """Teleoperation substates."""
    NONE = "none"
    MANUAL_CONTROL = "manual_control"
    SEMI_AUTONOMOUS = "semi_autonomous"


class AdaptiveStateMachine(LifecycleNode):
    """
    Adaptive State Machine for URC 2026 rover.

    Provides ROS2 service interface for BT orchestrator to query system state
    and manages state transitions based on system conditions and commands.
    """

    def __init__(self):
        super().__init__('adaptive_state_machine')

        # State management
        self.current_state = SystemState.BOOT
        self.current_substate = AutonomousSubstate.NONE
        self.current_teleop_substate = TeleoperationSubstate.NONE
        self.state_metadata = {}
        self.last_transition_time = time.time()

        # Service clients for system monitoring
        self.system_monitor_client = None
        self.navigation_status_client = None

        # Publishers for state changes
        self.state_publisher = self.create_publisher(
            autonomy_interfaces.msg.SystemState,
            '/adaptive_state_machine/state',
            10
        )

        self.get_logger().info('Adaptive State Machine initialized')

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure the state machine."""
        self.get_logger().info('Configuring Adaptive State Machine...')

        # Create service server
        self.state_service = self.create_service(
            GetSystemState,
            '/adaptive_state_machine/get_state',
            self.get_state_callback
        )

        # Create subscription for state change commands
        self.state_command_sub = self.create_subscription(
            std_msgs.msg.String,
            '/adaptive_state_machine/commands',
            self.state_command_callback,
            10
        )

        # Create clients for system monitoring
        self.system_monitor_client = self.create_client(
            autonomy_interfaces.srv.GetSystemHealth,
            '/system_monitor/get_health'
        )

        self.navigation_status_client = self.create_client(
            autonomy_interfaces.srv.GetNavigationStatus,
            '/navigation/get_status'
        )

        self.get_logger().info('Adaptive State Machine configured')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Activate the state machine."""
        self.get_logger().info('Activating Adaptive State Machine...')

        # Transition to IDLE state
        self.transition_to(SystemState.IDLE)

        self.get_logger().info('Adaptive State Machine activated')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Deactivate the state machine."""
        self.get_logger().info('Deactivating Adaptive State Machine...')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Clean up the state machine."""
        self.get_logger().info('Cleaning up Adaptive State Machine...')

        # Clean up service and subscriptions
        if hasattr(self, 'state_service'):
            self.destroy_service(self.state_service)

        if hasattr(self, 'state_command_sub'):
            self.destroy_subscription(self.state_command_sub)

        return TransitionCallbackReturn.SUCCESS

    def get_state_callback(self, request, response):
        """Handle get state service requests from BT orchestrator."""
        response.current_state = self.current_state.value
        response.current_substate = self.current_substate.value
        response.current_teleop_substate = self.current_teleop_substate.value
        response.state_metadata = json.dumps(self.state_metadata)
        response.last_transition_time = self.last_transition_time

        # Add system health information if available
        try:
            if self.system_monitor_client.service_is_ready():
                health_request = autonomy_interfaces.srv.GetSystemHealth.Request()
                future = self.system_monitor_client.call_async(health_request)
                # Note: In production, this should be handled asynchronously
                response.system_healthy = True  # Placeholder
            else:
                response.system_healthy = True  # Assume healthy if service unavailable
        except:
            response.system_healthy = True

        self.get_logger().debug(f'State query response: {response.current_state}')
        return response

    def state_command_callback(self, msg):
        """Handle state change commands."""
        try:
            command = json.loads(msg.data)

            if command.get('action') == 'transition':
                target_state = command.get('target_state')
                if target_state:
                    self.handle_state_transition(target_state, command.get('metadata', {}))

            elif command.get('action') == 'set_substate':
                substate_type = command.get('substate_type', 'autonomous')
                substate = command.get('substate')

                if substate_type == 'autonomous':
                    try:
                        self.current_substate = AutonomousSubstate(substate)
                    except ValueError:
                        self.get_logger().warn(f'Invalid autonomous substate: {substate}')
                elif substate_type == 'teleoperation':
                    try:
                        self.current_teleop_substate = TeleoperationSubstate(substate)
                    except ValueError:
                        self.get_logger().warn(f'Invalid teleop substate: {substate}')

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid state command JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing state command: {e}')

    def handle_state_transition(self, target_state_str: str, metadata: Dict[str, Any]):
        """Handle state transition requests."""
        try:
            target_state = SystemState(target_state_str)

            # Validate transition
            if self.can_transition_to(target_state):
                self.transition_to(target_state, metadata)
            else:
                self.get_logger().warn(f'Invalid transition from {self.current_state.value} to {target_state_str}')

        except ValueError:
            self.get_logger().error(f'Unknown target state: {target_state_str}')

    def can_transition_to(self, target_state: SystemState) -> bool:
        """Check if transition to target state is allowed."""
        # Define valid transitions
        valid_transitions = {
            SystemState.BOOT: [SystemState.IDLE],
            SystemState.IDLE: [SystemState.AUTONOMOUS, SystemState.TELEOPERATION],
            SystemState.AUTONOMOUS: [SystemState.IDLE, SystemState.EMERGENCY_STOP],
            SystemState.TELEOPERATION: [SystemState.IDLE, SystemState.EMERGENCY_STOP],
            SystemState.EMERGENCY_STOP: [SystemState.IDLE],
            SystemState.ERROR: [SystemState.BOOT, SystemState.IDLE]
        }

        # Emergency stop can be triggered from any state
        if target_state == SystemState.EMERGENCY_STOP:
            return True

        return target_state in valid_transitions.get(self.current_state, [])

    def transition_to(self, new_state: SystemState, metadata: Optional[Dict[str, Any]] = None):
        """Perform state transition."""
        old_state = self.current_state

        self.current_state = new_state
        self.last_transition_time = time.time()
        self.state_metadata.update(metadata or {})

        # Reset substates when entering certain states
        if new_state == SystemState.IDLE:
            self.current_substate = AutonomousSubstate.NONE
            self.current_teleop_substate = TeleoperationSubstate.NONE
        elif new_state == SystemState.EMERGENCY_STOP:
            self.current_substate = AutonomousSubstate.NONE
            self.current_teleop_substate = TeleoperationSubstate.NONE

        # Publish state change
        state_msg = autonomy_interfaces.msg.SystemState()
        state_msg.current_state = self.current_state.value
        state_msg.current_substate = self.current_substate.value
        state_msg.current_teleop_substate = self.current_teleop_substate.value
        state_msg.state_metadata = json.dumps(self.state_metadata)
        state_msg.last_transition_time = self.last_transition_time

        self.state_publisher.publish(state_msg)

        self.get_logger().info(f'State transition: {old_state.value} -> {new_state.value}')

    def update_substate(self, substate: AutonomousSubstate, metadata: Optional[Dict[str, Any]] = None):
        """Update autonomous substate."""
        if self.current_state == SystemState.AUTONOMOUS:
            self.current_substate = substate
            self.state_metadata.update(metadata or {})

            # Publish updated state
            state_msg = autonomy_interfaces.msg.SystemState()
            state_msg.current_state = self.current_state.value
            state_msg.current_substate = self.current_substate.value
            state_msg.current_teleop_substate = self.current_teleop_substate.value
            state_msg.state_metadata = json.dumps(self.state_metadata)
            state_msg.last_transition_time = self.last_transition_time

            self.state_publisher.publish(state_msg)

            self.get_logger().info(f'Substate updated: {substate.value}')

    def get_current_state_info(self) -> Dict[str, Any]:
        """Get comprehensive current state information."""
        return {
            'state': self.current_state.value,
            'substate': self.current_substate.value,
            'teleop_substate': self.current_teleop_substate.value,
            'metadata': self.state_metadata,
            'last_transition': self.last_transition_time,
            'time_in_state': time.time() - self.last_transition_time
        }


def main(args=None):
    """Main function."""
    rclpy.init(args=args)

    # Create lifecycle node
    node = AdaptiveStateMachine()

    # Use lifecycle manager
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    try:
        # Configure and activate the node
        ret = node.trigger_configure()
        if ret == TransitionCallbackReturn.SUCCESS:
            ret = node.trigger_activate()
            if ret == TransitionCallbackReturn.SUCCESS:
                executor.spin()
            else:
                node.get_logger().error('Failed to activate node')
        else:
            node.get_logger().error('Failed to configure node')
    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown
        node.trigger_deactivate()
        node.trigger_cleanup()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



