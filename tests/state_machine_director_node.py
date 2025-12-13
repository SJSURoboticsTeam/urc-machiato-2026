#!/usr/bin/env python3
"""
State Machine Director ROS2 Node for Integration Testing

Provides the ROS2 services and topics needed for state machine integration tests.
This is a simplified mock implementation for testing purposes.
"""

import signal
import sys
import time

import rclpy
from rclpy.node import Node

# Import message types (use standard ROS2 types)
try:
    from autonomy_interfaces.msg import SystemState
    from autonomy_interfaces.srv import ChangeState
except ImportError:
    # Create mock message types if not available
    from std_msgs.msg import String

    # Mock SystemState message
    class SystemState:
        def __init__(self):
            self.current_state = ""

    from std_srvs.srv import SetBool as ChangeState

from std_srvs.srv import Trigger


def signal_handler(signum, frame):
    """Handle shutdown signals."""
    print("\n🛑 Shutting down state machine director...")
    sys.exit(0)


class StateMachineDirector(Node):
    """ROS2 node that provides state machine services for integration testing."""

    def __init__(self):
        super().__init__('state_machine_director')

        # Current state
        self.current_state = "BOOT"
        self.available_states = ["BOOT", "IDLE", "TELEOPERATION", "AUTONOMOUS", "SAFESTOP"]

        # Publishers
        from std_msgs.msg import String
        self.state_publisher = self.create_publisher(
            String, '/state_machine/current_state', 10
        )

        # Services
        self.change_state_service = self.create_service(
            ChangeState, '/state_machine/change_state', self.change_state_callback
        )

        # Simple service for mission execution (instead of action)
        from std_srvs.srv import Trigger
        self.mission_service = self.create_service(
            Trigger, '/state_machine/execute_mission', self.execute_mission_callback
        )

        # Timer for periodic state publishing
        self.create_timer(0.5, self.publish_current_state)

        self.get_logger().info('State Machine Director initialized')

    def publish_current_state(self):
        """Publish current state periodically."""
        from std_msgs.msg import String
        msg = String()
        msg.data = self.current_state
        self.state_publisher.publish(msg)

    def change_state_callback(self, request, response):
        """Handle state change requests."""
        try:
            # For mock implementation, accept any state change
            if hasattr(request, 'data'):
                # SetBool service format
                new_state = "AUTONOMOUS" if request.data else "IDLE"
            else:
                # ChangeState service format (if available)
                new_state = getattr(request, 'new_state', 'IDLE')

            if new_state in self.available_states:
                self.current_state = new_state
                response.success = True
                response.message = f"State changed to {new_state}"
                self.get_logger().info(f'State changed to {new_state}')
            else:
                response.success = False
                response.message = f"Invalid state: {new_state}"

        except Exception as e:
            response.success = False
            response.message = f"Error changing state: {str(e)}"

        return response

    def execute_mission_callback(self, request, response):
        """Handle mission execution requests."""
        self.get_logger().info('Received mission execution request')

        # Simulate mission execution
        import time
        time.sleep(0.5)  # Brief simulation

        response.success = True
        response.message = "Mission executed successfully"
        return response


def main():
    """Main entry point."""
    # Set up signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    rclpy.init()
    node = StateMachineDirector()

    try:
        print("🚀 State Machine Director running...")
        print("📡 Providing services and topics for integration testing")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print("✅ State Machine Director stopping...")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
