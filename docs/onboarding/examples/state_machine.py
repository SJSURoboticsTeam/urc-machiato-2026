"""
State Machine Example

Standalone ROS2 state machine node pattern for testing the frontend
without the full autonomy stack. Valid transitions and publishers
follow the same contract as the production state management.

See: docs/onboarding/PILLAR_2_COGNITION.md, src/core/state_management.py
"""

import json
import time
from typing import Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class StandaloneStateMachine(Node):
    """Standalone state machine node for testing."""

    def __init__(self) -> None:
        super().__init__("standalone_state_machine")
        self.current_state = "CALIBRATION"
        self.current_substate = "INITIALIZING"
        self.last_transition_time = time.time()

        self.state_pub = self.create_publisher(
            String, "/state_machine/current_state", 10
        )
        self.transition_pub = self.create_publisher(
            String, "/state_machine/state_transition", 10
        )
        self.substate_pub = self.create_publisher(
            String, "/state_machine/substate", 10
        )

        self.create_service(
            Trigger, "/state_machine/change_state", self._handle_change_state
        )
        self.create_service(
            Trigger, "/state_machine/get_current_state", self._handle_get_state
        )
        self.create_timer(1.0, self._publish_state)

    def _valid_transitions(self) -> Dict[str, list]:
        """Valid state transitions (same contract as production)."""
        return {
            "BOOT": ["CALIBRATION"],
            "CALIBRATION": ["IDLE"],
            "IDLE": ["TELEOPERATION", "AUTONOMOUS", "SAFETY"],
            "TELEOPERATION": ["IDLE", "AUTONOMOUS", "SAFETY"],
            "AUTONOMOUS": ["IDLE", "TELEOPERATION", "SAFETY"],
            "SAFETY": ["IDLE", "TELEOPERATION", "AUTONOMOUS"],
            "SHUTDOWN": ["BOOT"],
        }

    def _handle_change_state(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """Handle state change requests with valid transitions."""
        valid = self._valid_transitions()
        if self.current_state not in valid:
            response.success = False
            response.message = f"Unknown state: {self.current_state}"
            return response
        possible = valid[self.current_state]
        new_state = possible[0] if len(possible) == 1 else possible[0]
        if self.current_state == new_state:
            response.success = False
            response.message = f"Already in state: {self.current_state}"
            return response
        old_state = self.current_state
        self.current_state = new_state
        self.last_transition_time = time.time()
        payload = {
            "from_state": old_state,
            "to_state": new_state,
            "timestamp": self.last_transition_time,
            "success": True,
        }
        self.transition_pub.publish(String(data=json.dumps(payload)))
        response.success = True
        response.message = f"State changed from {old_state} to {new_state}"
        return response

    def _handle_get_state(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """Return current state."""
        response.success = True
        response.message = self.current_state
        return response

    def _publish_state(self) -> None:
        """Publish current state periodically."""
        payload = {
            "state": self.current_state,
            "substate": self.current_substate,
            "timestamp": time.time(),
            "last_transition": self.last_transition_time,
        }
        self.state_pub.publish(String(data=json.dumps(payload)))
        self.substate_pub.publish(
            String(data=json.dumps({"substate": self.current_substate, "timestamp": time.time()}))
        )


def main() -> None:
    rclpy.init()
    node = StandaloneStateMachine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
