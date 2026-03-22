#!/usr/bin/env python3
"""
ROS2 state machine bridge: publishes JSON state for mission control and services.

Minimal implementation for integration tests and Docker `state-management-service`.
Topics match tests/integration/core/test_ros2_state_machine_bridge.py.
"""

from __future__ import annotations

import json
import time
from enum import Enum
from typing import Any, Dict, Optional

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    _RCLPY_AVAILABLE = True
except ImportError:
    rclpy = None  # type: ignore
    Node = object  # type: ignore
    String = None  # type: ignore
    Trigger = None  # type: ignore
    _RCLPY_AVAILABLE = False


class SystemState(str, Enum):
    """System states exposed on /state_machine/current_state (JSON)."""

    BOOT = "BOOT"
    IDLE = "IDLE"
    AUTONOMOUS = "AUTONOMOUS"
    TELEOPERATION = "TELEOPERATION"
    EMERGENCY_STOP = "EMERGENCY_STOP"
    ERROR = "ERROR"
    SHUTDOWN = "SHUTDOWN"


class ROS2StateMachineBridge(Node):
    """Publishes state JSON and handles transition requests."""

    def __init__(self) -> None:
        super().__init__("ros2_state_machine_bridge")
        self._state: SystemState = SystemState.IDLE
        self._transition_pub = self.create_publisher(String, "/state_machine/state_transition", 10)
        self._state_pub = self.create_publisher(String, "/state_machine/current_state", 10)
        self._mission_pub = self.create_publisher(String, "/state_machine/for_mission_control", 10)
        self.create_subscription(String, "/mission/state_request", self._on_state_request, 10)
        self.create_service(
            Trigger, "/state_machine/can_transition_to_autonomous", self._on_autonomous_check
        )
        self._timer = self.create_timer(0.5, self._publish_state)
        self._publish_state()
        self.get_logger().info("ros2_state_machine_bridge started")

    def _payload(self) -> Dict[str, Any]:
        return {
            "state": self._state.value,
            "timestamp": time.time(),
            "reason": "bridge",
        }

    def _publish_state(self) -> None:
        msg = String()
        msg.data = json.dumps(self._payload())
        self._state_pub.publish(msg)
        mc = String()
        mc.data = json.dumps({**self._payload(), "for_mission_control": True})
        self._mission_pub.publish(mc)

    def _on_state_request(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            raw = str(data.get("state", "")).upper()
            # Map string to enum if valid
            for s in SystemState:
                if s.value == raw or s.name == raw:
                    self._transition_to(s, str(data.get("reason", "request")))
                    return
            self.get_logger().warning(f"Ignoring invalid state request: {raw}")
        except Exception as e:
            self.get_logger().error(f"state_request parse error: {e}")

    def _transition_to(self, new_state: SystemState, reason: str) -> None:
        old = self._state
        self._state = new_state
        tmsg = String()
        tmsg.data = json.dumps(
            {"from": old.value, "to": new_state.value, "reason": reason, "time": time.time()}
        )
        self._transition_pub.publish(tmsg)
        self.get_logger().info(f"Transition {old.value} -> {new_state.value} ({reason})")

    def _on_autonomous_check(self, _request, response):
        response.success = self._state in (SystemState.IDLE, SystemState.TELEOPERATION)
        response.message = "ok" if response.success else "not_allowed"
        return response


def main(args: Optional[Any] = None) -> None:
    if not _RCLPY_AVAILABLE:
        raise RuntimeError("rclpy required for ros2_state_machine_bridge")
    rclpy.init(args=args)
    node = ROS2StateMachineBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
