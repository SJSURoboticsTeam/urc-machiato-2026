"""
Safety System Example

Standalone ROS2 safety node pattern: emergency stop, recovery, and
health publishing. Matches the safety pillar contract for testing
the frontend without the full autonomy stack.

See: docs/onboarding/PILLAR_3_MOTION_CONTROL.md, docs/onboarding/PILLAR_1_PERCEPTION.md
"""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class StandaloneSafetySystem(Node):
    """Standalone safety system node for testing."""

    def __init__(self) -> None:
        super().__init__("standalone_safety_system")
        self.emergency_stop_active = False
        self.system_healthy = True
        self.active_alerts: list = []
        self.last_emergency_time = 0.0

        self.system_health_pub = self.create_publisher(
            String, "/safety/system_health", 10
        )
        self.active_alerts_pub = self.create_publisher(
            String, "/safety/active_alerts", 10
        )
        self.emergency_status_pub = self.create_publisher(
            String, "/safety/emergency_status", 10
        )

        self.create_service(
            Trigger, "/safety/emergency_stop", self._handle_emergency_stop
        )
        self.create_service(
            Trigger, "/safety/recover_from_safety", self._handle_recover
        )
        self.create_timer(1.0, self._publish_system_health)
        self.create_timer(2.0, self._publish_active_alerts)

    def _handle_emergency_stop(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Handle emergency stop service calls."""
        self.emergency_stop_active = True
        self.last_emergency_time = time.time()
        self.system_healthy = False
        self.active_alerts.append({
            "severity": "CRITICAL",
            "message": "Emergency stop activated",
            "timestamp": time.time(),
            "source": "safety_system",
        })
        payload = {
            "emergency_active": True,
            "emergency_type": "SOFTWARE_ESTOP",
            "timestamp": time.time(),
            "operator_initiated": True,
        }
        self.emergency_status_pub.publish(String(data=json.dumps(payload)))
        response.success = True
        response.message = "Emergency stop activated successfully"
        return response

    def _handle_recover(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Handle safety recovery service calls."""
        if not self.emergency_stop_active:
            response.success = False
            response.message = "No active safety condition to recover from"
            return response
        self.emergency_stop_active = False
        self.system_healthy = True
        self.active_alerts = [
            a for a in self.active_alerts if a.get("severity") != "CRITICAL"
        ]
        response.success = True
        response.message = "Safety recovery successful"
        return response

    def _publish_system_health(self) -> None:
        """Publish system health status."""
        payload = {
            "system_health": "CRITICAL" if self.emergency_stop_active else "HEALTHY",
            "emergency_stop_active": self.emergency_stop_active,
            "overall_status": "EMERGENCY" if self.emergency_stop_active else "NOMINAL",
            "timestamp": time.time(),
            "active_alerts_count": len(self.active_alerts),
            "last_emergency_time": self.last_emergency_time,
        }
        self.system_health_pub.publish(String(data=json.dumps(payload)))

    def _publish_active_alerts(self) -> None:
        """Publish current active alerts."""
        payload = {
            "active_alerts": self.active_alerts,
            "alert_count": len(self.active_alerts),
            "timestamp": time.time(),
            "system_status": "EMERGENCY" if self.emergency_stop_active else "NOMINAL",
        }
        self.active_alerts_pub.publish(String(data=json.dumps(payload)))


def main() -> None:
    rclpy.init()
    node = StandaloneSafetySystem()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
