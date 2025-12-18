#!/usr/bin/env python3
"""
Communication Redundancy Manager
Provides fallback between WebSocket and direct ROS2 communication
"""

import asyncio
import json
import threading
import time
from typing import Any, Callable, Dict, Optional

import rclpy
import websockets
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


class CommunicationRedundancyManager(Node):
    """
    Manages communication redundancy between WebSocket and ROS2 direct connections.

    Primary Channel: WebSocket Bridge (simulation_bridge)
    Fallback Channel: Direct ROS2 communication

    Failover Logic:
    - Monitor WebSocket health via heartbeat
    - Switch to ROS2 direct when WebSocket fails
    - Automatically switch back when WebSocket recovers
    """

    def __init__(self):
        super().__init__("communication_redundancy_manager")

        # Configuration
        self.websocket_url = "ws://localhost:8766"
        self.heartbeat_interval = 5.0  # seconds
        self.failover_timeout = 10.0  # seconds after heartbeat loss

        # State management
        self.current_channel = "websocket"  # websocket | ros2_direct
        self.websocket_healthy = True
        self.last_heartbeat = time.time()
        self.failover_active = False

        # ROS2 Publishers (for direct ROS2 communication)
        self.state_pub = self.create_publisher(
            String, "/state_machine/current_state", 10
        )
        self.mission_pub = self.create_publisher(String, "/mission/status", 10)
        self.telemetry_pub = self.create_publisher(String, "/system/telemetry", 10)

        # ROS2 Subscribers (to receive data from core systems)
        self.callback_group = ReentrantCallbackGroup()
        self.state_sub = self.create_subscription(
            String,
            "/state_machine/for_mission_control",
            self._state_callback,
            10,
            callback_group=self.callback_group,
        )
        self.mission_sub = self.create_subscription(
            String,
            "/mission/telemetry",
            self._mission_callback,
            10,
            callback_group=self.callback_group,
        )

        # Health monitoring
        self.health_pub = self.create_publisher(
            String, "/system/communication_health", 10
        )
        self.create_timer(1.0, self._health_monitor_callback)

        # WebSocket connection management
        self.websocket_connection = None
        self.websocket_task = None

        self.get_logger().info("Communication Redundancy Manager initialized")
        self.get_logger().info(f"Primary channel: {self.current_channel}")

    async def websocket_monitor(self):
        """Monitor WebSocket connection and handle failover."""
        while rclpy.ok():
            try:
                async with websockets.connect(self.websocket_url) as websocket:
                    self.websocket_connection = websocket
                    self.websocket_healthy = True
                    self.last_heartbeat = time.time()

                    # Send initial heartbeat
                    await websocket.send(
                        json.dumps({"type": "heartbeat", "timestamp": time.time()})
                    )

                    async for message in websocket:
                        try:
                            data = json.loads(message)
                            if data.get("type") == "heartbeat_response":
                                self.last_heartbeat = time.time()
                                self.websocket_healthy = True

                                # If we were in failover, switch back
                                if self.failover_active:
                                    await self._switch_to_websocket()
                            else:
                                # Forward WebSocket data to ROS2
                                await self._forward_websocket_to_ros2(data)

                        except json.JSONDecodeError:
                            continue

            except (
                websockets.exceptions.ConnectionClosed,
                ConnectionRefusedError,
                OSError,
            ) as e:
                self.websocket_healthy = False
                self.websocket_connection = None

                # Check if we need to failover
                if (
                    not self.failover_active
                    and (time.time() - self.last_heartbeat) > self.failover_timeout
                ):
                    await self._initiate_failover()

                self.get_logger().warning(f"WebSocket connection lost: {e}")
                await asyncio.sleep(2.0)  # Retry delay

    async def _switch_to_websocket(self):
        """Switch back to WebSocket primary channel."""
        self.failover_active = False
        self.current_channel = "websocket"
        self.get_logger().info("[SUCCESS] Switched back to WebSocket primary channel")

    async def _initiate_failover(self):
        """Initiate failover to direct ROS2 communication."""
        self.failover_active = True
        self.current_channel = "ros2_direct"
        self.get_logger().warning(
            "[UPDATE] FAILOVER: Switching to ROS2 direct communication"
        )
        self.get_logger().warning("   WebSocket bridge appears to be down")

    async def _forward_websocket_to_ros2(self, data: Dict[str, Any]):
        """Forward WebSocket data to ROS2 topics."""
        if data.get("type") == "state_update":
            msg = String()
            msg.data = json.dumps(data)
            self.state_pub.publish(msg)

        elif data.get("type") == "mission_update":
            msg = String()
            msg.data = json.dumps(data)
            self.mission_pub.publish(msg)

    def _state_callback(self, msg):
        """Handle state updates from ROS2."""
        if self.failover_active:
            # Forward to WebSocket if available, otherwise handle directly
            asyncio.create_task(self._forward_ros2_to_websocket(msg.data, "state"))

    def _mission_callback(self, msg):
        """Handle mission updates from ROS2."""
        if self.failover_active:
            # Forward to WebSocket if available, otherwise handle directly
            asyncio.create_task(self._forward_ros2_to_websocket(msg.data, "mission"))

    async def _forward_ros2_to_websocket(self, data: str, data_type: str):
        """Forward ROS2 data to WebSocket during failover."""
        if self.websocket_connection:
            try:
                message = {
                    "type": f"{data_type}_update",
                    "data": data,
                    "timestamp": time.time(),
                    "source": "ros2_failover",
                }
                await self.websocket_connection.send(json.dumps(message))
            except Exception as e:
                self.get_logger().error(
                    f"Failed to forward ROS2 data to WebSocket: {e}"
                )

    def _health_monitor_callback(self):
        """Monitor communication health and publish status."""
        current_time = time.time()

        # Check WebSocket health
        websocket_age = current_time - self.last_heartbeat
        if websocket_age > self.heartbeat_interval * 2:
            self.websocket_healthy = False

        # Publish health status
        health_data = {
            "timestamp": current_time,
            "websocket_healthy": self.websocket_healthy,
            "websocket_age_seconds": websocket_age,
            "current_channel": self.current_channel,
            "failover_active": self.failover_active,
            "last_heartbeat": self.last_heartbeat,
        }

        health_msg = String()
        health_msg.data = json.dumps(health_data)
        self.health_pub.publish(health_msg)

        # Log status periodically or on changes
        if int(current_time) % 30 == 0:  # Every 30 seconds
            status = (
                "HEALTHY"
                if self.websocket_healthy and not self.failover_active
                else "DEGRADED"
            )
            self.get_logger().info(
                f"Communication Status: {status} (Channel: {self.current_channel})"
            )

    def send_command_via_redundant_channel(self, command: Dict[str, Any]) -> bool:
        """
        Send command using current active channel.
        Returns True if sent successfully.
        """
        try:
            if self.current_channel == "websocket" and self.websocket_connection:
                # Send via WebSocket
                asyncio.create_task(self._send_websocket_command(command))
                return True
            else:
                # Send via ROS2 direct
                return self._send_ros2_command(command)
        except Exception as e:
            self.get_logger().error(f"Failed to send command: {e}")
            return False

    async def _send_websocket_command(self, command: Dict[str, Any]):
        """Send command via WebSocket."""
        if self.websocket_connection:
            try:
                await self.websocket_connection.send(json.dumps(command))
            except Exception as e:
                self.get_logger().error(f"WebSocket command send failed: {e}")

    def _send_ros2_command(self, command: Dict[str, Any]) -> bool:
        """Send command via ROS2 direct."""
        try:
            msg = String()
            msg.data = json.dumps(command)

            if command.get("target") == "state_machine":
                self.state_pub.publish(msg)
            elif command.get("target") == "mission":
                self.mission_pub.publish(msg)
            else:
                self.telemetry_pub.publish(msg)

            return True
        except Exception as e:
            self.get_logger().error(f"ROS2 command send failed: {e}")
            return False

    def get_status(self) -> Dict[str, Any]:
        """Get current communication status."""
        return {
            "current_channel": self.current_channel,
            "websocket_healthy": self.websocket_healthy,
            "failover_active": self.failover_active,
            "last_heartbeat_age": time.time() - self.last_heartbeat,
            "failover_timeout": self.failover_timeout,
        }


async def main():
    """Main entry point for redundancy manager."""
    rclpy.init()

    manager = CommunicationRedundancyManager()
    executor = MultiThreadedExecutor()
    executor.add_node(manager)

    # Start WebSocket monitoring in background
    websocket_task = asyncio.create_task(manager.websocket_monitor())

    try:
        # Run ROS2 executor
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
            await asyncio.sleep(0.1)

    except KeyboardInterrupt:
        pass
    finally:
        websocket_task.cancel()
        try:
            await websocket_task
        except asyncio.CancelledError:
            pass

        manager.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    asyncio.run(main())
