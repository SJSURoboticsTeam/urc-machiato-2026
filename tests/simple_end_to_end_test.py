#!/usr/bin/env python3
"""
Simple End-to-End Network Test

Tests the complete data flow from ROS2 publishers through the competition bridge
to WebSocket clients with performance measurement.

Author: URC 2026 Autonomy Team
"""

import asyncio
import json
import subprocess
import sys
import threading
import time
from typing import Dict, List

# Add src to path
sys.path.insert(0, "/home/ubuntu/urc-machiato-2026/src")

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Import autonomy interfaces
from autonomy_interfaces.msg import SystemState, SafetyStatus

# WebSocket client for testing
import websockets
import websockets.exceptions


class SimpleROSPublisher(Node):
    """Simple ROS2 node that publishes test data."""

    def __init__(self):
        super().__init__('simple_publisher')
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)

        self.system_state_pub = self.create_publisher(SystemState, '/autonomy/system_state', qos_profile)
        self.safety_status_pub = self.create_publisher(SafetyStatus, '/autonomy/safety_status', qos_profile)

        self.timer = self.create_timer(1.0, self.publish_data)  # 1 Hz
        self.message_count = 0

        self.get_logger().info("Simple ROS Publisher started")

    def publish_data(self):
        """Publish test data."""
        current_time = self.get_clock().now().to_msg()

        # System State
        system_state = SystemState()
        system_state.header.stamp = current_time
        system_state.header.frame_id = "base_link"
        system_state.state = "active"
        system_state.mission_phase = "testing"
        system_state.battery_level = 85.0
        system_state.temperature = 35.0
        system_state.sequence_number = self.message_count
        self.system_state_pub.publish(system_state)

        # Safety Status
        safety_status = SafetyStatus()
        safety_status.header.stamp = current_time
        safety_status.emergency_stop_active = False
        safety_status.systems_nominal = True
        safety_status.last_safety_check = current_time
        safety_status.safety_violations = []
        self.safety_status_pub.publish(safety_status)

        self.message_count += 1
        if self.message_count % 5 == 0:
            self.get_logger().info(f"Published {self.message_count} messages")


class WebSocketPerformanceClient:
    """WebSocket client that measures performance."""

    def __init__(self, client_id: str, websocket_url: str = "ws://localhost:8080"):
        self.client_id = client_id
        self.websocket_url = websocket_url
        self.messages_received = 0
        self.message_times = []
        self.data_types = set()
        self.connected = False
        self.start_time = 0

    async def run_test(self, duration: float = 15.0):
        """Run WebSocket performance test."""
        self.start_time = time.time()

        try:
            async with websockets.connect(self.websocket_url) as websocket:
                self.connected = True
                print(f"[PASS] WebSocket client {self.client_id} connected")

                while time.time() - self.start_time < duration:
                    try:
                        # Set a timeout for receiving messages
                        message = await asyncio.wait_for(websocket.recv(), timeout=2.0)
                        self._process_message(message)

                    except asyncio.TimeoutError:
                        # Send ping to keep connection alive
                        await websocket.ping()
                        continue

        except Exception as e:
            print(f"[FAIL] WebSocket client {self.client_id} error: {e}")
            self.connected = False

        print(f"[GRAPH] Client {self.client_id}: {self.messages_received} messages, {len(self.data_types)} data types")

    def _process_message(self, message: str):
        """Process received message."""
        try:
            data = json.loads(message)
            self.messages_received += 1
            self.message_times.append(time.time())

            if 'type' in data:
                self.data_types.add(data['type'])

            # Keep only recent times for performance calc
            if len(self.message_times) > 50:
                self.message_times = self.message_times[-50:]

        except json.JSONDecodeError:
            pass  # Ignore malformed messages

    def get_stats(self) -> Dict:
        """Get performance statistics."""
        if len(self.message_times) < 2:
            return {"messages_per_second": 0, "connected": self.connected}

        time_span = self.message_times[-1] - self.message_times[0]
        messages_per_second = len(self.message_times) / time_span if time_span > 0 else 0

        return {
            "messages_per_second": messages_per_second,
            "total_messages": self.messages_received,
            "data_types": list(self.data_types),
            "connected": self.connected
        }


def run_end_to_end_test():
    """Run the complete end-to-end test."""
    print("[IGNITE] END-TO-END NETWORK TEST")
    print("=" * 50)

    # Test configuration
    test_duration = 15  # seconds
    ros_domain_id = 200

    results = {
        "ros_publisher": {"status": "starting"},
        "competition_bridge": {"status": "starting"},
        "websocket_clients": [],
        "performance": {},
        "overall_status": "running"
    }

    try:
        # Phase 1: Start ROS2 Publisher
        print("\n[ANTENNA] Phase 1: Starting ROS2 Publisher...")
        import os
        os.environ['ROS_DOMAIN_ID'] = str(ros_domain_id)

        # Start ROS2 publisher in separate thread
        def start_ros_publisher():
            rclpy.init()
            publisher = SimpleROSPublisher()
            rclpy.spin(publisher)
            rclpy.shutdown()

        ros_thread = threading.Thread(target=start_ros_publisher, daemon=True)
        ros_thread.start()
        time.sleep(2)  # Let it start
        results["ros_publisher"]["status"] = "running"
        print("[PASS] ROS2 Publisher started")

        # Phase 2: Start Competition Bridge
        print("\n[CONNECT] Phase 2: Starting Competition Bridge...")

        env = os.environ.copy()
        env['PYTHONPATH'] = '/home/ubuntu/urc-machiato-2026/src'
        env['ROS_DOMAIN_ID'] = str(ros_domain_id)

        bridge_process = subprocess.Popen(
            [sys.executable, '-m', 'bridges.competition_bridge'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env=env,
            cwd='/home/ubuntu/urc-machiato-2026'
        )

        time.sleep(3)  # Let bridge start
        results["competition_bridge"]["status"] = "running"
        print("[PASS] Competition Bridge started")

        # Phase 3: Start WebSocket Clients
        print("\n[PLUG] Phase 3: Starting WebSocket Clients...")

        async def run_clients():
            clients = []
            for i in range(2):
                client = WebSocketPerformanceClient(f"client_{i+1}")
                clients.append(client)

                # Start client
                asyncio.create_task(client.run_test(test_duration))

            # Wait for test duration
            await asyncio.sleep(test_duration + 2)

            return clients

        # Run WebSocket clients
        clients = asyncio.run(run_clients())

        results["websocket_clients"] = [client.get_stats() for client in clients]

        # Phase 4: Analyze Results
        print("\n[GRAPH] Phase 4: Analyzing Results...")

        total_messages = sum(client["total_messages"] for client in results["websocket_clients"])
        avg_mps = sum(client["messages_per_second"] for client in results["websocket_clients"]) / len(clients)

        connected_clients = sum(1 for client in results["websocket_clients"] if client["connected"])

        results["performance"] = {
            "total_messages_received": total_messages,
            "average_messages_per_second": avg_mps,
            "connected_clients": connected_clients,
            "total_clients": len(clients),
            "test_duration": test_duration
        }

        # Print results
        print("\n[OBJECTIVE] PERFORMANCE RESULTS:")
        print(f"   Total Messages: {total_messages}")
        print(".1f")
        print(f"   Connected Clients: {connected_clients}/{len(clients)}")

        for i, client_stats in enumerate(results["websocket_clients"]):
            print(f"   Client {i+1}: {client_stats['total_messages']} msgs, {client_stats['messages_per_second']:.1f} mps")

        # Assess success
        success = (
            results["ros_publisher"]["status"] == "running" and
            results["competition_bridge"]["status"] == "running" and
            connected_clients > 0 and
            total_messages > 0
        )

        results["overall_status"] = "PASSED" if success else "FAILED"

        print("\n[ACHIEVEMENT] FINAL RESULT:")
        print(f"   Status: {'[PASS] PASSED' if success else '[FAIL] FAILED'}")
        print(f"   ROS2 Publisher: {'[PASS]' if results['ros_publisher']['status'] == 'running' else '[FAIL]'}")
        print(f"   Competition Bridge: {'[PASS]' if results['competition_bridge']['status'] == 'running' else '[FAIL]'}")
        print(f"   WebSocket Connections: {'[PASS]' if connected_clients > 0 else '[FAIL]'} {connected_clients} connected")
        print(f"   Data Flow: {'[PASS]' if total_messages > 0 else '[FAIL]'} {total_messages} messages")

        return results

    except Exception as e:
        print(f"\n[FAIL] Test failed with error: {e}")
        results["overall_status"] = "ERROR"
        results["error"] = str(e)
        return results

    finally:
        # Cleanup
        print("\n[SWEEP] Cleaning up...")
        try:
            bridge_process.terminate()
            bridge_process.wait(timeout=5)
        except:
            bridge_process.kill()


if __name__ == '__main__':
    import logging
    logging.basicConfig(level=logging.WARNING)  # Reduce noise

    results = run_end_to_end_test()

    # Exit with appropriate code
    exit(0 if results.get("overall_status") == "PASSED" else 1)
