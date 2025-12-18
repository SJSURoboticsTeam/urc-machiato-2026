#!/usr/bin/env python3
"""
ROS2 Universal Monitor - Monitor ALL ROS2 Topics and Services

This script monitors all active ROS2 topics and services, displaying their data
in real-time. It connects to the competition bridge's WebSocket stream and also
monitors ROS2 topics directly.

Usage:
    python3 ros2_monitor_all.py [--websocket-only] [--dashboard]

Features:
- Real-time monitoring of all ROS2 topics
- WebSocket telemetry from competition bridge
- Service discovery and status
- Dashboard data visualization
- Emergency stop monitoring
- State machine status tracking
"""

import asyncio
import json
import os
import signal
import subprocess
import sys
import time
from typing import Any, Dict, List

import websockets

# Add project path for imports
sys.path.append(os.path.dirname(os.path.abspath(__file__)))


class ROS2UniversalMonitor:
    """Monitor all ROS2 topics, services, and WebSocket streams."""

    def __init__(self) -> None:
        self.running = True
        self.topics_data = {}
        self.services_data = {}
        self.websocket_data = {}
        self.last_update = time.time()

        # ROS2 environment
        self.ros2_env = os.environ.copy()
        self.ros2_env["ROS_DOMAIN_ID"] = "42"

        # WebSocket connection
        self.websocket_connected = False
        self.websocket_url = "ws://localhost:8080"  # Competition bridge default

        # Signal handler for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, signum: int, frame) -> None:
        """Handle shutdown signals."""
        print("\n Received shutdown signal...")
        self.running = False

    def setup_ros2_environment(self) -> None:
        """Set up ROS2 environment."""
        try:
            # Set up environment variables directly
            self.ros2_env["AMENT_PREFIX_PATH"] = "/opt/ros/humble"
            self.ros2_env["CMAKE_PREFIX_PATH"] = "/opt/ros/humble"
            self.ros2_env["COLCON_PREFIX_PATH"] = "/opt/ros/humble"
            self.ros2_env["LD_LIBRARY_PATH"] = "/opt/ros/humble/lib"
            self.ros2_env[
                "PATH"
            ] = f"/opt/ros/humble/bin:{self.ros2_env.get('PATH', '')}"
            self.ros2_env["PYTHONPATH"] = (
                f"/opt/ros/humble/lib/python3.10/site-packages:"
                f"{self.ros2_env.get('PYTHONPATH', '')}"
            )
            self.ros2_env["ROS_PYTHON_VERSION"] = "3"
            self.ros2_env["ROS_VERSION"] = "2"
            self.ros2_env["ROS_DISTRO"] = "humble"

            # Add workspace to Python path and environment
            workspace_path = os.path.dirname(os.path.abspath(__file__))
            install_path = os.path.join(workspace_path, "install")
            if os.path.exists(install_path):
                self.ros2_env[
                    "AMENT_PREFIX_PATH"
                ] = f"{install_path}:{self.ros2_env.get('AMENT_PREFIX_PATH', '')}"
                self.ros2_env[
                    "CMAKE_PREFIX_PATH"
                ] = f"{install_path}:{self.ros2_env.get('CMAKE_PREFIX_PATH', '')}"
                self.ros2_env[
                    "COLCON_PREFIX_PATH"
                ] = f"{install_path}:{self.ros2_env.get('COLCON_PREFIX_PATH', '')}"
                self.ros2_env[
                    "LD_LIBRARY_PATH"
                ] = f"{install_path}/lib:{self.ros2_env.get('LD_LIBRARY_PATH', '')}"
                self.ros2_env["PYTHONPATH"] = (
                    f"{install_path}/lib/python3.10/site-packages:"
                    f"{self.ros2_env.get('PYTHONPATH', '')}"
                )

            # Test ROS2 availability
            result = subprocess.run(
                ["ros2", "topic", "list"],
                env=self.ros2_env,
                capture_output=True,
                text=True,
                timeout=10,
            )

            if result.returncode == 0:
                print("[PASS] ROS2 environment ready")
                return True
            else:
                print(f"[FAIL] ROS2 setup failed: {result.stderr}")
                return False

        except Exception as e:
            print(f"[FAIL] ROS2 environment setup error: {e}")
            return False

    def get_topic_list(self) -> List[str]:
        """Get list of all active ROS2 topics."""
        try:
            result = subprocess.run(
                ["ros2", "topic", "list"],
                env=self.ros2_env,
                capture_output=True,
                text=True,
                timeout=5,
            )

            if result.returncode == 0:
                topics = [
                    line.strip() for line in result.stdout.split("\n") if line.strip()
                ]
                return topics
            else:
                print(f"[FAIL] Failed to get topic list: {result.stderr}")
                return []

        except Exception as e:
            print(f"[FAIL] Error getting topic list: {e}")
            return []

    def get_service_list(self) -> List[str]:
        """Get list of all active ROS2 services."""
        try:
            result = subprocess.run(
                ["ros2", "service", "list"],
                env=self.ros2_env,
                capture_output=True,
                text=True,
                timeout=5,
            )

            if result.returncode == 0:
                services = [
                    line.strip() for line in result.stdout.split("\n") if line.strip()
                ]
                return services
            else:
                print(f"[FAIL] Failed to get service list: {result.stderr}")
                return []

        except Exception as e:
            print(f"[FAIL] Error getting service list: {e}")
            return []

    def monitor_topic_data(self, topic: str, max_messages: int = 3) -> Dict[str, Any]:
        """Monitor data from a specific ROS2 topic."""
        try:
            result = subprocess.run(
                ["ros2", "topic", "echo", "--once", topic],
                env=self.ros2_env,
                capture_output=True,
                text=True,
                timeout=2,
            )

            if result.returncode == 0 and result.stdout.strip():
                return {
                    "topic": topic,
                    "data": result.stdout.strip(),
                    "timestamp": time.time(),
                    "success": True,
                }
            else:
                return {
                    "topic": topic,
                    "data": "No data available",
                    "timestamp": time.time(),
                    "success": False,
                }

        except subprocess.TimeoutExpired:
            return {
                "topic": topic,
                "data": "Timeout - topic may be inactive",
                "timestamp": time.time(),
                "success": False,
            }
        except Exception as e:
            return {
                "topic": topic,
                "data": f"Error: {str(e)}",
                "timestamp": time.time(),
                "success": False,
            }

    def monitor_service_info(self, service: str) -> Dict[str, Any]:
        """Get information about a ROS2 service."""
        try:
            # Try to get service type
            result = subprocess.run(
                ["ros2", "service", "type", service],
                env=self.ros2_env,
                capture_output=True,
                text=True,
                timeout=2,
            )

            if result.returncode == 0:
                service_type = result.stdout.strip()
                return {
                    "service": service,
                    "type": service_type,
                    "available": True,
                    "timestamp": time.time(),
                }
            else:
                return {
                    "service": service,
                    "type": "Unknown",
                    "available": False,
                    "timestamp": time.time(),
                }

        except Exception as e:
            return {
                "service": service,
                "type": "Error",
                "available": False,
                "error": str(e),
                "timestamp": time.time(),
            }

    async def websocket_monitor(self) -> None:
        """Monitor WebSocket telemetry from competition bridge."""
        while self.running:
            try:
                print(f"[PLUG] Connecting to WebSocket at {self.websocket_url}...")
                async with websockets.connect(self.websocket_url) as websocket:
                    self.websocket_connected = True
                    print("[PASS] WebSocket connected - receiving telemetry data")

                    # Send request for telemetry data
                    request = {"type": "request_telemetry"}
                    await websocket.send(json.dumps(request))

                    async for message in websocket:
                        try:
                            data = json.loads(message)
                            self.websocket_data = {
                                "data": data,
                                "timestamp": time.time(),
                                "connected": True,
                            }

                            # Print WebSocket data periodically
                            if time.time() - self.last_update > 2.0:  # Every 2 seconds
                                self.display_websocket_data()

                        except json.JSONDecodeError:
                            pass  # Ignore non-JSON messages

            except Exception as e:
                self.websocket_connected = False
                self.websocket_data = {
                    "error": str(e),
                    "timestamp": time.time(),
                    "connected": False,
                }
                print(f"[FAIL] WebSocket connection failed: {e}")
                await asyncio.sleep(5)  # Retry after 5 seconds

    def display_websocket_data(self) -> None:
        """Display WebSocket telemetry data."""
        if not self.websocket_data.get("connected", False):
            return

        data = self.websocket_data.get("data", {})
        timestamp = self.websocket_data.get("timestamp", 0)

        print(
            f"\n[NETWORK] WebSocket Telemetry Data "
            f"({time.strftime('%H:%M:%S', time.localtime(timestamp))})"
        )
        print("=" * 60)

        # System health
        if "system_health" in data:
            print(f" System Health: {data['system_health']}")

        # Mission status
        if "current_mission" in data and data["current_mission"] != "none":
            print(
                f"[OBJECTIVE] Mission: {data['current_mission']} | "
                f"Status: {data.get('mission_status', 'unknown')}"
            )

        # GPS data
        if "gps_position" in data:
            gps = data["gps_position"]
            print(
                f" GPS: {gps.get('lat', 'N/A'):.6f}, {gps.get('lon', 'N/A'):.6f} | "
                f"Alt: {gps.get('alt', 'N/A'):.1f}m"
            )

        # IMU data
        if "imu_data" in data:
            imu = data["imu_data"]
            accel = imu.get("accel", [0, 0, 0])
            print(
                f"[REFRESH] IMU: Accel [{accel[0]:.2f}, {accel[1]:.2f}, "
                f"{accel[2]:.2f}] m/s²"
            )

        # Battery
        if "battery_level" in data:
            battery = data["battery_level"]
            current = data.get("current", 0)
            print(f" Battery: {battery:.1f}% | Current: {current:.2f}A")

        # Emergency stop
        if data.get("emergency_stop", False):
            print(" EMERGENCY STOP ACTIVE!")

        # Autonomous mode
        if data.get("autonomous_mode", False):
            print(" AUTONOMOUS MODE ACTIVE")

    def display_topic_data(self) -> None:
        """Display data from all monitored ROS2 topics."""
        topics = self.get_topic_list()

        print(f"\n[ANTENNA] ROS2 Topics Data ({len(topics)} active topics)")
        print("=" * 60)

        # Priority topics to show first
        priority_topics = [
            "/state_machine/current_state",
            "/emergency_stop",
            "/hardware/battery_state",
            "/hardware/gps",
            "/hardware/imu",
            "/mission/status",
            "/odom",
        ]

        # Show priority topics first
        shown_topics = []
        for topic in priority_topics:
            if topic in topics:
                data = self.monitor_topic_data(topic)
                self.display_single_topic(data)
                shown_topics.append(topic)
                time.sleep(0.1)  # Small delay to avoid overwhelming

        # Show remaining topics
        for topic in topics:
            if (
                topic not in shown_topics
                and not topic.startswith("/parameter_events")
                and topic != "/rosout"
            ):
                data = self.monitor_topic_data(topic)
                if data["success"] or "state_machine" in topic or "mission" in topic:
                    self.display_single_topic(data)
                    time.sleep(0.1)

    def display_single_topic(self, topic_data: Dict[str, Any]):
        """Display data from a single topic."""
        topic = topic_data["topic"]
        data = topic_data["data"]
        success = topic_data["success"]

        # Format topic name
        topic_short = (
            topic.replace("/state_machine/", "SM/")
            .replace("/hardware/", "HW/")
            .replace("/mission/", "MS/")
        )

        if success:
            print(
                f"[PASS] {topic_short}: {data[:100]}{'...' if len(data) > 100 else ''}"
            )
        else:
            print(f"[FAIL] {topic_short}: {data}")

    def display_service_data(self) -> None:
        """Display information about all ROS2 services."""
        services = self.get_service_list()

        print(f"\n[TOOL] ROS2 Services ({len(services)} active services)")
        print("=" * 60)

        for service in services:
            if (
                not service.endswith("/describe_parameters")
                and not service.endswith("/get_parameter_types")
                and not service.endswith("/get_parameters")
                and not service.endswith("/list_parameters")
                and not service.endswith("/set_parameters")
                and not service.endswith("/set_parameters_atomically")
            ):
                info = self.monitor_service_info(service)
                if info["available"]:
                    print(f"[PASS] {service} ({info['type']})")
                else:
                    print(f"[FAIL] {service} (Unavailable)")

    def display_system_overview(self) -> None:
        """Display overall system status overview."""
        print("\n" + "=" * 80)
        print("[IGNITE] ROS2 UNIVERSAL MONITOR - SYSTEM OVERVIEW")
        print("=" * 80)

        # ROS2 nodes
        try:
            result = subprocess.run(
                ["ros2", "node", "list"],
                env=self.ros2_env,
                capture_output=True,
                text=True,
                timeout=5,
            )
            nodes = [line.strip() for line in result.stdout.split("\n") if line.strip()]
            print(f" ROS2 Nodes: {len(nodes)} active")
            for node in nodes[:5]:  # Show first 5
                print(f"   • {node}")
            if len(nodes) > 5:
                print(f"   ... and {len(nodes) - 5} more")
        except Exception as e:
            print(f" ROS2 Nodes: Unable to query - {e}")

        # Topics count
        topics = self.get_topic_list()
        print(f"[ANTENNA] ROS2 Topics: {len(topics)} active")

        # Services count
        services = self.get_service_list()
        print(f"[TOOL] ROS2 Services: {len(services)} active")

        # WebSocket status
        ws_status = (
            "[PASS] Connected"
            if self.websocket_data.get("connected", False)
            else "[FAIL] Disconnected"
        )
        print(f"[NETWORK] WebSocket Bridge: {ws_status}")

        print("=" * 80)

    async def run_monitor(self) -> None:
        """Main monitoring loop."""
        print("[IGNITE] Starting ROS2 Universal Monitor...")
        print("This will monitor ALL ROS2 topics, services, and WebSocket telemetry")
        print("Press Ctrl+C to stop\n")

        if not self.setup_ros2_environment():
            print("[FAIL] Failed to set up ROS2 environment. Exiting.")
            return

        # Start WebSocket monitoring in background
        websocket_task = asyncio.create_task(self.websocket_monitor())

        try:
            while self.running:
                # Display system overview
                self.display_system_overview()

                # Display WebSocket data (if available)
                if self.websocket_data.get("connected", False):
                    self.display_websocket_data()

                # Display ROS2 services
                self.display_service_data()

                # Display ROS2 topic data
                self.display_topic_data()

                print(
                    f"\n⏰ Next update in 5 seconds... "
                    f"(Last update: {time.strftime('%H:%M:%S')})"
                )
                print("-" * 80)

                # Wait before next update
                await asyncio.sleep(5)

        except KeyboardInterrupt:
            print("\n Stopping monitor...")
        finally:
            self.running = False
            websocket_task.cancel()
            try:
                await websocket_task
            except asyncio.CancelledError:
                pass


def main() -> None:
    """Main entry point."""
    import argparse

    parser = argparse.ArgumentParser(description="ROS2 Universal Monitor")
    parser.add_argument(
        "--websocket-only", action="store_true", help="Monitor only WebSocket data"
    )
    parser.add_argument("--dashboard", action="store_true", help="Launch web dashboard")
    parser.add_argument(
        "--websocket-url",
        default="ws://localhost:8080",
        help="WebSocket URL for telemetry (default: ws://localhost:8080)",
    )

    args = parser.parse_args()

    monitor = ROS2UniversalMonitor()
    monitor.websocket_url = args.websocket_url

    if args.websocket_only:
        # WebSocket-only monitoring
        print("[NETWORK] Starting WebSocket-only monitoring...")
        asyncio.run(monitor.websocket_monitor())
    else:
        # Full monitoring
        asyncio.run(monitor.run_monitor())


if __name__ == "__main__":
    main()
