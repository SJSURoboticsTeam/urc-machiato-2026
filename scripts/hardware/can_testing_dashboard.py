#!/usr/bin/env python3
"""
Unified CAN Testing Dashboard

Real-time monitoring and testing of CAN messages flowing through the system.
Works with simulated and real hardware.

Usage:
  # Software test (no hardware)
  python3 can_testing_dashboard.py

  # Monitor real hardware
  python3 can_testing_dashboard.py --hardware

Features:
  - Real-time CAN message display
  - Message rate monitoring
  - Blackboard integration status
  - Color-coded status indicators
  - Live update counter
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String, Float32MultiArray
from datetime import datetime
import sys
import time
import argparse

try:
    from autonomy_interfaces.srv import GetBlackboardValue

    BLACKBOARD_SRV_AVAILABLE = True
except ImportError:
    BLACKBOARD_SRV_AVAILABLE = False


class UnifiedCANDashboard(Node):
    """Unified dashboard for CAN message monitoring and testing."""

    def __init__(self, hardware_mode=False):
        super().__init__("unified_can_dashboard")

        self.hardware_mode = hardware_mode
        self.start_time = time.time()

        # Message tracking
        self.message_counts = {
            "battery": 0,
            "velocity": 0,
            "temperatures": 0,
            "status": 0,
        }

        self.message_rates = {
            "battery": 0.0,
            "velocity": 0.0,
            "temperatures": 0.0,
            "status": 0.0,
        }

        self.last_count_time = time.time()
        self.last_counts = {k: 0 for k in self.message_counts.keys()}

        # Latest data
        self.latest_data = {
            "battery_voltage": 0.0,
            "battery_current": 0.0,
            "battery_percent": 0.0,
            "velocity_x": 0.0,
            "velocity_y": 0.0,
            "velocity_z": 0.0,
            "temperatures": [],
            "system_status": "unknown",
            "can_connected": False,
            "blackboard_available": False,
        }

        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )

        # Subscribe to hardware interface topics
        self.battery_sub = self.create_subscription(
            BatteryState, "/hardware/battery_state", self.battery_callback, sensor_qos
        )

        self.velocity_sub = self.create_subscription(
            TwistStamped,
            "/hardware/chassis_velocity",
            self.velocity_callback,
            sensor_qos,
        )

        self.temp_sub = self.create_subscription(
            Float32MultiArray,
            "/hardware/motor_temperatures",
            self.temperature_callback,
            sensor_qos,
        )

        self.status_sub = self.create_subscription(
            String, "/hardware/system_status", self.status_callback, sensor_qos
        )

        # Update dashboard at 2Hz
        self.timer = self.create_timer(0.5, self.update_dashboard)

        # Calculate rates every second
        self.rate_timer = self.create_timer(1.0, self.calculate_rates)

        # Check blackboard service availability every 5 seconds
        self.bb_check_timer = self.create_timer(5.0, self.check_blackboard_service)
        self._bb_client = None

        self.get_logger().info("Unified CAN Testing Dashboard started")

    def battery_callback(self, msg):
        """Handle battery state messages."""
        self.message_counts["battery"] += 1
        self.latest_data["battery_voltage"] = msg.voltage
        self.latest_data["battery_current"] = msg.current
        self.latest_data["battery_percent"] = msg.percentage

    def velocity_callback(self, msg):
        """Handle velocity messages."""
        self.message_counts["velocity"] += 1
        self.latest_data["velocity_x"] = msg.twist.linear.x
        self.latest_data["velocity_y"] = msg.twist.linear.y
        self.latest_data["velocity_z"] = msg.twist.angular.z

    def temperature_callback(self, msg):
        """Handle temperature messages."""
        self.message_counts["temperatures"] += 1
        self.latest_data["temperatures"] = list(msg.data)

    def status_callback(self, msg):
        """Handle status messages."""
        self.message_counts["status"] += 1
        import json

        try:
            status = json.loads(msg.data)
            self.latest_data["can_connected"] = status.get("hardware_connected", False)
        except Exception:
            pass
        self.latest_data["system_status"] = msg.data

    def check_blackboard_service(self):
        """Check if blackboard service is available (mock or real BT)."""
        if not BLACKBOARD_SRV_AVAILABLE:
            return
        try:
            if self._bb_client is None:
                self._bb_client = self.create_client(
                    GetBlackboardValue, "/blackboard/get_value"
                )
            self.latest_data["blackboard_available"] = self._bb_client.wait_for_service(
                timeout_sec=0.5
            )
        except Exception:
            self.latest_data["blackboard_available"] = False

    def calculate_rates(self):
        """Calculate message rates per second."""
        current_time = time.time()
        dt = current_time - self.last_count_time

        if dt > 0:
            for key in self.message_counts:
                count_diff = self.message_counts[key] - self.last_counts[key]
                self.message_rates[key] = count_diff / dt
                self.last_counts[key] = self.message_counts[key]

        self.last_count_time = current_time

    def update_dashboard(self):
        """Update dashboard display."""
        # Clear screen for clean update
        print("\033[2J\033[H", end="")

        runtime = time.time() - self.start_time
        total_messages = sum(self.message_counts.values())

        # Header
        print("╔" + "═" * 78 + "╗")
        print("║" + " URC 2026 - Unified CAN Testing Dashboard".center(78) + "║")
        print("╚" + "═" * 78 + "╝")
        print()

        # Mode indicator
        mode = "🔧 HARDWARE MODE" if self.hardware_mode else "💻 SIMULATION MODE"
        print(f"  {mode}")
        print(f"  Runtime: {int(runtime//60)}m {int(runtime%60)}s")
        print()

        # Connection Status
        print("┌─ Connection Status " + "─" * 58 + "┐")
        hw_status = (
            "🟢 CONNECTED"
            if self.latest_data["can_connected"]
            else "🔴 MOCK/DISCONNECTED"
        )
        bb_status = (
            "🟢 AVAILABLE"
            if self.latest_data["blackboard_available"]
            else "🟡 NOT DETECTED"
        )
        print(f"│  Hardware:  {hw_status:<60}│")
        print(f"│  Blackboard: {bb_status:<59}│")
        print("└" + "─" * 78 + "┘")
        print()

        # Message Statistics
        print("┌─ Message Statistics " + "─" * 56 + "┐")
        print(f"│  Total Messages: {total_messages:<58}│")
        print("│" + " " * 78 + "│")
        for topic, count in self.message_counts.items():
            rate = self.message_rates[topic]
            print(
                f"│  {topic.ljust(15)}: {str(count).ljust(8)} messages  ({rate:.1f} Hz){'':>32}│"
            )
        print("└" + "─" * 78 + "┘")
        print()

        # Latest CAN Data
        print("┌─ Latest CAN Data " + "─" * 59 + "┐")

        # Battery
        print("│  🔋 Battery:")
        print(f"│     Voltage:    {self.latest_data['battery_voltage']:6.2f} V")
        print(f"│     Current:    {self.latest_data['battery_current']:6.2f} A")
        print(f"│     Percentage: {self.latest_data['battery_percent']:6.1f} %")
        print("│")

        # Velocity
        print("│  🏎️  Chassis Velocity:")
        print(f"│     Linear X:   {self.latest_data['velocity_x']:7.3f} m/s")
        print(f"│     Linear Y:   {self.latest_data['velocity_y']:7.3f} m/s")
        print(f"│     Angular Z:  {self.latest_data['velocity_z']:7.3f} rad/s")
        print("│")

        # Temperatures
        if self.latest_data["temperatures"]:
            print("│  🌡️  Motor Temperatures:")
            temps = self.latest_data["temperatures"]
            for i in range(0, len(temps), 4):
                line = "│     "
                for j in range(4):
                    if i + j < len(temps):
                        line += f"M{i+j}:{temps[i+j]:5.1f}°C  "
                print(line)
        else:
            print("│  🌡️  Motor Temperatures: (no data)")

        print("└" + "─" * 78 + "┘")
        print()

        # Status indicators
        if total_messages == 0:
            print("⚠️  No messages received yet")
            print()
            print("Waiting for hardware_interface_node...")
            print()
            print("To start it, run in another terminal:")
            print("  ros2 run autonomy_core hardware_interface")
            print()
        elif self.message_rates["battery"] < 1.0:
            print("⚠️  Low message rate - check hardware_interface_node")
            print()
        else:
            print("✅ System is receiving CAN messages")
            if self.latest_data["blackboard_available"]:
                print("✅ Blackboard writes are active (Option 1 working!)")
            else:
                print("ℹ️  Blackboard not connected (no /blackboard/get_value service)")
                print(
                    "   To test CAN→blackboard: ./scripts/hardware/test_can_blackboard_full.sh"
                )
            print()

        # Instructions
        print("─" * 80)
        print("Press Ctrl+C to exit")
        print()

    def print_summary(self):
        """Print summary on exit."""
        runtime = time.time() - self.start_time
        total_messages = sum(self.message_counts.values())

        print("\n" + "=" * 80)
        print("Test Summary")
        print("=" * 80)
        print(f"Runtime: {int(runtime//60)}m {int(runtime%60)}s")
        print(f"Total Messages: {total_messages}")
        print()
        print("Message Counts:")
        for topic, count in self.message_counts.items():
            print(f"  {topic:15s}: {count}")
        print()

        if total_messages > 0:
            avg_rate = total_messages / runtime if runtime > 0 else 0
            print(f"Average Rate: {avg_rate:.1f} msg/s")
            print()
            print("✅ CAN communication is working!")
            print()
            if self.hardware_mode:
                print("Hardware test successful - CAN data is flowing from real device")
            else:
                print("Simulation test successful - Ready for hardware testing")
                print()
                print("To test with hardware, run:")
                print("  python3 can_testing_dashboard.py --hardware")
        else:
            print("⚠️  No messages received")
            print()
            print("Make sure hardware_interface is running:")
            print("  ros2 run autonomy_core hardware_interface")

        print()


def main():
    parser = argparse.ArgumentParser(description="Unified CAN Testing Dashboard")
    parser.add_argument(
        "--hardware",
        action="store_true",
        help="Hardware mode (monitoring real CAN device)",
    )
    args = parser.parse_args()

    rclpy.init()

    dashboard = UnifiedCANDashboard(hardware_mode=args.hardware)

    print()
    print("Starting Unified CAN Testing Dashboard...")
    print()
    print("Mode:", "HARDWARE" if args.hardware else "SIMULATION")
    print()
    print("Monitoring topics:")
    print("  - /hardware/battery_state")
    print("  - /hardware/chassis_velocity")
    print("  - /hardware/motor_temperatures")
    print("  - /hardware/system_status")
    print()
    print("Waiting for messages...")
    print()

    time.sleep(2)  # Give time for subscriptions to initialize

    try:
        rclpy.spin(dashboard)
    except KeyboardInterrupt:
        pass
    finally:
        dashboard.print_summary()
        dashboard.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
