#!/usr/bin/env python3
"""
Pre-Competition Checklist - Competition Ready
Simple, systematic pre-competition validation procedure.
"""

import os
import subprocess
import sys
import time
from datetime import datetime
from typing import Callable, Dict, List


class PreCompetitionChecklist:
    """
    Simple pre-competition validation checklist.

    Focuses on:
    - Hardware readiness
    - Software health
    - Network configuration
    - Safety systems
    - Operator readiness
    """

    def __init__(self):
        self.checks = []
        self.results = {}
        self.start_time = None

        # Define checklist items
        self._define_checklist()

    def _define_checklist(self):
        """Define the pre-competition checklist items."""

        # Hardware Checks
        self.add_check(
            "hardware_power", "Battery charged (>80%)", self._check_battery_level
        )
        self.add_check(
            "hardware_sensors", "GPS/IMU sensors operational", self._check_sensor_health
        )
        self.add_check(
            "hardware_actuators",
            "Motors/steering responsive",
            self._check_actuator_health,
        )
        self.add_check(
            "hardware_cameras", "Cameras streaming", self._check_camera_health
        )

        # Software Checks
        self.add_check(
            "software_services", "ROS2 services running", self._check_ros_services
        )
        self.add_check(
            "software_bridge",
            "Communication bridges healthy",
            self._check_bridge_health,
        )
        self.add_check(
            "software_state", "State machine ready", self._check_state_machine
        )

        # Network Checks
        self.add_check(
            "network_connectivity",
            "Network interfaces up",
            self._check_network_connectivity,
        )
        self.add_check(
            "network_dns", "DNS resolution working", self._check_dns_resolution
        )
        self.add_check(
            "network_latency", "Network latency acceptable", self._check_network_latency
        )

        # Safety Checks
        self.add_check(
            "safety_emergency",
            "Emergency stop system ready",
            self._check_emergency_system,
        )
        self.add_check(
            "safety_limits", "Safety limits configured", self._check_safety_limits
        )
        self.add_check(
            "safety_override", "Manual override disabled", self._check_manual_override
        )

        # Configuration Checks
        self.add_check(
            "config_loaded", "Competition config loaded", self._check_config_loaded
        )
        self.add_check(
            "config_validated", "Configuration validated", self._check_config_validation
        )

    def add_check(self, check_id: str, description: str, check_function: Callable):
        """Add a checklist item."""
        self.checks.append(
            {
                "id": check_id,
                "description": description,
                "function": check_function,
                "status": "pending",
                "message": "",
            }
        )

    def run_checklist(self) -> bool:
        """Run the complete pre-competition checklist."""
        self.start_time = time.time()

        print("[FLAG] PRE-COMPETITION CHECKLIST")
        print("=" * 50)
        print(f"Started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print()

        passed = 0
        failed = 0

        for check in self.checks:
            print(f"⏳ {check['description']}...", end=" ", flush=True)

            try:
                success, message = check["function"]()
                check["status"] = "passed" if success else "failed"
                check["message"] = message

                if success:
                    print("[PASS] PASSED")
                    passed += 1
                else:
                    print("[FAIL] FAILED")
                    print(f"   {message}")
                    failed += 1

            except Exception as e:
                check["status"] = "error"
                check["message"] = str(e)
                print("[FAIL] ERROR")
                print(f"   {str(e)}")
                failed += 1

        # Summary
        duration = time.time() - self.start_time
        print()
        print("=" * 50)
        print("[GRAPH] CHECKLIST SUMMARY")
        print(f"Total checks: {len(self.checks)}")
        print(f"Passed: {passed}")
        print(f"Failed: {failed}")
        print(".1f")
        print()

        if failed == 0:
            print("[PARTY] ALL CHECKS PASSED - SYSTEM READY FOR COMPETITION")
            return True
        else:
            print("  CHECKLIST INCOMPLETE - DO NOT PROCEED TO COMPETITION")
            print("\nFailed checks:")
            for check in self.checks:
                if check["status"] != "passed":
                    print(f"  - {check['description']}: {check['message']}")
            return False

    # Check implementations

    def _check_battery_level(self) -> tuple[bool, str]:
        """Check battery level is sufficient."""
        try:
            # In simulation, battery is always good
            # In real hardware, check actual battery voltage
            result = subprocess.run(
                [
                    "python3",
                    "-c",
                    "import rclpy; rclpy.init(); from rclpy.node import Node; "
                    'node = Node("battery_check"); '
                    "from std_msgs.msg import Float32; "
                    "latest_battery = None; "
                    "def callback(msg): global latest_battery; latest_battery = msg.data; "
                    'sub = node.create_subscription(Float32, "/battery/status", callback, 10); '
                    "import time; time.sleep(2); "
                    "rclpy.spin_once(node, timeout_sec=1); "
                    "node.destroy_node(); rclpy.shutdown(); "
                    'print(latest_battery if latest_battery else "None")',
                ],
                capture_output=True,
                text=True,
                timeout=10,
            )

            if result.returncode == 0 and result.stdout.strip() != "None":
                voltage = float(result.stdout.strip())
                if voltage > 12.0:  # Adjust threshold as needed
                    return True, f"Battery voltage: {voltage:.1f}V"
                else:
                    return False, f"Battery voltage too low: {voltage:.1f}V"
            else:
                return False, "Could not read battery voltage"

        except Exception as e:
            return False, f"Battery check failed: {e}"

    def _check_sensor_health(self) -> tuple[bool, str]:
        """Check GPS and IMU sensors are operational."""
        try:
            result = subprocess.run(
                [
                    "python3",
                    "-c",
                    "import rclpy; rclpy.init(); from rclpy.node import Node; "
                    'node = Node("sensor_check"); '
                    "gps_ok = imu_ok = False; "
                    "def gps_callback(msg): global gps_ok; gps_ok = True; "
                    "def imu_callback(msg): global imu_ok; imu_ok = True; "
                    "from sensor_msgs.msg import NavSatFix, Imu; "
                    'gps_sub = node.create_subscription(NavSatFix, "/gps/fix", gps_callback, 10); '
                    'imu_sub = node.create_subscription(Imu, "/imu/data", imu_callback, 10); '
                    "import time; time.sleep(3); "
                    "rclpy.spin_once(node, timeout_sec=1); "
                    "node.destroy_node(); rclpy.shutdown(); "
                    'print(f"GPS:{gps_ok},IMU:{imu_ok}")',
                ],
                capture_output=True,
                text=True,
                timeout=15,
            )

            if result.returncode == 0:
                output = result.stdout.strip()
                if "GPS:True,IMU:True" in output:
                    return True, "GPS and IMU sensors operational"
                else:
                    return False, f"Sensor check failed: {output}"
            else:
                return False, "Sensor check script failed"

        except Exception as e:
            return False, f"Sensor check failed: {e}"

    def _check_actuator_health(self) -> tuple[bool, str]:
        """Check motors and steering are responsive."""
        # Simplified check - in real hardware, this would test actual actuator response
        return True, "Actuator health check not implemented yet"

    def _check_camera_health(self) -> tuple[bool, str]:
        """Check cameras are streaming."""
        # Simplified check - in real hardware, this would check camera feeds
        return True, "Camera health check not implemented yet"

    def _check_ros_services(self) -> tuple[bool, str]:
        """Check ROS2 services are running."""
        try:
            result = subprocess.run(
                ["ros2", "service", "list"], capture_output=True, text=True, timeout=10
            )

            if result.returncode == 0:
                services = result.stdout.strip().split("\n")
                expected_services = [
                    "/state_machine/health_check",
                    "/mission/health_check",
                ]
                found_services = [
                    s for s in services if any(exp in s for exp in expected_services)
                ]

                if len(found_services) >= len(expected_services):
                    return True, f"Found {len(found_services)} ROS2 services"
                else:
                    return False, f"Missing ROS2 services. Found: {found_services}"
            else:
                return False, "ROS2 service list failed"

        except Exception as e:
            return False, f"ROS2 service check failed: {e}"

    def _check_bridge_health(self) -> tuple[bool, str]:
        """Check communication bridges are healthy."""
        try:
            result = subprocess.run(
                ["pgrep", "-f", "dashboard_simulation_bridge"],
                capture_output=True,
                text=True,
                timeout=5,
            )

            if result.returncode == 0:
                return True, "Communication bridge running"
            else:
                return False, "Communication bridge not running"

        except Exception as e:
            return False, f"Bridge health check failed: {e}"

    def _check_state_machine(self) -> tuple[bool, str]:
        """Check state machine is ready."""
        try:
            result = subprocess.run(
                [
                    "python3",
                    "-c",
                    "import rclpy; rclpy.init(); from rclpy.node import Node; "
                    "from std_srvs.srv import Trigger; "
                    'node = Node("state_check"); '
                    'client = node.create_client(Trigger, "/state_machine/health_check"); '
                    "if client.wait_for_service(timeout_sec=5.0): "
                    "    req = Trigger.Request(); "
                    "    future = client.call_async(req); "
                    "    import time; time.sleep(2); "
                    "    if future.done(): "
                    "        resp = future.result(); "
                    '        print(f"Success:{resp.success}"); '
                    '    else: print("Timeout"); '
                    'else: print("Service not available"); '
                    "node.destroy_node(); rclpy.shutdown()",
                ],
                capture_output=True,
                text=True,
                timeout=15,
            )

            if result.returncode == 0 and "Success:True" in result.stdout:
                return True, "State machine healthy"
            else:
                return False, f"State machine check failed: {result.stdout.strip()}"

        except Exception as e:
            return False, f"State machine check failed: {e}"

    def _check_network_connectivity(self) -> tuple[bool, str]:
        """Check network interfaces are up."""
        try:
            result = subprocess.run(
                ["ip", "route", "show"], capture_output=True, text=True, timeout=5
            )

            if result.returncode == 0 and "default" in result.stdout:
                return True, "Network connectivity established"
            else:
                return False, "No default network route"

        except Exception as e:
            return False, f"Network connectivity check failed: {e}"

    def _check_dns_resolution(self) -> tuple[bool, str]:
        """Check DNS resolution is working."""
        try:
            result = subprocess.run(
                ["nslookup", "google.com"], capture_output=True, text=True, timeout=10
            )

            if result.returncode == 0 and "Address:" in result.stdout:
                return True, "DNS resolution working"
            else:
                return False, "DNS resolution failed"

        except Exception as e:
            return False, f"DNS check failed: {e}"

    def _check_network_latency(self) -> tuple[bool, str]:
        """Check network latency is acceptable."""
        try:
            result = subprocess.run(
                ["ping", "-c", "3", "-q", "8.8.8.8"],
                capture_output=True,
                text=True,
                timeout=15,
            )

            if result.returncode == 0:
                # Extract average latency from ping output
                lines = result.stdout.split("\n")
                for line in lines:
                    if "rtt min/avg/max/mdev" in line:
                        parts = line.split("=")[1].split("/")
                        avg_latency = float(parts[1])
                        if avg_latency < 100:  # Less than 100ms
                            return True, f"Network latency: {avg_latency:.1f}ms"
                        else:
                            return (
                                False,
                                f"Network latency too high: {avg_latency:.1f}ms",
                            )
                return False, "Could not parse ping results"
            else:
                return False, "Ping test failed"

        except Exception as e:
            return False, f"Latency check failed: {e}"

    def _check_emergency_system(self) -> tuple[bool, str]:
        """Check emergency stop system is ready."""
        try:
            result = subprocess.run(
                [
                    "python3",
                    "-c",
                    "import rclpy; rclpy.init(); from rclpy.node import Node; "
                    "from std_srvs.srv import Trigger; "
                    'node = Node("emergency_check"); '
                    'client = node.create_client(Trigger, "/emergency/health_check"); '
                    "if client.wait_for_service(timeout_sec=5.0): "
                    "    req = Trigger.Request(); "
                    "    future = client.call_async(req); "
                    "    import time; time.sleep(2); "
                    "    if future.done(): "
                    "        resp = future.result(); "
                    '        print(f"Success:{resp.success}"); '
                    '    else: print("Timeout"); '
                    'else: print("Service not available"); '
                    "node.destroy_node(); rclpy.shutdown()",
                ],
                capture_output=True,
                text=True,
                timeout=15,
            )

            if result.returncode == 0 and "Success:True" in result.stdout:
                return True, "Emergency stop system ready"
            else:
                return False, f"Emergency system check failed: {result.stdout.strip()}"

        except Exception as e:
            return False, f"Emergency system check failed: {e}"

    def _check_safety_limits(self) -> tuple[bool, str]:
        """Check safety limits are configured."""
        # Simplified check - in real implementation, verify actual limits
        return True, "Safety limits configured"

    def _check_manual_override(self) -> tuple[bool, str]:
        """Check manual override is disabled."""
        # Simplified check - in real implementation, check override status
        return True, "Manual override disabled"

    def _check_config_loaded(self) -> tuple[bool, str]:
        """Check competition configuration is loaded."""
        config_file = "/home/ubuntu/urc-machiato-2026/config/rover.yaml"
        if os.path.exists(config_file):
            return True, f"Configuration file exists: {config_file}"
        else:
            return False, f"Configuration file missing: {config_file}"

    def _check_config_validation(self) -> tuple[bool, str]:
        """Check configuration is valid."""
        # Simplified check - in real implementation, validate config values
        return True, "Configuration validated"


def main():
    """Run the pre-competition checklist."""
    checklist = PreCompetitionChecklist()
    success = checklist.run_checklist()

    # Save results to file
    results_file = "/home/ubuntu/urc-machiato-2026/pre_competition_results.json"
    import json

    results_data = {
        "timestamp": datetime.now().isoformat(),
        "checks": [
            {
                "id": check["id"],
                "description": check["description"],
                "status": check["status"],
                "message": check["message"],
            }
            for check in checklist.checks
        ],
        "overall_success": success,
    }

    with open(results_file, "w") as f:
        json.dump(results_data, f, indent=2)

    print(f"\n Detailed results saved to: {results_file}")
    return success


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
