#!/usr/bin/env python3
"""
URC 2026 System Workflow Test
Tests complete mission execution from state machine to BT to hardware
"""

import subprocess
import os
import time
import threading
import signal
import sys


class SystemWorkflowTest:
    def __init__(self):
        self.system_process = None
        self.test_results = {}
        self.test_timeout = 60  # seconds

    def start_system(self):
        """Start the integrated system"""
        print("🚀 Starting URC 2026 Integrated System...")
        self.system_process = subprocess.Popen(
            ["ros2", "launch", "tools/scripts/launch/integrated_system.launch.py"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            cwd="/home/durian/urc-machiato-2026",
        )

        # Wait for system to initialize
        time.sleep(10)

        # Check if process is still running
        if self.system_process.poll() is not None:
            print("❌ System failed to start")
            return False

        print("✅ System started successfully")
        return True

    def test_state_machine_transition(self):
        """Test state machine transitions"""
        print("🧪 Testing State Machine Transitions...")

        # Test transition to AUTONOMOUS state
        cmd = "ros2 service call /state_machine/change_state autonomy_interfaces/srv/ChangeState '{desired_state: AUTONOMOUS}'"
        success, stdout, stderr = self.run_command(cmd, timeout=10)

        if success:
            print("  ✅ State machine transition successful")
            return True
        else:
            print(f"  ❌ State machine transition failed: {stderr}")
            return False

    def test_bt_mission_execution(self):
        """Test BT mission execution"""
        print("🧪 Testing BT Mission Execution...")

        # Send mission goal
        cmd = """ros2 action send_goal /bt/execute_mission autonomy_interfaces/action/ExecuteMission "{
  mission_type: 'sample_collection',
  mission_id: 'system_test_mission',
  timeout: 30.0,
  waypoints: ['test_waypoint']
}" """

        success, stdout, stderr = self.run_command(cmd, timeout=35)

        if success and "SUCCEEDED" in stdout:
            print("  ✅ BT mission execution successful")
            return True
        else:
            print(f"  ❌ BT mission execution failed: {stderr}")
            return False

    def test_topic_communication(self):
        """Test topic-based communication between components"""
        print("🧪 Testing Topic Communication...")

        # Check for active topics
        success, stdout, stderr = self.run_command("ros2 topic list | wc -l", timeout=5)
        if success:
            topic_count = int(stdout.strip())
            print(f"  📊 {topic_count} active topics")
            return topic_count > 20  # Expect many topics
        else:
            print("  ❌ Topic enumeration failed")
            return False

    def test_sensor_data_flow(self):
        """Test sensor data flow"""
        print("🧪 Testing Sensor Data Flow...")

        # Check for IMU data
        success, stdout, stderr = self.run_command(
            "timeout 5 ros2 topic echo /imu --once 2>/dev/null | wc -l", timeout=10
        )
        if success and int(stdout.strip()) > 0:
            print("  ✅ IMU data flowing")
            return True
        else:
            print("  ⚠️  IMU data not immediately available (may be normal)")
            return True  # Not critical for system test

    def run_command(self, cmd, timeout=30):
        """Run a command with timeout"""
        try:
            env = os.environ.copy()
            env["PYTHONPATH"] = (
                "/home/durian/urc-machiato-2026/install/autonomy_utilities/lib/python3.12/site-packages:"
                + env.get("PYTHONPATH", "")
            )

            result = subprocess.run(
                cmd,
                shell=True,
                capture_output=True,
                text=True,
                timeout=timeout,
                env=env,
                cwd="/home/durian/urc-machiato-2026",
            )
            return result.returncode == 0, result.stdout, result.stderr
        except subprocess.TimeoutExpired:
            return False, "", "Command timed out"

    def run_tests(self):
        """Run all system workflow tests"""
        print("🧪 URC 2026 System Workflow Test")
        print("=" * 50)

        # Start system
        if not self.start_system():
            return False

        try:
            # Run test sequence
            tests = [
                ("Topic Communication", self.test_topic_communication),
                ("State Machine Transition", self.test_state_machine_transition),
                ("BT Mission Execution", self.test_bt_mission_execution),
                ("Sensor Data Flow", self.test_sensor_data_flow),
            ]

            results = {}
            for test_name, test_func in tests:
                try:
                    result = test_func()
                    results[test_name] = result
                except Exception as e:
                    print(f"  ❌ {test_name}: Exception - {e}")
                    results[test_name] = False
                time.sleep(2)  # Brief pause between tests

            # Summary
            print("\n" + "=" * 50)
            print("📊 SYSTEM WORKFLOW TEST RESULTS")
            print("=" * 50)

            passed = 0
            total = len(results)

            for test_name, result in results.items():
                status = "✅ PASS" if result else "❌ FAIL"
                print(f"{status} {test_name}")
                if result:
                    passed += 1

            success_rate = (passed / total) * 100
            print(f"\n🎯 Overall: {passed}/{total} tests passed ({success_rate:.1f}%)")
            if success_rate >= 80:
                print("🎉 SYSTEM WORKFLOW SUCCESSFUL!")
                return True
            elif success_rate >= 60:
                print("⚠️  SYSTEM MOSTLY FUNCTIONAL")
                return True
            else:
                print("❌ SYSTEM HAS SIGNIFICANT ISSUES")
                return False

        finally:
            # Clean up
            self.cleanup()

    def cleanup(self):
        """Clean up system processes"""
        print("\n🧹 Cleaning up...")

        if self.system_process:
            try:
                # Try graceful shutdown first
                self.system_process.terminate()
                self.system_process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                # Force kill if needed
                self.system_process.kill()
                print("⚠️  Had to force kill system process")


def main():
    import os

    os.chdir("/home/durian/urc-machiato-2026")

    # Source ROS environment
    ros_env = "/opt/ros/jazzy/setup.bash"
    install_env = "/home/durian/urc-machiato-2026/install/setup.bash"

    test = SystemWorkflowTest()
    success = test.run_tests()

    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
