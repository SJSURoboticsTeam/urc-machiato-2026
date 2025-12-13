#!/usr/bin/env python3
"""
Integration Test Runner - URC 2026
Runs integration tests after unit tests pass.

Integration tests focus on:
- Component interactions and interfaces
- Data flow between modules
- API contracts and message passing
- Cross-subsystem coordination

Author: URC 2026 Autonomy Team
"""

import atexit
import json
import os
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional


class ROS2EnvironmentManager:
    """Manages ROS2 environment for integration testing."""

    def __init__(self, project_root: Path):
        self.project_root = project_root
        self.ros_daemon_process: Optional[subprocess.Popen] = None
        self.launch_process: Optional[subprocess.Popen] = None
        self.environment_ready = False

    def setup_environment(self) -> bool:
        """Set up complete ROS2 environment for testing."""
        try:
            print("🔧 Setting up ROS2 test environment...")

            # Check if ROS2 can be sourced and basic command works
            ros2_check_script = "source /opt/ros/humble/setup.bash && ros2 --help | head -1"
            try:
                result = subprocess.run(
                    ['bash', '-c', ros2_check_script],
                    capture_output=True,
                    text=True,
                    timeout=10
                )
                if result.returncode != 0:
                    print("❌ ROS2 setup failed - check ROS2 installation")
                    print(f"Error: {result.stderr}")
                    return False
                print("✅ ROS2 sourced successfully")
            except (subprocess.TimeoutExpired, FileNotFoundError):
                print("❌ Cannot source ROS2 - install ROS2 Humble")
                return False

            # Set ROS2 environment variables for all subsequent calls
            self.ros2_env = os.environ.copy()
            self.ros2_env['ROS_DOMAIN_ID'] = '42'
            self.ros2_env['PYTHONPATH'] = f"{self.project_root}:{self.project_root}/Autonomy:{self.project_root}/Autonomy/code"

            # Stop any existing ROS2 daemon first
            print("🛑 Ensuring clean ROS2 state...")
            try:
                subprocess.run(['ros2', 'daemon', 'stop'], capture_output=True, timeout=5)
                time.sleep(2)
            except:
                pass  # Ignore errors if daemon wasn't running

            # Start ROS2 daemon
            print("📡 Starting ROS2 daemon...")
            daemon_cmd = "source /opt/ros/humble/setup.bash && ros2 daemon start"
            self.ros_daemon_process = subprocess.Popen(
                ['bash', '-c', daemon_cmd],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                env=self.ros2_env
            )

            # Wait for daemon to start
            time.sleep(5)

            # Verify daemon is running
            status_cmd = "source /opt/ros/humble/setup.bash && ros2 daemon status"
            result = subprocess.run(
                ['bash', '-c', status_cmd],
                capture_output=True,
                text=True,
                env=self.ros2_env,
                timeout=10
            )

            if 'running' not in result.stdout.lower():
                print("❌ ROS2 daemon failed to start")
                print(f"Daemon status output: {result.stdout}")
                return False

            print("✅ ROS2 daemon running")

            # Launch mock topics publisher
            print("🚀 Starting mock topics publisher...")
            mock_publisher_cmd = f"cd {self.project_root} && PYTHONPATH={self.project_root}:{self.project_root}/Autonomy:{self.project_root}/Autonomy/code:$PYTHONPATH python3 tests/mock_topics_publisher.py"
            self.mock_publisher_process = subprocess.Popen(
                ['bash', '-c', mock_publisher_cmd],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                env=self.ros2_env
            )

            # Launch state machine director
            print("🚀 Starting ROS2 integration nodes...")
            print("🚀 Starting state machine director...")
            state_machine_cmd = f"cd {self.project_root} && source /opt/ros/humble/setup.bash && ROS_DOMAIN_ID=42 PYTHONPATH={self.project_root}:{self.project_root}/Autonomy:{self.project_root}/Autonomy/code:$PYTHONPATH python3 tests/state_machine_director_node.py"
            print(f"   Command: {state_machine_cmd}")
            self.state_machine_process = subprocess.Popen(
                ['bash', '-c', state_machine_cmd],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                env=self.ros2_env
            )
            print(f"   Process started with PID: {self.state_machine_process.pid}")

            # Launch SLAM nodes
            print("🚀 Starting SLAM processing nodes...")
            slam_cmd = f"cd {self.project_root} && source /opt/ros/humble/setup.bash && ROS_DOMAIN_ID=42 PYTHONPATH={self.project_root}:{self.project_root}/Autonomy:{self.project_root}/Autonomy/code:$PYTHONPATH python3 tests/slam_nodes.py"
            print(f"   Command: {slam_cmd}")
            self.slam_process = subprocess.Popen(
                ['bash', '-c', slam_cmd],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                env=self.ros2_env
            )
            print(f"   Process started with PID: {self.slam_process.pid}")

            # Launch navigation service
            print("🚀 Starting navigation service...")
            navigation_cmd = f"cd {self.project_root} && source /opt/ros/humble/setup.bash && ROS_DOMAIN_ID=42 PYTHONPATH={self.project_root}:{self.project_root}/Autonomy:{self.project_root}/Autonomy/code:$PYTHONPATH python3 tests/navigation_service_node.py"
            print(f"   Command: {navigation_cmd}")
            self.navigation_process = subprocess.Popen(
                ['bash', '-c', navigation_cmd],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                env=self.ros2_env
            )
            print(f"   Process started with PID: {self.navigation_process.pid}")

            # Wait for all nodes to start and publish initial topics
            time.sleep(12)

            # Launch integrated system (rosbridge only)
            print("🔗 Launching ROS2 infrastructure...")
            launch_file = self.project_root / "launch" / "integrated_system.launch.py"

            if launch_file.exists():
                launch_cmd = f"source /opt/ros/humble/setup.bash && ros2 launch {launch_file}"
                self.launch_process = subprocess.Popen(
                    ['bash', '-c', launch_cmd],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    env=self.ros2_env
                )
                time.sleep(5)  # Brief wait for launch
            else:
                print(f"⚠️  Launch file not found: {launch_file} - continuing with mock publisher only")

            # Wait for system to initialize
            print("⏳ Waiting for ROS2 system to stabilize (20 seconds)...")
            time.sleep(20)

            # Verify critical topics exist
            topics_cmd = "source /opt/ros/humble/setup.bash && ros2 topic list"
            result = subprocess.run(
                ['bash', '-c', topics_cmd],
                capture_output=True,
                text=True,
                env=self.ros2_env,
                timeout=10
            )

            critical_topics = [
                '/teleoperation/joint_states',
                '/teleoperation/system_status',
                '/cmd_vel',
                '/gps/fix',
                '/imu/data'
            ]

            found_topics = result.stdout.strip().split('\n')
            missing_topics = [topic for topic in critical_topics if topic not in found_topics]

            if missing_topics:
                print(f"⚠️  Some expected topics missing: {missing_topics}")
                print("🔍 Available topics:")
                for topic in found_topics[:10]:  # Show first 10
                    if topic.strip():
                        print(f"   {topic}")
                if len(found_topics) > 10:
                    print(f"   ... and {len(found_topics) - 10} more")

                # Continue anyway - some tests might still work
                print("⚠️  Continuing with partial ROS2 environment...")
            else:
                print("✅ ROS2 environment fully ready!")

            self.environment_ready = True
            return True

        except Exception as e:
            print(f"❌ Failed to set up ROS2 environment: {e}")
            self.cleanup_environment()
            return False

    def cleanup_environment(self):
        """Clean up ROS2 environment."""
        print("\n🧹 Cleaning up ROS2 test environment...")

        # Stop launch process
        if self.launch_process:
            try:
                print("🛑 Stopping ROS2 launch process...")
                self.launch_process.terminate()
                self.launch_process.wait(timeout=10)
                print("✅ Launch process stopped")
            except subprocess.TimeoutExpired:
                print("⚠️  Force killing launch process...")
                self.launch_process.kill()
            except Exception as e:
                print(f"⚠️  Error stopping launch process: {e}")

        # Stop all ROS2 node processes
        processes_to_stop = [
            ("Mock publisher", getattr(self, 'mock_publisher_process', None)),
            ("State machine director", getattr(self, 'state_machine_process', None)),
            ("SLAM nodes", getattr(self, 'slam_process', None)),
            ("Navigation service", getattr(self, 'navigation_process', None)),
        ]

        for name, process in processes_to_stop:
            if process:
                try:
                    print(f"🛑 Stopping {name}...")
                    process.terminate()
                    process.wait(timeout=5)
                    print(f"✅ {name} stopped")
                except subprocess.TimeoutExpired:
                    print(f"⚠️  Force killing {name}...")
                    process.kill()
                except Exception as e:
                    print(f"⚠️  Error stopping {name}: {e}")

        # Stop ROS2 daemon
        try:
            print("🛑 Stopping ROS2 daemon...")
            daemon_stop_cmd = "source /opt/ros/humble/setup.bash && ros2 daemon stop"
            result = subprocess.run(
                ['bash', '-c', daemon_stop_cmd],
                capture_output=True,
                env=self.ros2_env if hasattr(self, 'ros2_env') else None,
                timeout=10
            )
            if result.returncode == 0:
                print("✅ ROS2 daemon stopped")
            else:
                print("⚠️  ROS2 daemon may still be running")
        except Exception as e:
            print(f"⚠️  Error stopping ROS2 daemon: {e}")

        self.environment_ready = False

    def is_ready(self) -> bool:
        """Check if ROS2 environment is ready."""
        return self.environment_ready


class IntegrationTestRunner:
    """Runner for integration tests with comprehensive reporting."""

    def __init__(self):
        self.project_root = Path(__file__).parent.parent
        self.test_results = []
        self.start_time = None
        self.end_time = None
        self.ros2_manager = ROS2EnvironmentManager(self.project_root)

        # Register cleanup on exit
        atexit.register(self._cleanup_on_exit)

    def _cleanup_on_exit(self):
        """Cleanup ROS2 environment on exit."""
        self.ros2_manager.cleanup_environment()

    def run_integration_tests(self) -> Dict[str, Any]:
        """Run all integration tests and return results."""
        self.start_time = time.time()

        print("🔗 URC 2026 - INTEGRATION TEST SUITE")
        print("=" * 50)
        print("Testing component interactions and data flow...")
        print("Validates how modules work together")
        print("=" * 50)
        print()

        # ROS2 environment is set up automatically by pytest fixtures in conftest.py
        # Set up ROS2 environment manually
        if not self.ros2_manager.setup_environment():
            return {
                "error": "Failed to set up ROS2 environment",
                "ros2_setup_failed": True
            }

        print("✅ ROS2 environment ready for integration testing")
        print()

        # Define integration test modules by category
        integration_categories = {
            "Core Integration": [
                "tests/integration/test_basic_ros2_integration.py",
                "tests/integration/test_full_system_integration.py",
                "tests/integration/test_streamlined_autonomy.py",
            ],
            "Messaging & Communication": [
                "tests/integration/test_message_contracts.py",
                "tests/integration/test_messaging_consistency.py",
                "tests/integration/test_dataflow_consistency.py",
                "tests/integration/test_ros_topic_comprehensive.py",
            ],
            "Navigation & Control": [
                "tests/integration/test_navigation_comprehensive.py",
                "tests/integration/test_safety_navigation_integration.py",
                "tests/integration/test_autonomy_teleoperation_integration.py",
                "tests/integration/test_full_autonomy_teleoperation.py",
            ],
            "Vision & Perception": [
                "tests/integration/test_vision_control_integration.py",
                "tests/integration/test_vision_aruco_degradation.py",
                "tests/integration/test_vision_degradation.py",
                "tests/integration/test_mission_aruco.py",
                "tests/integration/test_typing_aruco.py",
            ],
            "State Management": [
                "tests/integration/test_state_machine_integration.py",
                "tests/integration/test_state_machine_comprehensive.py",
                "tests/integration/test_state_machine_concurrent.py",
            ],
            "Mission & Tasks": [
                "tests/integration/test_mission_system.py",
                "tests/integration/test_mission_execution_comprehensive.py",
                "tests/integration/test_follow_me.py",
            ],
            "Safety & Arm Control": [
                "tests/integration/test_advanced_safety.py",
                "tests/integration/test_arm_control.py",
                "tests/integration/test_led_integration.py",
                "tests/integration/test_led_coordination.py",
            ],
            "SLAM & Mapping": [
                "tests/integration/test_slam_integration.py",
                "tests/integration/test_sensor_bridge.py",
            ]
        }

        results = []
        total_tests = 0
        passed_tests = 0
        failed_tests = 0

        for category, modules in integration_categories.items():
            print(f"📁 {category}")
            print("-" * 40)

            for module in modules:
                module_path = self.project_root / module
                if not module_path.exists():
                    print(f"⚠️  Skipping {module} - file not found")
                    continue

                print(f"  📋 Running {os.path.basename(module)}...")

                try:
                    # Create a temporary directory with just this test file to avoid collection issues
                    import shutil
                    import tempfile

                    with tempfile.TemporaryDirectory() as temp_dir:
                        # Copy the test file to temp directory
                        temp_test_file = Path(temp_dir) / module_path.name
                        shutil.copy2(module_path, temp_test_file)

                        # Copy any __init__.py files needed
                        init_file = module_path.parent / "__init__.py"
                        if init_file.exists():
                            shutil.copy2(init_file, Path(temp_dir) / "__init__.py")

                        # Set up environment with ROS2
                        env = self.ros2_manager.ros2_env.copy() if hasattr(self.ros2_manager, 'ros2_env') else os.environ.copy()
                        env['PYTHONPATH'] = f"{self.project_root}:{self.project_root}/Autonomy:{self.project_root}/Autonomy/code:{env.get('PYTHONPATH', '')}"

                        result = subprocess.run([
                            sys.executable, "-m", "pytest", temp_test_file.name,
                            "-v", "--tb=short"
                        ], cwd=temp_dir, env=env, capture_output=True, text=True, timeout=600)

                    # Parse pytest output for test results
                    output = result.stdout + result.stderr
                    passed_count = 0
                    failed_count = 0

                    # Look for pytest summary lines
                    lines = output.strip().split('\n')
                    for line in lines:
                        line = line.strip()
                        if 'passed' in line and 'failed' in line:
                            # Parse "3 passed, 1 failed"
                            parts = line.replace(',', '').split()
                            for i, part in enumerate(parts):
                                if part == 'passed' and i > 0:
                                    try:
                                        passed_count = int(parts[i-1])
                                    except (ValueError, IndexError):
                                        passed_count = 0
                                elif part == 'failed' and i > 0:
                                    try:
                                        failed_count = int(parts[i-1])
                                    except (ValueError, IndexError):
                                        failed_count = 0
                            break
                        elif line.startswith('=') and 'passed' in line and 'failed' not in line:
                            # Summary line like "====== 3 passed in 1.23s ======"
                            parts = line.split()
                            for part in parts:
                                if part.isdigit():
                                    passed_count = int(part)
                                    failed_count = 0
                                    break

                    total_module_tests = passed_count + failed_count

                    if result.returncode == 0:
                        # All tests passed
                        total_tests += total_module_tests if total_module_tests > 0 else 1
                        passed_tests += total_module_tests if total_module_tests > 0 else 1
                        status_icon = "✅"
                        test_info = f"{passed_count} passed" if passed_count > 0 else "passed"
                        print(f"    {status_icon} {os.path.basename(module)}: {test_info}")
                    elif result.returncode == 1:
                        # Some tests failed
                        total_tests += total_module_tests
                        passed_tests += passed_count
                        failed_tests += failed_count
                        status_icon = "❌"
                        print(f"    {status_icon} {os.path.basename(module)}: {passed_count} passed, {failed_count} failed")
                    else:
                        # Other error (import issues, etc.)
                        failed_tests += 1
                        print(f"    ⚠️  {os.path.basename(module)}: Error (exit code {result.returncode})")

                except subprocess.TimeoutExpired:
                    print(f"    ⏰ {os.path.basename(module)}: TIMEOUT (10 minutes)")
                    failed_tests += 1
                except Exception as e:
                    print(f"    ❌ {os.path.basename(module)}: ERROR - {e}")
                    failed_tests += 1

            print()

        self.end_time = time.time()

        # ROS2 cleanup is handled automatically by pytest fixtures

        # Generate summary
        summary = {
            "test_type": "integration_tests",
            "timestamp": datetime.now().isoformat(),
            "duration_seconds": self.end_time - self.start_time,
            "ros2_environment_setup": "managed_by_pytest_fixtures",
            "total_categories": len(integration_categories),
            "total_modules": sum(len(modules) for modules in integration_categories.values()),
            "total_tests": total_tests,
            "passed_tests": passed_tests,
            "failed_tests": failed_tests,
            "pass_rate": (passed_tests / total_tests * 100) if total_tests > 0 else 0,
            "categories": list(integration_categories.keys()),
            "results": results
        }

        return summary

    def _parse_pytest_results(self, json_file: str, module: str, category: str) -> List[Dict[str, Any]]:
        """Parse pytest JSON results."""
        try:
            if os.path.exists(json_file):
                with open(json_file, 'r') as f:
                    data = json.load(f)

                results = []
                for test in data.get("tests", []):
                    results.append({
                        "category": category,
                        "module": module,
                        "nodeid": test.get("nodeid", ""),
                        "outcome": test.get("outcome", "unknown"),
                        "duration": test.get("duration", 0),
                        "call": test.get("call", {})
                    })
                return results
        except Exception as e:
            print(f"    ⚠️  Error parsing results for {module}: {e}")

        return []

    def save_results(self, results: Dict[str, Any], output_file: str = None):
        """Save test results to file."""
        if not output_file:
            output_file = self.project_root / "test_reports" / "integration_test_results.json"

        output_file.parent.mkdir(exist_ok=True)

        with open(output_file, 'w') as f:
            json.dump(results, f, indent=2, default=str)

        print(f"\n📄 Results saved to: {output_file}")

    def print_summary(self, results: Dict[str, Any]):
        """Print test summary."""
        print("\n" + "=" * 50)
        print("📊 INTEGRATION TEST RESULTS SUMMARY")
        print("=" * 50)

        # Show ROS2 environment status
        ros2_setup = results.get('ros2_environment_setup', 'unknown')
        if ros2_setup == 'managed_by_pytest_fixtures':
            ros2_status = "✅ Managed by pytest fixtures"
        else:
            ros2_status = "✅ Ready" if ros2_setup else "❌ Failed"
        print(f"ROS2 Environment: {ros2_status}")
        print(f"Total Categories: {results['total_categories']}")
        print(f"Total Test Modules: {results['total_modules']}")
        print(f"Total Tests: {results['total_tests']}")
        print(f"Passed: {results['passed_tests']}")
        print(f"Failed: {results['failed_tests']}")
        print(".1f")
        print(f"Duration: {results['duration_seconds']:.2f} seconds")
        print()

        # Show results by category
        print("By Category:")
        category_results = {}
        for result in results.get('results', []):
            cat = result.get('category', 'unknown')
            if cat not in category_results:
                category_results[cat] = {'total': 0, 'passed': 0}
            category_results[cat]['total'] += 1
            if result.get('outcome') == 'passed':
                category_results[cat]['passed'] += 1

        for category, stats in category_results.items():
            pass_rate = (stats['passed'] / stats['total'] * 100) if stats['total'] > 0 else 0
            status_icon = "✅" if stats['passed'] == stats['total'] else "⚠️"
            print(f"  {status_icon} {category}: {stats['passed']}/{stats['total']} ({pass_rate:.1f}%)")

        if not category_results:
            print("  No test results available")

        print()

        if results['failed_tests'] > 0:
            print("⚠️  INTEGRATION TESTS HAVE ISSUES")
            print("Review integration failures - components may not work together properly")
            return False
        else:
            print("✅ ALL INTEGRATION TESTS PASSED")
            print("Ready to proceed to simulation and network testing")
            return True


def main():
    """Main entry point."""
    runner = IntegrationTestRunner()
    results = runner.run_integration_tests()

    # Check for ROS2 setup failure
    if "ros2_setup_failed" in results and results["ros2_setup_failed"]:
        print("\n❌ ROS2 ENVIRONMENT SETUP FAILED")
        print("Cannot run integration tests without ROS2 environment")
        print("\nTroubleshooting:")
        print("1. Ensure ROS2 Humble is installed and sourced")
        print("2. Check that launch files exist in launch/ directory")
        print("3. Verify ROS2 daemon can start")
        return 1

    # Save results
    runner.save_results(results)

    # Print summary and return appropriate exit code
    success = runner.print_summary(results)
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
