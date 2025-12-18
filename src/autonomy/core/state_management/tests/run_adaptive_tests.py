#!/usr/bin/env python3
"""
Adaptive State Machine Test Runner

Comprehensive test execution script for the adaptive state machine components.
Supports different test suites, performance benchmarking, and CI/CD integration.

Features:
- Automated ROS2 environment setup
- Comprehensive test result reporting
- HTML coverage reports and performance metrics
- CI/CD integration with JUnit XML output
"""

import argparse
import json
import os
import shutil
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional


class AdaptiveTestRunner:
    """Test runner for adaptive state machine components."""

    def __init__(self):
        self.test_dir = Path(__file__).parent
        self.project_root = self.test_dir.parent.parent.parent.parent
        self.reports_dir = self.test_dir / "reports"
        self.start_time = None
        self.end_time = None
        self.test_results = {}

        # Create reports directory
        self.reports_dir.mkdir(exist_ok=True)

        # Environment detection
        self.ros2_available = self._check_ros2_availability()
        self.python_path = self._setup_python_path()

    def _check_ros2_availability(self) -> bool:
        """Check if ROS2 is available and properly configured."""
        try:
            # Check if ROS_DISTRO is set
            ros_distro = os.environ.get('ROS_DISTRO')
            if not ros_distro:
                # Try to detect from common locations
                ros_paths = ['/opt/ros/humble', '/opt/ros/foxy', '/opt/ros/galactic']
                for path in ros_paths:
                    if os.path.exists(path):
                        ros_distro = path.split('/')[-1]
                        os.environ['ROS_DISTRO'] = ros_distro
                        break

            if not ros_distro:
                self.logger.warning("ROS2 distribution not detected")
                return False

            # Check if setup.bash exists
            setup_path = f'/opt/ros/{ros_distro}/setup.bash'
            if not os.path.exists(setup_path):
                self.logger.warning(f"ROS2 setup file not found: {setup_path}")
                return False

            # Try to source and test
            result = subprocess.run(
                ['bash', '-c', f'source {setup_path} && python3 -c "import rclpy; print(\'OK\')"'],
                capture_output=True, text=True, cwd=self.project_root
            )

            return result.returncode == 0 and 'OK' in result.stdout

        except Exception as e:
            self.logger.error(f"ROS2 availability check failed: {e}")
            return False

    def _setup_python_path(self) -> str:
        """Set up Python path for imports."""
        paths = [
            str(self.test_dir.parent),  # autonomy_state_machine package
            str(self.project_root / 'src' / 'autonomy' / 'interfaces'),  # autonomy_interfaces
            str(self.project_root / 'src'),  # Additional src paths
        ]

        python_path = ':'.join(paths)
        os.environ['PYTHONPATH'] = python_path
        return python_path

    def setup_test_environment(self) -> bool:
        """Set up the complete test environment."""
        try:
            self.logger.info("[FIX] Setting up test environment...")

            # Ensure required packages are built
            if not self._ensure_packages_built():
                self.logger.error("[ERROR] Package building failed")
                return False

            # Setup ROS2 if available
            if self.ros2_available:
                if not self._source_ros2():
                    self.logger.warning("ROS2 setup failed, tests may not work properly")
                    return False
            else:
                self.logger.warning("ROS2 not available, using mock interfaces")

            # Setup Python path
            os.environ['PYTHONPATH'] = self.python_path

            # Additional environment setup
            os.environ['TESTING'] = '1'  # Flag for test mode
            os.environ['ROS_DOMAIN_ID'] = '42'  # Use specific domain for tests

            self.logger.info("[SUCCESS] Test environment setup complete")
            return True

        except Exception as e:
            self.logger.error(f"Environment setup failed: {e}")
            return False

    def _ensure_packages_built(self) -> bool:
        """Ensure required ROS2 packages are built."""
        required_packages = ['autonomy_interfaces', 'autonomy_state_management']

        for package in required_packages:
            if not self._is_package_built(package):
                self.logger.info(f"🔨 Building package: {package}")
                if not self._build_package(package):
                    self.logger.error(f"[ERROR] Failed to build package: {package}")
                    return False

        return True

    def _is_package_built(self, package_name: str) -> bool:
        """Check if a package is already built."""
        install_path = self.project_root / 'install' / package_name
        return install_path.exists()

    def _build_package(self, package_name: str) -> bool:
        """Build a specific ROS2 package."""
        try:
            ros_distro = os.environ.get('ROS_DISTRO', 'humble')

            cmd = [
                'bash', '-c',
                f'cd {self.project_root} && '
                f'source /opt/ros/{ros_distro}/setup.bash && '
                f'colcon build --packages-select {package_name} --cmake-clean-cache'
            ]

            result = subprocess.run(
                cmd,
                capture_output=False,  # Show build output
                text=True,
                timeout=300  # 5 minute timeout
            )

            return result.returncode == 0

        except subprocess.TimeoutExpired:
            self.logger.error(f"Package build timed out: {package_name}")
            return False
        except Exception as e:
            self.logger.error(f"Package build failed: {e}")
            return False

    def _source_ros2(self) -> bool:
        """Source ROS2 environment."""
        try:
            ros_distro = os.environ.get('ROS_DISTRO', 'humble')
            setup_script = f'/opt/ros/{ros_distro}/setup.bash'

            if not os.path.exists(setup_script):
                return False

            # Source ROS2 in the current environment
            result = subprocess.run(
                ['bash', '-c', f'source {setup_script} && env'],
                capture_output=True, text=True, cwd=self.project_root
            )

            if result.returncode == 0:
                # Update current environment with ROS2 variables
                for line in result.stdout.split('\n'):
                    if '=' in line:
                        key, value = line.split('=', 1)
                        if key.startswith(('ROS_', 'AMENT_', 'COLCON_')) or key in ['PATH', 'PYTHONPATH', 'LD_LIBRARY_PATH']:
                            os.environ[key] = value

                self.logger.info(f"[SUCCESS] ROS2 {ros_distro} environment sourced")
                return True

            return False

        except Exception as e:
            self.logger.error(f"ROS2 sourcing failed: {e}")
            return False

    def _run_command_with_ros2(self, cmd: List[str]) -> bool:
        """Run a command with ROS2 environment sourced."""
        if self.ros2_available:
            ros_distro = os.environ.get('ROS_DISTRO', 'humble')

            # Create a wrapper script that sources ROS2 and then runs the command
            wrapper_script = f"""#!/bin/bash
set -e
source /opt/ros/{ros_distro}/setup.bash
# Also source the built packages
if [ -f "/home/ubuntu/urc-machiato-2026/install/setup.bash" ]; then
    source /home/ubuntu/urc-machiato-2026/install/setup.bash
fi
export PYTHONPATH="{self.python_path}:$PYTHONPATH"
export ROS_DOMAIN_ID=42
export TESTING=1
cd /home/ubuntu/urc-machiato-2026/src/autonomy/core/state_management/tests
exec {" ".join(cmd)}
"""
            # Write wrapper script to temp file
            import tempfile
            with tempfile.NamedTemporaryFile(mode='w', suffix='.sh', delete=False) as f:
                f.write(wrapper_script.strip())
                wrapper_path = f.name

            # Make it executable
            os.chmod(wrapper_path, 0o755)

            full_cmd = ['bash', wrapper_path]
        else:
            # Fall back to regular command if ROS2 not available
            full_cmd = cmd

        return self._run_command(full_cmd)

    @property
    def logger(self):
        """Get logger instance."""
        import logging
        return logging.getLogger('AdaptiveTestRunner')

    def run_unit_tests(self, verbose: bool = True, coverage: bool = True) -> bool:
        """Run unit tests for all adaptive components."""
        self.logger.info("[TEST] Running Adaptive State Machine Unit Tests")

        # Run all unit test files
        test_files = [
            "test_adaptive_state_machine.py",
            "test_safety_manager.py",
            "test_error_handling.py",
            "test_transition_manager.py",
            "test_monitoring_service.py",
            "test_environment_setup.py"
        ]

        cmd = ["python3", "-m", "pytest"]
        cmd.extend([str(self.test_dir / tf) for tf in test_files])

        if verbose:
            cmd.append("-v")
        if coverage:
            cmd.extend([
                "--cov=autonomy_state_machine",
                "--cov-report=term-missing",
                f"--cov-report=html:{self.reports_dir}/coverage_html",
                f"--cov-report=xml:{self.reports_dir}/coverage.xml"
            ])

        # Source ROS2 environment for pytest
        success = self._run_command_with_ros2(cmd)
        self.test_results['unit_tests'] = success
        return success

    def run_performance_tests(self, benchmark: bool = True) -> bool:
        """Run performance and stress tests."""
        self.logger.info("[PERFORMANCE] Running Adaptive State Machine Performance Tests")

        cmd = ["python3", "-m", "pytest", str(self.test_dir / "test_adaptive_performance.py")]

        if benchmark:
            # Use pytest-benchmark if available
            try:
                import pytest_benchmark
                cmd.extend(["--benchmark-only", "--benchmark-json", f"{self.reports_dir}/benchmarks.json"])
            except ImportError:
                self.logger.warning("pytest-benchmark not available, running basic performance tests")

        cmd.extend(["-v", "--tb=short"])
        success = self._run_command_with_ros2(cmd)
        self.test_results['performance_tests'] = success
        return success

    def run_scenario_tests(self, scenario_filter: str = None) -> bool:
        """Run scenario-based tests."""
        self.logger.info("[ACTION] Running Adaptive State Machine Scenario Tests")

        cmd = ["python3", "-m", "pytest", str(self.test_dir / "test_adaptive_scenarios.py")]

        if scenario_filter:
            cmd.extend(["-k", scenario_filter])

        cmd.extend(["-v", "--tb=short"])
        success = self._run_command_with_ros2(cmd)
        self.test_results['scenario_tests'] = success
        return success

    def run_integration_tests(self) -> bool:
        """Run integration tests."""
        self.logger.info("[CONNECT] Running Adaptive State Machine Integration Tests")

        # Run integration tests from main test file
        cmd = [
            "python3", "-m", "pytest",
            str(self.test_dir / "test_adaptive_state_machine.py"),
            "-k", "integration",
            "-v", "--tb=short"
        ]

        success = self._run_command_with_ros2(cmd)
        self.test_results['integration_tests'] = success
        return success

    def run_all_tests(self) -> bool:
        """Run complete test suite with environment setup and comprehensive reporting."""
        self.get_logger().info("[START] Running Complete Adaptive State Machine Test Suite")

        self.start_time = datetime.now()

        # Setup environment
        if not self.setup_test_environment():
            self.logger.error("[ERROR] Environment setup failed, aborting tests")
            return False

        test_results = []

        # Unit tests
        self.logger.info("Running unit tests...")
        test_results.append(("Unit Tests", self.run_unit_tests(verbose=False)))

        # Performance tests
        self.logger.info("Running performance tests...")
        test_results.append(("Performance Tests", self.run_performance_tests()))

        # Scenario tests
        self.logger.info("Running scenario tests...")
        test_results.append(("Scenario Tests", self.run_scenario_tests()))

        # Integration tests
        self.logger.info("Running integration tests...")
        test_results.append(("Integration Tests", self.run_integration_tests()))

        self.end_time = datetime.now()

        # Generate comprehensive report
        self._generate_comprehensive_report(test_results)

        # Summary
        self.get_logger().info("\n[STATUS] Test Suite Summary")

        all_passed = True
        for test_name, passed in test_results:
            status = "[SUCCESS] PASSED" if passed else "[ERROR] FAILED"
        self.get_logger().info("25")
        all_passed = all_passed and passed

        overall_status = "[SUCCESS] ALL TESTS PASSED" if all_passed else "[ERROR] SOME TESTS FAILED"
        self.get_logger().info(f"\n[TARGET] Overall Result: {overall_status}")
        # Show report location
        report_path = self.reports_dir / "test_summary.html"
        if report_path.exists():
            self.get_logger().info(f"📄 Detailed report: {report_path}")
        return all_passed

    def _generate_comprehensive_report(self, test_results: List[tuple]):
        """Generate comprehensive HTML test report."""
        try:
            duration = self.end_time - self.start_time
            total_duration_seconds = duration.total_seconds()

            # Collect system information
            system_info = self._get_system_info()

            # Generate HTML report
            html_content = self._create_html_report(test_results, total_duration_seconds, system_info)

            # Write report files
            report_path = self.reports_dir / "test_summary.html"
            with open(report_path, 'w') as f:
                f.write(html_content)

            # Generate JSON summary
            json_summary = self._create_json_summary(test_results, total_duration_seconds, system_info)
            json_path = self.reports_dir / "test_summary.json"
            with open(json_path, 'w') as f:
                json.dump(json_summary, f, indent=2)

            self.logger.info(f"📄 HTML report generated: {report_path}")
            self.logger.info(f"📄 JSON summary generated: {json_path}")

        except Exception as e:
            self.logger.error(f"Report generation failed: {e}")

    def _get_system_info(self) -> Dict[str, Any]:
        """Get system information for reporting."""
        try:
            import platform

            import psutil

            return {
                'python_version': platform.python_version(),
                'platform': platform.platform(),
                'cpu_count': psutil.cpu_count(),
                'memory_total': psutil.virtual_memory().total // (1024**3),  # GB
                'ros2_available': self.ros2_available,
                'ros2_distro': os.environ.get('ROS_DISTRO', 'Not set'),
                'test_environment': os.environ.get('CI', 'Local development')
            }
        except Exception:
            return {'error': 'System info collection failed'}

    def _create_html_report(self, test_results: List[tuple], duration: float,
                          system_info: Dict[str, Any]) -> str:
        """Create HTML test report."""
        passed_count = sum(1 for _, passed in test_results if passed)
        failed_count = len(test_results) - passed_count

        html = f"""
<!DOCTYPE html>
<html>
<head>
    <title>Adaptive State Machine Test Report</title>
    <style>
        body {{ font-family: Arial, sans-serif; margin: 40px; }}
        .header {{ background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
                 color: white; padding: 20px; border-radius: 10px; margin-bottom: 30px; }}
        .summary {{ display: flex; gap: 20px; margin-bottom: 30px; }}
        .metric {{ background: #f8f9fa; padding: 20px; border-radius: 8px; flex: 1; text-align: center; }}
        .metric.success {{ border-left: 4px solid #28a745; }}
        .metric.failure {{ border-left: 4px solid #dc3545; }}
        .metric.info {{ border-left: 4px solid #17a2b8; }}
        .test-result {{ margin-bottom: 10px; padding: 10px; border-radius: 5px; }}
        .test-result.pass {{ background: #d4edda; border-left: 4px solid #28a745; }}
        .test-result.fail {{ background: #f8d7da; border-left: 4px solid #dc3545; }}
        .system-info {{ background: #e9ecef; padding: 20px; border-radius: 8px; margin-bottom: 30px; }}
        .footer {{ text-align: center; color: #6c757d; margin-top: 40px; }}
    </style>
</head>
<body>
    <div class="header">
        <h1>[START] Adaptive State Machine Test Report</h1>
        <p>Generated on {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}</p>
    </div>

    <div class="summary">
        <div class="metric success">
            <h3>{passed_count}</h3>
            <p>Tests Passed</p>
        </div>
        <div class="metric failure">
            <h3>{failed_count}</h3>
            <p>Tests Failed</p>
        </div>
        <div class="metric info">
            <h3>{duration:.1f}s</h3>
            <p>Total Duration</p>
        </div>
        <div class="metric info">
            <h3>{len(test_results)}</h3>
            <p>Test Suites</p>
        </div>
    </div>

    <div class="system-info">
        <h3>[SCREEN] System Information</h3>
        <ul>
            <li><strong>Python Version:</strong> {system_info.get('python_version', 'Unknown')}</li>
            <li><strong>Platform:</strong> {system_info.get('platform', 'Unknown')}</li>
            <li><strong>CPU Cores:</strong> {system_info.get('cpu_count', 'Unknown')}</li>
            <li><strong>Memory:</strong> {system_info.get('memory_total', 'Unknown')} GB</li>
            <li><strong>ROS2 Available:</strong> {'[SUCCESS]' if system_info.get('ros2_available') else '[ERROR]'}</li>
            <li><strong>ROS2 Distro:</strong> {system_info.get('ros2_distro', 'Unknown')}</li>
        </ul>
    </div>

    <h3>[STATUS] Test Results</h3>
"""

        for test_name, passed in test_results:
            css_class = "pass" if passed else "fail"
            icon = "[SUCCESS]" if passed else "[ERROR]"
            html += f"""
    <div class="test-result {css_class}">
        <strong>{icon} {test_name}</strong>
    </div>"""

        html += """
    <div class="footer">
        <p>Report generated by Adaptive State Machine Test Runner</p>
        <p>🔬 Ensuring reliability for URC 2026 Mars Rover autonomy</p>
    </div>
</body>
</html>"""

        return html

    def _create_json_summary(self, test_results: List[tuple], duration: float,
                           system_info: Dict[str, Any]) -> Dict[str, Any]:
        """Create JSON test summary."""
        passed_count = sum(1 for _, passed in test_results if passed)
        failed_count = len(test_results) - passed_count

        return {
            'timestamp': datetime.now().isoformat(),
            'duration_seconds': duration,
            'summary': {
                'total_tests': len(test_results),
                'passed': passed_count,
                'failed': failed_count,
                'success_rate': passed_count / len(test_results) if test_results else 0
            },
            'test_results': [
                {'name': name, 'passed': passed}
                for name, passed in test_results
            ],
            'system_info': system_info,
            'reports_generated': [
                str(self.reports_dir / "test_summary.html"),
                str(self.reports_dir / "test_summary.json"),
                str(self.reports_dir / "coverage_html" / "index.html") if (self.reports_dir / "coverage.xml").exists() else None,
                str(self.reports_dir / "benchmarks.json") if (self.reports_dir / "benchmarks.json").exists() else None
            ]
        }

    def run_ci_tests(self) -> bool:
        """Run tests optimized for CI/CD environment."""
        self.logger.info("[UPDATE] Running Adaptive State Machine CI Tests")

        # Setup environment for CI
        if not self.setup_test_environment():
            self.logger.error("[ERROR] CI environment setup failed")
            return False

        os.environ['ROS_DOMAIN_ID'] = '42'  # Use specific domain for CI

        # Run tests with coverage and JUnit output
        cmd = [
            "python3", "-m", "pytest",
            str(self.test_dir),
            "--cov=autonomy_state_machine",
            "--cov-report=term-missing",
            f"--cov-report=xml:{self.reports_dir}/coverage.xml",
            f"--junitxml={self.reports_dir}/junit.xml",
            "--tb=short",
            "-x",  # Stop on first failure
            "--disable-warnings",
            "-q"   # Quiet mode for CI
        ]

        success = self._run_command_with_ros2(cmd)

        if success:
            self.logger.info("[SUCCESS] CI Tests Passed")
            # Generate CI-specific summary
            self._generate_ci_summary()
        else:
            self.logger.error("[ERROR] CI Tests Failed")

        return success

    def _generate_ci_summary(self):
        """Generate CI-specific summary for automated systems."""
        try:
            ci_summary = {
                'status': 'success',
                'timestamp': datetime.now().isoformat(),
                'reports': {
                    'coverage': str(self.reports_dir / "coverage.xml"),
                    'junit': str(self.reports_dir / "junit.xml"),
                    'html_summary': str(self.reports_dir / "test_summary.html")
                },
                'artifacts': [
                    str(self.reports_dir / "coverage_html"),
                    str(self.reports_dir / "benchmarks.json") if (self.reports_dir / "benchmarks.json").exists() else None
                ]
            }

            ci_path = self.reports_dir / "ci_summary.json"
            with open(ci_path, 'w') as f:
                json.dump(ci_summary, f, indent=2)

            self.logger.info(f"📄 CI summary generated: {ci_path}")

        except Exception as e:
            self.logger.error(f"CI summary generation failed: {e}")

    def run_stress_test(self, duration: int = 60) -> bool:
        """Run stress tests for extended periods."""
        self.get_logger().info(f"[HOT] Running Adaptive State Machine Stress Test ({duration}s)")

        start_time = time.time()

        try:
            # Run performance tests repeatedly
            while time.time() - start_time < duration:
                if not self.run_performance_tests():
                    self.get_logger().info("[ERROR] Stress test failed during execution")
                    return False

                # Brief pause between test runs
                time.sleep(1)
        self.get_logger().info(f"[SUCCESS] Stress test completed successfully ({duration}s)")
            return True

        except KeyboardInterrupt:
        self.get_logger().info("[STOP] Stress test interrupted by user")
            return False
        except Exception as e:
        self.get_logger().info(f"[ERROR] Stress test failed with error: {e}")
            return False

    def generate_test_report(self) -> None:
        """Generate a comprehensive test report."""
        self.get_logger().info("[LIST] Generating Adaptive State Machine Test Report")

        report_path = self.test_dir / "test_report.md"

        with open(report_path, 'w') as f:
            f.write("# Adaptive State Machine Test Report\n\n")
            f.write(f"Generated: {time.strftime('%Y-%m-%d %H:%M:%S')}\n\n")

            # Test structure
            f.write("## Test Structure\n\n")
            f.write("```\n")
            f.write("tests/\n")
            f.write("├── test_adaptive_state_machine.py    # Core unit tests\n")
            f.write("├── test_adaptive_performance.py      # Performance & reliability\n")
            f.write("├── test_adaptive_scenarios.py        # Mission scenario tests\n")
            f.write("├── pytest.ini                        # Test configuration\n")
            f.write("└── run_adaptive_tests.py            # Test runner (this file)\n")
            f.write("```\n\n")

            # Coverage areas
            f.write("## Test Coverage Areas\n\n")
            f.write("- **Context Evaluation**: System monitoring and context assessment\n")
            f.write("- **Policy Engine**: Adaptive decision making and policy execution\n")
            f.write("- **State Machine**: Enhanced state transitions with adaptation\n")
            f.write("- **Monitoring Service**: Real-time monitoring and analytics\n")
            f.write("- **Performance**: Scalability, reliability, and resource usage\n")
            f.write("- **Scenarios**: Realistic URC mission situations\n")
            f.write("- **Integration**: End-to-end system behavior\n\n")

            # Key metrics
            f.write("## Key Test Metrics\n\n")
            f.write("- **Unit Test Coverage**: >80% code coverage required\n")
            f.write("- **Performance**: <50ms context evaluation, <20ms policy decisions\n")
            f.write("- **Reliability**: Graceful degradation under failure conditions\n")
            f.write("- **Memory Usage**: <50MB additional memory for adaptive features\n")
            f.write("- **Mission Success**: >95% success rate in adaptive scenarios\n\n")
        self.get_logger().info(f"📄 Test report generated: {report_path}")
    def _run_command(self, cmd: List[str]) -> bool:
        """Run a command and return success status."""
        try:
            result = subprocess.run(
                cmd,
                cwd=self.project_root,
                capture_output=False,
                text=True,
                timeout=300  # 5 minute timeout
            )
            return result.returncode == 0
        except subprocess.TimeoutExpired:
        self.get_logger().info("[ERROR] Command timed out")
            return False
        except Exception as e:
        self.get_logger().info(f"[ERROR] Command failed with error: {e}")
            return False


def main():
    """Main entry point for test runner."""
    # Setup logging
    import logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
        datefmt='%H:%M:%S'
    )

    parser = argparse.ArgumentParser(
        description="Adaptive State Machine Test Runner",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python run_adaptive_tests.py unit          # Run unit tests
  python run_adaptive_tests.py performance   # Run performance tests
  python run_adaptive_tests.py scenario      # Run scenario tests
  python run_adaptive_tests.py all           # Run all tests
  python run_adaptive_tests.py ci            # Run CI-optimized tests
  python run_adaptive_tests.py stress --duration 120  # 2-minute stress test
  python run_adaptive_tests.py report        # Generate test report

Environment Variables:
  ROS_DISTRO      ROS2 distribution (auto-detected if not set)
  ROS_DOMAIN_ID   ROS2 domain ID (default: 42 for tests)
  PYTHONPATH      Additional Python paths (auto-configured)
        """
    )

    parser.add_argument(
        'command',
        choices=['unit', 'performance', 'scenario', 'integration', 'all', 'ci', 'stress', 'report'],
        help='Test command to execute'
    )

    parser.add_argument(
        '--duration',
        type=int,
        default=60,
        help='Duration for stress tests (seconds)'
    )

    parser.add_argument(
        '--scenario-filter',
        type=str,
        help='Filter for scenario tests (e.g., "battery" or "emergency")'
    )

    parser.add_argument(
        '--verbose', '-v',
        action='store_true',
        help='Enable verbose output'
    )

    parser.add_argument(
        '--no-setup',
        action='store_true',
        help='Skip automatic environment setup'
    )

    args = parser.parse_args()

    # Create test runner
    runner = AdaptiveTestRunner()

    # Print banner
        self.get_logger().info("🔬 Adaptive State Machine Test Runner")

        self.get_logger().info(f"ROS2 Available: {'[SUCCESS]' if runner.ros2_available else '[ERROR]'}")
        self.get_logger().info(f"Reports Directory: {runner.reports_dir}")

    success = False

    try:
        if args.command == 'unit':
            success = runner.run_unit_tests(verbose=args.verbose)
        elif args.command == 'performance':
            success = runner.run_performance_tests()
        elif args.command == 'scenario':
            success = runner.run_scenario_tests(args.scenario_filter)
        elif args.command == 'integration':
            success = runner.run_integration_tests()
        elif args.command == 'all':
            if not args.no_setup:
        self.get_logger().info("[FIX] Setting up test environment...")
            success = runner.run_all_tests()
        elif args.command == 'ci':
            success = runner.run_ci_tests()
        elif args.command == 'stress':
            if not args.no_setup:
                runner.setup_test_environment()
            success = runner.run_stress_test(args.duration)
        elif args.command == 'report':
            runner.generate_test_report()
            success = True

        # Print final status
        if success:
        self.get_logger().info("\n[SUCCESS] Test execution completed successfully")
            if hasattr(runner, 'reports_dir'):
        self.get_logger().info(f"📄 Reports available in: {runner.reports_dir}")
        else:
        self.get_logger().info("\n[ERROR] Test execution failed")
        self.get_logger().info("Check the logs above for details")
    except KeyboardInterrupt:
        self.get_logger().info("\n[STOP] Test execution interrupted by user")
        success = False
    except Exception as e:
        self.get_logger().info(f"\n[FAIL] Test execution failed with error: {e}")
        success = False

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
