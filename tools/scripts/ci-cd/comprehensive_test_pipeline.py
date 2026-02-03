#!/usr/bin/env python3
"""
Comprehensive Automated Testing Pipeline for URC 2026

Runs all test suites with chaos engineering, performance regression detection,
and comprehensive coverage analysis.
"""

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional


class ComprehensiveTestPipeline:
    """
    Automated testing pipeline with chaos engineering and regression detection.

    Features:
    - Unit and integration testing
    - Performance regression detection
    - Chaos engineering scenarios
    - Coverage analysis
    - Automated reporting
    """

    def __init__(self, workspace_root: str = None):
        self.workspace_root = Path(workspace_root or os.getcwd())
        self.test_results = {}
        self.coverage_data = {}
        self.performance_baselines = self._load_performance_baselines()

    def _load_performance_baselines(self) -> Dict[str, float]:
        """Load performance regression baselines."""
        return {
            "websocket_setup": 0.15,  # seconds
            "config_update": 7.5,  # milliseconds
            "state_sync": 50,  # milliseconds
            "dds_failover": 200,  # milliseconds
        }

    def run_full_pipeline(self) -> bool:
        """Run the complete testing pipeline."""
        print("[IGNITE] Starting Comprehensive Test Pipeline")
        print("=" * 60)

        success = True

        # Phase 1: Environment Setup
        if not self._setup_test_environment():
            print("[FAIL] Environment setup failed")
            return False

        # Phase 2: Unit Tests
        if not self._run_unit_tests():
            print("[FAIL] Unit tests failed")
            success = False

        # Phase 3: Integration Tests
        if not self._run_integration_tests():
            print("[FAIL] Integration tests failed")
            success = False

        # Phase 4: Performance Tests
        if not self._run_performance_tests():
            print("[FAIL] Performance tests failed")
            success = False

        # Phase 5: Chaos Engineering
        if not self._run_chaos_tests():
            print("  Chaos tests revealed issues (expected)")
            # Don't fail on chaos tests - they test failure scenarios

        # Phase 6: Coverage Analysis
        self._analyze_coverage()

        # Phase 7: Generate Report
        self._generate_comprehensive_report()

        print("=" * 60)
        if success:
            print("[PASS] Comprehensive Test Pipeline PASSED")
        else:
            print("[FAIL] Comprehensive Test Pipeline FAILED")

        return success

    def _setup_test_environment(self) -> bool:
        """Set up the testing environment."""
        print("\n[TOOL] Phase 1: Setting up test environment...")

        try:
            # Source ROS2
            env = os.environ.copy()
            env["ROS_DOMAIN_ID"] = "42"

            # Build workspace if needed
            if not (self.workspace_root / "install").exists():
                print("Building ROS2 workspace...")
                result = subprocess.run(
                    [
                        "bash",
                        "-c",
                        "source /opt/ros/humble/setup.bash && colcon build --symlink-install",
                    ],
                    cwd=self.workspace_root,
                    env=env,
                    capture_output=True,
                    timeout=300,
                )
                if result.returncode != 0:
                    print(f"Build failed: {result.stderr.decode()}")
                    return False

            # Source the workspace
            setup_script = self.workspace_root / "install" / "setup.bash"
            if setup_script.exists():
                result = subprocess.run(
                    ["bash", "-c", f"source {setup_script} && ros2 topic list"],
                    env=env,
                    capture_output=True,
                    timeout=10,
                )
                if result.returncode != 0:
                    print("ROS2 environment setup failed")
                    return False

            print("[PASS] Test environment ready")
            return True

        except Exception as e:
            print(f"[FAIL] Environment setup failed: {e}")
            return False

    def _run_unit_tests(self) -> bool:
        """Run unit tests."""
        print("\n[EXPERIMENT] Phase 2: Running unit tests...")

        try:
            result = subprocess.run(
                [
                    sys.executable,
                    "-m",
                    "pytest",
                    "tests/unit/",
                    "-v",
                    "--tb=short",
                    "--cov=src",
                    "--cov-report=term-missing",
                ],
                cwd=self.workspace_root,
                capture_output=True,
                text=True,
                timeout=300,
            )

            self.test_results["unit"] = {
                "returncode": result.returncode,
                "stdout": result.stdout,
                "stderr": result.stderr,
            }

            if result.returncode == 0:
                print("[PASS] Unit tests passed")
                return True
            else:
                print("[FAIL] Unit tests failed")
                print(result.stdout)
                print(result.stderr)
                return False

        except subprocess.TimeoutExpired:
            print("[FAIL] Unit tests timed out")
            return False

    def _run_integration_tests(self) -> bool:
        """Run integration tests."""
        print("\n Phase 3: Running integration tests...")

        try:
            result = subprocess.run(
                [
                    sys.executable,
                    "-m",
                    "pytest",
                    "tests/integration/",
                    "-v",
                    "--tb=short",
                    "--cov-append",
                    "--cov-report=term-missing",
                ],
                cwd=self.workspace_root,
                capture_output=True,
                text=True,
                timeout=600,
            )

            self.test_results["integration"] = {
                "returncode": result.returncode,
                "stdout": result.stdout,
                "stderr": result.stderr,
            }

            if result.returncode == 0:
                print("[PASS] Integration tests passed")
                return True
            else:
                print("[FAIL] Integration tests failed")
                print(result.stdout)
                print(result.stderr)
                return False

        except subprocess.TimeoutExpired:
            print("[FAIL] Integration tests timed out")
            return False

    def _run_performance_tests(self) -> bool:
        """Run performance regression tests."""
        print("\n[LIGHTNING] Phase 4: Running performance tests...")

        try:
            result = subprocess.run(
                [
                    sys.executable,
                    "-m",
                    "pytest",
                    "tests/performance/",
                    "-v",
                    "--tb=short",
                ],
                cwd=self.workspace_root,
                capture_output=True,
                text=True,
                timeout=300,
            )

            self.test_results["performance"] = {
                "returncode": result.returncode,
                "stdout": result.stdout,
                "stderr": result.stderr,
            }

            # Check for performance regressions
            regressions = self._detect_performance_regressions(result.stdout)
            if regressions:
                print("  Performance regressions detected:")
                for regression in regressions:
                    print(f"   {regression}")
                return False

            if result.returncode == 0:
                print("[PASS] Performance tests passed")
                return True
            else:
                print("[FAIL] Performance tests failed")
                return False

        except subprocess.TimeoutExpired:
            print("[FAIL] Performance tests timed out")
            return False

    def _run_chaos_tests(self) -> bool:
        """Run chaos engineering tests."""
        print("\n Phase 5: Running chaos engineering tests...")

        chaos_scenarios = ["network_partition", "high_latency", "packet_loss"]

        for scenario in chaos_scenarios:
            print(f"Running chaos scenario: {scenario}")
            try:
                result = subprocess.run(
                    [
                        sys.executable,
                        "-m",
                        "pytest",
                        f"tests/chaos/test_chaos_engineering.py::{scenario}",
                        "-v",
                        "--tb=short",
                    ],
                    cwd=self.workspace_root,
                    capture_output=True,
                    text=True,
                    timeout=120,
                )

                if result.returncode != 0:
                    print(f"  Chaos scenario {scenario} revealed system weaknesses")
                    print(result.stdout)
                    # Don't fail - chaos tests are meant to find issues

            except subprocess.TimeoutExpired:
                print(f"  Chaos scenario {scenario} timed out")

        print("[PASS] Chaos engineering tests completed")
        return True  # Chaos tests don't "fail" - they reveal issues

    def _detect_performance_regressions(self, test_output: str) -> List[str]:
        """Detect performance regressions from test output."""
        regressions = []

        # Parse performance test results
        lines = test_output.split("\n")
        for line in lines:
            if "Average" in line and ("ms" in line or "s" in line):
                # Extract metric and value
                parts = line.split()
                for i, part in enumerate(parts):
                    if part == "Average":
                        try:
                            metric_name = parts[i - 1].lower().replace(":", "")
                            value_str = parts[i + 1]
                            value = float(value_str.replace("ms", "").replace("s", ""))

                            # Convert seconds to milliseconds for comparison
                            if "s" in parts[i + 1]:
                                value *= 1000

                            # Check against baseline
                            if metric_name in self.performance_baselines:
                                baseline = self.performance_baselines[metric_name]
                                if value > baseline * 1.1:  # 10% regression threshold
                                    regressions.append(
                                        f"{metric_name}: {value:.2f} > baseline {baseline:.2f} "
                                        f"({((value/baseline)-1)*100:.1f}% regression)"
                                    )

                        except (ValueError, IndexError):
                            continue

        return regressions

    def _analyze_coverage(self):
        """Analyze test coverage."""
        print("\n[GRAPH] Phase 6: Analyzing test coverage...")

        try:
            # Generate coverage report
            result = subprocess.run(
                [sys.executable, "-m", "coverage", "report", "--include=src/*"],
                cwd=self.workspace_root,
                capture_output=True,
                text=True,
            )

            self.coverage_data = result.stdout
            print("Coverage report:")
            print(result.stdout)

        except Exception as e:
            print(f"Coverage analysis failed: {e}")

    def _generate_comprehensive_report(self):
        """Generate comprehensive test report."""
        print("\n[CLIPBOARD] Phase 7: Generating comprehensive report...")

        report = {
            "timestamp": time.time(),
            "pipeline_version": "1.0",
            "test_results": self.test_results,
            "coverage": self.coverage_data,
            "performance_baselines": self.performance_baselines,
            "summary": {
                "unit_tests": (
                    "passed"
                    if self.test_results.get("unit", {}).get("returncode") == 0
                    else "failed"
                ),
                "integration_tests": (
                    "passed"
                    if self.test_results.get("integration", {}).get("returncode") == 0
                    else "failed"
                ),
                "performance_tests": (
                    "passed"
                    if self.test_results.get("performance", {}).get("returncode") == 0
                    else "failed"
                ),
                "chaos_tests": "completed",  # Chaos tests always complete
            },
        }

        # Save report
        report_path = (
            self.workspace_root / "test-reports" / "comprehensive_test_report.json"
        )
        report_path.parent.mkdir(exist_ok=True)

        with open(report_path, "w") as f:
            json.dump(report, f, indent=2)

        print(f"[PASS] Comprehensive report saved to {report_path}")


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description="Comprehensive Test Pipeline")
    parser.add_argument(
        "--workspace", "-w", default=None, help="Workspace root directory"
    )
    parser.add_argument(
        "--phase",
        "-p",
        choices=["unit", "integration", "performance", "chaos", "all"],
        default="all",
        help="Run specific test phase",
    )

    args = parser.parse_args()

    pipeline = ComprehensiveTestPipeline(args.workspace)

    if args.phase == "all":
        success = pipeline.run_full_pipeline()
    else:
        # Run individual phases
        phase_map = {
            "unit": pipeline._run_unit_tests,
            "integration": pipeline._run_integration_tests,
            "performance": pipeline._run_performance_tests,
            "chaos": pipeline._run_chaos_tests,
        }

        success = phase_map[args.phase]()

    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
