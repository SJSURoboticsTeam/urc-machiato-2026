#!/usr/bin/env python3
"""
Smart Test Runner for URC 2026

Intelligent test execution based on development context:
- Development: Fast feedback for active development
- Pre-commit: Quality gates before commits
- CI/CD: Comprehensive validation for merges
- Hardware: Tests requiring physical systems
- Performance: Load and performance validation
- Stable: Stable unit tests only (no ROS2)
- Mock: Dashboard mock data test suite
- System: Comprehensive system tests
- Validate: Critical systems validation

Usage:
    python scripts/run_tests.py [mode] [options]

Modes:
    dev        - Fast development feedback (<30s)
    pre-commit - Quality gates before commit (<2min)
    ci         - Full CI/CD validation (<10min)
    hardware   - Hardware-specific tests
    performance- Performance and load tests
    stable     - Stable unit tests only (no ROS2)
    mock       - Dashboard mock data test suite
    system     - System tests organized by categories
    validate   - Validation of critical systems
    custom     - Custom test selection
"""

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import List, Optional

# Project paths
PROJECT_ROOT = Path(__file__).parent.parent
TESTS_DIR = PROJECT_ROOT / "tests"


class TestRunner:
    """Smart test runner with context-aware execution."""

    def __init__(self):
        self.start_time = time.time()
        self.results = {}

    def run_command(self, cmd: List[str], description: str, timeout: int = 300) -> bool:
        """Run a command with timing and error handling."""
        print(f"\n🔍 {description}")
        print(f"   Command: {' '.join(cmd)}")

        try:
            start = time.time()
            result = subprocess.run(
                cmd, cwd=PROJECT_ROOT, timeout=timeout, capture_output=False, text=True
            )
            duration = time.time() - start

            if result.returncode == 0:
                print(f"   ✅ SUCCESS: Command completed in {duration:.1f}s")
                return True
            else:
                print(f"   ❌ FAILED: Command failed after {duration:.1f}s")
                return False

        except subprocess.TimeoutExpired:
            print(f"   ❌ TIMEOUT: Command took longer than {timeout}s")
            return False
        except Exception as e:
            print(f"   ❌ ERROR: {e}")
            return False

    def run_dev_mode(self) -> bool:
        print("🚀 Development Mode: Fast feedback for active development")
        tests_passed = 0
        total_tests = 0

        total_tests += 1
        if self.run_command(
            ["python", "-m", "pytest", "tests/unit/", "-v", "--tb=short", "--maxfail=3", "-x"],
            "Unit Tests (Fast Feedback)",
            timeout=30,
        ):
            tests_passed += 1

        total_tests += 1
        if self.run_command(
            ["python", "-m", "pytest", "tests/integration/unit_level/", "-v", "--tb=line", "--maxfail=1", "-k", "not slow"],
            "Integration Smoke Tests",
            timeout=20,
        ):
            tests_passed += 1

        total_tests += 1
        if self.run_command(
            ["python", "-m", "ruff", "check", "services/", "shared/"],
            "Code Quality Check",
            timeout=15,
        ):
            tests_passed += 1

        return tests_passed == total_tests

    def run_pre_commit_mode(self) -> bool:
        print("🔒 Pre-commit Mode: Quality gates before merging")
        tests_passed = 0
        total_tests = 0

        total_tests += 1
        if self.run_command(
            ["python", "-m", "pytest", "tests/unit/", "-v", "--tb=short", "--cov=services", "--cov=shared", "--cov-report=term-missing", "--cov-fail-under=85", "-n", "auto"],
            "Unit Tests with Coverage",
            timeout=60,
        ):
            tests_passed += 1

        total_tests += 1
        if self.run_command(
            ["python", "-m", "pytest", "tests/integration/", "-k", "not (slow or hardware or endurance)", "-v", "--tb=short", "--maxfail=5", "-n", "2"],
            "Integration Tests (Fast)",
            timeout=60,
        ):
            tests_passed += 1

        total_tests += 1
        if self.run_command(
            ["python", "-c", "import subprocess, sys; sys.exit(subprocess.run(['python', '-m', 'ruff', 'check', 'services/', 'shared/']).returncode)"],
            "Code Quality Suite",
            timeout=45,
        ):
            tests_passed += 1

        return tests_passed == total_tests

    def run_ci_mode(self) -> bool:
        print("🔬 CI/CD Mode: Comprehensive validation for merges")
        tests_passed = 0
        total_tests = 0

        total_tests += 1
        if self.run_command(
            ["python", "-m", "pytest", "tests/", "-k", "not (hardware or endurance or chaos)", "--cov=services", "--cov=shared", "--cov-report=xml", "--cov-fail-under=85", "-n", "auto", "--durations=20"],
            "Full Test Suite with Coverage",
            timeout=480,
        ):
            tests_passed += 1

        return tests_passed == total_tests

    def run_hardware_mode(self) -> bool:
        print("🔧 Hardware Mode: Tests requiring physical hardware")
        return self.run_command(
            ["python", "-m", "pytest", "tests/hardware/", "tests/integration/hardware/", "-v", "--tb=short", "-k", "hardware"],
            "Hardware Tests",
            timeout=300,
        )

    def run_performance_mode(self) -> bool:
        print("⚡ Performance Mode: Load and performance validation")
        return self.run_command(
            ["python", "-m", "pytest", "tests/performance/", "-v", "--tb=short", "--durations=0"],
            "Performance Tests",
            timeout=600,
        )

    def run_stable_mode(self) -> bool:
        print("✅ Stable Mode: Stable unit tests only (no ROS2)")
        return self.run_command(
            ["python", "-m", "pytest", "tests/unit/", "-v", "--tb=short", "--ignore=tests/unit/autonomy", "--ignore=tests/unit/core", "--ignore=tests/unit/infrastructure", "--ignore=tests/unit/simulation"],
            "Stable Unit Tests",
            timeout=120,
        )

    def run_mock_mode(self) -> bool:
        print("🎭 Mock Mode: Dashboard mock data test suite")
        return self.run_command(
            ["npm", "run", "test", "--prefix", "services/dashboard"],
            "Dashboard Mock Tests",
            timeout=120,
        )

    def run_system_mode(self) -> bool:
        print("🌌 System Mode: Comprehensive system tests")
        return self.run_command(
            ["python", "-m", "pytest", "tests/integration/system/", "-v", "--tb=short"],
            "System Tests",
            timeout=300,
        )

    def run_validate_mode(self) -> bool:
        print("🛡️ Validate Mode: Critical systems validation")
        return self.run_command(
            ["python", "-m", "pytest", "tests/critical/", "-v", "--tb=short"],
            "Critical Systems Validation",
            timeout=300,
        )

    def run_custom_mode(self, test_path: Optional[str] = None, marker: Optional[str] = None) -> bool:
        print("🎯 Custom Mode: Selective test execution")
        cmd = ["python", "-m", "pytest", "-v", "--tb=short"]
        if marker:
            cmd.extend(["-k", marker])
        elif test_path:
            cmd.append(test_path)
        else:
            cmd.append("tests/unit/")
        return self.run_command(cmd, "Custom Test Selection", timeout=120)


def main():
    parser = argparse.ArgumentParser(
        description="Smart Test Runner for URC 2026",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "mode",
        choices=["dev", "pre-commit", "ci", "hardware", "performance", "stable", "mock", "system", "validate", "custom"],
        help="Test execution mode",
    )
    parser.add_argument("path", nargs="?", help="Custom test path (for custom mode)")
    parser.add_argument("--marker", "-k", help="Pytest marker for custom mode")

    args = parser.parse_args()
    runner = TestRunner()

    mode_map = {
        "dev": runner.run_dev_mode,
        "pre-commit": runner.run_pre_commit_mode,
        "ci": runner.run_ci_mode,
        "hardware": runner.run_hardware_mode,
        "performance": runner.run_performance_mode,
        "stable": runner.run_stable_mode,
        "mock": runner.run_mock_mode,
        "system": runner.run_system_mode,
        "validate": runner.run_validate_mode,
    }

    if args.mode == "custom":
        success = runner.run_custom_mode(args.path, args.marker)
    else:
        success = mode_map[args.mode]()

    total_time = time.time() - runner.start_time
    print(f"\n⏱️  Total execution time: {total_time:.1f}s")

    if success:
        print("🎉 All tests passed!")
        sys.exit(0)
    else:
        print("❌ Some tests failed. Check output above.")
        sys.exit(1)


if __name__ == "__main__":
    main()
