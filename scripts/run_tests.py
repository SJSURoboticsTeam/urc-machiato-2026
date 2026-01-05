#!/usr/bin/env python3
"""
Smart Test Runner for URC 2026

Intelligent test execution based on development context:
- Development: Fast feedback for active development
- Pre-commit: Quality gates before commits
- CI/CD: Comprehensive validation for merges
- Hardware: Tests requiring physical systems
- Performance: Load and performance validation

Usage:
    python scripts/run_tests.py [mode] [options]

Modes:
    dev        - Fast development feedback (<30s)
    pre-commit - Quality gates before commit (<2min)
    ci         - Full CI/CD validation (<10min)
    hardware   - Hardware-specific tests
    performance- Performance and load tests
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
                cmd,
                cwd=PROJECT_ROOT,
                timeout=timeout,
                capture_output=False,
                text=True
            )
            duration = time.time() - start

            if result.returncode == 0:
                print(".1f"                return True
            else:
                print(".1f"                return False

        except subprocess.TimeoutExpired:
            print(f"   ❌ TIMEOUT: Command took longer than {timeout}s")
            return False
        except Exception as e:
            print(f"   ❌ ERROR: {e}")
            return False

    def run_dev_mode(self) -> bool:
        """Fast development feedback - run most relevant tests quickly."""
        print("🚀 Development Mode: Fast feedback for active development")
        print("   Target: <30 seconds, catch obvious issues")

        tests_passed = 0
        total_tests = 0

        # 1. Unit tests (fastest, most important)
        total_tests += 1
        if self.run_command([
            "python", "-m", "pytest",
            "tests/unit/",
            "-v", "--tb=short",
            "--maxfail=3",
            "-x"  # Stop on first failure for fast feedback
        ], "Unit Tests (Fast Feedback)", timeout=30):
            tests_passed += 1

        # 2. Quick integration smoke tests
        total_tests += 1
        if self.run_command([
            "python", "-m", "pytest",
            "tests/integration/unit_level/",
            "-v", "--tb=line",
            "--maxfail=1",
            "-k", "not slow"
        ], "Integration Smoke Tests", timeout=20):
            tests_passed += 1

        # 3. Basic linting check
        total_tests += 1
        if self.run_command([
            "python", "-m", "flake8",
            "src/autonomy/core/",
            "--max-line-length=88",
            "--extend-ignore=E203,W503",
            "--max-complexity=10"
        ], "Code Quality Check", timeout=15):
            tests_passed += 1

        return tests_passed == total_tests

    def run_pre_commit_mode(self) -> bool:
        """Pre-commit quality gates - ensure code meets standards."""
        print("🔒 Pre-commit Mode: Quality gates before merging")
        print("   Target: <2 minutes, comprehensive but not exhaustive")

        tests_passed = 0
        total_tests = 0

        # 1. All unit tests
        total_tests += 1
        if self.run_command([
            "python", "-m", "pytest",
            "tests/unit/",
            "-v", "--tb=short",
            "--cov=src", "--cov-report=term-missing",
            "--cov-fail-under=85",
            "-n", "auto"
        ], "Unit Tests with Coverage", timeout=60):
            tests_passed += 1

        # 2. Fast integration tests
        total_tests += 1
        if self.run_command([
            "python", "-m", "pytest",
            "tests/integration/",
            "-k", "not (slow or hardware or endurance)",
            "-v", "--tb=short",
            "--maxfail=5",
            "-n", "2"
        ], "Integration Tests (Fast)", timeout=60):
            tests_passed += 1

        # 3. Code quality checks
        total_tests += 1
        if self.run_command([
            "python", "-c", """
import subprocess
import sys
checks = [
    ['python', '-m', 'flake8', 'src/', '--max-line-length=88', '--extend-ignore=E203,W503'],
    ['python', '-m', 'isort', '--check-only', '--diff', 'src/'],
    ['python', '-m', 'mypy', 'src/', '--ignore-missing-imports', '--no-strict-optional']
]
for cmd in checks:
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print(f'❌ {\" \".join(cmd)} failed')
        sys.exit(1)
print('✅ All code quality checks passed')
"""
        ], "Code Quality Suite", timeout=45):
            tests_passed += 1

        return tests_passed == total_tests

    def run_ci_mode(self) -> bool:
        """Full CI/CD validation - comprehensive testing."""
        print("🔬 CI/CD Mode: Comprehensive validation for merges")
        print("   Target: <10 minutes, complete test coverage")

        tests_passed = 0
        total_tests = 0

        # 1. Complete test suite with coverage
        total_tests += 1
        if self.run_command([
            "python", "-m", "pytest",
            "tests/",
            "-k", "not (hardware or endurance or chaos)",
            "--cov=src", "--cov-report=xml",
            "--cov-fail-under=85",
            "-n", "auto",
            "--durations=20"
        ], "Full Test Suite with Coverage", timeout=480):
            tests_passed += 1

        # 2. ROS2 package validation
        total_tests += 1
        if self.run_command([
            "bash", "-c", """
source /opt/ros/humble/setup.bash 2>/dev/null || true
cd /workspace
colcon build --symlink-install --packages-skip urc-machiato-2026 --parallel-workers 2 --continue-on-error
"""
        ], "ROS2 Package Build", timeout=180):
            tests_passed += 1

        # 3. Documentation build
        total_tests += 1
        if self.run_command([
            "bash", "-c", """
cd docs
pip install -r requirements.txt >/dev/null 2>&1 || true
make html >/dev/null 2>&1
"""
        ], "Documentation Build", timeout=60):
            tests_passed += 1

        return tests_passed == total_tests

    def run_hardware_mode(self) -> bool:
        """Hardware-specific tests - require physical systems."""
        print("🔧 Hardware Mode: Tests requiring physical hardware")
        print("   Target: Hardware-dependent validation")

        return self.run_command([
            "python", "-m", "pytest",
            "tests/hardware/",
            "tests/integration/hardware/",
            "-v", "--tb=short",
            "-k", "hardware"
        ], "Hardware Tests", timeout=300)

    def run_performance_mode(self) -> bool:
        """Performance and load testing."""
        print("⚡ Performance Mode: Load and performance validation")
        print("   Target: System performance under load")

        return self.run_command([
            "python", "-m", "pytest",
            "tests/performance/",
            "-v", "--tb=short",
            "--durations=0"  # Show all durations
        ], "Performance Tests", timeout=600)

    def run_custom_mode(self, test_path: Optional[str] = None, marker: Optional[str] = None) -> bool:
        """Custom test selection for specific debugging."""
        print("🎯 Custom Mode: Selective test execution")

        cmd = ["python", "-m", "pytest", "-v", "--tb=short"]

        if marker:
            cmd.extend(["-k", marker])
        elif test_path:
            cmd.append(test_path)
        else:
            cmd.append("tests/unit/")  # Default to unit tests

        return self.run_command(cmd, "Custom Test Selection", timeout=120)


def main():
    """Main entry point with argument parsing."""
    parser = argparse.ArgumentParser(
        description="Smart Test Runner for URC 2026",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python scripts/run_tests.py dev                    # Fast development feedback
  python scripts/run_tests.py pre-commit            # Quality gates
  python scripts/run_tests.py ci                     # Full CI/CD validation
  python scripts/run_tests.py hardware              # Hardware tests
  python scripts/run_tests.py custom tests/unit/    # Specific test path
  python scripts/run_tests.py custom --marker slow  # Tests with marker
        """
    )

    parser.add_argument(
        "mode",
        choices=["dev", "pre-commit", "ci", "hardware", "performance", "custom"],
        help="Test execution mode"
    )

    parser.add_argument(
        "path",
        nargs="?",
        help="Custom test path (for custom mode)"
    )

    parser.add_argument(
        "--marker", "-k",
        help="Pytest marker for custom mode"
    )

    args = parser.parse_args()

    runner = TestRunner()

    # Execute appropriate mode
    success = False
    if args.mode == "dev":
        success = runner.run_dev_mode()
    elif args.mode == "pre-commit":
        success = runner.run_pre_commit_mode()
    elif args.mode == "ci":
        success = runner.run_ci_mode()
    elif args.mode == "hardware":
        success = runner.run_hardware_mode()
    elif args.mode == "performance":
        success = runner.run_performance_mode()
    elif args.mode == "custom":
        success = runner.run_custom_mode(args.path, args.marker)

    # Report results
    total_time = time.time() - runner.start_time
    print(".1f"
    if success:
        print("🎉 All tests passed!"        sys.exit(0)
    else:
        print("❌ Some tests failed. Check output above."        sys.exit(1)


if __name__ == "__main__":
    main()

