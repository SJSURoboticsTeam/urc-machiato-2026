#!/usr/bin/env python3
"""
Test Runner Script for Autonomy System

Provides convenient commands to run different types of tests and generate reports.
"""

import argparse
import subprocess
import sys
from pathlib import Path


class TestRunner:
    """Test runner for the autonomy system."""

    def __init__(self):
        self.project_root = Path(__file__).parent.parent
        self.tests_dir = self.project_root / "tests"

    def run_unit_tests(self, coverage=True, verbose=True):
        """Run unit tests."""
        cmd = ["python3", "-m", "pytest", "tests/unit/"]

        if verbose:
            cmd.append("-v")
        if coverage:
            cmd.extend(["--cov=Autonomy", "--cov-report=term-missing"])

        return self._run_command(cmd, "Unit Tests")

    def run_integration_tests(self, coverage=True, verbose=True):
        """Run integration tests."""
        cmd = ["python3", "-m", "pytest", "tests/integration/", "-m", "integration"]

        if verbose:
            cmd.append("-v")
        if coverage:
            cmd.extend(["--cov=Autonomy", "--cov-report=term-missing"])

        return self._run_command(cmd, "Integration Tests")

    def run_system_tests(self, coverage=True, verbose=True):
        """Run system tests."""
        cmd = ["python3", "-m", "pytest", "tests/system/", "-m", "system"]

        if verbose:
            cmd.append("-v")
        if coverage:
            cmd.extend(["--cov=Autonomy", "--cov-report=term-missing"])

        return self._run_command(cmd, "System Tests")

    def run_performance_tests(self, verbose=True):
        """Run performance tests."""
        cmd = ["python3", "-m", "pytest", "tests/performance/", "-m", "performance"]

        if verbose:
            cmd.append("-v")

        return self._run_command(cmd, "Performance Tests")

    def run_safety_tests(self, verbose=True):
        """Run safety-critical tests."""
        cmd = ["python3", "-m", "pytest", "-m", "safety"]

        if verbose:
            cmd.append("-v")

        return self._run_command(cmd, "Safety Tests")

    def run_aoi_tests(self, coverage=True, verbose=True):
        """Run automated AoI tests."""
        print("[EXPERIMENT] Running Automated AoI Tests...")
        print("This will spin up the system, run tests, and spin down automatically")

        cmd = ["python3", "tests/automated_test_platform.py", "aoi_tests"]

        if verbose:
            cmd.append("--ci-mode")

        return self._run_command(cmd, "AoI Tests")

    def run_all_automated_tests(self, coverage=True, verbose=True):
        """Run all tests using automated platform."""
        print(" Running Complete Automated Test Suite...")
        print("This will run all test suites with full lifecycle management")

        cmd = ["python3", "tests/automated_test_platform.py", "all"]

        if verbose:
            cmd.append("--ci-mode")

        return self._run_command(cmd, "Automated Test Suite")

    def run_all_tests(self, coverage=True, verbose=True):
        """Run all tests with coverage."""
        cmd = ["python3", "-m", "pytest"]

        if verbose:
            cmd.append("-v")
        if coverage:
            cmd.extend(
                [
                    "--cov=Autonomy",
                    "--cov-report=html:output/reports/coverage_html",
                    "--cov-report=xml:output/reports/coverage.xml",
                    "--cov-report=term-missing",
                    "--junitxml=output/reports/junit.xml",
                ]
            )

        return self._run_command(cmd, "All Tests")

    def generate_reports(self):
        """Generate test reports."""
        print("[GRAPH] Generating Test Reports...")

        # Note: Report generation script has been deprecated
        # Test reports are now generated directly to output/reports/
        print("[INFO] Test reports are generated directly to output/reports/")
        print("   - output/reports/coverage_html/ (HTML coverage)")
        print("   - output/reports/coverage.xml (XML coverage)")
        print("   - output/reports/junit.xml (JUnit results)")

        return 0

    def check_test_status(self):
        """Check current test status."""
        print("[CLIPBOARD] Current Test Status")
        print("=" * 50)

        # Check if test directories exist
        unit_tests = (self.tests_dir / "unit").exists()
        integration_tests = (self.tests_dir / "integration").exists()
        system_tests = (self.tests_dir / "system").exists()
        fixtures = (self.tests_dir / "fixtures").exists()

        print(f"Unit Tests Directory: {'[PASS]' if unit_tests else '[FAIL]'}")
        print(
            f"Integration Tests Directory: {'[PASS]' if integration_tests else '[FAIL]'}"
        )
        print(f"System Tests Directory: {'[PASS]' if system_tests else '[FAIL]'}")
        print(f"Test Fixtures: {'[PASS]' if fixtures else '[FAIL]'}")

        # Check pytest configuration
        pytest_ini = self.tests_dir / "pytest.ini"
        conftest = self.tests_dir / "conftest.py"

        print(f"Pytest Config: {'[PASS]' if pytest_ini.exists() else '[FAIL]'}")
        print(
            f"Test Fixtures (conftest.py): {'[PASS]' if conftest.exists() else '[FAIL]'}"
        )

        # Count test files
        test_files = list(self.tests_dir.rglob("test_*.py"))
        print(f"Test Files Found: {len(test_files)}")

        for test_file in test_files[:5]:  # Show first 5
            print(f"  - {test_file.relative_to(self.project_root)}")

        if len(test_files) > 5:
            print(f"  ... and {len(test_files) - 5} more")

        print("\n Next Steps:")
        print("1. Run automated AoI tests: python tests/run_tests.py --aoi")
        print("2. Run complete automated suite: python tests/run_tests.py --automated")
        print("3. Run traditional unit tests: python tests/run_tests.py --unit")
        print("4. Run all tests: python tests/run_tests.py --all")
        print("5. Generate reports: python tests/run_tests.py --reports")

        return len(test_files) > 0

    def _run_command(self, cmd, test_type):
        """Run a command and return exit code."""
        print(f"[IGNITE] Running {test_type}...")
        print(f"Command: {' '.join(cmd)}")
        print("-" * 50)

        try:
            result = subprocess.run(
                cmd,
                cwd=self.project_root,
                capture_output=False,  # Show output live
                text=True,
            )
            print("-" * 50)
            if result.returncode == 0:
                print(f"[PASS] {test_type} PASSED")
            else:
                print(f"[FAIL] {test_type} FAILED (exit code: {result.returncode})")
            return result.returncode
        except Exception as e:
            print(f"[FAIL] Error running {test_type}: {e}")
            return 1


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description="Autonomy System Test Runner")
    parser.add_argument("--unit", action="store_true", help="Run unit tests")
    parser.add_argument(
        "--integration", action="store_true", help="Run integration tests"
    )
    parser.add_argument("--system", action="store_true", help="Run system tests")
    parser.add_argument(
        "--performance", action="store_true", help="Run performance tests"
    )
    parser.add_argument("--safety", action="store_true", help="Run safety tests")
    parser.add_argument("--aoi", action="store_true", help="Run automated AoI tests")
    parser.add_argument(
        "--automated", action="store_true", help="Run complete automated test suite"
    )
    parser.add_argument("--all", action="store_true", help="Run all tests")
    parser.add_argument("--reports", action="store_true", help="Generate test reports")
    parser.add_argument("--status", action="store_true", help="Check test status")
    parser.add_argument(
        "--no-coverage", action="store_true", help="Skip coverage reporting"
    )
    parser.add_argument("--quiet", action="store_true", help="Quiet output")

    args = parser.parse_args()

    runner = TestRunner()
    verbose = not args.quiet
    coverage = not args.no_coverage

    # Handle different test modes
    if args.status:
        return runner.check_test_status()
    elif args.reports:
        return runner.generate_reports() == 0
    elif args.unit:
        return runner.run_unit_tests(coverage, verbose) == 0
    elif args.integration:
        return runner.run_integration_tests(coverage, verbose) == 0
    elif args.system:
        return runner.run_system_tests(coverage, verbose) == 0
    elif args.performance:
        return runner.run_performance_tests(verbose) == 0
    elif args.safety:
        return runner.run_safety_tests(verbose) == 0
    elif args.aoi:
        return runner.run_aoi_tests(coverage, verbose) == 0
    elif args.automated:
        return runner.run_all_automated_tests(coverage, verbose) == 0
    elif args.all:
        success = runner.run_all_tests(coverage, verbose) == 0
        if success:
            runner.generate_reports()
        return success
    else:
        # Default: show status
        print("[EXPERIMENT] Autonomy System Test Runner")
        print("=" * 40)
        print("Available commands:")
        print("  --status          Check current test infrastructure status")
        print("  --unit           Run unit tests")
        print("  --integration    Run integration tests")
        print("  --system         Run system tests")
        print("  --performance    Run performance tests")
        print("  --safety         Run safety-critical tests")
        print("  --aoi            Run automated AoI tests (spin-up → test → spin-down)")
        print("  --automated      Run complete automated test suite")
        print("  --all            Run all tests with coverage")
        print("  --reports        Generate test reports")
        print("  --no-coverage    Skip coverage reporting")
        print("  --quiet          Quiet output")
        print()
        print("Examples:")
        print("  python tests/run_tests.py --status")
        print("  python tests/run_tests.py --unit")
        print("  python tests/run_tests.py --aoi              # Automated AoI testing")
        print(
            "  python tests/run_tests.py --automated        # Complete automated suite"
        )
        print("  python tests/run_tests.py --all --reports")
        return runner.check_test_status()


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
