#!/usr/bin/env python3
"""
Dashboard Validation Script

Quick validation that the dashboard is working properly.
Can be used locally or in CI/CD pipelines.

Usage:
  python3 scripts/validate_dashboard.py [--quick|--full]
"""

import argparse
import subprocess
import sys
import time
from pathlib import Path


def run_quick_validation():
    """Run quick validation checks."""
    print("[LIGHTNING] Quick Dashboard Validation")
    print("=" * 40)

    # Check if services are accessible
    checks = [
        (
            "Frontend",
            "curl -s -o /dev/null -w '%{http_code}' http://localhost:5173",
            "200",
        ),
        (
            "Backend",
            "timeout 2 bash -c '</dev/tcp/localhost/8766' 2>/dev/null && echo 'OK' || echo 'FAIL'",
            "OK",
        ),
    ]

    all_passed = True
    for name, command, expected in checks:
        try:
            result = subprocess.run(
                command, shell=True, capture_output=True, text=True, timeout=5
            )
            output = result.stdout.strip()
            if expected in output:
                print(f"[PASS] {name}: OK")
            else:
                print(f"[FAIL] {name}: FAILED (got '{output}', expected '{expected}')")
                all_passed = False
        except Exception as e:
            print(f"[FAIL] {name}: ERROR - {e}")
            all_passed = False

    return all_passed


def run_full_validation():
    """Run comprehensive validation."""
    print("[MAGNIFY] Full Dashboard Validation")
    print("=" * 40)

    # Run the comprehensive health check
    try:
        result = subprocess.run(
            [
                sys.executable,
                "scripts/health-checks/health_check.py",
                "--ci",
                "--start-services",
            ],
            cwd=Path(__file__).parent.parent,
            timeout=120,
        )

        return result.returncode == 0
    except subprocess.TimeoutExpired:
        print("[FAIL] Validation timed out")
        return False
    except Exception as e:
        print(f"[FAIL] Validation failed: {e}")
        return False


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description="Dashboard Validation")
    parser.add_argument(
        "--quick", action="store_true", help="Run quick validation only"
    )
    parser.add_argument(
        "--full",
        action="store_true",
        default=True,
        help="Run full validation (default)",
    )

    args = parser.parse_args()

    success = False

    if args.quick:
        success = run_quick_validation()
    else:
        success = run_full_validation()

    print("\n" + "=" * 40)
    if success:
        print("[PARTY] Dashboard validation PASSED!")
        sys.exit(0)
    else:
        print("[FAIL] Dashboard validation FAILED!")
        print("Run with --full for detailed diagnostics")
        sys.exit(1)


if __name__ == "__main__":
    main()
