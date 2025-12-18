#!/usr/bin/env python3
"""
Comprehensive Health Check Script

Similar to pre-commit but for runtime health validation.
Ensures both frontend and backend are working properly.

Usage:
  python3 scripts/health-checks/health_check.py [--frontend-only|--backend-only|--ci]
"""

import argparse
import subprocess
import sys
import time
from pathlib import Path


def run_frontend_checks() -> bool:
    """Run frontend health checks."""
    print("🌐 Running Frontend Health Checks...")
    try:
        result = subprocess.run(
            [sys.executable, "scripts/health-checks/check_frontend.py"],
            capture_output=True,
            text=True,
            cwd=Path(__file__).parent.parent.parent,
        )

        print(result.stdout)
        if result.stderr:
            print("STDERR:", result.stderr)

        return result.returncode == 0
    except Exception as e:
        print(f"❌ Failed to run frontend checks: {e}")
        return False


def run_backend_checks() -> bool:
    """Run backend health checks."""
    print("🔧 Running Backend Health Checks...")
    try:
        result = subprocess.run(
            [sys.executable, "scripts/health-checks/check_backend.py"],
            capture_output=True,
            text=True,
            cwd=Path(__file__).parent.parent.parent,
        )

        print(result.stdout)
        if result.stderr:
            print("STDERR:", result.stderr)

        return result.returncode == 0
    except Exception as e:
        print(f"❌ Failed to run backend checks: {e}")
        return False


def ensure_services_running() -> bool:
    """Ensure that dashboard services are running."""
    print("🚀 Ensuring Dashboard Services are Running...")

    try:
        # Check if services are already running
        frontend_check = subprocess.run(
            [
                "curl",
                "-s",
                "-o",
                "/dev/null",
                "-w",
                "%{http_code}",
                "http://localhost:5173",
            ],
            capture_output=True,
            text=True,
        )

        backend_check = subprocess.run(
            ["timeout", "2", "bash", "-c", "</dev/tcp/localhost/8766"],
            capture_output=True,
        )

        frontend_running = frontend_check.stdout.strip() == "200"
        backend_running = backend_check.returncode == 0

        if frontend_running and backend_running:
            print("✅ Services already running")
            return True

        print("⚠️  Services not running, starting them...")

        # Start the dashboard
        result = subprocess.run(
            [sys.executable, "start.py", "dev", "dashboard"],
            cwd=Path(__file__).parent.parent.parent,
            timeout=30,
        )

        if result.returncode != 0:
            print("❌ Failed to start dashboard services")
            return False

        # Wait for services to start
        print("⏳ Waiting for services to start...")
        time.sleep(10)

        # Recheck
        frontend_check = subprocess.run(
            [
                "curl",
                "-s",
                "-o",
                "/dev/null",
                "-w",
                "%{http_code}",
                "http://localhost:5173",
            ],
            capture_output=True,
            text=True,
        )

        backend_check = subprocess.run(
            ["timeout", "2", "bash", "-c", "</dev/tcp/localhost/8766"],
            capture_output=True,
        )

        frontend_running = frontend_check.stdout.strip() == "200"
        backend_running = backend_check.returncode == 0

        if frontend_running and backend_running:
            print("✅ Services started successfully")
            return True
        else:
            print("❌ Services failed to start properly")
            return False

    except Exception as e:
        print(f"❌ Error ensuring services: {e}")
        return False


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description="Comprehensive Health Check")
    parser.add_argument(
        "--frontend-only", action="store_true", help="Only check frontend"
    )
    parser.add_argument(
        "--backend-only", action="store_true", help="Only check backend"
    )
    parser.add_argument(
        "--ci",
        action="store_true",
        help="CI mode - ensure services are running before checks",
    )
    parser.add_argument(
        "--start-services", action="store_true", help="Start services if not running"
    )

    args = parser.parse_args()

    print("🏥 URC 2026 Health Check Suite")
    print("=" * 50)

    # Start services if requested or in CI mode
    if args.start_services or args.ci:
        if not ensure_services_running():
            print("❌ Cannot proceed without running services")
            sys.exit(1)

    # Run checks
    frontend_success = True
    backend_success = True

    if not args.backend_only:
        frontend_success = run_frontend_checks()
        print()

    if not args.frontend_only:
        backend_success = run_backend_checks()
        print()

    # Summary
    print("=" * 50)
    if frontend_success and backend_success:
        print("🎉 ALL HEALTH CHECKS PASSED!")
        print("Dashboard is ready for testing.")
        sys.exit(0)
    else:
        failed_components = []
        if not frontend_success:
            failed_components.append("Frontend")
        if not backend_success:
            failed_components.append("Backend")

        print(f"❌ HEALTH CHECKS FAILED: {', '.join(failed_components)}")
        print("Please fix the issues before proceeding.")
        sys.exit(1)


if __name__ == "__main__":
    main()
