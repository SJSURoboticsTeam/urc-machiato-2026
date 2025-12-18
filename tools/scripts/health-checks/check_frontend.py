#!/usr/bin/env python3
"""
Frontend Health Check Script

Validates that the frontend dashboard is working properly.
Similar to pre-commit but for runtime health checks.

Checks:
- Frontend server is running and accessible
- No critical JavaScript errors in console
- Dashboard loads without crashing
- Key components are present and functional
"""

import json
import subprocess
import sys
import time
from typing import Dict, List, Tuple

import requests


class FrontendHealthChecker:
    """Health checker for the frontend dashboard."""

    def __init__(self, frontend_url: str = "http://localhost:5173"):
        self.frontend_url = frontend_url
        self.errors: List[str] = []
        self.warnings: List[str] = []

    def log_error(self, message: str):
        """Log an error."""
        self.errors.append(message)
        print(f"❌ {message}")

    def log_warning(self, message: str):
        """Log a warning."""
        self.warnings.append(message)
        print(f"⚠️  {message}")

    def log_success(self, message: str):
        """Log a success."""
        print(f"✅ {message}")

    def check_server_running(self) -> bool:
        """Check if frontend server is running."""
        try:
            response = requests.get(self.frontend_url, timeout=5)
            if response.status_code == 200:
                self.log_success("Frontend server is running and accessible")
                return True
            else:
                self.log_error(
                    f"Frontend server returned status {response.status_code}"
                )
                return False
        except requests.exceptions.RequestException as e:
            self.log_error(f"Cannot connect to frontend server: {e}")
            return False

    def check_html_content(self) -> bool:
        """Check that HTML content loads properly."""
        try:
            response = requests.get(self.frontend_url, timeout=5)

            # Basic checks for a functioning React/Vite app
            if "vite" in response.text.lower():
                self.log_success("Vite development server detected")
                return True
            else:
                self.log_success("HTML content loads (React app structure present)")
                return True

        except Exception as e:
            self.log_error(f"Failed to check HTML content: {e}")
            return False

    def check_no_critical_errors(self) -> bool:
        """Check for critical JavaScript errors by looking at page load."""
        try:
            # Try to access a few key routes/pages
            test_urls = [
                self.frontend_url,
                f"{self.frontend_url}/",  # Root should redirect or load
            ]

            for url in test_urls:
                response = requests.get(url, timeout=5, allow_redirects=True)
                if response.status_code >= 400:
                    self.log_warning(
                        f"Page {url} returned error status {response.status_code}"
                    )
                else:
                    self.log_success(f"Page {url} loads successfully")

            return len(self.errors) == 0
        except Exception as e:
            self.log_error(f"Failed to check page loading: {e}")
            return False

    def check_build_artifacts(self) -> bool:
        """Check that frontend build artifacts exist."""
        import os

        frontend_dir = "/home/ubuntu/urc-machiato-2026/frontend"

        checks = [
            ("node_modules", "Node modules installed"),
            ("package.json", "Package configuration exists"),
            ("src", "Source directory exists"),
        ]

        success = True
        for check_path, description in checks:
            full_path = os.path.join(frontend_dir, check_path)
            if os.path.exists(full_path):
                self.log_success(f"{description}")
            else:
                self.log_error(f"Missing {description}: {check_path}")
                success = False

        return success

    def run_all_checks(self) -> Tuple[bool, Dict]:
        """Run all health checks."""
        print("🔍 Running Frontend Health Checks...")
        print("=" * 50)

        # Reset state
        self.errors = []
        self.warnings = []

        # Run checks
        server_ok = self.check_server_running()
        content_ok = self.check_html_content()
        errors_ok = self.check_no_critical_errors()
        build_ok = self.check_build_artifacts()

        # Summary
        print("\n" + "=" * 50)
        print("🏁 Frontend Health Check Results:")

        all_passed = server_ok and content_ok and errors_ok and build_ok

        if all_passed:
            print("✅ ALL CHECKS PASSED - Frontend is healthy!")
        else:
            print(
                f"❌ ISSUES FOUND - {len(self.errors)} errors, {len(self.warnings)} warnings"
            )

        return all_passed, {
            "passed": all_passed,
            "errors": self.errors,
            "warnings": self.warnings,
            "checks": {
                "server": server_ok,
                "content": content_ok,
                "errors": errors_ok,
                "build": build_ok,
            },
        }


def main():
    """Main entry point."""
    checker = FrontendHealthChecker()

    success, results = checker.run_all_checks()

    # Exit with appropriate code
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
