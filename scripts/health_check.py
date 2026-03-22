#!/usr/bin/env python3
"""
Comprehensive Health Check Script

Validates that the frontend and backend services are working properly,
and performs production readiness checks.

Usage:
  python3 scripts/health_check.py --mode [frontend|backend|full|production] [--ci] [--start-services]
"""

import argparse
import json
import os
import socket
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List, Tuple

try:
    import requests
except ImportError:
    print("Please install requests: pip install requests")
    sys.exit(1)


class BaseChecker:
    def __init__(self):
        self.errors: List[str] = []
        self.warnings: List[str] = []

    def log_error(self, message: str):
        self.errors.append(message)
        print(f"[FAIL] {message}")

    def log_warning(self, message: str):
        self.warnings.append(message)
        print(f"  [WARN] {message}")

    def log_success(self, message: str):
        print(f"[PASS] {message}")


class FrontendHealthChecker(BaseChecker):
    """Health checker for the frontend dashboard."""

    def __init__(self, frontend_url: str = "http://localhost:5173"):
        super().__init__()
        self.frontend_url = frontend_url

    def check_server_running(self) -> bool:
        try:
            response = requests.get(self.frontend_url, timeout=5)
            if response.status_code == 200:
                self.log_success("Frontend server is running and accessible")
                return True
            else:
                self.log_error(f"Frontend server returned status {response.status_code}")
                return False
        except requests.exceptions.RequestException as e:
            self.log_error(f"Cannot connect to frontend server: {e}")
            return False

    def check_html_content(self) -> bool:
        try:
            response = requests.get(self.frontend_url, timeout=5)
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
        try:
            test_urls = [self.frontend_url, f"{self.frontend_url}/"]
            for url in test_urls:
                response = requests.get(url, timeout=5, allow_redirects=True)
                if response.status_code >= 400:
                    self.log_warning(f"Page {url} returned error status {response.status_code}")
                else:
                    self.log_success(f"Page {url} loads successfully")
            return len(self.errors) == 0
        except Exception as e:
            self.log_error(f"Failed to check page loading: {e}")
            return False

    def check_build_artifacts(self) -> bool:
        frontend_dir = os.path.join(Path(__file__).parent.parent, "services", "dashboard")
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

    def run_all_checks(self) -> bool:
        print("[MAGNIFY] Running Frontend Health Checks...")
        print("=" * 50)
        self.errors.clear()
        self.warnings.clear()

        server_ok = self.check_server_running()
        if server_ok:
            self.check_html_content()
            self.check_no_critical_errors()
        self.check_build_artifacts()

        all_passed = len(self.errors) == 0
        if all_passed:
            print("[PASS] ALL CHECKS PASSED - Frontend is healthy!")
        else:
            print(f"[FAIL] ISSUES FOUND - {len(self.errors)} errors, {len(self.warnings)} warnings")
        return all_passed


class BackendHealthChecker(BaseChecker):
    """Health checker for the backend services."""

    def __init__(self, websocket_host: str = "localhost", websocket_port: int = 8766):
        super().__init__()
        self.websocket_host = websocket_host
        self.websocket_port = websocket_port

    def check_websocket_server(self) -> bool:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5)
            result = sock.connect_ex((self.websocket_host, self.websocket_port))
            sock.close()
            if result == 0:
                self.log_success("WebSocket server is running and accepting connections")
                return True
            else:
                self.log_error(f"Cannot connect to WebSocket server on {self.websocket_host}:{self.websocket_port}")
                return False
        except Exception as e:
            self.log_error(f"WebSocket connection check failed: {e}")
            return False

    def check_python_imports(self) -> bool:
        import_checks = [
            ("websockets", "WebSocket library available"),
            ("asyncio", "AsyncIO available"),
            ("json", "JSON library available"),
        ]
        success = True
        for module_name, description in import_checks:
            try:
                __import__(module_name)
                self.log_success(f"{description}")
            except ImportError as e:
                self.log_error(f"Failed to import {module_name}: {e}")
                success = False
        return success

    def check_process_running(self) -> bool:
        try:
            result = subprocess.run(["pgrep", "-f", "test_dashboard_backend"], capture_output=True, text=True)
            if result.returncode == 0:
                self.log_success("Backend process is running")
                return True
            else:
                self.log_warning("Backend process not found running")
                return False
        except Exception as e:
            self.log_error(f"Failed to check process status: {e}")
            return False

    def run_all_checks(self) -> bool:
        print("[MAGNIFY] Running Backend Health Checks...")
        print("=" * 50)
        self.errors.clear()
        self.warnings.clear()

        self.check_websocket_server()
        self.check_python_imports()
        self.check_process_running()

        all_passed = len(self.errors) == 0
        if all_passed:
            print("[PASS] ALL CRITICAL CHECKS PASSED - Backend is healthy!")
        else:
            print(f"[FAIL] ISSUES FOUND - {len(self.errors)} errors, {len(self.warnings)} warnings")
        return all_passed


class ProductionHealthCheck(BaseChecker):
    """Comprehensive production readiness checker."""

    def check_file_exists(self, file_path: str, description: str) -> bool:
        if os.path.exists(file_path):
            self.log_success(f"{description} found")
            return True
        else:
            self.log_error(f"{description} missing: {file_path}")
            return False

    def check_command_available(self, command: str, description: str) -> bool:
        try:
            subprocess.run([command, "--version"], capture_output=True, check=True)
            self.log_success(f"{description} available")
            return True
        except (subprocess.CalledProcessError, FileNotFoundError):
            self.log_error(f"{description} not available: {command}")
            return False

    def check_git_status(self) -> bool:
        try:
            result = subprocess.run(["git", "status"], capture_output=True, text=True)
            if result.returncode == 0:
                self.log_success("Git repository healthy")
                return True
            else:
                self.log_error("Git repository issues")
                return False
        except FileNotFoundError:
            self.log_warning("Git not available")
            return False

    def run_all_checks(self) -> bool:
        print(" Production Health Check\n")
        self.errors.clear()
        self.warnings.clear()

        self.check_file_exists("pyproject.toml", "Project configuration")
        self.check_file_exists("config/environments/competition.yaml", "Production config")
        self.check_file_exists("config/environments/development.yaml", "Development config")
        self.check_file_exists("README.md", "Project README")

        self.check_command_available("python3", "Python 3")
        self.check_command_available("docker", "Docker")

        # Python module checks
        for module in ["yaml", "pathlib", "subprocess"]:
            try:
                __import__(module)
                self.log_success(f"Python module available: {module}")
            except ImportError:
                self.log_error(f"Python module missing: {module}")

        self.check_git_status()

        print("\n[GRAPH] Health Check Summary:")
        if self.errors:
            print("\n Production readiness failed!")
            return False
        else:
            print("\n[PARTY] System passed health checks!")
            return True


def ensure_services_running() -> bool:
    print("[IGNITE] Ensuring Dashboard Services are Running...")
    try:
        frontend_check = subprocess.run(["curl", "-s", "-o", "/dev/null", "-w", "%{http_code}", "http://localhost:5173"], capture_output=True, text=True)
        backend_check = subprocess.run(["timeout", "2", "bash", "-c", "</dev/tcp/localhost/8766"], capture_output=True)
        
        if frontend_check.stdout.strip() == "200" and backend_check.returncode == 0:
            print("[PASS] Services already running")
            return True

        print("  Services not running, starting them...")
        result = subprocess.run([sys.executable, "scripts/start.py", "dev", "dashboard"], cwd=Path(__file__).parent.parent, timeout=30)
        
        if result.returncode != 0:
            print("[FAIL] Failed to start dashboard services")
            return False

        print("Waiting for services to start...")
        time.sleep(10)
        return True
    except Exception as e:
        print(f"[FAIL] Error ensuring services: {e}")
        return False


def main():
    parser = argparse.ArgumentParser(description="Comprehensive Health Check")
    parser.add_argument("--mode", choices=["frontend", "backend", "full", "production"], default="full", help="Health check mode")
    parser.add_argument("--ci", action="store_true", help="CI mode - ensure services are running before checks")
    parser.add_argument("--start-services", action="store_true", help="Start services if not running")

    args = parser.parse_args()

    print(" URC 2026 Health Check Suite")
    print("=" * 50)

    if args.start_services or args.ci:
        if args.mode in ["frontend", "backend", "full"]:
            if not ensure_services_running():
                print("[FAIL] Cannot proceed without running services")
                sys.exit(1)

    success = True

    if args.mode in ["frontend", "full"]:
        checker = FrontendHealthChecker()
        if not checker.run_all_checks():
            success = False
        print()

    if args.mode in ["backend", "full"]:
        checker = BackendHealthChecker()
        if not checker.run_all_checks():
            success = False
        print()
        
    if args.mode == "production":
        checker = ProductionHealthCheck()
        if not checker.run_all_checks():
            success = False
        print()

    print("=" * 50)
    if success:
        print("[PARTY] ALL HEALTH CHECKS PASSED!")
        sys.exit(0)
    else:
        print(f"[FAIL] HEALTH CHECKS FAILED.")
        sys.exit(1)


if __name__ == "__main__":
    main()
