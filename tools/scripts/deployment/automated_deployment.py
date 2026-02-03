#!/usr/bin/env python3
"""
Automated Deployment System - Competition Ready
Simple, validated deployment with pre-flight checks.
"""

import os
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional


class AutomatedDeployment:
    """
    Simple automated deployment system for competition.

    Features:
    - Pre-deployment validation
    - Service startup with health checks
    - Configuration validation
    - Rollback capability
    - Status monitoring
    """

    def __init__(self, project_root: str = None):
        self.project_root = Path(project_root or os.getcwd())
        self.deployment_log = []
        self.services_started = []
        self.start_time = None

    def deploy_competition_system(self, validate_only: bool = False) -> bool:
        """
        Deploy the complete competition system.

        Steps:
        1. Pre-deployment validation
        2. Environment setup
        3. Service startup with health checks
        4. Post-deployment validation
        5. Status reporting
        """
        self.start_time = time.time()
        self._log("[IGNITE] Starting competition system deployment")

        try:
            # Step 1: Pre-deployment validation
            if not self._run_pre_deployment_checks():
                self._log("[FAIL] Pre-deployment checks failed")
                return False

            if validate_only:
                self._log("[PASS] Validation only - deployment checks passed")
                return True

            # Step 2: Environment setup
            if not self._setup_deployment_environment():
                self._log("[FAIL] Environment setup failed")
                return False

            # Step 3: Service deployment
            if not self._deploy_services():
                self._log("[FAIL] Service deployment failed")
                self._rollback_deployment()
                return False

            # Step 4: Post-deployment validation
            if not self._run_post_deployment_checks():
                self._log("[FAIL] Post-deployment checks failed")
                self._rollback_deployment()
                return False

            # Step 5: Final status
            deployment_time = time.time() - self.start_time
            self._log(f"Deployment completed in {deployment_time:.1f}s")
            self._log("[PARTY] COMPETITION SYSTEM DEPLOYMENT COMPLETE")
            self._log("System is ready for competition operation")

            return True

        except Exception as e:
            self._log(f"[FAIL] Deployment failed with error: {e}")
            self._rollback_deployment()
            return False

    def _run_pre_deployment_checks(self) -> bool:
        """Run pre-deployment validation checks."""
        self._log("[MAGNIFY] Running pre-deployment checks...")

        checks = [
            ("Hardware validation", self._check_hardware_readiness),
            ("Software dependencies", self._check_software_dependencies),
            ("Configuration validation", self._check_configuration_validity),
            ("Network readiness", self._check_network_readiness),
            ("Disk space", self._check_disk_space),
        ]

        passed = 0
        total = len(checks)

        for check_name, check_func in checks:
            self._log(f"  Checking {check_name}...")
            try:
                if check_func():
                    self._log(f"    [PASS] {check_name}: PASSED")
                    passed += 1
                else:
                    self._log(f"    [FAIL] {check_name}: FAILED")
            except Exception as e:
                self._log(f"    [FAIL] {check_name}: ERROR - {e}")

        self._log(f"Pre-deployment checks: {passed}/{total} passed")
        return passed == total

    def _setup_deployment_environment(self) -> bool:
        """Set up deployment environment."""
        self._log("[TOOL] Setting up deployment environment...")

        try:
            # Set environment variables
            os.environ["URC_ENV"] = "competition"
            os.environ["ROS_DOMAIN_ID"] = "42"

            # Source ROS2 environment
            ros_setup = "/opt/ros/humble/setup.bash"
            if os.path.exists(ros_setup):
                result = subprocess.run(
                    f"source {ros_setup} && env",
                    shell=True,
                    capture_output=True,
                    text=True,
                )
                if result.returncode == 0:
                    # Update environment with ROS2 variables
                    for line in result.stdout.split("\n"):
                        if "=" in line:
                            key, value = line.split("=", 1)
                            os.environ[key] = value

            # Create necessary directories
            log_dir = self.project_root / "logs"
            log_dir.mkdir(exist_ok=True)

            self._log("[PASS] Deployment environment ready")
            return True

        except Exception as e:
            self._log(f"[FAIL] Environment setup failed: {e}")
            return False

    def _deploy_services(self) -> bool:
        """Deploy system services with health checks."""
        self._log("[IGNITE] Deploying system services...")

        services = [
            {
                "name": "communication_redundancy_manager",
                "command": "python3 bridges/communication_redundancy_manager.py",
                "health_check": self._check_service_health,
                "timeout": 10,
            },
            {
                "name": "emergency_stop_system",
                "command": "python3 autonomy/code/safety_system/emergency_stop_system.py",
                "health_check": self._check_service_health,
                "timeout": 10,
            },
            {
                "name": "service_health_monitor",
                "command": "python3 scripts/monitoring/service_health_monitor.py",
                "health_check": self._check_service_health,
                "timeout": 15,
            },
            {
                "name": "system_monitor",
                "command": "python3 scripts/monitoring/system_monitor.py",
                "health_check": self._check_service_health,
                "timeout": 10,
            },
            {
                "name": "state_machine_bridge",
                "command": "python3 bridges/ros2_state_machine_bridge.py",
                "health_check": self._check_service_health,
                "timeout": 10,
            },
            {
                "name": "mission_bridge",
                "command": "python3 bridges/ros2_mission_bridge.py",
                "health_check": self._check_service_health,
                "timeout": 10,
            },
            {
                "name": "simulation_bridge",
                "command": "python3 bridges/dashboard_simulation_bridge.py",
                "health_check": self._check_service_health,
                "timeout": 15,
            },
        ]

        for service in services:
            if not self._start_service(service):
                return False

        self._log("[PASS] All services deployed successfully")
        return True

    def _start_service(self, service_config: Dict[str, Any]) -> bool:
        """Start a single service with health check."""
        name = service_config["name"]
        command = service_config["command"]
        health_check = service_config["health_check"]
        timeout = service_config["timeout"]

        self._log(f"  Starting {name}...")

        try:
            # Start service in background
            log_file = self.project_root / "logs" / f"{name}.log"
            with open(log_file, "w") as f:
                process = subprocess.Popen(
                    command.split(),
                    stdout=f,
                    stderr=f,
                    cwd=self.project_root,
                    env=os.environ.copy(),
                )

            self.services_started.append(
                {
                    "name": name,
                    "process": process,
                    "pid": process.pid,
                    "start_time": time.time(),
                }
            )

            # Wait for service to start
            time.sleep(2)

            # Health check with timeout
            health_ok = False
            start_check = time.time()

            while (time.time() - start_check) < timeout:
                if health_check(name):
                    health_ok = True
                    break
                time.sleep(1)

            if health_ok:
                self._log(f"    [PASS] {name}: STARTED (PID: {process.pid})")
                return True
            else:
                self._log(f"    [FAIL] {name}: HEALTH CHECK FAILED")
                self._stop_service(name)
                return False

        except Exception as e:
            self._log(f"    [FAIL] {name}: STARTUP ERROR - {e}")
            return False

    def _run_post_deployment_checks(self) -> bool:
        """Run post-deployment validation."""
        self._log("[MAGNIFY] Running post-deployment validation...")

        checks = [
            ("System health", self._check_overall_system_health),
            ("ROS2 communication", self._check_ros2_communication),
            ("Service integration", self._check_service_integration),
            ("Performance baseline", self._check_performance_baseline),
        ]

        passed = 0
        total = len(checks)

        for check_name, check_func in checks:
            self._log(f"  Validating {check_name}...")
            try:
                if check_func():
                    self._log(f"    [PASS] {check_name}: PASSED")
                    passed += 1
                else:
                    self._log(f"    [FAIL] {check_name}: FAILED")
            except Exception as e:
                self._log(f"    [FAIL] {check_name}: ERROR - {e}")

        self._log(f"Post-deployment validation: {passed}/{total} passed")
        return passed == total

    def _rollback_deployment(self):
        """Rollback deployment on failure."""
        self._log("[REFRESH] Rolling back deployment...")

        for service in reversed(self.services_started):
            self._stop_service(service["name"])

        self.services_started = []
        self._log("[PASS] Deployment rollback complete")

    def _stop_service(self, service_name: str):
        """Stop a specific service."""
        for service in self.services_started:
            if service["name"] == service_name:
                try:
                    service["process"].terminate()
                    service["process"].wait(timeout=5)
                    self._log(f"     Stopped {service_name}")
                except subprocess.TimeoutExpired:
                    service["process"].kill()
                    self._log(f"     Force killed {service_name}")
                except Exception as e:
                    self._log(f"    [FAIL] Error stopping {service_name}: {e}")
                break

    # Validation check implementations

    def _check_hardware_readiness(self) -> bool:
        """Check hardware is ready."""
        # Run hardware validation suite
        try:
            result = subprocess.run(
                [sys.executable, "tests/hardware/hardware_validation_suite.py"],
                capture_output=True,
                text=True,
                cwd=self.project_root,
                timeout=60,
            )

            return result.returncode == 0
        except subprocess.TimeoutExpired:
            return False

    def _check_software_dependencies(self) -> bool:
        """Check software dependencies."""
        try:
            # Check Python imports
            import psutil
            import rclpy
            import yaml

            # Check ROS2
            result = subprocess.run(
                ["ros2", "--version"], capture_output=True, timeout=10
            )
            return result.returncode == 0
        except:
            return False

    def _check_configuration_validity(self) -> bool:
        """Check configuration is valid."""
        try:
            sys.path.insert(0, str(self.project_root))
            from src.infrastructure.config import get_config_manager, get_config

            get_config_manager()
            get_config()
            return True
        except Exception:
            return False

    def _check_network_readiness(self) -> bool:
        """Check network is ready."""
        try:
            # Check internet connectivity
            result = subprocess.run(
                ["ping", "-c", "1", "8.8.8.8"], capture_output=True, timeout=5
            )
            return result.returncode == 0
        except:
            return False

    def _check_disk_space(self) -> bool:
        """Check sufficient disk space."""
        try:
            result = subprocess.run(
                ["df", "/"], capture_output=True, text=True, timeout=5
            )

            # Check if at least 5GB free
            for line in result.stdout.split("\n"):
                if "/" in line:
                    parts = line.split()
                    if len(parts) > 3:
                        available_kb = int(parts[3])
                        available_gb = available_kb / (1024 * 1024)
                        return available_gb > 5.0
            return False
        except:
            return False

    def _check_service_health(self, service_name: str) -> bool:
        """Check if a service is healthy."""
        try:
            # Simple process check
            result = subprocess.run(
                ["pgrep", "-f", service_name], capture_output=True, timeout=5
            )
            return result.returncode == 0
        except:
            return False

    def _check_overall_system_health(self) -> bool:
        """Check overall system health."""
        # Check that all critical services are running
        critical_services = [
            "state_machine_bridge",
            "mission_bridge",
            "simulation_bridge",
        ]
        return all(self._check_service_health(service) for service in critical_services)

    def _check_ros2_communication(self) -> bool:
        """Check ROS2 communication is working."""
        try:
            result = subprocess.run(
                ["ros2", "topic", "list"], capture_output=True, timeout=10
            )
            return result.returncode == 0 and len(result.stdout.strip()) > 0
        except:
            return False

    def _check_service_integration(self) -> bool:
        """Check services are properly integrated."""
        # Check for key ROS2 topics
        required_topics = [
            "/state_machine/current_state",
            "/mission/status",
            "/emergency/status",
        ]
        try:
            result = subprocess.run(
                ["ros2", "topic", "list"], capture_output=True, text=True, timeout=10
            )

            available_topics = result.stdout.strip().split("\n")
            return all(topic in available_topics for topic in required_topics)
        except:
            return False

    def _check_performance_baseline(self) -> bool:
        """Check system meets performance baseline."""
        # Quick performance check
        try:
            result = subprocess.run(
                [
                    sys.executable,
                    "tests/performance/competition_performance_profiler.py",
                ],
                capture_output=True,
                text=True,
                cwd=self.project_root,
                timeout=30,
            )

            # Just check if it runs without crashing
            return result.returncode == 0
        except subprocess.TimeoutExpired:
            return False

    def _log(self, message: str):
        """Log a deployment message."""
        timestamp = datetime.now().strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}"
        print(log_entry)
        self.deployment_log.append(log_entry)

    def get_deployment_report(self) -> Dict[str, Any]:
        """Get deployment report."""
        return {
            "success": len(
                [s for s in self.services_started if s["process"].poll() is None]
            )
            > 0,
            "services_started": len(self.services_started),
            "services_running": len(
                [s for s in self.services_started if s["process"].poll() is None]
            ),
            "deployment_time_s": (
                time.time() - self.start_time if self.start_time else 0
            ),
            "log": self.deployment_log,
        }


def main():
    """Main deployment function."""
    import argparse

    parser = argparse.ArgumentParser(description="Automated Competition Deployment")
    parser.add_argument(
        "--validate-only",
        action="store_true",
        help="Run validation checks only, do not deploy",
    )
    parser.add_argument("--project-root", default=None, help="Project root directory")

    args = parser.parse_args()

    deployer = AutomatedDeployment(args.project_root)
    success = deployer.deploy_competition_system(validate_only=args.validate_only)

    # Save deployment report
    report = deployer.get_deployment_report()
    report_file = f"deployment_report_{int(time.time())}.json"

    import json

    with open(report_file, "w") as f:
        json.dump(report, f, indent=2, default=str)

    print(f"\n Deployment report saved to: {report_file}")

    if success:
        print("[PARTY] Deployment successful!")
        sys.exit(0)
    else:
        print("[FAIL] Deployment failed!")
        sys.exit(1)


if __name__ == "__main__":
    main()
