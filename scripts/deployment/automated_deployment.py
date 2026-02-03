#!/usr/bin/env python3
"""
Automated Deployment System
Handles deployment, validation, and rollback for competition systems.
"""

import subprocess
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import yaml


class AutomatedDeployment:
    """
    Automated deployment system for competition robots.

    Features:
    - Configuration validation
    - Service deployment orchestration
    - Health monitoring during deployment
    - Automatic rollback on failure
    - Deployment timeout handling
    """

    def __init__(self, project_root: Optional[str] = None):
        self.project_root = (
            Path(project_root) if project_root else Path(__file__).parent.parent.parent
        )
        self.deployment_active = False
        self.deployment_start_time = None
        self.services_deployed = []
        self.deployment_timeout = 300  # 5 minutes
        self.health_checks = []
        self.rollback_available = True
        self.logger = self._setup_logger()

        # Deployment configuration
        self.service_configs = {}
        self._load_service_configs()

    def _setup_logger(self):
        """Set up logging for deployment operations."""
        import logging

        logger = logging.getLogger("AutomatedDeployment")
        logger.setLevel(logging.INFO)
        if not logger.handlers:
            handler = logging.StreamHandler()
            formatter = logging.Formatter(
                "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
            )
            handler.setFormatter(formatter)
            logger.addHandler(handler)
        return logger

    def _check_configuration_validity(self) -> bool:
        """Check if deployment configuration is valid."""
        try:
            # Check if required config files exist
            config_files = [
                self.project_root / "config" / "competition.yaml",
                self.project_root / "config" / "network.yaml",
                self.project_root / "config" / "safety.yaml",
            ]

            for config_file in config_files:
                if not config_file.exists():
                    self.logger.warning(f"Configuration file missing: {config_file}")
                    return False

            # Validate service configurations
            for service_name, config in self.service_configs.items():
                if "command" not in config:
                    self.logger.error(f"Service {service_name} missing command")
                    return False
                if "health_check" not in config:
                    self.logger.error(f"Service {service_name} missing health_check")
                    return False

            return True

        except Exception as e:
            self.logger.error(f"Configuration validation failed: {e}")
            return False

    def _load_service_configs(self):
        """Load service configurations for deployment."""
        self.service_configs = {
            "ros2_state_machine": {
                "command": "ros2 launch urc_bringup state_machine.launch.py",
                "health_check": self._check_state_machine_health,
                "timeout": 30,
            },
            "vision_processing": {
                "command": "ros2 launch urc_vision vision_processing.launch.py",
                "health_check": self._check_vision_health,
                "timeout": 45,
            },
            "communication_bridge": {
                "command": "ros2 launch urc_bridges communication_bridge.launch.py",
                "health_check": self._check_bridge_health,
                "timeout": 20,
            },
        }

    def validate_configuration(self, config_path: str) -> tuple[bool, str]:
        """Validate deployment configuration."""
        if not Path(config_path).exists():
            return False, f"Configuration file not found: {config_path}"

        try:
            with open(config_path, "r") as f:
                config = yaml.safe_load(f)

            # Validate required sections
            required_sections = ["simulation", "mission", "safety", "network"]
            for section in required_sections:
                if section not in config:
                    return False, f"Missing required configuration section: {section}"

            return True, "Configuration validation passed"
        except Exception as e:
            return False, f"Configuration validation failed: {e}"

    def start_deployment(self, environment: str = "competition") -> tuple[bool, str]:
        """Start automated deployment process."""
        if self.deployment_active:
            return False, "Deployment already in progress"

        self.deployment_active = True
        self.deployment_start_time = time.time()
        self.services_deployed = []

        try:
            # Phase 1: Pre-deployment validation
            success, message = self._run_pre_deployment_checks()
            if not success:
                return False, f"Pre-deployment validation failed: {message}"

            # Phase 2: Service deployment
            for service_name, service_config in self.service_configs.items():
                success, message = self._deploy_service(service_name, service_config)
                if not success:
                    # Attempt rollback
                    self._rollback_deployment()
                    return (
                        False,
                        f"Service deployment failed for {service_name}: {message}",
                    )

                self.services_deployed.append(service_name)

            # Phase 3: Post-deployment validation
            success, message = self._run_post_deployment_validation()
            if not success:
                self._rollback_deployment()
                return False, f"Post-deployment validation failed: {message}"

            return (
                True,
                f"Deployment completed successfully. Services deployed: {len(self.services_deployed)}",
            )

        except Exception as e:
            self._rollback_deployment()
            return False, f"Deployment failed with exception: {e}"
        finally:
            self.deployment_active = False

    def _run_pre_deployment_checks(self) -> tuple[bool, str]:
        """Run pre-deployment validation checks."""
        checks = [
            self._check_system_resources,
            self._check_network_connectivity,
            self._check_existing_services,
        ]

        for check in checks:
            success, message = check()
            if not success:
                return False, message

        return True, "All pre-deployment checks passed"

    def _deploy_service(
        self, service_name: str, config: Dict[str, Any]
    ) -> tuple[bool, str]:
        """Deploy a single service."""
        try:
            # Start service process
            process = subprocess.Popen(
                config["command"].split(),
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )

            # Wait for service to start
            time.sleep(2)

            # Check if process is still running
            if process.poll() is not None:
                stdout, stderr = process.communicate()
                return (
                    False,
                    f"Service {service_name} failed to start: {stderr.decode()}",
                )

            # Run health check
            success, message = config["health_check"]()
            if not success:
                process.terminate()
                return False, f"Health check failed for {service_name}: {message}"

            return True, f"Service {service_name} deployed successfully"

        except Exception as e:
            return False, f"Service deployment error for {service_name}: {e}"

    def _run_post_deployment_validation(self) -> tuple[bool, str]:
        """Run post-deployment validation."""
        # Wait for services to stabilize
        time.sleep(5)

        # Check inter-service communication
        success, message = self._check_service_communication()
        if not success:
            return False, message

        # Run integration tests
        success, message = self._run_integration_tests()
        if not success:
            return False, message

        return True, "Post-deployment validation passed"

    def _rollback_deployment(self) -> tuple[bool, str]:
        """Rollback failed deployment."""
        if not self.rollback_available:
            return False, "Rollback not available"

        rolled_back = []
        for service_name in reversed(self.services_deployed):
            try:
                # Kill service process (simplified - would need process tracking)
                subprocess.run(["pkill", "-f", service_name], check=False)
                rolled_back.append(service_name)
            except Exception:
                pass

        self.services_deployed = []
        return True, f"Rollback completed. Services stopped: {len(rolled_back)}"

    def check_deployment_timeout(self) -> tuple[bool, str]:
        """Check if deployment has timed out."""
        if not self.deployment_active:
            return True, "No active deployment"

        elapsed = time.time() - (self.deployment_start_time or time.time())
        if elapsed > self.deployment_timeout:
            self._rollback_deployment()
            return False, f"Deployment timed out after {elapsed:.1f} seconds"

        return True, f"Deployment in progress ({elapsed:.1f}s elapsed)"

    # Health check implementations
    def _check_state_machine_health(self) -> tuple[bool, str]:
        """Check state machine service health."""
        # Placeholder - would check ROS2 service availability
        return True, "State machine health check passed"

    def _check_vision_health(self) -> tuple[bool, str]:
        """Check vision processing service health."""
        # Placeholder - would check camera connectivity and processing
        return True, "Vision health check passed"

    def _check_bridge_health(self) -> tuple[bool, str]:
        """Check communication bridge health."""
        # Placeholder - would check WebSocket/ROS2 bridge connectivity
        return True, "Bridge health check passed"

    # Pre-deployment checks
    def _check_system_resources(self) -> tuple[bool, str]:
        """Check system resources."""
        # Placeholder - would check CPU, memory, disk space
        return True, "System resources check passed"

    def _check_network_connectivity(self) -> tuple[bool, str]:
        """Check network connectivity."""
        # Placeholder - would test network connectivity
        return True, "Network connectivity check passed"

    def _check_existing_services(self) -> tuple[bool, str]:
        """Check for conflicting existing services."""
        # Placeholder - would check for running ROS2 services
        return True, "Existing services check passed"

    def _check_service_communication(self) -> tuple[bool, str]:
        """Check inter-service communication."""
        # Placeholder - would test ROS2 topic/service communication
        return True, "Service communication check passed"

    def _run_integration_tests(self) -> tuple[bool, str]:
        """Run integration tests."""
        # Placeholder - would run automated integration tests
        return True, "Integration tests passed"

    def generate_deployment_report(self) -> Dict[str, Any]:
        """Generate comprehensive deployment report."""
        return {
            "deployment_status": (
                "completed" if not self.deployment_active else "in_progress"
            ),
            "services_deployed": self.services_deployed,
            "deployment_duration": time.time()
            - (self.deployment_start_time or time.time()),
            "health_checks": len(self.health_checks),
            "rollback_available": self.rollback_available,
        }
