#!/usr/bin/env python3
"""
Migration Manager - Smooth Transition from Mock to Hardware Components

Manages the gradual migration from simulation/mock components to real hardware
during integration testing. Ensures compatibility and provides rollback capabilities.

Author: URC 2026 Autonomy Team
"""

import logging
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Dict, List, Optional

logger = logging.getLogger(__name__)


class MigrationPhase(Enum):
    """Defined migration phases for hardware integration."""

    MOCK_ONLY = "mock_only"
    CAN_BUS_INTEGRATION = "can_bus_integration"
    DRIVE_SYSTEM_INTEGRATION = "drive_system_integration"
    ARM_INTEGRATION = "arm_integration"
    SCIENCE_PAYLOAD_INTEGRATION = "science_payload_integration"
    SENSORS_INTEGRATION = "sensors_integration"
    FULL_HARDWARE = "full_hardware"


@dataclass
class MigrationStep:
    """Individual migration step."""

    component_name: str
    from_mode: str  # 'mock' or 'hardware'
    to_mode: str  # 'mock' or 'hardware'
    description: str
    prerequisites: List[str] = field(
        default_factory=list
    )  # Other steps that must complete first
    validation_tests: List[str] = field(
        default_factory=list
    )  # Tests to run after migration
    rollback_supported: bool = True
    critical: bool = False  # If True, failure blocks further migration


@dataclass
class MigrationResult:
    """Result of a migration operation."""

    success: bool
    step: Optional[MigrationStep] = None
    error_message: Optional[str] = None
    validation_results: Dict[str, Any] = field(default_factory=dict)
    timestamp: float = field(default_factory=time.time)
    duration: float = 0.0


class MigrationManager:
    """
    Manages migration from mock to hardware components.

    Provides phased migration with validation, rollback, and monitoring
    capabilities for safe hardware integration.
    """

    def __init__(self):
        """Initialize migration manager."""
        self.current_phase: Optional[MigrationPhase] = MigrationPhase.MOCK_ONLY
        self.migration_history: List[MigrationResult] = []
        self.component_states: Dict[str, str] = {}  # component -> mode
        self.validation_callbacks: Dict[str, Callable] = {}

        # Define migration phases and their configurations
        self._define_migration_phases()

        logger.info("Migration manager initialized")

    def _define_migration_phases(self):
        """Define the migration phases and their component configurations."""
        self.phase_configurations = {
            MigrationPhase.MOCK_ONLY: {
                "can_bus": "mock",
                "motor_controller": "mock",
                "drive_system": "mock",
                "robotic_arm": "mock",
                "science_payload": "mock",
                "power_system": "mock",
                "sensor_imu": "mock",
                "sensor_gps": "mock",
            },
            MigrationPhase.CAN_BUS_INTEGRATION: {
                "can_bus": "hardware",
                "motor_controller": "mock",
                "drive_system": "mock",
                "robotic_arm": "mock",
                "science_payload": "mock",
                "power_system": "mock",
                "sensor_imu": "mock",
                "sensor_gps": "mock",
            },
            MigrationPhase.DRIVE_SYSTEM_INTEGRATION: {
                "can_bus": "hardware",
                "motor_controller": "hardware",
                "drive_system": "hardware",
                "robotic_arm": "mock",
                "science_payload": "mock",
                "power_system": "mock",
                "sensor_imu": "mock",
                "sensor_gps": "mock",
            },
            MigrationPhase.ARM_INTEGRATION: {
                "can_bus": "hardware",
                "motor_controller": "hardware",
                "drive_system": "hardware",
                "robotic_arm": "hardware",
                "science_payload": "mock",
                "power_system": "mock",
                "sensor_imu": "mock",
                "sensor_gps": "mock",
            },
            MigrationPhase.SCIENCE_PAYLOAD_INTEGRATION: {
                "can_bus": "hardware",
                "motor_controller": "hardware",
                "drive_system": "hardware",
                "robotic_arm": "hardware",
                "science_payload": "hardware",
                "power_system": "mock",
                "sensor_imu": "mock",
                "sensor_gps": "mock",
            },
            MigrationPhase.SENSORS_INTEGRATION: {
                "can_bus": "hardware",
                "motor_controller": "hardware",
                "drive_system": "hardware",
                "robotic_arm": "hardware",
                "science_payload": "hardware",
                "power_system": "hardware",
                "sensor_imu": "hardware",
                "sensor_gps": "hardware",
            },
            MigrationPhase.FULL_HARDWARE: {
                "can_bus": "hardware",
                "motor_controller": "hardware",
                "drive_system": "hardware",
                "robotic_arm": "hardware",
                "science_payload": "hardware",
                "power_system": "hardware",
                "sensor_imu": "hardware",
                "sensor_gps": "hardware",
            },
        }

    def migrate_to_phase(
        self, target_phase: MigrationPhase, validate: bool = True
    ) -> MigrationResult:
        """
        Migrate system to a specific integration phase.

        Args:
            target_phase: Target migration phase
            validate: Whether to run validation after migration

        Returns:
            MigrationResult with success/failure status
        """
        start_time = time.time()

        logger.info(f"Starting migration to phase: {target_phase.value}")

        # Validate that target phase is defined
        if target_phase not in self.phase_configurations:
            error_msg = f"Unknown migration phase: {target_phase}"
            logger.error(error_msg)
            return MigrationResult(
                success=False,
                error_message=error_msg,
                duration=time.time() - start_time,
            )

        # Get target configuration
        target_config = self.phase_configurations[target_phase]

        # Check if we're already at this phase
        if self._is_current_configuration(target_config):
            logger.info(f"Already at phase {target_phase.value}")
            return MigrationResult(success=True, duration=time.time() - start_time)

        # Generate migration steps
        migration_steps = self._generate_migration_steps(target_config)

        # Execute migration steps
        success = True
        failed_step = None
        validation_results = {}

        for step in migration_steps:
            logger.info(f"Executing migration step: {step.description}")

            step_result = self._execute_migration_step(step)
            self.migration_history.append(step_result)

            if not step_result.success:
                logger.error(f"Migration step failed: {step.description}")
                logger.error(f"Error: {step_result.error_message}")
                success = False
                failed_step = step

                # Stop migration if critical step failed
                if step.critical:
                    break

                # Continue with non-critical steps
                continue

            # Run validation if requested
            if validate and step.validation_tests:
                validation_results[step.component_name] = self._run_validation_tests(
                    step.component_name, step.validation_tests
                )

        # Update current phase if migration successful
        if success:
            self.current_phase = target_phase
            logger.info(f"[PASS] Successfully migrated to phase: {target_phase.value}")
        else:
            logger.error(
                f"[FAIL] Migration to {target_phase.value} failed at step: {failed_step.description if failed_step else 'unknown'}"
            )

        return MigrationResult(
            success=success,
            step=failed_step,
            error_message=failed_step.error_message if failed_step else None,
            validation_results=validation_results,
            duration=time.time() - start_time,
        )

    def incremental_migration(
        self, steps: List[MigrationStep]
    ) -> List[MigrationResult]:
        """
        Perform incremental migration with specific steps.

        Args:
            steps: List of migration steps to execute

        Returns:
            List of MigrationResult for each step
        """
        results = []

        for step in steps:
            logger.info(f"Executing incremental migration: {step.description}")

            start_time = time.time()
            result = self._execute_migration_step(step)

            result.duration = time.time() - start_time
            results.append(result)
            self.migration_history.append(result)

            if not result.success and step.critical:
                logger.error(f"Critical migration step failed: {step.description}")
                break

        return results

    def rollback_to_phase(self, target_phase: MigrationPhase) -> MigrationResult:
        """
        Rollback to a previous migration phase.

        Args:
            target_phase: Phase to rollback to

        Returns:
            MigrationResult with rollback status
        """
        start_time = time.time()

        logger.info(f"Rolling back to phase: {target_phase.value}")

        # Get target configuration
        target_config = self.phase_configurations[target_phase]

        # Generate rollback steps (reverse of migration)
        rollback_steps = self._generate_rollback_steps(target_config)

        # Execute rollback
        success = True
        for step in rollback_steps:
            step_result = self._execute_migration_step(step)
            if not step_result.success:
                success = False
                logger.error(f"Rollback step failed: {step.description}")
                break

        if success:
            self.current_phase = target_phase
            logger.info(
                f"[PASS] Successfully rolled back to phase: {target_phase.value}"
            )

        return MigrationResult(success=success, duration=time.time() - start_time)

    def register_validation_callback(
        self, component_name: str, callback: Callable[[str, Dict[str, Any]], bool]
    ):
        """
        Register validation callback for a component.

        Args:
            callback: Function that takes (component_name, validation_results) and returns bool
        """
        self.validation_callbacks[component_name] = callback

    def get_current_phase(self) -> Optional[MigrationPhase]:
        """Get current migration phase."""
        return self.current_phase

    def get_migration_history(self) -> List[MigrationResult]:
        """Get migration operation history."""
        return self.migration_history.copy()

    def get_component_states(self) -> Dict[str, str]:
        """Get current component states."""
        return self.component_states.copy()

    def get_available_phases(self) -> List[MigrationPhase]:
        """Get all available migration phases."""
        return list(self.phase_configurations.keys())

    def _generate_migration_steps(
        self, target_config: Dict[str, str]
    ) -> List[MigrationStep]:
        """Generate migration steps to reach target configuration."""
        steps = []

        for component, target_mode in target_config.items():
            current_mode = self.component_states.get(component, "mock")

            if current_mode != target_mode:
                step = MigrationStep(
                    component_name=component,
                    from_mode=current_mode,
                    to_mode=target_mode,
                    description=f"Switch {component} from {current_mode} to {target_mode}",
                    validation_tests=[f"test_{component}_basic_functionality"],
                )

                # Mark critical components
                if component in ["can_bus", "safety_system"]:
                    step.critical = True

                steps.append(step)

        return steps

    def _generate_rollback_steps(
        self, target_config: Dict[str, str]
    ) -> List[MigrationStep]:
        """Generate rollback steps to reach target configuration."""
        # Rollback is just migration in reverse (hardware -> mock)
        rollback_config = {comp: "mock" for comp in target_config.keys()}
        return self._generate_migration_steps(rollback_config)

    def _execute_migration_step(self, step: MigrationStep) -> MigrationResult:
        """Execute a single migration step."""
        start_time = time.time()

        try:
            # Import hardware abstraction factory
            from hardware_abstraction.hardware_interface_factory import (
                HardwareInterfaceFactory,
            )

            # Validate prerequisites
            if not self._check_prerequisites(step):
                return MigrationResult(
                    success=False,
                    step=step,
                    error_message=f"Prerequisites not met for {step.component_name}",
                    duration=time.time() - start_time,
                )

            # Switch component mode
            config_update = {step.component_name: step.to_mode}
            validation_errors = HardwareInterfaceFactory.validate_configuration(
                config_update
            )

            if validation_errors:
                return MigrationResult(
                    success=False,
                    step=step,
                    error_message=f"Configuration validation failed: {validation_errors}",
                    duration=time.time() - start_time,
                )

            # Apply configuration
            success = HardwareInterfaceFactory.set_configuration(config_update)

            if success:
                # Update local state
                self.component_states[step.component_name] = step.to_mode

                # Run validation tests
                validation_results = {}
                if step.validation_tests:
                    validation_results = self._run_validation_tests(
                        step.component_name, step.validation_tests
                    )

                return MigrationResult(
                    success=True,
                    step=step,
                    validation_results=validation_results,
                    duration=time.time() - start_time,
                )
            else:
                return MigrationResult(
                    success=False,
                    step=step,
                    error_message="Failed to apply hardware configuration",
                    duration=time.time() - start_time,
                )

        except Exception as e:
            logger.error(f"Migration step execution failed: {e}")
            return MigrationResult(
                success=False,
                step=step,
                error_message=str(e),
                duration=time.time() - start_time,
            )

    def _check_prerequisites(self, step: MigrationStep) -> bool:
        """Check if prerequisites are met for a migration step."""
        for prereq in step.prerequisites:
            if prereq not in self.component_states:
                continue

            # Hardware components require CAN bus
            if (
                step.to_mode == "hardware"
                and prereq == "can_bus"
                and self.component_states.get("can_bus") != "hardware"
            ):
                return False

        return True

    def _run_validation_tests(
        self, component_name: str, test_names: List[str]
    ) -> Dict[str, Any]:
        """Run validation tests for a component."""
        results = {}

        # Check if we have a registered validation callback
        if component_name in self.validation_callbacks:
            callback = self.validation_callbacks[component_name]
            results = callback(component_name, {})

        # Run basic connectivity test
        try:
            from hardware_abstraction.hardware_interface_factory import (
                HardwareInterfaceFactory,
            )

            # Try to create the component
            if component_name == "can_bus":
                interface = HardwareInterfaceFactory.create_can_bus()
            elif component_name == "drive_system":
                interface = HardwareInterfaceFactory.create_drive_system()
            elif component_name == "robotic_arm":
                interface = HardwareInterfaceFactory.create_robotic_arm()
            # Add other components as needed

            # Test basic functionality
            if hasattr(interface, "get_status"):
                status = interface.get_status()
                results["connectivity_test"] = {"passed": True, "status": status}
            else:
                results["connectivity_test"] = {
                    "passed": False,
                    "error": "No status method available",
                }

        except Exception as e:
            results["connectivity_test"] = {"passed": False, "error": str(e)}

        return results

    def _is_current_configuration(self, config: Dict[str, str]) -> bool:
        """Check if current component states match the given configuration."""
        for component, expected_mode in config.items():
            current_mode = self.component_states.get(component, "mock")
            if current_mode != expected_mode:
                return False
        return True


# Global migration manager instance
_manager = None


def get_migration_manager() -> MigrationManager:
    """Get the global migration manager instance."""
    global _manager
    if _manager is None:
        _manager = MigrationManager()
    return _manager


def migrate_to_phase(phase: MigrationPhase, validate: bool = True) -> MigrationResult:
    """Migrate to a specific integration phase."""
    return get_migration_manager().migrate_to_phase(phase, validate)


def get_current_phase() -> Optional[MigrationPhase]:
    """Get current migration phase."""
    return get_migration_manager().get_current_phase()


if __name__ == "__main__":
    # Example usage
    manager = MigrationManager()

    print(f"Current phase: {manager.get_current_phase()}")

    # Migrate to CAN bus integration
    result = manager.migrate_to_phase(MigrationPhase.CAN_BUS_INTEGRATION)
    print(f"Migration result: {result.success}")

    if result.success:
        print(f"New phase: {manager.get_current_phase()}")

        # Check component states
        states = manager.get_component_states()
        print("Component states:")
        for component, mode in states.items():
            print(f"  {component}: {mode}")
