#!/usr/bin/env python3
"""
Recovery Coordinator

Orchestrates coordinated recovery across all advanced systems during complex
failure scenarios. Ensures systems recover in the correct order without conflicts.

Author: URC 2026 Autonomy Team
"""

import logging
import threading
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Dict, List, Optional

logger = logging.getLogger(__name__)


class RecoveryPhase(Enum):
    """Phases of coordinated recovery."""

    ASSESSMENT = "assessment"
    DDS_RECOVERY = "dds_recovery"
    STATE_RECOVERY = "state_recovery"
    CONFIG_RECOVERY = "config_recovery"
    WEBSOCKET_RECOVERY = "websocket_recovery"
    VALIDATION = "validation"
    COMPLETE = "COMPLETE"


class FailureSeverity(Enum):
    """Severity levels for failures."""

    MINOR = "minor"  # Single component, automatic recovery
    MODERATE = "moderate"  # Multiple components, coordinated recovery
    SEVERE = "severe"  # System-wide, manual intervention may be needed
    CRITICAL = "critical"  # Complete system failure


@dataclass
class RecoveryCheckpoint:
    """A checkpoint in the recovery process."""

    phase: RecoveryPhase
    timestamp: float = field(default_factory=time.time)
    status: str = "pending"
    description: str = ""
    error_message: Optional[str] = None


@dataclass
class FailureAssessment:
    """Assessment of a system failure."""

    severity: FailureSeverity
    affected_systems: List[str]
    primary_failure: str
    estimated_recovery_time: float
    can_auto_recover: bool = True
    requires_manual_intervention: bool = False


class RecoveryCoordinator:
    """
    Coordinates recovery across all advanced systems.

    Ensures proper recovery sequencing:
    1. DDS Domain (foundation layer)
    2. State Synchronization (data consistency)
    3. Configuration (system settings)
    4. WebSocket (client connectivity)
    """

    def __init__(self):
        self.recovery_active = False
        self.current_phase = RecoveryPhase.ASSESSMENT
        self.checkpoints: List[RecoveryCheckpoint] = []
        self.system_managers = {}

        # Recovery coordination
        self.recovery_lock = threading.RLock()
        self.phase_timeout = 30.0  # 30 seconds per phase
        self.overall_timeout = 300.0  # 5 minutes total
        self.max_checkpoints = 20  # Limit checkpoint history to reduce memory

        # Callbacks
        self.progress_callbacks: List[Callable] = []
        self.completion_callbacks: List[Callable] = []

    def register_system_manager(self, system_name: str, manager: Any):
        """Register a system manager for coordinated recovery."""
        self.system_managers[system_name] = manager
        logger.info(f"Registered system manager: {system_name}")

    def add_progress_callback(self, callback: Callable[[RecoveryPhase, str], None]):
        """Add callback for recovery progress updates."""
        self.progress_callbacks.append(callback)

    def add_completion_callback(self, callback: Callable[[bool, str], None]):
        """Add callback for recovery completion."""
        self.completion_callbacks.append(callback)

    def initiate_recovery(self, failure_description: str) -> bool:
        """Initiate coordinated recovery process."""
        with self.recovery_lock:
            if self.recovery_active:
                logger.warning("Recovery already in progress")
                return False

            logger.info(
                f"[ALERT] Initiating coordinated recovery: {failure_description}"
            )
            self.recovery_active = True
            self.checkpoints = []
            self.current_phase = RecoveryPhase.ASSESSMENT

            # Start recovery in background thread
            recovery_thread = threading.Thread(
                target=self._execute_recovery, daemon=True
            )
            recovery_thread.start()

            return True

    def _execute_recovery(self):
        """Execute the coordinated recovery process."""
        start_time = time.time()
        success = False
        error_message = ""

        try:
            # Phase 1: Assess the failure
            self._set_phase(RecoveryPhase.ASSESSMENT)
            assessment = self._assess_failure()

            if assessment.severity == FailureSeverity.CRITICAL:
                error_message = (
                    "Critical failure detected - manual intervention required"
                )
                self._fail_recovery(error_message)
                return

            # Phase 2: DDS Domain Recovery (foundation)
            if "dds" in assessment.affected_systems:
                self._set_phase(RecoveryPhase.DDS_RECOVERY)
                if not self._recover_dds_domain():
                    error_message = "DDS domain recovery failed"
                    self._fail_recovery(error_message)
                    return

            # Phase 3: State Synchronization Recovery
            if "state" in assessment.affected_systems:
                self._set_phase(RecoveryPhase.STATE_RECOVERY)
                if not self._recover_state_synchronization():
                    error_message = "State synchronization recovery failed"
                    self._fail_recovery(error_message)
                    return

            # Phase 4: Configuration Recovery
            if "config" in assessment.affected_systems:
                self._set_phase(RecoveryPhase.CONFIG_RECOVERY)
                if not self._recover_configuration():
                    error_message = "Configuration recovery failed"
                    self._fail_recovery(error_message)
                    return

            # Phase 5: WebSocket Recovery
            if "websocket" in assessment.affected_systems:
                self._set_phase(RecoveryPhase.WEBSOCKET_RECOVERY)
                if not self._recover_websocket():
                    error_message = "WebSocket recovery failed"
                    self._fail_recovery(error_message)
                    return

            # Phase 6: Validation
            self._set_phase(RecoveryPhase.VALIDATION)
            if not self._validate_recovery():
                error_message = "Recovery validation failed"
                self._fail_recovery(error_message)
                return

            # Success!
            self._set_phase(RecoveryPhase.COMPLETE)
            success = True
            logger.info("[SUCCESS] Coordinated recovery completed successfully")

        except Exception as e:
            error_message = f"Recovery failed with exception: {e}"
            logger.error(error_message)
            self._fail_recovery(error_message)

        finally:
            self.recovery_active = False
            duration = time.time() - start_time

            # Notify completion callbacks
            for callback in self.completion_callbacks:
                try:
                    callback(success, error_message if not success else "")
                except Exception as e:
                    logger.error(f"Completion callback error: {e}")

            if success:
                logger.info(".1f")
            else:
                logger.error(".1f")

    def _assess_failure(self) -> FailureAssessment:
        """Assess the current system failure state."""
        logger.info("[SEARCH] Assessing system failure state...")

        affected_systems = []
        primary_failure = "unknown"

        # Check each system
        for system_name, manager in self.system_managers.items():
            try:
                if system_name == "dds":
                    # Check DDS health
                    status = manager.get_system_status()
                    if status.get("current_domain") is None:
                        affected_systems.append("dds")
                        primary_failure = "dds"
                elif system_name == "state":
                    # Check state sync health
                    status = manager.get_system_status()
                    if status.get("role") == "unknown":
                        affected_systems.append("state")
                        if not primary_failure or primary_failure == "unknown":
                            primary_failure = "state"
                elif system_name == "config":
                    # Check config health
                    status = manager.get_system_status()
                    # Config system is usually healthy unless corrupted
                    pass
                elif system_name == "websocket":
                    # Check WebSocket health
                    status = manager.get_system_status()
                    unhealthy_count = sum(
                        1
                        for ep in status.get("endpoints", {}).values()
                        if not ep.get("is_healthy", True)
                    )
                    if unhealthy_count > 0:
                        affected_systems.append("websocket")
                        if not primary_failure or primary_failure == "unknown":
                            primary_failure = "websocket"
            except Exception as e:
                logger.error(f"Error assessing {system_name}: {e}")
                affected_systems.append(system_name)

        # Determine severity
        if len(affected_systems) == 0:
            severity = FailureSeverity.MINOR
        elif len(affected_systems) == 1:
            severity = FailureSeverity.MODERATE
        elif len(affected_systems) < 3:
            severity = FailureSeverity.SEVERE
        else:
            severity = FailureSeverity.CRITICAL

        # Estimate recovery time
        recovery_time = len(affected_systems) * 15.0  # 15 seconds per affected system

        assessment = FailureAssessment(
            severity=severity,
            affected_systems=affected_systems,
            primary_failure=primary_failure,
            estimated_recovery_time=recovery_time,
            can_auto_recover=severity != FailureSeverity.CRITICAL,
        )

        logger.info(
            f"Assessment: {severity.value} severity, {len(affected_systems)} systems affected"
        )
        return assessment

    def _recover_dds_domain(self) -> bool:
        """Recover DDS domain functionality."""
        logger.info("[CONNECT] Recovering DDS domain...")

        try:
            dds_manager = self.system_managers.get("dds")
            if not dds_manager:
                logger.error("DDS manager not registered")
                return False

            # Trigger domain failover to backup
            success = dds_manager.trigger_domain_failover(
                target_domain_id=43
            )  # Backup domain
            if success:
                logger.info("[SUCCESS] DDS domain failover successful")
                return True
            else:
                logger.error("[ERROR] DDS domain failover failed")
                return False

        except Exception as e:
            logger.error(f"DDS recovery error: {e}")
            return False

    def _recover_state_synchronization(self) -> bool:
        """Recover state synchronization."""
        logger.info("[UPDATE] Recovering state synchronization...")

        try:
            state_manager = self.system_managers.get("state")
            if not state_manager:
                logger.error("State manager not registered")
                return False

            # Force re-election
            state_manager._trigger_election()

            # Wait for election to complete
            time.sleep(2.0)

            # Check if we have a master
            status = state_manager.get_system_status()
            if status.get("master_node"):
                logger.info("[SUCCESS] State synchronization recovered")
                return True
            else:
                logger.error(
                    "[ERROR] State synchronization recovery failed - no master elected"
                )
                return False

        except Exception as e:
            logger.error(f"State recovery error: {e}")
            return False

    def _recover_configuration(self) -> bool:
        """Recover configuration system."""
        logger.info("[CONFIG] Recovering configuration system...")

        try:
            config_manager = self.system_managers.get("config")
            if not config_manager:
                logger.error("Config manager not registered")
                return False

            # Configuration system usually recovers itself
            # Just validate it's working
            status = config_manager.get_system_status()
            if status.get("current_version") is not None:
                logger.info("[SUCCESS] Configuration system healthy")
                return True
            else:
                logger.error("[ERROR] Configuration system not responding")
                return False

        except Exception as e:
            logger.error(f"Configuration recovery error: {e}")
            return False

    def _recover_websocket(self) -> bool:
        """Recover WebSocket connectivity."""
        logger.info("[NETWORK] Recovering WebSocket connectivity...")

        try:
            ws_manager = self.system_managers.get("websocket")
            if not ws_manager:
                logger.error("WebSocket manager not registered")
                return False

            # Check current status
            status = ws_manager.get_system_status()

            # Count healthy endpoints
            healthy_endpoints = sum(
                1
                for ep in status.get("endpoints", {}).values()
                if ep.get("is_healthy", False)
            )

            if healthy_endpoints > 0:
                logger.info(
                    f"[SUCCESS] WebSocket system recovered ({healthy_endpoints} healthy endpoints)"
                )
                return True
            else:
                logger.error("[ERROR] No healthy WebSocket endpoints available")
                return False

        except Exception as e:
            logger.error(f"WebSocket recovery error: {e}")
            return False

    def _validate_recovery(self) -> bool:
        """Validate that recovery was successful."""
        logger.info("[SUCCESS] Validating recovery...")

        try:
            # Check all systems are healthy
            all_healthy = True

            for system_name, manager in self.system_managers.items():
                try:
                    if system_name == "dds":
                        status = manager.get_system_status()
                        if not status.get("current_domain"):
                            all_healthy = False
                            logger.error(f"DDS system not healthy")
                    elif system_name == "state":
                        status = manager.get_system_status()
                        if not status.get("master_node"):
                            all_healthy = False
                            logger.error(f"State system not healthy")
                    elif system_name == "config":
                        status = manager.get_system_status()
                        if status.get("current_version") is None:
                            all_healthy = False
                            logger.error(f"Config system not healthy")
                    elif system_name == "websocket":
                        status = manager.get_system_status()
                        healthy_count = sum(
                            1
                            for ep in status.get("endpoints", {}).values()
                            if ep.get("is_healthy", False)
                        )
                        if healthy_count == 0:
                            all_healthy = False
                            logger.error(f"WebSocket system not healthy")
                except Exception as e:
                    logger.error(f"Error validating {system_name}: {e}")
                    all_healthy = False

            if all_healthy:
                logger.info("[SUCCESS] All systems validated as healthy")
                return True
            else:
                logger.error("[ERROR] Some systems failed validation")
                return False

        except Exception as e:
            logger.error(f"Recovery validation error: {e}")
            return False

    def _set_phase(self, phase: RecoveryPhase):
        """Set the current recovery phase."""
        self.current_phase = phase

        checkpoint = RecoveryCheckpoint(
            phase=phase, description=f"Starting {phase.value} phase"
        )
        self.checkpoints.append(checkpoint)

        # Limit checkpoint history to reduce memory usage
        if len(self.checkpoints) > self.max_checkpoints:
            self.checkpoints.pop(0)  # Remove oldest checkpoint

        logger.info(f"[UPDATE] Recovery phase: {phase.value}")

        # Notify progress callbacks
        for callback in self.progress_callbacks:
            try:
                callback(phase, f"Starting {phase.value} phase")
            except Exception as e:
                logger.error(f"Progress callback error: {e}")

    def _fail_recovery(self, error_message: str):
        """Mark recovery as failed."""
        logger.error(f"[ERROR] Recovery failed: {error_message}")

        # Update final checkpoint
        if self.checkpoints:
            self.checkpoints[-1].status = "failed"
            self.checkpoints[-1].error_message = error_message

        self.current_phase = RecoveryPhase.COMPLETE

    def get_recovery_status(self) -> Dict[str, Any]:
        """Get current recovery status."""
        return {
            "active": self.recovery_active,
            "current_phase": self.current_phase.value if self.current_phase else None,
            "checkpoints": [
                {
                    "phase": cp.phase.value,
                    "timestamp": cp.timestamp,
                    "status": cp.status,
                    "description": cp.description,
                    "error": cp.error_message,
                }
                for cp in self.checkpoints
            ],
            "registered_systems": list(self.system_managers.keys()),
        }


# Global recovery coordinator instance
_recovery_coordinator = None


def get_recovery_coordinator() -> RecoveryCoordinator:
    """Get the global recovery coordinator instance."""
    global _recovery_coordinator
    if _recovery_coordinator is None:
        _recovery_coordinator = RecoveryCoordinator()
    return _recovery_coordinator
