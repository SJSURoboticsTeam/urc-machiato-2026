#!/usr/bin/env python3
"""
Dynamic Configuration Manager

Provides runtime parameter updates for ROS2 nodes without requiring restarts.
Supports configuration versioning, rollback, and validation.

Author: URC 2026 Autonomy Team
"""

import copy
import hashlib
import json
import logging
import threading
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Dict, List, Optional, Set

from bridges.constants import CONFIG_UPDATE_TIMEOUT


class ConfigStatus(Enum):
    """Status of a configuration update."""

    PENDING = "pending"
    APPLYING = "applying"
    ACTIVE = "active"
    FAILED = "failed"
    ROLLED_BACK = "rolled_back"


class ValidationResult(Enum):
    """Result of configuration validation."""

    VALID = "valid"
    INVALID = "invalid"
    WARNING = "warning"


@dataclass
class ConfigChange:
    """A single configuration parameter change."""

    node_name: str
    parameter_name: str
    old_value: Any
    new_value: Any
    timestamp: float = field(default_factory=time.time)
    applied: bool = False
    validated: bool = True


@dataclass
class ConfigSnapshot:
    """A snapshot of the complete system configuration."""

    version: int
    timestamp: float
    changes: List[ConfigChange]
    status: ConfigStatus = ConfigStatus.PENDING
    checksum: str = ""
    _change_count: int = field(init=False, default=0)

    def __post_init__(self):
        """Generate checksum after initialization."""
        self._change_count = len(self.changes)
        self.checksum = self._calculate_checksum()

    def _calculate_checksum(self) -> str:
        """Calculate optimized checksum of the configuration snapshot."""
        # Use MD5 for speed (still cryptographically secure enough for this use case)
        # Include only essential data to reduce computation
        data = f"{self.version}:{self.timestamp}:{len(self.changes)}:{sum(hash(str(c.__dict__)) for c in self.changes)}"
        return hashlib.md5(data.encode()).hexdigest()[:12]

    def is_valid(self) -> bool:
        """Verify the integrity of this configuration snapshot."""
        # Quick validation - check if change count matches
        if len(self.changes) != self._change_count:
            return False
        # Only compute checksum if change count is valid
        return self.checksum == self._calculate_checksum()


@dataclass
class NodeConfig:
    """Configuration for a specific ROS2 node."""

    node_name: str
    current_parameters: Dict[str, Any] = field(default_factory=dict)
    pending_changes: Dict[str, Any] = field(default_factory=dict)
    last_update: float = 0.0
    config_version: int = 0


class DynamicConfigManager:
    """
    Manages dynamic configuration updates for ROS2 systems.

    Features:
    - Runtime parameter updates without node restarts
    - Configuration versioning and rollback
    - Validation and safety checks
    - Atomic configuration changes
    - Configuration history and auditing
    """

    def __init__(self, debug_enabled: bool = False, logger=None):
        # Configuration storage
        self.nodes: Dict[str, NodeConfig] = {}
        self.config_history: List[ConfigSnapshot] = []
        self.current_version = 0

        # Logger setup
        self.logger = logger or logging.getLogger(__name__)

        # Performance optimization flags
        self.debug_enabled = debug_enabled

        # Control
        self.update_lock = threading.RLock()
        self.validation_enabled = True
        self.auto_backup = True
        self.running = False

        # Callbacks
        self.update_callbacks: List[Callable] = []
        self.validation_callbacks: List[Callable] = []
        self.rollback_callbacks: List[Callable] = []

        # Configuration limits
        self.max_history_size = 10  # Keep last 10 configurations
        self.update_timeout = (
            CONFIG_UPDATE_TIMEOUT  # Configurable timeout to apply changes
        )
        self.validation_timeout = 5.0  # 5 seconds for validation

    def start(self):
        """Start the dynamic configuration manager."""
        self.running = True

    def stop(self):
        """Stop the dynamic configuration manager."""
        self.running = False

    def register_node(
        self, node_name: str, initial_config: Optional[Dict[str, Any]] = None
    ):
        """Register a ROS2 node for dynamic configuration."""
        with self.update_lock:
            if node_name not in self.nodes:
                self.nodes[node_name] = NodeConfig(
                    node_name=node_name,
                    current_parameters=initial_config or {},
                    last_update=time.time(),
                )

    def unregister_node(self, node_name: str):
        """Remove a node from dynamic configuration management."""
        with self.update_lock:
            if node_name in self.nodes:
                del self.nodes[node_name]

    def update_node_config(
        self, node_name: str, parameter_name: str, new_value: Any, validate: bool = True
    ) -> bool:
        """Update a single parameter for a specific node."""
        changes = [
            {
                "node_name": node_name,
                "parameter_name": parameter_name,
                "new_value": new_value,
            }
        ]
        return self.update_multiple_configs(changes, validate=validate)

    def update_multiple_configs(
        self,
        config_updates: List[Dict[str, Any]],
        validate: bool = True,
        description: str = "",
    ) -> bool:
        """
        Update multiple configuration parameters atomically.

        config_updates format:
        [
            {
                'node_name': 'competition_bridge',
                'parameter_name': 'telemetry_rate_hz',
                'new_value': 10.0
            },
            ...
        ]
        """
        with self.update_lock:
            try:
                # Create configuration snapshot
                self.current_version += 1
                changes = []

                # Validate all nodes exist and prepare changes
                for update in config_updates:
                    node_name = update["node_name"]
                    param_name = update["parameter_name"]
                    new_value = update["new_value"]

                    if node_name not in self.nodes:
                        raise ValueError(f"Node {node_name} not registered")

                    node_config = self.nodes[node_name]
                    old_value = node_config.current_parameters.get(param_name)

                    change = ConfigChange(
                        node_name=node_name,
                        parameter_name=param_name,
                        old_value=old_value,
                        new_value=new_value,
                    )
                    changes.append(change)

                # Create snapshot
                snapshot = ConfigSnapshot(
                    version=self.current_version,
                    timestamp=time.time(),
                    changes=changes,
                    status=ConfigStatus.PENDING,
                )

                # Validate configuration if requested
                if validate and self.validation_enabled:
                    validation_result = self._validate_config_snapshot(snapshot)
                    if validation_result == ValidationResult.INVALID:
                        snapshot.status = ConfigStatus.FAILED
                        self.config_history.append(snapshot)
                        return False

                # Apply configuration changes
                snapshot.status = ConfigStatus.APPLYING
                success = self._apply_config_snapshot(snapshot)

                if success:
                    snapshot.status = ConfigStatus.ACTIVE
                    self._trigger_update_callbacks(snapshot)
                else:
                    snapshot.status = ConfigStatus.FAILED
                    # Attempt automatic rollback
                    self.rollback_to_version(self.current_version - 1)

                # Store in history
                self.config_history.append(snapshot)
                self._cleanup_history()

                return success

            except Exception as e:
                self.logger.info(f"Configuration update failed: {e}")
                return False

    def rollback_to_version(self, target_version: int) -> bool:
        """Rollback configuration to a previous version."""
        with self.update_lock:
            # Find all snapshots after the target version that need to be rolled back
            snapshots_to_rollback = []
            for snapshot in self.config_history:
                if (
                    snapshot.version > target_version
                    and snapshot.status == ConfigStatus.ACTIVE
                ):
                    snapshots_to_rollback.append(snapshot)

            if not snapshots_to_rollback:
                self.logger.info(
                    f"No changes to rollback (already at version {target_version})"
                )
                return True

            try:
                # Create rollback snapshot that undoes all changes after target version
                rollback_changes = []
                for snapshot in reversed(
                    snapshots_to_rollback
                ):  # Process in reverse order
                    for change in snapshot.changes:
                        # Reverse each change
                        rollback_change = ConfigChange(
                            node_name=change.node_name,
                            parameter_name=change.parameter_name,
                            old_value=change.new_value,  # Current value becomes old
                            new_value=change.old_value,  # Old value becomes new
                        )
                        rollback_changes.append(rollback_change)

                rollback_snapshot = ConfigSnapshot(
                    version=self.current_version + 1,
                    timestamp=time.time(),
                    changes=rollback_changes,
                    status=ConfigStatus.APPLYING,
                )

                # Apply rollback
                success = self._apply_config_snapshot(rollback_snapshot)

                if success:
                    rollback_snapshot.status = ConfigStatus.ROLLED_BACK
                    self.current_version = rollback_snapshot.version
                    self._trigger_rollback_callbacks(rollback_snapshot)
                else:
                    rollback_snapshot.status = ConfigStatus.FAILED

                self.config_history.append(rollback_snapshot)
                self._cleanup_history()

                return success

            except Exception as e:
                self.logger.info(f"Configuration rollback failed: {e}")
                return False

    def get_node_config(self, node_name: str) -> Optional[Dict[str, Any]]:
        """Get the current configuration for a specific node."""
        with self.update_lock:
            node_config = self.nodes.get(node_name)
            return node_config.current_parameters.copy() if node_config else None

    def get_system_config(self) -> Dict[str, Any]:
        """Get the complete current system configuration."""
        with self.update_lock:
            return {
                node_name: node_config.current_parameters.copy()
                for node_name, node_config in self.nodes.items()
            }

    def get_config_history(self, limit: int = 5) -> List[Dict[str, Any]]:
        """Get recent configuration history."""
        with self.update_lock:
            history = []
            for snapshot in reversed(self.config_history[-limit:]):
                history.append(
                    {
                        "version": snapshot.version,
                        "timestamp": snapshot.timestamp,
                        "status": snapshot.status.value,
                        "changes_count": len(snapshot.changes),
                        "changes": [
                            {
                                "node": c.node_name,
                                "parameter": c.parameter_name,
                                "old_value": c.old_value,
                                "new_value": c.new_value,
                            }
                            for c in snapshot.changes
                        ],
                    }
                )
            return history

    def add_update_callback(self, callback: Callable[[ConfigSnapshot], None]):
        """Add callback for configuration updates."""
        self.update_callbacks.append(callback)

    def add_validation_callback(
        self, callback: Callable[[ConfigSnapshot], ValidationResult]
    ):
        """Add callback for configuration validation."""
        self.validation_callbacks.append(callback)

    def add_rollback_callback(self, callback: Callable[[ConfigSnapshot], None]):
        """Add callback for configuration rollbacks."""
        self.rollback_callbacks.append(callback)

    def _validate_config_snapshot(self, snapshot: ConfigSnapshot) -> ValidationResult:
        """Validate a configuration snapshot."""
        try:
            # Run custom validation callbacks
            for callback in self.validation_callbacks:
                result = callback(snapshot)
                if result == ValidationResult.INVALID:
                    return ValidationResult.INVALID

            # Basic validation
            for change in snapshot.changes:
                # Validate parameter types and ranges
                if not self._validate_parameter_change(change):
                    return ValidationResult.INVALID

            # Cross-parameter validation
            if not self._validate_parameter_dependencies(snapshot):
                return ValidationResult.INVALID

            return ValidationResult.VALID

        except Exception as e:
            self.logger.info(f"Configuration validation error: {e}")
            return ValidationResult.INVALID

    def _validate_parameter_change(self, change: ConfigChange) -> bool:
        """Validate a single parameter change."""
        # Basic type validation
        old_type = type(change.old_value) if change.old_value is not None else None
        new_type = type(change.new_value)

        if old_type and old_type != new_type:

            return False

        # Parameter-specific validation
        param_name = change.parameter_name

        if "rate" in param_name.lower() or "hz" in param_name.lower():
            # Rate parameters should be positive numbers
            if not isinstance(change.new_value, (int, float)) or change.new_value <= 0:
                return False

        elif "timeout" in param_name.lower():
            # Timeout parameters should be positive
            if not isinstance(change.new_value, (int, float)) or change.new_value <= 0:
                return False

        elif "port" in param_name.lower():
            # Port parameters should be valid port numbers
            if not isinstance(change.new_value, int) or not (
                1024 <= change.new_value <= 65535
            ):
                return False

        return True

    def _validate_parameter_dependencies(self, snapshot: ConfigSnapshot) -> bool:
        """Validate parameter dependencies and conflicts."""
        # Group changes by node
        node_changes = {}
        for change in snapshot.changes:
            if change.node_name not in node_changes:
                node_changes[change.node_name] = []
            node_changes[change.node_name].append(change)

        # Check for conflicting changes within each node
        for node_name, changes in node_changes.items():
            param_names = [c.parameter_name for c in changes]
            if len(param_names) != len(set(param_names)):
                self.logger.info(f"Duplicate parameter changes in {node_name}")
                return False

            # Check for related parameter consistency
            rate_params = [c for c in changes if "rate" in c.parameter_name.lower()]
            if len(rate_params) > 1:
                # Ensure rate parameters are reasonable relative to each other
                rates = [c.new_value for c in rate_params]
                if max(rates) / min(rates) > 10:  # No parameter should be 10x another
                    self.logger.info(f"Rate parameter ratio too extreme in {node_name}")
                    return False

        return True

    def _apply_config_snapshot(self, snapshot: ConfigSnapshot) -> bool:
        """Apply a configuration snapshot to all affected nodes."""
        try:
            # Group changes by node
            node_changes = {}
            for change in snapshot.changes:
                if change.node_name not in node_changes:
                    node_changes[change.node_name] = []
                node_changes[change.node_name].append(change)

            # Apply changes to each node
            success_count = 0
            for node_name, changes in node_changes.items():
                if self._apply_node_changes(node_name, changes):
                    success_count += 1

            # All nodes must succeed for the snapshot to be considered successful
            success = success_count == len(node_changes)

            if success:
                # Update node configurations
                for node_name, changes in node_changes.items():
                    node_config = self.nodes[node_name]
                    for change in changes:
                        node_config.current_parameters[
                            change.parameter_name
                        ] = change.new_value
                        change.applied = True
                    node_config.last_update = time.time()
                    node_config.config_version = snapshot.version

            return success

        except Exception as e:
            self.logger.info(f"Failed to apply configuration snapshot: {e}")
            return False

    def _apply_node_changes(self, node_name: str, changes: List[ConfigChange]) -> bool:
        """Apply configuration changes to a specific node."""
        try:
            # In a real ROS2 system, this would use the ROS2 parameter API
            # For now, we simulate the parameter update

            if self.debug_enabled:
                self.logger.info(
                    f"Applying {len(changes)} parameter changes to {node_name}"
                )
                for change in changes:
                    pass  # Placeholder for debug logging

            # Optimized: Remove artificial delay - real ROS2 parameter updates are near-instantaneous

            return True

        except Exception as e:
            self.logger.info(f"Failed to apply changes to node {node_name}: {e}")
            return False

    def _trigger_update_callbacks(self, snapshot: ConfigSnapshot):
        """Trigger configuration update callbacks."""
        for callback in self.update_callbacks:
            try:
                callback(snapshot)
            except Exception as e:
                self.logger.info(f"Update callback error: {e}")

    def _trigger_rollback_callbacks(self, snapshot: ConfigSnapshot):
        """Trigger configuration rollback callbacks."""
        for callback in self.rollback_callbacks:
            try:
                callback(snapshot)
            except Exception as e:
                self.logger.info(f"Rollback callback error: {e}")

    def _cleanup_history(self):
        """Clean up old configuration history."""
        while len(self.config_history) > self.max_history_size:
            self.config_history.pop(0)

    def get_system_status(self) -> Dict[str, Any]:
        """Get comprehensive system configuration status."""
        with self.update_lock:
            return {
                "current_version": self.current_version,
                "nodes": {
                    node_name: {
                        "parameters_count": len(node_config.current_parameters),
                        "last_update": node_config.last_update,
                        "config_version": node_config.config_version,
                        "pending_changes": len(node_config.pending_changes),
                    }
                    for node_name, node_config in self.nodes.items()
                },
                "history": {
                    "total_snapshots": len(self.config_history),
                    "recent_versions": [s.version for s in self.config_history[-3:]],
                },
                "validation_enabled": self.validation_enabled,
                "auto_backup": self.auto_backup,
                "timestamp": time.time(),
            }


# Global dynamic config manager instance
_config_manager = None


def get_dynamic_config_manager() -> DynamicConfigManager:
    """Get the global dynamic configuration manager instance."""
    global _config_manager
    if _config_manager is None:
        _config_manager = DynamicConfigManager()
    return _config_manager


# Example usage:
