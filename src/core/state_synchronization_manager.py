"""
Compatibility stub for tests that reference core.state_synchronization_manager.

DistributedStateManager and related types were never implemented in this codebase.
This module provides minimal stubs so existing tests can import without failure.
New code should use StateSynchronizationManager from state_management.py.
"""

from enum import Enum
from typing import Any, Dict, List, Optional

try:
    from src.core.state_management import (
        StateSynchronizationManager as _StateSyncManager,
        get_state_manager as _get_state_manager,
    )
except ImportError:
    from core.state_management import (
        StateSynchronizationManager as _StateSyncManager,
        get_state_manager as _get_state_manager,
    )


class NodeRole(Enum):
    """Node role in distributed state (stub for test compatibility)."""
    MASTER = "master"
    SLAVE = "slave"


class NodeStatus:
    """Node status (stub for test compatibility)."""

    def __init__(
        self,
        node_id: str,
        role: NodeRole = NodeRole.SLAVE,
        is_healthy: bool = True,
        state_version: int = 0,
        last_heartbeat: float = 0.0,
    ):
        self.node_id = node_id
        self.role = role
        self.is_healthy = is_healthy
        self.state_version = state_version
        self.last_heartbeat = last_heartbeat


class DistributedStateManager:
    """
    Stub for tests that expect a distributed state manager.

    Wraps StateSynchronizationManager and adds test-expected attributes
    (register_node, register_slave_manager, nodes, _trigger_election, get_system_status).
    """

    def __init__(self, node_id: str):
        self.node_id = node_id
        self.role = NodeRole.SLAVE
        self.master_node_id: Optional[str] = None
        self.nodes: Dict[str, NodeStatus] = {}
        self._slave_managers: List["DistributedStateManager"] = []
        self._sync = _StateSyncManager()

    def register_node(self, node_id: str) -> None:
        """Register a node (stub)."""
        if node_id not in self.nodes:
            self.nodes[node_id] = NodeStatus(node_id=node_id)

    def register_slave_manager(self, other: "DistributedStateManager") -> None:
        """Register a slave manager (stub)."""
        if other not in self._slave_managers:
            self._slave_managers.append(other)

    def update_state(self, name: str, value: Any) -> bool:
        """Update state (delegate to StateSynchronizationManager)."""
        if name not in self._sync.states:
            self._sync.register_state(name, value)
            return True
        return self._sync.update_state(name, value)

    def _trigger_election(self) -> None:
        """Stub - no-op for tests."""
        pass

    def get_system_status(self) -> Dict[str, Any]:
        """Return system status dict expected by tests."""
        return {
            "node_id": self.node_id,
            "role": self.role.value if isinstance(self.role, NodeRole) else str(self.role),
            "master_node_id": self.master_node_id,
            "nodes_count": len(self.nodes),
        }


def get_state_manager():
    """Re-export for test compatibility."""
    return _get_state_manager()
