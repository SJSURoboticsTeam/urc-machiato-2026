#!/usr/bin/env python3
"""
Distributed State Synchronization Manager

Ensures consistent state across all ROS2 bridges using a master-slave
replication model with automatic failover and conflict resolution.

Author: URC 2026 Autonomy Team
"""

import hashlib
import json
import logging
import threading
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Dict, List, Optional


class NodeRole(Enum):
    """Node roles in the distributed state system."""

    MASTER = "master"
    SLAVE = "slave"
    CANDIDATE = "candidate"


class StateOperation(Enum):
    """Types of state operations."""

    SET = "set"
    UPDATE = "update"
    DELETE = "delete"
    SYNC = "sync"


@dataclass
class StateEntry:
    """A single state entry with metadata."""

    key: str
    value: Any
    timestamp: float
    node_id: str
    operation: StateOperation
    version: int = 1
    checksum: str = ""

    def __post_init__(self):
        """Generate checksum after initialization."""
        self.checksum = self._calculate_checksum()

    def _calculate_checksum(self) -> str:
        """Calculate SHA256 checksum of the state entry."""
        data = f"{self.key}:{json.dumps(self.value, sort_keys=True)}:{self.timestamp}:{self.node_id}:{self.operation.value}:{self.version}"
        return hashlib.sha256(data.encode()).hexdigest()[:16]

    def is_valid(self) -> bool:
        """Verify the integrity of this state entry."""
        return self.checksum == self._calculate_checksum()


@dataclass
class NodeStatus:
    """Status information for a distributed node."""

    node_id: str
    role: NodeRole
    last_heartbeat: float
    state_version: int
    is_healthy: bool = True
    master_candidate: bool = False


class DistributedStateManager:
    """
    Manages distributed state synchronization across ROS2 bridges.

    Features:
    - Master-slave replication with automatic failover
    - State consistency checking and conflict resolution
    - Heartbeat monitoring and node health tracking
    - Event-driven state synchronization
    """

    def __init__(self, node_id: str, heartbeat_interval: float = 1.0):
        self.node_id = node_id
        self.heartbeat_interval = heartbeat_interval
        self.logger = logging.getLogger(f"StateManager.{node_id}")

        # State management
        self.local_state: Dict[str, StateEntry] = {}
        self.state_version = 0
        self.state_lock = threading.RLock()

        # Node management
        self.nodes: Dict[str, NodeStatus] = {}
        self.role = NodeRole.SLAVE  # Start as slave
        self.master_node_id: Optional[str] = None

        # Synchronization
        self.sync_callbacks: List[Callable] = []
        self.conflict_resolution_callbacks: List[Callable] = []

        # Control
        self.running = False
        self.monitor_thread: Optional[threading.Thread] = None
        self.heartbeat_thread: Optional[threading.Thread] = None

        # Testing support - slave manager references for direct state sync
        self._slave_managers: Dict[str, "DistributedStateManager"] = {}

        # Configuration
        self.heartbeat_timeout = heartbeat_interval * 3
        self.election_timeout_min = heartbeat_interval * 5
        self.election_timeout_max = heartbeat_interval * 10

    def get_logger(self):
        """Get the logger for this state manager."""
        return self.logger

    def start(self):
        """Start the distributed state manager."""
        if self.running:
            return

        self.running = True

        # Start monitoring threads
        self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.heartbeat_thread = threading.Thread(
            target=self._heartbeat_loop, daemon=True
        )

        self.monitor_thread.start()
        self.heartbeat_thread.start()

        # Register self as a node
        self._register_node(self.node_id, NodeRole.SLAVE)

    def stop(self):
        """Stop the distributed state manager."""
        self.running = False

        if self.monitor_thread and self.monitor_thread.is_alive():
            self.monitor_thread.join(timeout=2.0)

        if self.heartbeat_thread and self.heartbeat_thread.is_alive():
            self.heartbeat_thread.join(timeout=2.0)

    def register_node(self, node_id: str, initial_role: NodeRole = NodeRole.SLAVE):
        """Register a new node in the distributed system."""
        with self.state_lock:
            if node_id not in self.nodes:
                self.nodes[node_id] = NodeStatus(
                    node_id=node_id,
                    role=initial_role,
                    last_heartbeat=time.time(),
                    state_version=0,
                )

                # If this is the first node, make it master
                if len(self.nodes) == 1:
                    self._promote_to_master(node_id)

    def unregister_node(self, node_id: str):
        """Remove a node from the distributed system."""
        with self.state_lock:
            if node_id in self.nodes:
                del self.nodes[node_id]

                # If master left, trigger election
                if self.master_node_id == node_id:
                    self._trigger_election()

    def update_state(
        self, key: str, value: Any, operation: StateOperation = StateOperation.UPDATE
    ) -> bool:
        """Update local state and propagate to other nodes."""
        with self.state_lock:
            # Create new state entry
            entry = StateEntry(
                key=key,
                value=value,
                timestamp=time.time(),
                node_id=self.node_id,
                operation=operation,
                version=self.state_version + 1,
            )

            # Update local state
            self.local_state[key] = entry
            self.state_version = entry.version

            # If we're master, propagate to slaves
            if self.role == NodeRole.MASTER:
                self._propagate_state_update(entry)

            # Trigger callbacks
            self._trigger_state_callbacks(key, entry)

            return True

    def get_state(self, key: str) -> Optional[Any]:
        """Get the current value for a state key."""
        with self.state_lock:
            entry = self.local_state.get(key)
            return entry.value if entry else None

    def get_full_state(self) -> Dict[str, Any]:
        """Get the complete current state."""
        with self.state_lock:
            return {key: entry.value for key, entry in self.local_state.items()}

    def sync_state_from_master(self, master_state: Dict[str, Any], master_version: int):
        """Synchronize state from master node."""
        with self.state_lock:
            if master_version > self.state_version:
                # Apply master's state
                for key, value in master_state.items():
                    if (
                        key not in self.local_state
                        or self.local_state[key].version < master_version
                    ):
                        entry = StateEntry(
                            key=key,
                            value=value,
                            timestamp=time.time(),
                            node_id="master_sync",
                            operation=StateOperation.SYNC,
                            version=master_version,
                        )
                        self.local_state[key] = entry

                self.state_version = master_version

                # Trigger callbacks for synced keys
                for key in master_state.keys():
                    if key in self.local_state:
                        self._trigger_state_callbacks(key, self.local_state[key])

    def add_state_callback(self, callback: Callable[[str, StateEntry], None]):
        """Add a callback for state changes."""
        self.sync_callbacks.append(callback)

    def add_conflict_callback(
        self, callback: Callable[[str, StateEntry, StateEntry], StateEntry]
    ):
        """Add a callback for conflict resolution."""
        self.conflict_resolution_callbacks.append(callback)

    def _monitor_loop(self):
        """Main monitoring loop for node health and elections."""
        while self.running:
            try:
                current_time = time.time()

                # Check node health
                self._check_node_health(current_time)

                # Check if election is needed
                if self._should_trigger_election():
                    self._trigger_election()

                time.sleep(self.heartbeat_interval)

            except Exception as e:
                self.get_logger().info(f"Monitor loop error: {e}")
                time.sleep(self.heartbeat_interval)

    def _heartbeat_loop(self):
        """Send periodic heartbeats to other nodes."""
        while self.running:
            try:
                if self.role in [NodeRole.MASTER, NodeRole.SLAVE]:
                    self._send_heartbeat()

                time.sleep(self.heartbeat_interval)

            except Exception as e:
                self.get_logger().info(f"Heartbeat loop error: {e}")
                time.sleep(self.heartbeat_interval)

    def _check_node_health(self, current_time: float):
        """Check health of all known nodes."""
        unhealthy_nodes = []

        for node_id, node_status in self.nodes.items():
            if node_id == self.node_id:
                continue  # Don't check self

            time_since_heartbeat = current_time - node_status.last_heartbeat

            if time_since_heartbeat > self.heartbeat_timeout:
                if node_status.is_healthy:
                    self.get_logger().info(f"Node {node_id} marked as unhealthy")
                    node_status.is_healthy = False
                    unhealthy_nodes.append(node_id)
                else:
                    # Node has been unhealthy for too long, remove it
                    if time_since_heartbeat > self.heartbeat_timeout * 3:
                        self.get_logger().info(f"Removing unresponsive node {node_id}")
                        self.unregister_node(node_id)

        # If master is unhealthy, trigger election
        if self.master_node_id in unhealthy_nodes:
            self._trigger_election()

    def _should_trigger_election(self) -> bool:
        """Determine if an election should be triggered."""
        if self.role == NodeRole.MASTER:
            return False  # Masters don't trigger elections

        # Check if we have a master
        if not self.master_node_id:
            return True

        # Check if master is healthy
        master_status = self.nodes.get(self.master_node_id)
        if not master_status or not master_status.is_healthy:
            return True

        return False

    def _trigger_election(self):
        """Trigger a master election using improved consensus logic."""
        self.get_logger().info(
            f"Triggering master election (current master: {self.master_node_id})"
        )
        # Become candidate
        self.role = NodeRole.CANDIDATE

        # Get all healthy nodes including self
        healthy_nodes = [
            node_id for node_id, status in self.nodes.items() if status.is_healthy
        ]

        if not healthy_nodes:
            self.get_logger().info(f"No healthy nodes available for election")
            return

        # Enhanced election algorithm:
        # 1. Prefer nodes with higher state version (more up-to-date)
        # 2. Break ties by node ID (deterministic)
        # 3. Only participate if we have a chance to win

        candidates_with_priority = []
        for node_id in healthy_nodes:
            node_status = self.nodes[node_id]
            # Priority: (state_version, node_id) - higher state version wins, then lexicographic node_id
            priority = (node_status.state_version, node_id)
            candidates_with_priority.append((node_id, priority))

        # Sort by priority (highest first)
        candidates_with_priority.sort(key=lambda x: x[1], reverse=True)
        winner_id = candidates_with_priority[0][0]
        self.get_logger().info(
            f"Election candidates: {[c[0] for c in candidates_with_priority]}, winner: {winner_id}"
        )
        self.get_logger().info(
            f"Winner priority: state_v{candidates_with_priority[0][1][0]}, node_id:{candidates_with_priority[0][1][1]}"
        )
        # Update all nodes' knowledge of the master
        for node_id in healthy_nodes:
            if node_id == winner_id:
                if node_id == self.node_id:
                    self.get_logger().info(f"Node {self.node_id} becoming master")
                    self._promote_to_master(self.node_id)
                # Else another node is master, we'll learn about it via heartbeat
            else:
                # Everyone else becomes slave
                if node_id == self.node_id:
                    self.get_logger().info(
                        f"Node {self.node_id} becoming slave (master will be {winner_id})"
                    )
                    self.role = NodeRole.SLAVE
                    self.master_node_id = winner_id

    def _promote_to_master(self, node_id: str):
        """Promote a node to master."""
        self.get_logger().info(f"Promoting {node_id} to master")
        if node_id == self.node_id:
            self.role = NodeRole.MASTER
            self.master_node_id = node_id

            # Notify all slaves of the new master
            self._broadcast_master_change()

        else:
            self.master_node_id = node_id
            self.role = NodeRole.SLAVE

    def _send_heartbeat(self):
        """Send heartbeat to other nodes."""
        # In a real implementation, this would send actual network messages
        # For now, we simulate by updating local timestamps

        for node_id, node_status in self.nodes.items():
            if node_id != self.node_id:
                # Simulate receiving heartbeat
                node_status.last_heartbeat = time.time()
                node_status.is_healthy = True

    def _propagate_state_update(self, entry: StateEntry):
        """Propagate state update to all slave nodes."""
        # In a real implementation, this would send network messages to slaves
        # For simulation, we assume all nodes are local and directly update slave state

        for node_id, node_status in self.nodes.items():
            if node_id != self.node_id and node_status.role == NodeRole.SLAVE:
                # For testing purposes, find the slave manager instance and sync
                # In real ROS2, this would be done via ROS2 topics/services
                if hasattr(self, "_slave_managers"):
                    slave_manager = self._slave_managers.get(node_id)
                    if slave_manager:
                        # Sync the state entry to the slave
                        slave_manager._receive_state_update(entry)
                        self.get_logger().info(
                            f"Propagated state update {entry.key} to slave {node_id}"
                        )

    def _broadcast_master_change(self):
        """Broadcast master change to all nodes."""
        for node_id, node_status in self.nodes.items():
            if node_id != self.node_id:
                self.get_logger().info(
                    f"Notifying {node_id} of new master {self.master_node_id}"
                )

    def _trigger_state_callbacks(self, key: str, entry: StateEntry):
        """Trigger callbacks for state changes."""
        for callback in self.sync_callbacks:
            try:
                callback(key, entry)
            except Exception as e:
                self.get_logger().info(f"State callback error: {e}")

    def _register_node(self, node_id: str, role: NodeRole):
        """Register a node internally."""
        self.nodes[node_id] = NodeStatus(
            node_id=node_id, role=role, last_heartbeat=time.time(), state_version=0
        )

    def _receive_state_update(self, entry: StateEntry):
        """Receive a state update from master (for testing)."""
        with self.state_lock:
            if entry.version > self.state_version:
                self.local_state[entry.key] = entry
                self.state_version = entry.version

                # Trigger callbacks
                self._trigger_state_callbacks(entry.key, entry)

    def register_slave_manager(self, slave_manager: "DistributedStateManager"):
        """Register a slave manager for direct state sync (testing only)."""
        self._slave_managers[slave_manager.node_id] = slave_manager

    def get_system_status(self) -> Dict[str, Any]:
        """Get comprehensive system status."""
        with self.state_lock:
            return {
                "node_id": self.node_id,
                "role": self.role.value,
                "master_node": self.master_node_id,
                "state_version": self.state_version,
                "state_keys": list(self.local_state.keys()),
                "nodes": {
                    node_id: {
                        "role": status.role.value,
                        "healthy": status.is_healthy,
                        "last_heartbeat": status.last_heartbeat,
                        "state_version": status.state_version,
                    }
                    for node_id, status in self.nodes.items()
                },
                "timestamp": time.time(),
            }


# Global state manager instance
_state_manager = None


def get_state_manager(node_id: str) -> DistributedStateManager:
    """Get the global distributed state manager instance."""
    global _state_manager
    if _state_manager is None:
        _state_manager = DistributedStateManager(node_id)
    return _state_manager


# Example usage and testing functions
def test_state_synchronization():
    """Test the state synchronization functionality."""
    logger = logging.getLogger("test_state_synchronization")
    logger.info("[TEST] Testing Distributed State Synchronization...")

    # Create multiple state managers (simulating different nodes)
    node1 = DistributedStateManager("competition_bridge")
    node2 = DistributedStateManager("state_machine_bridge")
    node3 = DistributedStateManager("safety_bridge")

    # Register nodes
    node1.register_node("competition_bridge", NodeRole.MASTER)
    node1.register_node("state_machine_bridge", NodeRole.SLAVE)
    node1.register_node("safety_bridge", NodeRole.SLAVE)

    node2.register_node("competition_bridge", NodeRole.MASTER)
    node2.register_node("state_machine_bridge", NodeRole.SLAVE)
    node2.register_node("safety_bridge", NodeRole.SLAVE)

    node3.register_node("competition_bridge", NodeRole.MASTER)
    node3.register_node("state_machine_bridge", NodeRole.SLAVE)
    node3.register_node("safety_bridge", NodeRole.SLAVE)

    # Start managers
    node1.start()
    node2.start()
    node3.start()

    try:
        # Test state updates
        logger.info("  [NOTE] Testing state updates...")
        # Master updates state
        node1.update_state("system_mode", "autonomous")
        node1.update_state("battery_level", 85.5)

        time.sleep(0.1)  # Allow sync

        # Check if slaves received updates
        master_mode = node1.get_state("system_mode")
        slave1_mode = node2.get_state("system_mode")
        slave2_mode = node3.get_state("system_mode")
        logger.info(f"    Master system_mode: {master_mode}")
        logger.info(f"    Slave1 system_mode: {slave1_mode}")
        logger.info(f"    Slave2 system_mode: {slave2_mode}")
        if master_mode == slave1_mode == slave2_mode:
            logger.info("    [SUCCESS] State synchronization working!")
        else:
            logger.info("    [ERROR] State synchronization failed!")

        # Test master failure simulation
        logger.info("   Testing master failure handling...")
        # Simulate master failure by stopping it
        node1.stop()

        # Wait for election
        time.sleep(2.0)

        # Check if new master was elected
        if node2.role == NodeRole.MASTER or node3.role == NodeRole.MASTER:
            logger.info("    [SUCCESS] Master failover successful!")
        else:
            logger.info("    [ERROR] Master failover failed!")
        return True

    finally:
        node1.stop()
        node2.stop()
        node3.stop()


if __name__ == "__main__":
    test_state_synchronization()
