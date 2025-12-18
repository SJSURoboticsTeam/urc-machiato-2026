#!/usr/bin/env python3
"""
DDS Domain Redundancy Manager

Provides ROS2 DDS domain failover capability to survive DDS middleware
failures and network partitioning events.

Author: URC 2026 Autonomy Team
"""

import os
import time
import subprocess
import threading
import signal
from typing import Dict, List, Any, Optional, Callable
from dataclasses import dataclass, field
from enum import Enum


class DomainStatus(Enum):
    """Status of a DDS domain."""
    ACTIVE = "active"
    STANDBY = "standby"
    FAILED = "failed"
    RECOVERING = "recovering"


class FailoverStrategy(Enum):
    """Strategies for domain failover."""
    IMMEDIATE = "immediate"      # Fail over immediately on detection
    GRACEFUL = "graceful"        # Allow current operations to complete
    CONSENSUS = "consensus"      # Require agreement from multiple nodes


@dataclass
class DDSDomain:
    """Configuration for a DDS domain."""
    domain_id: int
    name: str
    status: DomainStatus = DomainStatus.STANDBY
    node_count: int = 0
    last_health_check: float = 0.0
    health_score: float = 1.0  # 1.0 = perfect health
    failover_priority: int = 1  # Lower number = higher priority


@dataclass
class NodeInfo:
    """Information about a ROS2 node."""
    name: str
    namespace: str = ""
    pid: Optional[int] = None
    domain_id: int = 42
    restart_command: Optional[str] = None
    last_restart: float = 0.0
    restart_count: int = 0


class DDSDomainRedundancyManager:
    """
    Manages DDS domain redundancy for ROS2 systems.

    Provides:
    - Multi-domain DDS setup with automatic failover
    - Node lifecycle management across domains
    - Health monitoring of DDS domains
    - Seamless domain transitions
    """

    def __init__(self, primary_domain: int = 42):
        self.primary_domain_id = primary_domain
        self.current_domain_id = primary_domain

        # Domain management
        self.domains: Dict[int, DDSDomain] = {}
        self._init_domains()

        # Node management
        self.nodes: Dict[str, NodeInfo] = {}
        self.node_processes: Dict[str, subprocess.Popen] = {}

        # Health monitoring
        self.health_check_interval = 5.0  # seconds
        self.domain_failure_threshold = 3  # consecutive failures
        self.node_restart_delay = 2.0  # seconds between node restarts

        # Callbacks
        self.failover_callbacks: List[Callable] = []
        self.health_callbacks: List[Callable] = []

        # Control
        self.running = False
        self.monitor_thread: Optional[threading.Thread] = None
        self.failover_in_progress = False

        # Configuration
        self.failover_strategy = FailoverStrategy.GRACEFUL
        self.consensus_required = False  # For CONSENSUS strategy

    def _init_domains(self):
        """Initialize DDS domains."""
        domains_config = [
            (self.primary_domain_id, "primary", 1),
            (self.primary_domain_id + 1, "backup", 2),
            (self.primary_domain_id + 2, "emergency", 3)
        ]

        for domain_id, name, priority in domains_config:
            self.domains[domain_id] = DDSDomain(
                domain_id=domain_id,
                name=name,
                status=DomainStatus.ACTIVE if domain_id == self.primary_domain_id else DomainStatus.STANDBY,
                failover_priority=priority
            )

    def start(self):
        """Start the DDS domain redundancy manager."""
        if self.running:
            return

        self.running = True
        self.monitor_thread = threading.Thread(target=self._monitoring_loop, daemon=True)
        self.monitor_thread.start()

        # Set primary domain as active
        self._activate_domain(self.primary_domain_id)

    def stop(self):
        """Stop the DDS domain redundancy manager."""
        self.running = False

        if self.monitor_thread and self.monitor_thread.is_alive():
            self.monitor_thread.join(timeout=5.0)

        # Stop all managed processes
        self._stop_all_nodes()

    def register_node(self, node_name: str, restart_command: str,
                     namespace: str = "", domain_id: Optional[int] = None):
        """Register a ROS2 node for management."""
        if domain_id is None:
            domain_id = self.current_domain_id

        node_info = NodeInfo(
            name=node_name,
            namespace=namespace,
            domain_id=domain_id,
            restart_command=restart_command
        )

        self.nodes[node_name] = node_info

        # Update domain node count
        if domain_id in self.domains:
            self.domains[domain_id].node_count += 1

    def unregister_node(self, node_name: str):
        """Remove a node from management."""
        if node_name in self.nodes:
            node_info = self.nodes[node_name]
            if node_info.domain_id in self.domains:
                self.domains[node_info.domain_id].node_count -= 1

            # Stop process if running
            if node_name in self.node_processes:
                self._stop_node_process(node_name)

            del self.nodes[node_name]

    def trigger_domain_failover(self, target_domain_id: Optional[int] = None) -> bool:
        """Trigger a failover to a different DDS domain."""
        if self.failover_in_progress:
            return False

        if target_domain_id is None:
            # Find best available domain
            target_domain_id = self._select_failover_domain()

        if target_domain_id == self.current_domain_id:
            return False

        self.failover_in_progress = True

        try:
            success = self._execute_domain_failover(target_domain_id)

            if success:
                self._trigger_failover_callbacks(target_domain_id)

            return success

        finally:
            self.failover_in_progress = False

    def add_failover_callback(self, callback: Callable[[int, int], None]):
        """Add callback for failover events (old_domain, new_domain)."""
        self.failover_callbacks.append(callback)

    def add_health_callback(self, callback: Callable[[int, float], None]):
        """Add callback for health changes (domain_id, health_score)."""
        self.health_callbacks.append(callback)

    def get_system_status(self) -> Dict[str, Any]:
        """Get comprehensive system status."""
        return {
            'current_domain': self.current_domain_id,
            'domains': {
                domain_id: {
                    'name': domain.name,
                    'status': domain.status.value,
                    'node_count': domain.node_count,
                    'health_score': domain.health_score,
                    'last_health_check': domain.last_health_check
                }
                for domain_id, domain in self.domains.items()
            },
            'nodes': {
                node_name: {
                    'domain_id': node.domain_id,
                    'pid': node.pid,
                    'restart_count': node.restart_count,
                    'last_restart': node.last_restart
                }
                for node_name, node in self.nodes.items()
            },
            'failover_in_progress': self.failover_in_progress,
            'timestamp': time.time()
        }

    def _monitoring_loop(self):
        """Main monitoring loop for domain health."""
        while self.running:
            try:
                self._check_domain_health()
                self._check_node_health()

                # Check for automatic failover
                if self._should_trigger_failover():
                    self.trigger_domain_failover()

                time.sleep(self.health_check_interval)

            except Exception as e:
        self.get_logger().info(f"DDS monitoring error: {e}")
                time.sleep(self.health_check_interval)

    def _check_domain_health(self):
        """Check health of all DDS domains."""
        current_time = time.time()

        for domain_id, domain in self.domains.items():
            try:
                # Check if domain is accessible by testing node discovery
                health_score = self._measure_domain_health(domain_id)

                # Update domain status
                old_score = domain.health_score
                domain.health_score = health_score
                domain.last_health_check = current_time

                # Update domain status based on health
                if health_score > 0.8:
                    domain.status = DomainStatus.ACTIVE if domain_id == self.current_domain_id else DomainStatus.STANDBY
                elif health_score > 0.3:
                    domain.status = DomainStatus.RECOVERING
                else:
                    domain.status = DomainStatus.FAILED

                # Trigger health callbacks if score changed significantly
                if abs(old_score - health_score) > 0.1:
                    self._trigger_health_callbacks(domain_id, health_score)

            except Exception as e:
        self.get_logger().info(f"Domain health check error for {domain_id}: {e}")
                domain.health_score = 0.0
                domain.status = DomainStatus.FAILED

    def _check_node_health(self):
        """Check health of managed nodes and restart if necessary."""
        for node_name, node_info in self.nodes.items():
            if node_name in self.node_processes:
                process = self.node_processes[node_name]

                # Check if process is still running
                if process.poll() is not None:
        self.get_logger().info(f"Node {node_name} process died (exit code: {process.returncode})")
                    # Restart the node
                    self._restart_node(node_name)

    def _measure_domain_health(self, domain_id: int) -> float:
        """Measure the health of a DDS domain."""
        try:
            # Test DDS connectivity by attempting to list nodes in the domain
            env = os.environ.copy()
            env['ROS_DOMAIN_ID'] = str(domain_id)

            # Try to run a quick ROS2 command to test domain accessibility
            result = subprocess.run(
                ['ros2', 'node', 'list'],
                env=env,
                capture_output=True,
                text=True,
                timeout=2.0
            )

            if result.returncode == 0:
                # Domain is accessible - return health based on node count and response time
                node_count = len(result.stdout.strip().split('\n')) if result.stdout.strip() else 0

                # Health score based on node count and accessibility
                base_health = 0.5  # Base score for accessibility
                node_bonus = min(0.5, node_count * 0.1)  # Bonus for active nodes

                return base_health + node_bonus
            else:
                return 0.0  # Domain not accessible

        except subprocess.TimeoutExpired:
            return 0.1  # Slow but accessible
        except Exception:
            return 0.0  # Not accessible

    def _should_trigger_failover(self) -> bool:
        """Determine if a domain failover should be triggered."""
        current_domain = self.domains.get(self.current_domain_id)

        if not current_domain:
            return True

        # Check health threshold
        if current_domain.health_score < 0.3:
            return True

        # Check consecutive failures (simplified - in real implementation would track history)
        if current_domain.status == DomainStatus.FAILED:
            return True

        return False

    def _select_failover_domain(self) -> int:
        """Select the best domain for failover."""
        # Find healthy domains excluding current
        available_domains = [
            domain for domain in self.domains.values()
            if domain.domain_id != self.current_domain_id and
            domain.status in [DomainStatus.ACTIVE, DomainStatus.STANDBY] and
            domain.health_score > 0.5
        ]

        if not available_domains:
            # No healthy domains, use emergency domain
            emergency_domain = next(
                (d for d in self.domains.values() if d.failover_priority == 3),
                None
            )
            return emergency_domain.domain_id if emergency_domain else self.primary_domain_id

        # Select domain with best health score and highest priority (lowest number)
        best_domain = min(available_domains,
                         key=lambda d: (d.failover_priority, -d.health_score))

        return best_domain.domain_id

    def _execute_domain_failover(self, target_domain_id: int) -> bool:
        """Execute the actual domain failover."""
        self.get_logger().info(f"[ALERT] Executing DDS domain failover: {self.current_domain_id} → {target_domain_id}")
        try:
            # Step 1: Stop all nodes in current domain gracefully
            self._graceful_node_shutdown()

            # Step 2: Switch to new domain
            old_domain = self.current_domain_id
            self.current_domain_id = target_domain_id

            # Update domain statuses
            if old_domain in self.domains:
                self.domains[old_domain].status = DomainStatus.STANDBY
            if target_domain_id in self.domains:
                self.domains[target_domain_id].status = DomainStatus.ACTIVE

            # Step 3: Restart all nodes in new domain
            self._restart_all_nodes_in_domain(target_domain_id)
        self.get_logger().info(f"[SUCCESS] Domain failover completed successfully")
            return True

        except Exception as e:
        self.get_logger().info(f"[ERROR] Domain failover failed: {e}")
            # Attempt rollback
            try:
                self.current_domain_id = old_domain
                if old_domain in self.domains:
                    self.domains[old_domain].status = DomainStatus.ACTIVE
                self._restart_all_nodes_in_domain(old_domain)
        self.get_logger().info("[SUCCESS] Failover rollback completed")
            except Exception as rollback_error:
        self.get_logger().info(f"[ERROR] Failover rollback also failed: {rollback_error}")
            return False

    def _activate_domain(self, domain_id: int):
        """Activate a specific DDS domain."""
        if domain_id in self.domains:
            self.domains[domain_id].status = DomainStatus.ACTIVE
            self.current_domain_id = domain_id

    def _graceful_node_shutdown(self, timeout: float = 10.0):
        """Gracefully shut down all managed nodes."""
        self.get_logger().info("[STOP] Gracefully shutting down nodes...")
        shutdown_start = time.time()

        # Send SIGTERM to all processes
        for node_name in list(self.node_processes.keys()):
            self._stop_node_process(node_name, graceful=True)

        # Wait for processes to terminate
        while self.node_processes and (time.time() - shutdown_start) < timeout:
            still_running = [name for name, proc in self.node_processes.items()
                           if proc.poll() is None]
            if not still_running:
                break
            time.sleep(0.5)

        # Force kill any remaining processes
        for node_name in list(self.node_processes.keys()):
            if self.node_processes[node_name].poll() is None:
                self._stop_node_process(node_name, graceful=False)

    def _restart_all_nodes_in_domain(self, domain_id: int):
        """Restart all nodes in the specified domain."""
        self.get_logger().info(f"[UPDATE] Restarting nodes in domain {domain_id}...")
        for node_name, node_info in self.nodes.items():
            if node_info.domain_id == domain_id:
                self._restart_node(node_name)

    def _restart_node(self, node_name: str):
        """Restart a specific node."""
        node_info = self.nodes.get(node_name)
        if not node_info or not node_info.restart_command:
            return

        # Rate limit restarts
        time_since_last_restart = time.time() - node_info.last_restart
        if time_since_last_restart < self.node_restart_delay:
            time.sleep(self.node_restart_delay - time_since_last_restart)

        try:
            # Stop existing process if running
            if node_name in self.node_processes:
                self._stop_node_process(node_name)

            # Set environment for correct domain
            env = os.environ.copy()
            env['ROS_DOMAIN_ID'] = str(node_info.domain_id)
        self.get_logger().info(f"[START] Restarting {node_name} in domain {node_info.domain_id}")
            # Start new process
            process = subprocess.Popen(
                node_info.restart_command.split(),
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )

            self.node_processes[node_name] = process
            node_info.pid = process.pid
            node_info.last_restart = time.time()
            node_info.restart_count += 1

        except Exception as e:
        self.get_logger().info(f"Failed to restart node {node_name}: {e}")
    def _stop_node_process(self, node_name: str, graceful: bool = True):
        """Stop a node process."""
        if node_name not in self.node_processes:
            return

        process = self.node_processes[node_name]

        if process.poll() is None:  # Still running
            if graceful:
                process.terminate()
                try:
                    process.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            else:
                process.kill()
                process.wait()

        del self.node_processes[node_name]

    def _stop_all_nodes(self):
        """Stop all managed node processes."""
        for node_name in list(self.node_processes.keys()):
            self._stop_node_process(node_name, graceful=False)

    def _trigger_failover_callbacks(self, new_domain_id: int):
        """Trigger failover callbacks."""
        for callback in self.failover_callbacks:
            try:
                callback(self.current_domain_id, new_domain_id)
            except Exception as e:
        self.get_logger().info(f"Failover callback error: {e}")
    def _trigger_health_callbacks(self, domain_id: int, health_score: float):
        """Trigger health change callbacks."""
        for callback in self.health_callbacks:
            try:
                callback(domain_id, health_score)
            except Exception as e:
        self.get_logger().info(f"Health callback error: {e}")
# Global DDS redundancy manager instance
_dds_manager = None


def get_dds_redundancy_manager(primary_domain: int = 42) -> DDSDomainRedundancyManager:
    """Get the global DDS domain redundancy manager instance."""
    global _dds_manager
    if _dds_manager is None:
        _dds_manager = DDSDomainRedundancyManager(primary_domain)
    return _dds_manager


# Example usage and testing
def test_dds_domain_redundancy():
    """Test DDS domain redundancy functionality."""
        self.get_logger().info("[TEST] Testing DDS Domain Redundancy...")
    manager = DDSDomainRedundancyManager(primary_domain=42)

    # Register test nodes
    manager.register_node("competition_bridge", "python3 src/bridges/competition_bridge.py")
    manager.register_node("state_machine_bridge", "python3 src/bridges/ros2_state_machine_bridge.py")
    manager.register_node("sensor_bridge", "python3 src/autonomy/perception/sensor_bridge/sensor_bridge_node.py")

    # Start manager
    manager.start()

    try:
        self.get_logger().info("  [STATUS] Testing domain health monitoring...")
        time.sleep(2)

        status = manager.get_system_status()
        self.get_logger().info(f"    Current domain: {status['current_domain']}")
        self.get_logger().info(f"    Domains: {list(status['domains'].keys())}")
        self.get_logger().info(f"    Registered nodes: {list(status['nodes'].keys())}")
        # Simulate domain failure and failover
        self.get_logger().info("  [UPDATE] Testing domain failover...")
        success = manager.trigger_domain_failover()

        if success:
        self.get_logger().info("    [SUCCESS] Domain failover successful!")
            new_status = manager.get_system_status()
        self.get_logger().info(f"    New domain: {new_status['current_domain']}")
        else:
        self.get_logger().info("    [ERROR] Domain failover failed!")
        return True

    finally:
        manager.stop()


if __name__ == "__main__":
    test_dds_domain_redundancy()
