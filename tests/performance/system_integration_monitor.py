#!/usr/bin/env python3
"""
URC 2026 System Integration Monitor

Monitors full system integration including:
- Behavior trees status and health
- State machines transitions and validity
- Network connections and latency
- Simulator status and real-time factor
- System connections and uptime
- Performance metrics collection

Integrates with the metrics dashboard for real-time monitoring.

Author: URC 2026 Systems Integration Team
"""

import threading
import time
import json
import subprocess
import psutil
from typing import Dict, List, Any, Optional, Tuple
import sys
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from tests.performance.testing_metrics_dashboard import metrics_store


class SystemIntegrationMonitor:
    """Monitors full system integration status."""

    def __init__(self):
        self.monitoring_active = False
        self.monitor_thread = None
        self.system_components = {
            'behavior_trees': {},
            'state_machines': {},
            'network': {},
            'simulator': {},
            'connections': {},
            'uptime': {}
        }

    def start_monitoring(self):
        """Start system integration monitoring."""
        self.monitoring_active = True
        self.monitor_thread = threading.Thread(target=self._monitoring_loop, daemon=True)
        self.monitor_thread.start()
        print("🔗 System Integration Monitor started")

    def stop_monitoring(self):
        """Stop system integration monitoring."""
        self.monitoring_active = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=5)
        print("🔗 System Integration Monitor stopped")

    def _monitoring_loop(self):
        """Main monitoring loop."""
        while self.monitoring_active:
            try:
                # Monitor each subsystem
                self._monitor_behavior_trees()
                self._monitor_state_machines()
                self._monitor_network_connections()
                self._monitor_simulator_status()
                self._monitor_system_connections()
                self._monitor_system_uptime()

                # Update metrics store
                for component, subsystems in self.system_components.items():
                    for subsystem_name, subsystem_data in subsystems.items():
                        metrics_store.update_system_status(component, subsystem_name, subsystem_data)

                time.sleep(2.0)  # Monitor every 2 seconds

            except Exception as e:
                print(f"⚠️ System integration monitoring error: {e}")
                time.sleep(5.0)

    def _monitor_behavior_trees(self):
        """Monitor behavior tree status."""
        try:
            # Check for running BT processes
            bt_processes = []
            for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
                try:
                    if 'behavior' in ' '.join(proc.info['cmdline'] or []).lower() or \
                       'bt' in proc.info['name'].lower():
                        bt_processes.append(proc)
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue

            # Update BT status
            for i, proc in enumerate(bt_processes):
                bt_name = f"bt_process_{i+1}"
                self.system_components['behavior_trees'][bt_name] = {
                    'active': True,
                    'status': 'RUNNING',
                    'pid': proc.pid,
                    'cpu_percent': proc.cpu_percent(),
                    'memory_mb': proc.memory_info().rss / (1024 * 1024),
                    'last_check': time.time()
                }

            # Mark old BTs as inactive
            current_time = time.time()
            for bt_name in list(self.system_components['behavior_trees'].keys()):
                if current_time - self.system_components['behavior_trees'][bt_name].get('last_check', 0) > 10:
                    self.system_components['behavior_trees'][bt_name]['active'] = False
                    self.system_components['behavior_trees'][bt_name]['status'] = 'INACTIVE'

        except Exception as e:
            print(f"⚠️ BT monitoring error: {e}")

    def _monitor_state_machines(self):
        """Monitor state machine status."""
        try:
            # Check for SM-related processes
            sm_processes = []
            for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
                try:
                    cmdline = ' '.join(proc.info['cmdline'] or [])
                    if 'state' in cmdline.lower() or 'machine' in cmdline.lower() or \
                       'sm' in proc.info['name'].lower():
                        sm_processes.append(proc)
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue

            # Update SM status
            for i, proc in enumerate(sm_processes):
                sm_name = f"state_machine_{i+1}"
                last_transition_valid = True  # Assume valid unless we detect issues

                self.system_components['state_machines'][sm_name] = {
                    'active': True,
                    'current_state': 'RUNNING',  # Would need actual SM integration
                    'last_transition_valid': last_transition_valid,
                    'pid': proc.pid,
                    'uptime_seconds': time.time() - proc.create_time(),
                    'last_check': time.time()
                }

            # Clean up inactive SMs
            current_time = time.time()
            for sm_name in list(self.system_components['state_machines'].keys()):
                if current_time - self.system_components['state_machines'][sm_name].get('last_check', 0) > 10:
                    self.system_components['state_machines'][sm_name]['active'] = False

        except Exception as e:
            print(f"⚠️ SM monitoring error: {e}")

    def _monitor_network_connections(self):
        """Monitor network connections."""
        try:
            # Check network interfaces
            net_interfaces = psutil.net_if_stats()

            for interface_name, stats in net_interfaces.items():
                if stats.isup:  # Only monitor active interfaces
                    # Get network I/O
                    net_io = psutil.net_io_counters(pernic=True)
                    if interface_name in net_io:
                        io_stats = net_io[interface_name]

                        # Calculate connection health (simplified)
                        latency_ms = 5.0  # Would need actual ping test
                        connected = stats.isup and io_stats.bytes_sent > 0

                        self.system_components['network'][interface_name] = {
                            'connected': connected,
                            'latency_ms': latency_ms,
                            'bytes_sent': io_stats.bytes_sent,
                            'bytes_recv': io_stats.bytes_recv,
                            'speed_mbps': getattr(stats, 'speed', 1000),
                            'last_check': time.time()
                        }

        except Exception as e:
            print(f"⚠️ Network monitoring error: {e}")

    def _monitor_simulator_status(self):
        """Monitor simulator status."""
        try:
            # Check for simulator processes (Gazebo, etc.)
            sim_processes = []
            for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
                try:
                    name = proc.info['name'].lower()
                    cmdline = ' '.join(proc.info['cmdline'] or []).lower()
                    if 'gazebo' in name or 'gzserver' in name or 'gzclient' in name or \
                       'simulator' in cmdline or 'gazebo' in cmdline:
                        sim_processes.append(proc)
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue

            # Update simulator status
            for i, proc in enumerate(sim_processes):
                sim_name = f"simulator_{i+1}"

                # Estimate real-time factor (simplified - would need actual sim integration)
                cpu_usage = proc.cpu_percent()
                real_time_factor = max(0.1, 1.0 - (cpu_usage / 100.0) * 0.5)  # Rough estimate

                self.system_components['simulator'][sim_name] = {
                    'running': True,
                    'pid': proc.pid,
                    'cpu_percent': cpu_usage,
                    'memory_mb': proc.memory_info().rss / (1024 * 1024),
                    'real_time_factor': real_time_factor,
                    'physics_time': time.time() * real_time_factor,  # Simulated
                    'wall_time': time.time(),
                    'last_check': time.time()
                }

            # Clean up inactive simulators
            current_time = time.time()
            for sim_name in list(self.system_components['simulator'].keys()):
                if current_time - self.system_components['simulator'][sim_name].get('last_check', 0) > 10:
                    self.system_components['simulator'][sim_name]['running'] = False

        except Exception as e:
            print(f"⚠️ Simulator monitoring error: {e}")

    def _monitor_system_connections(self):
        """Monitor system-wide connections."""
        try:
            # Count active ROS2 connections (simplified)
            ros2_connections = 0
            try:
                # Check for ROS2 processes
                result = subprocess.run(['pgrep', '-f', 'ros2'], capture_output=True, text=True, timeout=2)
                if result.returncode == 0:
                    ros2_connections = len(result.stdout.strip().split('\n')) if result.stdout.strip() else 0
            except (subprocess.TimeoutExpired, FileNotFoundError):
                ros2_connections = 0

            # Count network connections
            network_connections = len([c for c in self.system_components['network'].values() if c.get('connected', False)])

            # Count IPC connections (shared memory)
            ipc_connections = len([bt for bt in self.system_components['behavior_trees'].values() if bt.get('active', False)])

            total_connections = ros2_connections + network_connections + ipc_connections

            self.system_components['connections']['system_overview'] = {
                'ros2_connections': ros2_connections,
                'network_connections': network_connections,
                'ipc_connections': ipc_connections,
                'total_connections': total_connections,
                'connection_quality': 'GOOD' if total_connections > 0 else 'NONE',
                'last_check': time.time()
            }

        except Exception as e:
            print(f"⚠️ Connection monitoring error: {e}")

    def _monitor_system_uptime(self):
        """Monitor system uptime."""
        try:
            # Get system boot time
            boot_time = psutil.boot_time()

            # Get current process start time
            process = psutil.Process()
            process_start_time = process.create_time()

            self.system_components['uptime']['system_status'] = {
                'system_boot_time': boot_time,
                'system_uptime_seconds': time.time() - boot_time,
                'process_start_time': process_start_time,
                'process_uptime_seconds': time.time() - process_start_time,
                'last_check': time.time()
            }

        except Exception as e:
            print(f"⚠️ Uptime monitoring error: {e}")

    def get_integration_status(self) -> Dict[str, Any]:
        """Get current integration status."""
        return {
            'timestamp': time.time(),
            'monitoring_active': self.monitoring_active,
            'components': self.system_components,
            'health_score': self._calculate_health_score()
        }

    def _calculate_health_score(self) -> float:
        """Calculate overall system health score."""
        scores = []

        # Behavior trees health
        bt_total = len(self.system_components['behavior_trees'])
        bt_active = len([bt for bt in self.system_components['behavior_trees'].values() if bt.get('active', False)])
        scores.append((bt_active / max(bt_total, 1)) * 100)

        # State machines health
        sm_total = len(self.system_components['state_machines'])
        sm_active = len([sm for sm in self.system_components['state_machines'].values() if sm.get('active', False)])
        scores.append((sm_active / max(sm_total, 1)) * 100)

        # Network health
        net_total = len(self.system_components['network'])
        net_connected = len([n for n in self.system_components['network'].values() if n.get('connected', False)])
        scores.append((net_connected / max(net_total, 1)) * 100)

        # Simulator health
        sim_total = len(self.system_components['simulator'])
        sim_running = len([s for s in self.system_components['simulator'].values() if s.get('running', False)])
        scores.append((sim_running / max(sim_total, 1)) * 100)

        return sum(scores) / len(scores) if scores else 0.0

    def simulate_test_data(self):
        """Simulate test data for demonstration."""
        print("🎭 Simulating system integration data...")

        # Simulate behavior trees
        for i in range(3):
            bt_name = f"mission_bt_{i+1}"
            self.system_components['behavior_trees'][bt_name] = {
                'active': True,
                'status': 'RUNNING',
                'current_node': f'waypoint_{i+1}',
                'last_check': time.time()
            }

        # Simulate state machines
        for i in range(2):
            sm_name = f"rover_sm_{i+1}"
            self.system_components['state_machines'][sm_name] = {
                'active': True,
                'current_state': 'NAVIGATING',
                'last_transition_valid': True,
                'last_check': time.time()
            }

        # Simulate network connections
        self.system_components['network']['wifi_main'] = {
            'connected': True,
            'latency_ms': 5.2,
            'signal_strength': -45,
            'last_check': time.time()
        }

        self.system_components['network']['lte_backup'] = {
            'connected': False,
            'latency_ms': 150.0,
            'last_check': time.time()
        }

        # Simulate simulator
        self.system_components['simulator']['gazebo_world'] = {
            'running': True,
            'real_time_factor': 0.95,
            'physics_time': time.time() * 0.95,
            'last_check': time.time()
        }

        # Update metrics store with simulated data
        for component, subsystems in self.system_components.items():
            for subsystem_name, subsystem_data in subsystems.items():
                metrics_store.update_system_status(component, subsystem_name, subsystem_data)

        print("✅ Test data simulation complete")


# Global monitor instance
system_monitor = SystemIntegrationMonitor()


def start_system_integration_monitoring():
    """Start system integration monitoring."""
    system_monitor.start_monitoring()
    return system_monitor


def stop_system_integration_monitoring():
    """Stop system integration monitoring."""
    system_monitor.stop_monitoring()


def get_system_integration_status():
    """Get current system integration status."""
    return system_monitor.get_integration_status()


if __name__ == "__main__":
    # Test the monitor
    print("Testing System Integration Monitor...")

    # Start monitoring
    system_monitor.start_monitoring()

    # Simulate some data
    system_monitor.simulate_test_data()

    # Run for a bit
    time.sleep(5)

    # Get status
    status = system_monitor.get_integration_status()
    print(f"System Health Score: {status.get('health_score', 0):.1f}%")

    # Stop monitoring
    system_monitor.stop_monitoring()

    print("System Integration Monitor test complete")
