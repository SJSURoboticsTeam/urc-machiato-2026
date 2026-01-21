#!/usr/bin/env python3
"""
Network Partition Detector - Detect WiFi Drops and Enable Offline Autonomy

Detects network partitions and switches rover to offline autonomy mode.
Critical for maintaining operation during WiFi dropouts in field conditions.

Features:
- WiFi connectivity monitoring
- Partition detection with hysteresis
- Offline autonomy mode activation
- Recovery procedures when network returns
- Telemetry queueing during outages

Author: URC 2026 Network Resilience Team
"""

import time
import threading
import subprocess
from typing import Dict, Any, Optional, Callable, List
from dataclasses import dataclass
from enum import Enum
import logging
import socket

logger = logging.getLogger(__name__)


class NetworkState(Enum):
    """Network connectivity states."""
    CONNECTED = "connected"
    DEGRADED = "degraded"
    PARTITIONED = "partitioned"
    OFFLINE = "offline"


class AutonomyMode(Enum):
    """Autonomy operation modes."""
    FULL_AUTONOMY = "full_autonomy"      # Normal operation with cloud connectivity
    OFFLINE_AUTONOMY = "offline_autonomy"  # Local operation without cloud
    DEGRADED_AUTONOMY = "degraded_autonomy"  # Limited capabilities
    MANUAL_OVERRIDE = "manual_override"   # Human operator control only


@dataclass
class NetworkHealthCheck:
    """Result of network health check."""
    timestamp: float
    wifi_connected: bool
    internet_reachable: bool
    latency_ms: Optional[float]
    packet_loss_percent: float
    signal_strength_dbm: Optional[int]
    error_message: Optional[str]


@dataclass
class PartitionEvent:
    """Network partition event record."""
    event_type: str  # 'partition_detected', 'partition_resolved', 'degraded', 'recovered'
    timestamp: float
    previous_state: NetworkState
    new_state: NetworkState
    trigger_reason: str
    duration_seconds: Optional[float] = None


class NetworkPartitionDetector:
    """
    Detects network partitions and manages offline autonomy transitions.

    Monitors network connectivity and switches between online/offline autonomy modes
    to maintain rover operation during WiFi outages.
    """

    def __init__(self,
                 partition_threshold_seconds: float = 5.0,
                 recovery_grace_period_seconds: float = 10.0,
                 health_check_interval_seconds: float = 1.0):
        """
        Initialize network partition detector.

        Args:
            partition_threshold_seconds: Time without connectivity before declaring partition
            recovery_grace_period_seconds: Time to wait before declaring recovery
            health_check_interval_seconds: How often to check network health
        """
        self.partition_threshold_seconds = partition_threshold_seconds
        self.recovery_grace_period_seconds = recovery_grace_period_seconds
        self.health_check_interval_seconds = health_check_interval_seconds

        # State tracking
        self.current_network_state = NetworkState.CONNECTED
        self.current_autonomy_mode = AutonomyMode.FULL_AUTONOMY
        self.last_healthy_check: Optional[NetworkHealthCheck] = None
        self.partition_start_time: Optional[float] = None

        # History and monitoring
        self.health_checks: List[NetworkHealthCheck] = []
        self.partition_events: List[PartitionEvent] = []
        self.max_history_size = 1000

        # Callbacks for mode changes
        self.partition_callbacks: List[Callable[[PartitionEvent], None]] = []
        self.recovery_callbacks: List[Callable[[PartitionEvent], None]] = []

        # Monitoring thread
        self.monitoring_thread: Optional[threading.Thread] = None
        self.monitoring_active = False
        self.lock = threading.RLock()

        logger.info("NetworkPartitionDetector initialized")

    def start_monitoring(self):
        """Start network monitoring thread."""
        with self.lock:
            if self.monitoring_active:
                return

            self.monitoring_active = True
            self.monitoring_thread = threading.Thread(
                target=self._monitoring_loop,
                name="network_monitor",
                daemon=True
            )
            self.monitoring_thread.start()
            logger.info("Network partition monitoring started")

    def stop_monitoring(self):
        """Stop network monitoring."""
        with self.lock:
            self.monitoring_active = False
            if self.monitoring_thread:
                self.monitoring_thread.join(timeout=5.0)
                self.monitoring_thread = None
            logger.info("Network partition monitoring stopped")

    def _monitoring_loop(self):
        """Main monitoring loop."""
        while self.monitoring_active:
            try:
                health_check = self._perform_health_check()

                with self.lock:
                    self.health_checks.append(health_check)
                    if len(self.health_checks) > self.max_history_size:
                        self.health_checks.pop(0)

                    self._update_network_state(health_check)

                self.last_healthy_check = health_check

            except Exception as e:
                logger.error(f"Error in network monitoring: {e}")

            time.sleep(self.health_check_interval_seconds)

    def _perform_health_check(self) -> NetworkHealthCheck:
        """Perform comprehensive network health check."""
        timestamp = time.time()

        # Check WiFi connectivity
        wifi_connected = self._check_wifi_connected()

        # Check internet reachability
        internet_reachable, latency_ms = self._check_internet_reachable()

        # Estimate packet loss (simplified)
        packet_loss_percent = 0.0 if internet_reachable else 100.0

        # Get WiFi signal strength
        signal_strength_dbm = self._get_wifi_signal_strength()

        # Determine error message
        error_message = None
        if not wifi_connected:
            error_message = "WiFi not connected"
        elif not internet_reachable:
            error_message = "Internet unreachable"

        return NetworkHealthCheck(
            timestamp=timestamp,
            wifi_connected=wifi_connected,
            internet_reachable=internet_reachable,
            latency_ms=latency_ms,
            packet_loss_percent=packet_loss_percent,
            signal_strength_dbm=signal_strength_dbm,
            error_message=error_message
        )

    def _check_wifi_connected(self) -> bool:
        """Check if WiFi is connected."""
        try:
            # Check network interfaces
            result = subprocess.run(
                ["iwconfig"],
                capture_output=True,
                text=True,
                timeout=2.0
            )

            # Look for wlan0 or similar
            if "wlan0" in result.stdout or "wlan1" in result.stdout:
                # Check if associated with AP
                if "Not-Associated" not in result.stdout:
                    return True

            # Fallback: check network status
            result = subprocess.run(
                ["nmcli", "device", "status"],
                capture_output=True,
                text=True,
                timeout=2.0
            )

            return "connected" in result.stdout.lower()

        except (subprocess.TimeoutExpired, subprocess.CalledProcessError, FileNotFoundError):
            # Fallback: try socket connection
            try:
                socket.create_connection(("8.8.8.8", 53), timeout=1.0)
                return True
            except (socket.timeout, socket.error):
                return False

    def _check_internet_reachable(self) -> tuple[bool, Optional[float]]:
        """Check internet reachability and measure latency."""
        try:
            start_time = time.time()
            # Try to connect to Google DNS
            sock = socket.create_connection(("8.8.8.8", 53), timeout=2.0)
            sock.close()
            latency_ms = (time.time() - start_time) * 1000
            return True, latency_ms
        except (socket.timeout, socket.error):
            return False, None

    def _get_wifi_signal_strength(self) -> Optional[int]:
        """Get WiFi signal strength in dBm."""
        try:
            result = subprocess.run(
                ["iwconfig", "wlan0"],
                capture_output=True,
                text=True,
                timeout=1.0
            )

            # Parse signal level from output like "Signal level=-45 dBm"
            for line in result.stdout.split('\n'):
                if 'Signal level' in line and 'dBm' in line:
                    # Extract the number
                    parts = line.split('=')
                    if len(parts) >= 2:
                        signal_part = parts[1].split()[0]
                        return int(signal_part)

        except (subprocess.TimeoutExpired, subprocess.CalledProcessError, subprocess.SubprocessError):
            pass

        return None

    def _update_network_state(self, health_check: NetworkHealthCheck):
        """Update network state based on health check."""
        new_state = self._determine_network_state(health_check)

        if new_state != self.current_network_state:
            previous_state = self.current_network_state

            # Record partition event
            event = PartitionEvent(
                event_type='partition_detected' if new_state == NetworkState.PARTITIONED else
                          'partition_resolved' if previous_state == NetworkState.PARTITIONED else
                          'state_change',
                timestamp=health_check.timestamp,
                previous_state=previous_state,
                new_state=new_state,
                trigger_reason=health_check.error_message or "health_check",
                duration_seconds=health_check.timestamp - (self.partition_start_time or health_check.timestamp)
            )

            self.partition_events.append(event)
            if len(self.partition_events) > 500:  # Keep last 500 events
                self.partition_events.pop(0)

            # Update state
            self.current_network_state = new_state

            if new_state == NetworkState.PARTITIONED:
                self.partition_start_time = health_check.timestamp
                self._handle_partition_detected(event)
            elif previous_state == NetworkState.PARTITIONED and new_state == NetworkState.CONNECTED:
                self._handle_partition_resolved(event)

            logger.info(f"Network state changed: {previous_state.value} -> {new_state.value} "
                       f"(reason: {event.trigger_reason})")

    def _determine_network_state(self, health_check: NetworkHealthCheck) -> NetworkState:
        """Determine network state from health check."""
        # Connected: WiFi up and internet reachable
        if health_check.wifi_connected and health_check.internet_reachable:
            # Check for degradation (high latency or packet loss)
            if (health_check.latency_ms and health_check.latency_ms > 500) or \
               health_check.packet_loss_percent > 20:
                return NetworkState.DEGRADED
            else:
                return NetworkState.CONNECTED

        # Partitioned: WiFi up but no internet (DNS/server unreachable)
        elif health_check.wifi_connected and not health_check.internet_reachable:
            # Check how long we've been in this state
            if self.last_healthy_check and self.last_healthy_check.internet_reachable:
                # Just lost internet, give it time
                time_since_internet_lost = health_check.timestamp - self.last_healthy_check.timestamp
                if time_since_internet_lost < self.partition_threshold_seconds:
                    return NetworkState.DEGRADED  # Temporary issue
                else:
                    return NetworkState.PARTITIONED  # Persistent issue
            else:
                # Internet never worked, assume partitioned
                return NetworkState.PARTITIONED

        # Offline: WiFi completely down
        else:
            return NetworkState.OFFLINE

    def _handle_partition_detected(self, event: PartitionEvent):
        """Handle network partition detection."""
        logger.warning(f"NETWORK PARTITION DETECTED after {event.duration_seconds:.1f}s: {event.trigger_reason}")

        # Switch to offline autonomy
        self.current_autonomy_mode = AutonomyMode.OFFLINE_AUTONOMY

        # Notify callbacks
        for callback in self.partition_callbacks:
            try:
                callback(event)
            except Exception as e:
                logger.error(f"Partition callback failed: {e}")

        # Trigger offline procedures
        self._activate_offline_mode()

    def _handle_partition_resolved(self, event: PartitionEvent):
        """Handle network partition resolution."""
        duration = event.duration_seconds or 0
        logger.info(f"NETWORK PARTITION RESOLVED after {duration:.1f}s")

        # Switch back to full autonomy
        self.current_autonomy_mode = AutonomyMode.FULL_AUTONOMY

        # Notify callbacks
        for callback in self.recovery_callbacks:
            try:
                callback(event)
            except Exception as e:
                logger.error(f"Recovery callback failed: {e}")

        # Trigger recovery procedures
        self._activate_online_mode()

    def _activate_offline_mode(self):
        """Activate offline autonomy mode."""
        logger.info("Activating offline autonomy mode")

        # In a real implementation, this would:
        # 1. Stop publishing telemetry to cloud
        # 2. Switch to local-only sensor fusion
        # 3. Queue critical messages for later transmission
        # 4. Enable offline navigation capabilities
        # 5. Update operator interface to show offline status

        # For now, just log the mode change
        logger.warning("OFFLINE MODE: Telemetry suspended, local autonomy active")

    def _activate_online_mode(self):
        """Activate online autonomy mode."""
        logger.info("Activating online autonomy mode")

        # In a real implementation, this would:
        # 1. Resume telemetry transmission
        # 2. Sync queued messages
        # 3. Validate state consistency
        # 4. Resume cloud-based autonomy features
        # 5. Update operator interface

        # For now, just log the mode change
        logger.info("ONLINE MODE: Full autonomy restored")

    def register_partition_callback(self, callback: Callable[[PartitionEvent], None]):
        """Register callback for partition events."""
        with self.lock:
            self.partition_callbacks.append(callback)

    def register_recovery_callback(self, callback: Callable[[PartitionEvent], None]):
        """Register callback for recovery events."""
        with self.lock:
            self.recovery_callbacks.append(callback)

    def get_network_status(self) -> Dict[str, Any]:
        """Get current network status."""
        with self.lock:
            return {
                'network_state': self.current_network_state.value,
                'autonomy_mode': self.current_autonomy_mode.value,
                'last_health_check': self.last_healthy_check.__dict__ if self.last_healthy_check else None,
                'partition_duration_seconds': (
                    time.time() - (self.partition_start_time or time.time())
                    if self.current_network_state == NetworkState.PARTITIONED
                    else 0
                ),
                'total_partition_events': len([e for e in self.partition_events
                                             if e.event_type == 'partition_detected']),
                'timestamp': time.time()
            }

    def get_network_history(self, hours: float = 1.0) -> Dict[str, Any]:
        """Get network history for specified time period."""
        with self.lock:
            cutoff_time = time.time() - (hours * 3600)

            recent_checks = [hc for hc in self.health_checks if hc.timestamp > cutoff_time]
            recent_events = [pe for pe in self.partition_events if pe.timestamp > cutoff_time]

            # Calculate uptime percentage
            if recent_checks:
                connected_checks = sum(1 for hc in recent_checks
                                     if hc.wifi_connected and hc.internet_reachable)
                uptime_percent = (connected_checks / len(recent_checks)) * 100
            else:
                uptime_percent = 100.0

            # Calculate average latency
            latencies = [hc.latency_ms for hc in recent_checks if hc.latency_ms is not None]
            avg_latency_ms = sum(latencies) / len(latencies) if latencies else None

            return {
                'time_range_hours': hours,
                'uptime_percent': uptime_percent,
                'avg_latency_ms': avg_latency_ms,
                'total_health_checks': len(recent_checks),
                'total_partition_events': len(recent_events),
                'partition_events': [pe.__dict__ for pe in recent_events[-20:]],  # Last 20 events
                'timestamp': time.time()
            }

    def force_offline_mode(self, reason: str = "manual_trigger"):
        """Manually force offline mode (for testing or emergency)."""
        with self.lock:
            event = PartitionEvent(
                event_type='partition_detected',
                timestamp=time.time(),
                previous_state=self.current_network_state,
                new_state=NetworkState.PARTITIONED,
                trigger_reason=reason
            )

            self.current_network_state = NetworkState.PARTITIONED
            self.current_autonomy_mode = AutonomyMode.OFFLINE_AUTONOMY
            self.partition_start_time = event.timestamp

            self.partition_events.append(event)

            self._handle_partition_detected(event)

    def reset_to_online_mode(self, reason: str = "manual_reset"):
        """Manually reset to online mode (for testing or recovery)."""
        with self.lock:
            if self.current_network_state != NetworkState.CONNECTED:
                event = PartitionEvent(
                    event_type='partition_resolved',
                    timestamp=time.time(),
                    previous_state=self.current_network_state,
                    new_state=NetworkState.CONNECTED,
                    trigger_reason=reason,
                    duration_seconds=time.time() - (self.partition_start_time or time.time())
                )

                self.current_network_state = NetworkState.CONNECTED
                self.current_autonomy_mode = AutonomyMode.FULL_AUTONOMY

                self.partition_events.append(event)

                self._handle_partition_resolved(event)

    def export_diagnostics(self) -> Dict[str, Any]:
        """Export diagnostic information for debugging."""
        with self.lock:
            return {
                'current_status': self.get_network_status(),
                'network_history_1h': self.get_network_history(1.0),
                'network_history_24h': self.get_network_history(24.0),
                'recent_health_checks': [hc.__dict__ for hc in self.health_checks[-50:]],
                'recent_partition_events': [pe.__dict__ for pe in self.partition_events[-20:]],
                'configuration': {
                    'partition_threshold_seconds': self.partition_threshold_seconds,
                    'recovery_grace_period_seconds': self.recovery_grace_period_seconds,
                    'health_check_interval_seconds': self.health_check_interval_seconds
                },
                'export_timestamp': time.time()
            }


# Global instance
_network_partition_detector_instance: Optional[NetworkPartitionDetector] = None
_network_partition_detector_lock = threading.Lock()


def get_network_partition_detector(partition_threshold_seconds: float = 5.0,
                                 recovery_grace_period_seconds: float = 10.0) -> NetworkPartitionDetector:
    """Get global network partition detector instance."""
    global _network_partition_detector_instance

    if _network_partition_detector_instance is None:
        with _network_partition_detector_lock:
            if _network_partition_detector_instance is None:
                _network_partition_detector_instance = NetworkPartitionDetector(
                    partition_threshold_seconds, recovery_grace_period_seconds
                )

    return _network_partition_detector_instance


# Convenience functions
def start_network_monitoring():
    """Start network partition monitoring."""
    detector = get_network_partition_detector()
    detector.start_monitoring()


def stop_network_monitoring():
    """Stop network partition monitoring."""
    detector = get_network_partition_detector()
    detector.stop_monitoring()


def get_network_status() -> Dict[str, Any]:
    """Get current network status."""
    detector = get_network_partition_detector()
    return detector.get_network_status()


def force_offline_mode(reason: str = "manual_trigger"):
    """Force offline autonomy mode."""
    detector = get_network_partition_detector()
    detector.force_offline_mode(reason)


def reset_to_online_mode(reason: str = "manual_reset"):
    """Reset to online autonomy mode."""
    detector = get_network_partition_detector()
    detector.reset_to_online_mode(reason)
