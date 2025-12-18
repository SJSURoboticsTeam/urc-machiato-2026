#!/usr/bin/env python3
"""
Spectrum Monitor - URC Competition Spectrum Compliance

Monitors spectrum usage and ensures FCC compliance per URC requirements.
Handles interference detection, bandwidth monitoring, and compliance reporting.
"""

import time
from typing import Any, Dict, List, Optional

from .constants import URC_900MHZ_MAX_BANDWIDTH


class SpectrumMonitor:
    """
    Monitors spectrum usage and ensures FCC compliance per URC requirements.

    Handles:
    - Bandwidth usage monitoring
    - Interference detection
    - FCC compliance validation
    - Spectrum violation reporting
    - URC band management
    """

    def __init__(self, logger):
        """
        Initialize the Spectrum Monitor.

        Args:
            logger: Logger instance for spectrum monitoring
        """
        self.logger = logger

        # URC Band Configuration (competition restrictions)
        self.urc_band_config = {
            "900mhz": {
                "max_bandwidth_mhz": URC_900MHZ_MAX_BANDWIDTH,  # 8 MHz max for entire band
                "sub_bands": {
                    "low": {"range": (902, 910), "active": False},
                    "mid": {"range": (911, 919), "active": False},
                    "high": {"range": (920, 928), "active": False},
                },
                "current_subband": None,
                "frequency_hopping": True,  # Required for interference tolerance
                "channel_selection": True,  # Automatic channel selection
            },
            "2_4ghz": {
                "max_bandwidth_mhz": None,  # No FCC restriction, but interference-prone
                "interference_tolerant": True,
                "frequency_hopping": True,
                "channel_selection": True,
            },
            "current_band": "unknown",
            "subband_switching_enabled": True,
        }

        # Spectrum Compliance Monitoring
        self.spectrum_compliance = {
            "fcc_compliant": True,
            "current_band": "unknown",
            "current_subband": None,
            "bandwidth_usage_mhz": 0.0,
            "interference_detected": False,
            "compliance_violations": [],
            "monitoring_active": True,
        }

        # 2.4 GHz Interference Monitoring
        self.interference_monitor = {
            "wifi_congestion_detected": False,
            "competing_teams_detected": False,
            "environmental_noise": False,
            "channel_overlap": False,
            "last_interference_check": 0,
            "interference_threshold": 0.8,  # 80% interference triggers safety protocol
        }

    def monitor_spectrum_compliance(self) -> None:
        """Monitor spectrum usage and ensure FCC compliance per URC requirements."""
        if not self.spectrum_compliance["monitoring_active"]:
            return

        current_band = self.urc_band_config["current_band"]

        # Check bandwidth limits
        current_usage = self._measure_current_bandwidth()

        if current_band == "900mhz":
            max_bandwidth = self.urc_band_config["900mhz"]["max_bandwidth_mhz"]
            if max_bandwidth and current_usage > max_bandwidth:
                self._report_bandwidth_violation(
                    current_usage, max_bandwidth, current_band
                )

        # Check sub-band compliance for 900MHz
        if current_band == "900mhz":
            current_subband = self.urc_band_config["900mhz"]["current_subband"]
            if not current_subband:
                self._report_subband_violation(current_band)

        # Check for interference (2.4 GHz specific)
        if current_band == "2.4ghz":
            interference_sources = self._detect_interference_sources()

            # Trigger safety protocols if interference is critical
            total_interference = sum(interference_sources.values())
            if total_interference > self.interference_monitor["interference_threshold"]:
                self._trigger_2_4ghz_safety_protocol(
                    "critical_interference", interference_sources
                )

        # Update compliance tracking
        self.spectrum_compliance["current_band"] = current_band
        self.spectrum_compliance["bandwidth_usage_mhz"] = current_usage

    def _measure_current_bandwidth(self) -> float:
        """
        Measure current bandwidth usage.

        Returns:
            Current bandwidth usage in MHz
        """
        # In a real implementation, this would measure actual network traffic
        # For simulation, return a mock value
        base_usage = 2.0  # Base 2 MHz usage

        # Add some variability based on current band
        current_band = self.urc_band_config["current_band"]
        if current_band == "900mhz":
            # 900MHz typically has lower usage
            return base_usage * 0.8
        elif current_band == "2.4ghz":
            # 2.4GHz can be more variable
            return base_usage * 1.2
        else:
            return base_usage

    def _report_bandwidth_violation(
        self, current_usage: float, limit: float, band: str
    ) -> None:
        """
        Report a bandwidth violation.

        Args:
            current_usage: Current bandwidth usage in MHz
            limit: Bandwidth limit in MHz
            band: Frequency band
        """
        violation = {
            "timestamp": time.time(),
            "type": "bandwidth_exceeded",
            "current_usage_mhz": current_usage,
            "limit_mhz": limit,
            "band": band,
        }

        self.spectrum_compliance["compliance_violations"].append(violation)
        self.spectrum_compliance["fcc_compliant"] = False

        self.logger.error(
            f"FCC bandwidth violation: {current_usage:.1f}MHz > {limit:.1f}MHz on {band}"
        )

    def _report_subband_violation(self, band: str) -> None:
        """
        Report a sub-band violation.

        Args:
            band: Frequency band
        """
        violation = {
            "timestamp": time.time(),
            "type": "no_subband_selected",
            "band": band,
        }

        self.spectrum_compliance["compliance_violations"].append(violation)
        self.spectrum_compliance["fcc_compliant"] = False

        self.logger.error(f"Sub-band violation: No subband selected for {band}")

    def _detect_interference_sources(self) -> Dict[str, float]:
        """
        Detect various interference sources on 2.4 GHz.

        Returns:
            Dictionary of interference sources and their severity levels (0.0-1.0)
        """
        interference_sources = {
            "wifi_congestion": self._detect_wifi_congestion(),
            "team_interference": self._detect_competing_teams(),
            "environmental_noise": self._detect_environmental_noise(),
            "channel_overlap": self._detect_channel_overlap(),
        }

        # Update interference monitor state
        self.interference_monitor.update(
            {
                "wifi_congestion_detected": interference_sources["wifi_congestion"]
                > 0.5,
                "competing_teams_detected": interference_sources["team_interference"]
                > 0.5,
                "environmental_noise": interference_sources["environmental_noise"]
                > 0.5,
                "channel_overlap": interference_sources["channel_overlap"] > 0.5,
                "last_interference_check": time.time(),
            }
        )

        return interference_sources

    def _detect_wifi_congestion(self) -> float:
        """
        Detect WiFi congestion on 2.4 GHz.

        Returns:
            Congestion level (0.0-1.0)
        """
        # In a real implementation, this would scan for WiFi networks
        # and measure channel utilization
        # For simulation, return a mock value
        return 0.3  # 30% congestion

    def _detect_competing_teams(self) -> float:
        """
        Detect interference from competing URC teams.

        Returns:
            Competition interference level (0.0-1.0)
        """
        # In a real implementation, this would detect other URC team signals
        # For simulation, assume some competition
        return 0.2  # 20% competition interference

    def _detect_environmental_noise(self) -> float:
        """
        Detect environmental noise and interference.

        Returns:
            Environmental noise level (0.0-1.0)
        """
        # In a real implementation, this would measure background noise
        # For simulation, return low environmental noise
        return 0.1  # 10% environmental noise

    def _detect_channel_overlap(self) -> float:
        """
        Detect overlapping WiFi channels.

        Returns:
            Channel overlap level (0.0-1.0)
        """
        # In a real implementation, this would check if current channel
        # overlaps with adjacent channels
        # For simulation, check current channel
        current_channel = getattr(self, "current_wifi_channel", 6)

        # Check if current channel overlaps with adjacent channels
        overlap_penalty = 0.0
        if current_channel in [1, 2, 3, 4, 5]:  # Overlaps with channel 6
            overlap_penalty = 0.4
        elif current_channel in [8, 9, 10, 11]:  # Overlaps with channel 6
            overlap_penalty = 0.4

        return overlap_penalty

    def _trigger_2_4ghz_safety_protocol(
        self, trigger_type: str, interference_sources: Optional[Dict[str, float]] = None
    ) -> None:
        """
        Trigger 2.4 GHz safety protocol in response to interference.

        Args:
            trigger_type: Type of trigger ("critical_interference", "communication_loss", etc.)
            interference_sources: Optional interference source data
        """
        self.logger.warning(f"2.4 GHz safety protocol triggered: {trigger_type}")

        if trigger_type == "critical_interference":
            # Immediate channel switch
            # In a real implementation, this would trigger emergency communicator
            self.logger.warning(
                "Triggering emergency channel switch due to critical interference"
            )

        elif trigger_type == "communication_loss":
            # Activate emergency communication mode
            # In a real implementation, this would activate emergency communicator
            self.logger.warning(
                "Activating emergency communication mode due to communication loss"
            )

        elif trigger_type == "bandwidth_degradation":
            # Reduce communication load
            # In a real implementation, this would reduce telemetry rate
            self.logger.warning(
                "Reducing communication load due to bandwidth degradation"
            )

        elif trigger_type == "jitter":
            # Handle latency variation
            self.logger.warning("Handling jitter due to latency variation")

        elif trigger_type == "weak_signal":
            # Handle weak signal conditions
            self.logger.warning("Handling weak signal conditions")

    def set_urc_band(self, band: str, subband: Optional[str] = None) -> None:
        """
        Set the current URC frequency band.

        Args:
            band: Band to switch to ('900mhz' or '2.4ghz')
            subband: Subband for 900MHz ('low', 'mid', 'high')
        """
        if band not in ["900mhz", "2.4ghz"]:
            self.logger.error(f"Invalid band: {band}")
            return

        self.urc_band_config["current_band"] = band

        if band == "900mhz" and subband:
            if subband in self.urc_band_config["900mhz"]["sub_bands"]:
                # Deactivate all subbands
                for sb in self.urc_band_config["900mhz"]["sub_bands"].values():
                    sb["active"] = False

                # Activate selected subband
                self.urc_band_config["900mhz"]["sub_bands"][subband]["active"] = True
                self.urc_band_config["900mhz"]["current_subband"] = subband

                self.logger.info(
                    f"Switched to URC 900MHz {subband} subband "
                    f"({self.urc_band_config['900mhz']['sub_bands'][subband]['range']} MHz)"
                )
            else:
                self.logger.error(f"Invalid 900MHz subband: {subband}")

        elif band == "2.4ghz":
            self.logger.info("Switched to 2.4GHz band (interference monitoring active)")

    def get_current_band_limit(self) -> Optional[float]:
        """
        Get current band bandwidth limit in Mbps.

        Returns:
            Bandwidth limit in Mbps, or None if no limit
        """
        current_band = self.urc_band_config["current_band"]

        if current_band == "900mhz":
            return self.urc_band_config["900mhz"]["max_bandwidth_mhz"]
        elif current_band == "2.4ghz":
            return None  # No limit, but interference-prone
        else:
            return None

    def get_spectrum_status(self) -> Dict[str, Any]:
        """
        Get current spectrum monitoring status.

        Returns:
            Dictionary with spectrum status information
        """
        return {
            "fcc_compliant": self.spectrum_compliance["fcc_compliant"],
            "current_band": self.spectrum_compliance["current_band"],
            "current_subband": self.spectrum_compliance["current_subband"],
            "bandwidth_usage_mhz": self.spectrum_compliance["bandwidth_usage_mhz"],
            "interference_detected": self.spectrum_compliance["interference_detected"],
            "violation_count": len(self.spectrum_compliance["compliance_violations"]),
            "monitoring_active": self.spectrum_compliance["monitoring_active"],
            "band_limits": {
                "900mhz_max_mhz": self.urc_band_config["900mhz"]["max_bandwidth_mhz"],
                "current_limit_mhz": self.get_current_band_limit(),
            },
            "interference_status": self.interference_monitor.copy(),
        }

    def reset_compliance_violations(self) -> None:
        """Reset compliance violation tracking."""
        self.spectrum_compliance["compliance_violations"] = []
        self.spectrum_compliance["fcc_compliant"] = True
        self.logger.info("Spectrum compliance violations reset")

    def enable_monitoring(self, enabled: bool = True) -> None:
        """
        Enable or disable spectrum monitoring.

        Args:
            enabled: Whether to enable monitoring
        """
        self.spectrum_compliance["monitoring_active"] = enabled
        status = "enabled" if enabled else "disabled"
        self.logger.info(f"Spectrum monitoring {status}")

    def get_compliance_report(self) -> Dict[str, Any]:
        """
        Generate a compliance report for judging.

        Returns:
            Comprehensive compliance report
        """
        return {
            "spectrum_compliance": self.get_spectrum_status(),
            "urc_band_config": self.urc_band_config.copy(),
            "compliance_violations": self.spectrum_compliance[
                "compliance_violations"
            ].copy(),
            "generated_at": time.time(),
        }
