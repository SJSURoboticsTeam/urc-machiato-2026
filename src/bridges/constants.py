#!/usr/bin/env python3
"""
Competition Bridge Constants

Centralized constants for the competition bridge to improve maintainability
and reduce magic numbers throughout the codebase.
"""

# WebSocket Configuration
DEFAULT_WEBSOCKET_PORT = 8080
DEFAULT_MAX_WEBSOCKET_CLIENTS = 10

# Telemetry Configuration
DEFAULT_TELEMETRY_RATE_HZ = 5.0
MIN_TELEMETRY_RATE_HZ = 1.0
MAX_TELEMETRY_RATE_HZ = 10.0

# DDS Configuration
DEFAULT_DDS_DOMAIN_ID = 42

# Bandwidth and Performance
BANDWIDTH_TARGET_UTILIZATION = 0.7  # 70% of max bandwidth
LATENCY_TARGET_MS = 100.0
BANDWIDTH_MEASUREMENT_WINDOW_SEC = 10.0
ADAPTATION_RATE = 0.1

# URC Band Configuration (MHz)
URC_900MHZ_MAX_BANDWIDTH = 8.0
URC_900MHZ_LOW_RANGE = (902, 910)
URC_900MHZ_MID_RANGE = (911, 919)
URC_900MHZ_HIGH_RANGE = (920, 928)

# History buffer sizes
LATENCY_HISTORY_SIZE = 100
PACKET_LOSS_HISTORY_SIZE = 100
HEALTH_HISTORY_SIZE = 50

# Timeouts and intervals (seconds)
WEBSOCKET_PING_INTERVAL = 30
WEBSOCKET_CLOSE_TIMEOUT = 1
HEALTH_CHECK_INTERVAL = 5.0
STATE_SYNC_INTERVAL = 2.0
CONFIG_UPDATE_TIMEOUT = 30.0

# Thresholds
HEALTH_SCORE_THRESHOLD = 0.7
LOAD_DEGRADED_THRESHOLD = 0.8  # 80% load triggers degraded status
LOAD_CRITICAL_THRESHOLD = 0.9  # 90% load triggers critical status

# File paths
DEFAULT_COMPETITION_LOG_FILE = "competition_telemetry.jsonl"

# String constants
PRIMARY_ROLE = "primary"
SECONDARY_ROLE = "secondary"
TERTIARY_ROLE = "tertiary"
EMERGENCY_ROLE = "emergency"

# QoS Profiles
DEFAULT_QOS_DEPTH = 10
RELIABLE_QOS_DEPTH = 100
BEST_EFFORT_QOS_DEPTH = 1
