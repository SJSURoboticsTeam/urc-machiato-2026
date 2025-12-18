# ContextUpdate.msg
# Simplified context update for dashboard display

# Core status indicators
float32 battery_level
string mission_status
float32 mission_progress
bool communication_active
bool safety_active

# Active adaptive actions
string[] active_adaptations

# Alert level
string alert_level  # NONE, WARNING, CRITICAL

# Quick action buttons available
string[] available_actions

# Timestamp
builtin_interfaces/Time timestamp


