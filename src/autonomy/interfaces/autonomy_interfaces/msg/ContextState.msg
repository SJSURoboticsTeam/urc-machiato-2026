# ContextState.msg
# Real-time context information for adaptive state machine

# Battery information
float32 battery_level
float32 battery_voltage
bool battery_critical
bool battery_warning

# Mission status
string mission_type
string mission_status  # IDLE, EXECUTING, COMPLETED, FAILED, PAUSED
float32 mission_progress  # 0.0 to 1.0
float32 mission_time_remaining  # seconds

# Communication health
bool communication_active
float32 communication_latency  # seconds
int32 communication_quality  # 0-100

# System performance
float32 cpu_usage
float32 memory_usage
float32 temperature

# Environmental conditions
bool obstacle_detected
float32 obstacle_distance
float32 terrain_difficulty  # 0.0 (easy) to 1.0 (difficult)
bool weather_adverse

# Safety status
bool safety_active
string safety_reason

# Timestamp
builtin_interfaces/Time timestamp
