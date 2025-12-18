# AdaptiveAction.msg
# Actions taken by the adaptive state machine

# Action type
string action_type  # EMERGENCY_RETURN, REDUCE_POWER, OBSTACLE_AVOIDANCE, etc.

# Action parameters
string[] parameters  # Key-value pairs as strings

# Context that triggered the action
ContextState trigger_context

# Action priority (higher = more urgent)
int32 priority  # 0-100

# Expected duration (seconds)
float32 expected_duration

# Success criteria
string success_criteria

# Timestamp
builtin_interfaces/Time timestamp


