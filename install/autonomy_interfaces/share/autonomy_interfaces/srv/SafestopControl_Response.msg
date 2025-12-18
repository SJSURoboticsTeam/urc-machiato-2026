
# Response
bool success                # Command executed successfully
string message              # Human-readable status message
string current_state        # "ENGAGED", "DISENGAGED", "IN_PROGRESS", "FAILED"
string[] affected_subsystems # Subsystems that were affected
float64 timestamp           # When command was processed
string operator_id          # Operator who executed the command
