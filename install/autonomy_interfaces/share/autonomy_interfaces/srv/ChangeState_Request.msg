# Request to change system state with validation

# Request
string desired_state             # Target state to transition to
string desired_substate          # Optional: target substate (empty string if not applicable)
string desired_calibration_substate  # Optional: target calibration substate (empty string if not applicable)
string reason                    # Human-readable reason for state change
string operator_id               # ID of operator/system requesting change
bool force                       # If true, skip some validation checks (use with caution)
string[] metadata                # Additional key=value metadata pairs
