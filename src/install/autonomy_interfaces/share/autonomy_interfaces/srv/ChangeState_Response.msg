
# Response

bool success                     # True if transition successful
string actual_state              # Actual state after transition attempt
string actual_substate           # Actual substate after transition
string actual_calibration_substate  # Actual calibration substate after transition
float64 transition_time          # Time taken to complete transition (seconds)
string message                   # Human-readable status message

# Validation details
bool preconditions_met           # Were all preconditions satisfied
string[] failed_preconditions    # List of preconditions that failed
string[] warnings                # Non-critical warnings about the transition
