# Initiate recovery from safety state

# Request
string recovery_method           # "AUTO", "MANUAL_GUIDED", "FULL_RESET"
string operator_id               # ID of operator initiating recovery
bool acknowledge_risks           # Operator acknowledges recovery risks
string[] completed_steps         # Manual steps already completed (for verification)
string notes                     # Additional notes about recovery conditions

