
# Response

bool success                     # True if recovery initiated/completed
string message                   # Human-readable status message
string recovery_state            # "IN_PROGRESS", "COMPLETED", "FAILED", "REQUIRES_ACTION"

# Recovery details
bool is_safe_to_proceed          # True if safe to continue operations
string[] remaining_steps         # Steps still needed for full recovery
string[] verified_systems        # Systems that passed verification
string[] failed_systems          # Systems that failed verification
float64 estimated_time           # Estimated time to complete recovery (seconds)

# Next state information
string recommended_next_state    # Recommended state after recovery
string[] restrictions            # Operational restrictions after recovery
