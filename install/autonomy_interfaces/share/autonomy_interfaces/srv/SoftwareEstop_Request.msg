# Software Emergency Stop Service
# Triggers full emergency shutdown via software (equivalent to hardware ESTOP)
#
# WARNING: This service triggers a critical emergency stop that requires manual
# intervention to recover from. Use only in actual emergency situations.
# Behavior is identical to pressing the physical emergency stop button.

# Request
string operator_id              # ID of operator triggering software ESTOP
string reason                   # Reason for triggering software ESTOP
bool acknowledge_criticality    # Operator acknowledges this is critical action
bool force_immediate            # Force immediate shutdown (no coordination)

