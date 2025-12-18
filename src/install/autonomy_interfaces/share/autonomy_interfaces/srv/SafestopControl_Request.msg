# Safestop Control Service
# Allows operator to engage/disengage/toggle safestop

# Request
string command              # "ENGAGE", "DISENGAGE", "TOGGLE"
string operator_id          # ID of operator initiating the command
bool acknowledge_risks      # Operator acknowledges any risks
string reason               # Optional reason for the command
string[] affected_subsystems # Specific subsystems to control (empty = all)

