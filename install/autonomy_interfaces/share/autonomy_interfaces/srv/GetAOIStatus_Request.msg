# GetAOIStatus.srv
# Service to query AoI status for sensors/systems

# Request
string sensor_name          # Specific sensor to query (empty for all)
bool include_history        # Whether to include historical data
uint32 history_samples      # Number of historical samples to return (0 = current only)

