
# Response
bool success               # Whether query was successful
string message             # Status message

# Current status
autonomy_interfaces/AOIStatus[] sensor_status  # Status for each sensor

# Historical data (if requested)
float64[] aoi_history       # Historical AoI values
builtin_interfaces/Time[] timestamp_history  # Corresponding timestamps

# System summary
autonomy_interfaces/AOIMetrics system_metrics  # Overall system AoI metrics
