# AOIMetrics.msg
# Detailed AoI metrics for system monitoring and diagnostics

std_msgs/Header header

# System-wide AoI summary
float64 system_average_aoi    # Average AoI across all monitored components
uint32 total_sensors         # Total number of sensors being monitored
uint32 fresh_sensors         # Number of sensors with fresh data
uint32 stale_sensors         # Number of sensors with stale data
uint32 critical_sensors      # Number of sensors with critical AoI

# AoI distribution (percentiles)
float64 aoi_p50             # 50th percentile AoI (median)
float64 aoi_p90             # 90th percentile AoI
float64 aoi_p95             # 95th percentile AoI
float64 aoi_p99             # 99th percentile AoI

# System health indicators
bool system_healthy         # Overall system AoI health
string health_status        # "HEALTHY", "WARNING", "CRITICAL"
float64 health_score        # 0.0-1.0 overall health score

# Performance metrics
float64 update_rate_hz      # Rate of AoI status updates
uint32 dropped_updates      # Number of updates dropped due to high load
float64 processing_latency  # Average processing latency (seconds)

# Alert information
string[] active_alerts      # List of active AoI alerts
uint32 alert_count          # Number of active alerts
builtin_interfaces/Time last_alert_time  # Time of last alert
