// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autonomy_interfaces:msg/AOIMetrics.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/msg/aoi_metrics.h"


#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__AOI_METRICS__STRUCT_H_
#define AUTONOMY_INTERFACES__MSG__DETAIL__AOI_METRICS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'health_status'
// Member 'active_alerts'
// Member 'network_recommendations'
#include "rosidl_runtime_c/string.h"
// Member 'last_alert_time'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in msg/AOIMetrics in the package autonomy_interfaces.
/**
  * AOIMetrics.msg
  * Detailed AoI metrics for system monitoring and diagnostics
 */
typedef struct autonomy_interfaces__msg__AOIMetrics
{
  std_msgs__msg__Header header;
  /// System-wide AoI summary
  /// Average AoI across all monitored components
  double system_average_aoi;
  /// Total number of sensors being monitored
  uint32_t total_sensors;
  /// Number of sensors with fresh data
  uint32_t fresh_sensors;
  /// Number of sensors with stale data
  uint32_t stale_sensors;
  /// Number of sensors with critical AoI
  uint32_t critical_sensors;
  /// AoI distribution (percentiles)
  /// 50th percentile AoI (median)
  double aoi_p50;
  /// 90th percentile AoI
  double aoi_p90;
  /// 95th percentile AoI
  double aoi_p95;
  /// 99th percentile AoI
  double aoi_p99;
  /// System health indicators
  /// Overall system AoI health
  bool system_healthy;
  /// "HEALTHY", "WARNING", "CRITICAL"
  rosidl_runtime_c__String health_status;
  /// 0.0-1.0 overall health score
  double health_score;
  /// Performance metrics
  /// Rate of AoI status updates
  double update_rate_hz;
  /// Number of updates dropped due to high load
  uint32_t dropped_updates;
  /// Average processing latency (seconds)
  double processing_latency;
  /// Alert information
  /// List of active AoI alerts
  rosidl_runtime_c__String__Sequence active_alerts;
  /// Number of active alerts
  uint32_t alert_count;
  /// Time of last alert
  builtin_interfaces__msg__Time last_alert_time;
  /// Network health metrics
  /// Number of sensors using serial transport
  uint32_t serial_sensors;
  /// Number of sensors using CAN transport
  uint32_t can_sensors;
  /// Number of sensors using ethernet transport
  uint32_t ethernet_sensors;
  /// Number of sensors using local transport
  uint32_t local_sensors;
  /// Average network latency across all sensors
  double avg_network_latency;
  /// Maximum network latency observed
  double max_network_latency;
  /// Number of transport links with congestion
  uint32_t congested_links;
  /// System recommendations
  /// Recommendations for network optimization
  rosidl_runtime_c__String__Sequence network_recommendations;
  /// 0.0-1.0 network health score
  double network_health_score;
} autonomy_interfaces__msg__AOIMetrics;

// Struct for a sequence of autonomy_interfaces__msg__AOIMetrics.
typedef struct autonomy_interfaces__msg__AOIMetrics__Sequence
{
  autonomy_interfaces__msg__AOIMetrics * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__msg__AOIMetrics__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__AOI_METRICS__STRUCT_H_
