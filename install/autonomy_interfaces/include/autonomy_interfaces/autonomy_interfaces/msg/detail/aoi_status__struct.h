// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autonomy_interfaces:msg/AOIStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/msg/aoi_status.h"


#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__AOI_STATUS__STRUCT_H_
#define AUTONOMY_INTERFACES__MSG__DETAIL__AOI_STATUS__STRUCT_H_

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
// Member 'sensor_name'
// Member 'sensor_type'
// Member 'freshness_status'
// Member 'transport_type'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/AOIStatus in the package autonomy_interfaces.
/**
  * AOIStatus.msg
  * Age of Information status for sensor/system monitoring
 */
typedef struct autonomy_interfaces__msg__AOIStatus
{
  std_msgs__msg__Header header;
  /// Sensor identification
  /// Name of sensor/system (e.g., "imu", "gps", "slam_pose")
  rosidl_runtime_c__String sensor_name;
  /// Type category (e.g., "sensor", "system", "fusion")
  rosidl_runtime_c__String sensor_type;
  /// AoI metrics (seconds)
  /// Current age of information
  double current_aoi;
  /// Rolling average AoI over last N samples
  double average_aoi;
  /// Maximum AoI observed in window
  double max_aoi;
  /// Minimum AoI observed in window
  double min_aoi;
  /// Quality assessment
  /// Whether current data is within acceptable age
  bool is_fresh;
  /// 0.0-1.0 quality score based on AoI
  double quality_score;
  /// "FRESH", "ACCEPTABLE", "STALE", "CRITICAL"
  rosidl_runtime_c__String freshness_status;
  /// Configuration
  /// Maximum acceptable AoI (seconds)
  double acceptable_threshold;
  /// Optimal AoI threshold (seconds)
  double optimal_threshold;
  /// Statistics
  /// Number of samples in current window
  uint32_t sample_count;
  /// Ratio of fresh samples (0.0-1.0)
  double freshness_ratio;
  /// Network-aware metrics
  /// Transport type: "SERIAL", "CAN", "ETHERNET", "LOCAL"
  rosidl_runtime_c__String transport_type;
  /// Network transmission latency (seconds)
  double network_latency;
  /// Transport-specific latency (serial/CAN delays)
  double transport_latency;
  /// Whether network congestion was detected
  bool congestion_detected;
  /// Current congestion multiplier (1.0 = no congestion)
  double congestion_factor;
  /// Predictive metrics
  /// Predicted AOI for next sample
  double predicted_aoi;
  /// AOI trend (-1 decreasing, 0 stable, 1 increasing)
  double aoi_trend;
} autonomy_interfaces__msg__AOIStatus;

// Struct for a sequence of autonomy_interfaces__msg__AOIStatus.
typedef struct autonomy_interfaces__msg__AOIStatus__Sequence
{
  autonomy_interfaces__msg__AOIStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__msg__AOIStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__AOI_STATUS__STRUCT_H_
