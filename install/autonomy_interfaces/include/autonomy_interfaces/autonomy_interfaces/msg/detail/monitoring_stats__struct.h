// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autonomy_interfaces:msg/MonitoringStats.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/msg/monitoring_stats.h"


#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__MONITORING_STATS__STRUCT_H_
#define AUTONOMY_INTERFACES__MSG__DETAIL__MONITORING_STATS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/MonitoringStats in the package autonomy_interfaces.
typedef struct autonomy_interfaces__msg__MonitoringStats
{
  int32_t total_evaluations;
  int32_t total_violations;
  double evaluation_rate;
} autonomy_interfaces__msg__MonitoringStats;

// Struct for a sequence of autonomy_interfaces__msg__MonitoringStats.
typedef struct autonomy_interfaces__msg__MonitoringStats__Sequence
{
  autonomy_interfaces__msg__MonitoringStats * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__msg__MonitoringStats__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__MONITORING_STATS__STRUCT_H_
