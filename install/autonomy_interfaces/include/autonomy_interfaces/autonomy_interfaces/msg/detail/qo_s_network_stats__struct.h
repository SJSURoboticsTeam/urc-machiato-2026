// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autonomy_interfaces:msg/QoSNetworkStats.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/msg/qo_s_network_stats.h"


#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__QO_S_NETWORK_STATS__STRUCT_H_
#define AUTONOMY_INTERFACES__MSG__DETAIL__QO_S_NETWORK_STATS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'current_band'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/QoSNetworkStats in the package autonomy_interfaces.
typedef struct autonomy_interfaces__msg__QoSNetworkStats
{
  double bandwidth_up_mbps;
  double bandwidth_down_mbps;
  double latency_ms;
  double packet_loss_rate;
  rosidl_runtime_c__String current_band;
  double signal_strength;
} autonomy_interfaces__msg__QoSNetworkStats;

// Struct for a sequence of autonomy_interfaces__msg__QoSNetworkStats.
typedef struct autonomy_interfaces__msg__QoSNetworkStats__Sequence
{
  autonomy_interfaces__msg__QoSNetworkStats * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__msg__QoSNetworkStats__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__QO_S_NETWORK_STATS__STRUCT_H_
