// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autonomy_interfaces:msg/ContextState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/msg/context_state.h"


#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__CONTEXT_STATE__STRUCT_H_
#define AUTONOMY_INTERFACES__MSG__DETAIL__CONTEXT_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'mission_type'
// Member 'mission_status'
// Member 'safety_reason'
#include "rosidl_runtime_c/string.h"
// Member 'timestamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in msg/ContextState in the package autonomy_interfaces.
/**
  * ContextState.msg
  * Real-time context information for adaptive state machine
 */
typedef struct autonomy_interfaces__msg__ContextState
{
  /// Battery information
  float battery_level;
  float battery_voltage;
  bool battery_critical;
  bool battery_warning;
  /// Mission status
  rosidl_runtime_c__String mission_type;
  /// IDLE, EXECUTING, COMPLETED, FAILED, PAUSED
  rosidl_runtime_c__String mission_status;
  /// 0.0 to 1.0
  float mission_progress;
  /// seconds
  float mission_time_remaining;
  /// Communication health
  bool communication_active;
  /// seconds
  float communication_latency;
  /// 0-100
  int32_t communication_quality;
  /// System performance
  float cpu_usage;
  float memory_usage;
  float temperature;
  /// Environmental conditions
  bool obstacle_detected;
  float obstacle_distance;
  /// 0.0 (easy) to 1.0 (difficult)
  float terrain_difficulty;
  bool weather_adverse;
  /// Safety status
  bool safety_active;
  rosidl_runtime_c__String safety_reason;
  /// Timestamp
  builtin_interfaces__msg__Time timestamp;
} autonomy_interfaces__msg__ContextState;

// Struct for a sequence of autonomy_interfaces__msg__ContextState.
typedef struct autonomy_interfaces__msg__ContextState__Sequence
{
  autonomy_interfaces__msg__ContextState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__msg__ContextState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__CONTEXT_STATE__STRUCT_H_
