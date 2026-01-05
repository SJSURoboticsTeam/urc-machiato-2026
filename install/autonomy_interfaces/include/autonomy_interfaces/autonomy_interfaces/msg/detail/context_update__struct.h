// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autonomy_interfaces:msg/ContextUpdate.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/msg/context_update.h"


#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__CONTEXT_UPDATE__STRUCT_H_
#define AUTONOMY_INTERFACES__MSG__DETAIL__CONTEXT_UPDATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'mission_status'
// Member 'active_adaptations'
// Member 'alert_level'
// Member 'available_actions'
#include "rosidl_runtime_c/string.h"
// Member 'timestamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in msg/ContextUpdate in the package autonomy_interfaces.
/**
  * ContextUpdate.msg
  * Simplified context update for dashboard display
 */
typedef struct autonomy_interfaces__msg__ContextUpdate
{
  /// Core status indicators
  float battery_level;
  rosidl_runtime_c__String mission_status;
  float mission_progress;
  bool communication_active;
  bool safety_active;
  /// Active adaptive actions
  rosidl_runtime_c__String__Sequence active_adaptations;
  /// Alert level
  /// NONE, WARNING, CRITICAL
  rosidl_runtime_c__String alert_level;
  /// Quick action buttons available
  rosidl_runtime_c__String__Sequence available_actions;
  /// Timestamp
  builtin_interfaces__msg__Time timestamp;
} autonomy_interfaces__msg__ContextUpdate;

// Struct for a sequence of autonomy_interfaces__msg__ContextUpdate.
typedef struct autonomy_interfaces__msg__ContextUpdate__Sequence
{
  autonomy_interfaces__msg__ContextUpdate * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__msg__ContextUpdate__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__CONTEXT_UPDATE__STRUCT_H_
