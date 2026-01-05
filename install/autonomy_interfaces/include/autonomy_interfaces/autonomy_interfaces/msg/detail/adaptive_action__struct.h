// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autonomy_interfaces:msg/AdaptiveAction.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "autonomy_interfaces/msg/adaptive_action.h"


#ifndef AUTONOMY_INTERFACES__MSG__DETAIL__ADAPTIVE_ACTION__STRUCT_H_
#define AUTONOMY_INTERFACES__MSG__DETAIL__ADAPTIVE_ACTION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'action_type'
// Member 'parameters'
// Member 'success_criteria'
#include "rosidl_runtime_c/string.h"
// Member 'trigger_context'
#include "autonomy_interfaces/msg/detail/context_state__struct.h"
// Member 'timestamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in msg/AdaptiveAction in the package autonomy_interfaces.
/**
  * AdaptiveAction.msg
  * Actions taken by the adaptive state machine
 */
typedef struct autonomy_interfaces__msg__AdaptiveAction
{
  /// Action type
  /// EMERGENCY_RETURN, REDUCE_POWER, OBSTACLE_AVOIDANCE, etc.
  rosidl_runtime_c__String action_type;
  /// Action parameters
  /// Key-value pairs as strings
  rosidl_runtime_c__String__Sequence parameters;
  /// Context that triggered the action
  autonomy_interfaces__msg__ContextState trigger_context;
  /// Action priority (higher = more urgent)
  /// 0-100
  int32_t priority;
  /// Expected duration (seconds)
  float expected_duration;
  /// Success criteria
  rosidl_runtime_c__String success_criteria;
  /// Timestamp
  builtin_interfaces__msg__Time timestamp;
} autonomy_interfaces__msg__AdaptiveAction;

// Struct for a sequence of autonomy_interfaces__msg__AdaptiveAction.
typedef struct autonomy_interfaces__msg__AdaptiveAction__Sequence
{
  autonomy_interfaces__msg__AdaptiveAction * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__msg__AdaptiveAction__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTONOMY_INTERFACES__MSG__DETAIL__ADAPTIVE_ACTION__STRUCT_H_
