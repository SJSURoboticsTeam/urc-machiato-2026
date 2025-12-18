// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autonomy_interfaces:srv/GetSystemState.idl
// generated code does not contain a copyright notice

#ifndef AUTONOMY_INTERFACES__SRV__DETAIL__GET_SYSTEM_STATE__STRUCT_H_
#define AUTONOMY_INTERFACES__SRV__DETAIL__GET_SYSTEM_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GetSystemState in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__GetSystemState_Request
{
  /// Request
  /// If true, include recent state history
  bool include_history;
  /// If true, include subsystem status details
  bool include_subsystems;
  /// Number of recent transitions to include (default: 10)
  int32_t history_limit;
} autonomy_interfaces__srv__GetSystemState_Request;

// Struct for a sequence of autonomy_interfaces__srv__GetSystemState_Request.
typedef struct autonomy_interfaces__srv__GetSystemState_Request__Sequence
{
  autonomy_interfaces__srv__GetSystemState_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__GetSystemState_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// Member 'current_state'
// Member 'substate'
// Member 'sub_substate'
// Member 'recent_states'
// Member 'transition_reasons'
// Member 'active_subsystems'
// Member 'inactive_subsystems'
// Member 'failed_subsystems'
#include "rosidl_runtime_c/string.h"
// Member 'state_entered'
// Member 'state_timestamps'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in srv/GetSystemState in the package autonomy_interfaces.
typedef struct autonomy_interfaces__srv__GetSystemState_Response
{
  /// True if query successful
  bool success;
  /// Status message
  rosidl_runtime_c__String message;
  /// Current state information
  /// Current top-level state
  rosidl_runtime_c__String current_state;
  /// Current substate
  rosidl_runtime_c__String substate;
  /// Current sub-substate
  rosidl_runtime_c__String sub_substate;
  /// Seconds in current state
  double time_in_state;
  /// When current state was entered
  builtin_interfaces__msg__Time state_entered;
  /// History (if requested)
  /// List of recent states
  rosidl_runtime_c__String__Sequence recent_states;
  /// Timestamps for each state
  builtin_interfaces__msg__Time__Sequence state_timestamps;
  /// Reasons for each transition
  rosidl_runtime_c__String__Sequence transition_reasons;
  /// Subsystem status (if requested)
  /// Currently active subsystems
  rosidl_runtime_c__String__Sequence active_subsystems;
  /// Inactive subsystems
  rosidl_runtime_c__String__Sequence inactive_subsystems;
  /// Failed/error subsystems
  rosidl_runtime_c__String__Sequence failed_subsystems;
} autonomy_interfaces__srv__GetSystemState_Response;

// Struct for a sequence of autonomy_interfaces__srv__GetSystemState_Response.
typedef struct autonomy_interfaces__srv__GetSystemState_Response__Sequence
{
  autonomy_interfaces__srv__GetSystemState_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autonomy_interfaces__srv__GetSystemState_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTONOMY_INTERFACES__SRV__DETAIL__GET_SYSTEM_STATE__STRUCT_H_
